/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021 Dmitry Salychev <dsl@mcusim.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * QBMan software portal helper routines.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/condvar.h>
#include <sys/lock.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <vm/vm.h>              /* for vtophys */
#include <vm/pmap.h>            /* for vtophys */

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_swp.h"
#include "dpaa2_mc.h"

#define CMD_SLEEP_TIMEOUT		1u	/* ms */
#define CMD_SLEEP_ATTEMPTS		2000u	/* max. 2000 ms */
#define CMD_SPIN_TIMEOUT		100u	/* us */
#define CMD_SPIN_ATTEMPTS		20000u	/* max. 2000 ms */

#define CMD_VERB_MASK			(0x7Fu)

/* Shifts in the VERB byte of the enqueue command descriptor. */
#define ENQ_CMD_ORP_ENABLE_SHIFT	2
#define ENQ_CMD_IRQ_ON_DISPATCH_SHIFT	3
#define ENQ_CMD_TARGET_TYPE_SHIFT	4
#define ENQ_CMD_DCA_EN_SHIFT		7
/* VERB byte options of the enqueue command descriptor. */
#define ENQ_CMD_EMPTY			(0u)
#define ENQ_CMD_RESPONSE_ALWAYS		(1u)
#define ENQ_CMD_REJECTS_TO_FQ		(2u)

#define ENQ_DESC_FD_OFFSET		(32u)

#define ENQ_DCA_IDXMASK			(0x0Fu)
#define ENQ_FLAG_DCA			(1ull << 31)

/* QBMan portal command codes. */
#define CMDID_SWP_MC_ACQUIRE		(0x30)
#define CMDID_SWP_WQCHAN_CONFIGURE	(0x46)

/* QBMan portal command result codes. */
#define QBMAN_CMD_RC_OK			(0xF0)

MALLOC_DEFINE(M_DPAA2_SWP, "dpaa2_swp", "DPAA2 QBMan Software Portal");

/* Forward declarations. */

static int	swp_init_portal(dpaa2_swp_t *swp, dpaa2_swp_desc_t *desc,
		    const uint16_t flags, const bool atomic);
static int	swp_enq_direct(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd);
static int	swp_enq_memback(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd);
static int	swp_enq_mult_direct(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n);
static int	swp_enq_mult_memback(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n);
static uint8_t	cyc_diff(const uint8_t ringsize, const uint8_t first,
		    const uint8_t last);

static int	exec_command(dpaa2_swp_t swp, dpaa2_swp_cmd_t cmd,
		    dpaa2_swp_rsp_t rsp, const uint8_t cmdid);
static void	send_command(dpaa2_swp_t swp, dpaa2_swp_cmd_t cmd,
		    const uint8_t cmdid);
static int	wait_for_command(dpaa2_swp_t swp, dpaa2_swp_cmd_t cmd,
		    dpaa2_swp_rsp_t rsp);

/* Management routines. */

int
dpaa2_swp_init_portal(dpaa2_swp_t *swp, dpaa2_swp_desc_t *desc,
    const uint16_t flags)
{
	return (swp_init_portal(swp, desc, flags, false));
}

int
dpaa2_swp_init_atomic(dpaa2_swp_t *swp, dpaa2_swp_desc_t *desc,
    const uint16_t flags)
{
	return (swp_init_portal(swp, desc, flags, true));
}

static int
swp_init_portal(dpaa2_swp_t *portal, dpaa2_swp_desc_t *desc,
    const uint16_t flags, const bool atomic)
{
	const int mflags = flags & DPAA2_SWP_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO) : (M_WAITOK | M_ZERO);
	struct dpaa2_swp *p;
	uint32_t reg;
	uint32_t mask_size;
	uint32_t eqcr_pi;

	if (!portal || !desc)
		return (DPAA2_SWP_STAT_EINVAL);

	p = malloc(sizeof(struct dpaa2_swp), M_DPAA2_SWP, mflags);
	if (!p)
		return (DPAA2_SWP_STAT_NO_MEMORY);

	if (atomic) {
		/*
		 * NOTE: Do not initialize cv for an atomic software portal:
		 * it's not possible to sleep on it in case of a spin mutex.
		 */
		mtx_init(&p->lock, "QBMan portal spin lock", NULL, MTX_SPIN);
	} else {
		mtx_init(&p->lock, "QBMan portal sleep lock", NULL, MTX_DEF);
		cv_init(&p->cv, "QBMan portal cv");
	}

	p->cfg.atomic = atomic;
	p->cfg.mem_backed = false;
	p->cfg.writes_cinh = true;

	p->desc = desc;
	p->flags = flags;
	p->mc.valid_bit = DPAA2_SWP_VALID_BIT;
	p->mr.valid_bit = DPAA2_SWP_VALID_BIT;

	p->cena_res = desc->cena_res;
	p->cena_map = desc->cena_map;
	p->cinh_res = desc->cinh_res;
	p->cinh_map = desc->cinh_map;

	/* Dequeue Response Ring configuration */
	p->dqrr.next_idx = 0;
	p->dqrr.valid_bit = DPAA2_SWP_VALID_BIT;
	if ((desc->swp_version & DPAA2_SWP_REV_MASK) < DPAA2_SWP_REV_4100) {
		p->dqrr.ring_size = 4;
		p->dqrr.reset_bug = 1;
	} else {
		p->dqrr.ring_size = 8;
		p->dqrr.reset_bug = 0;
	}

	if ((desc->swp_version & DPAA2_SWP_REV_MASK) < DPAA2_SWP_REV_5000) {
		reg = dpaa2_swp_set_cfg(
		    p->dqrr.ring_size, /* max. entries QMan writes to DQRR */
		    1, /* writes enabled in the CINH memory only */
		    0, /* EQCR_CI stashing threshold */
		    3, /* RPM: RCR in array mode */
		    2, /* DCM: Discrete consumption ack */
		    2, /* EPM: EQCR in ring mode (FIFO) */
		    1, /* mem stashing drop enable enable */
		    1, /* mem stashing priority enable */
		    1, /* mem stashing enable */
		    1, /* dequeue stashing priority enable */
		    0, /* dequeue stashing enable enable */
		    0  /* EQCR_CI stashing priority enable */
		);
	} else {
		bus_set_region_4(p->cena_map, 0, 0,
		    rman_get_size(p->cena_res) / 4);
		reg = dpaa2_swp_set_cfg(
		    p->dqrr.ring_size, /* max. entries QMan writes to DQRR */
		    1, /* writes enabled in the CINH memory only */
		    1, /* EQCR_CI stashing threshold */
		    3, /* RPM: RCR in array mode */
		    2, /* DCM: Discrete consumption ack */
		    0, /* EPM: EQCR in ring mode (FIFO) */
		    1, /* mem stashing drop enable */
		    1, /* mem stashing priority enable */
		    1, /* mem stashing enable */
		    1, /* dequeue stashing priority enable */
		    0, /* dequeue stashing enable */
		    0  /* EQCR_CI stashing priority enable */
		);
	}
	dpaa2_swp_write_reg(p, DPAA2_SWP_CINH_CFG, reg);
	reg = dpaa2_swp_read_reg(p, DPAA2_SWP_CINH_CFG);
	if (!reg) {
		free(p, M_DPAA2_SWP);
		return (DPAA2_SWP_STAT_PORTAL_DISABLED);
	}

	/*
	 * Static Dequeue Command Register needs to be initialized to 0 when no
	 * channels are being dequeued from or else the QMan HW will indicate an
	 * error. The values that were calculated above will be applied when
	 * dequeues from a specific channel are enabled.
	 */
	dpaa2_swp_write_reg(p, DPAA2_SWP_CINH_SDQCR, 0);

	p->enq = swp_enq_direct;
	p->enq_mult = swp_enq_mult_direct;
	p->eqcr.pi_ring_size = 8;

	if ((desc->swp_version & DPAA2_SWP_REV_MASK) >= DPAA2_SWP_REV_5000) {
		p->eqcr.pi_ring_size = 32;
		p->enq = swp_enq_memback;
		p->enq_mult = swp_enq_mult_memback;
		/* qbman_swp_enqueue_multiple_desc_ptr = */
		/*     qbman_swp_enqueue_multiple_desc_mem_back; */
		/* qbman_swp_pull_ptr = qbman_swp_pull_mem_back; */
		/* qbman_swp_dqrr_next_ptr = qbman_swp_dqrr_next_mem_back; */
		/* qbman_swp_release_ptr = qbman_swp_release_mem_back; */
	}

	for (mask_size = p->eqcr.pi_ring_size; mask_size > 0; mask_size >>= 1)
		p->eqcr.pi_ci_mask = (p->eqcr.pi_ci_mask << 1) + 1;

	eqcr_pi = dpaa2_swp_read_reg(p, DPAA2_SWP_CINH_EQCR_PI);
	p->eqcr.pi = eqcr_pi & p->eqcr.pi_ci_mask;
	p->eqcr.pi_vb = eqcr_pi & DPAA2_SWP_VALID_BIT;
	p->eqcr.ci = dpaa2_swp_read_reg(p, DPAA2_SWP_CINH_EQCR_CI)
	    & p->eqcr.pi_ci_mask;
	p->eqcr.available = p->eqcr.pi_ring_size;

	*portal = p;

	return (0);
}

void
dpaa2_swp_free_portal(dpaa2_swp_t swp)
{
	uint16_t flags;

	if (swp) {
		if (swp->cfg.atomic) {
			mtx_destroy(&swp->lock);
			free(swp, M_DPAA2_SWP);
		} else {
			/*
			 * Signal all threads sleeping on portal's cv that it's
			 * going to be destroyed.
			 */
			dpaa2_swp_lock(swp, &flags);
			swp->flags |= DPAA2_SWP_DESTROYED;
			cv_signal(&swp->cv);
			dpaa2_swp_unlock(swp);

			/* Let threads stop using this portal. */
			DELAY(DPAA2_SWP_TIMEOUT);

			mtx_destroy(&swp->lock);
			cv_destroy(&swp->cv);
			free(swp, M_DPAA2_SWP);
		}
	}
}

void
dpaa2_swp_lock(dpaa2_swp_t swp, uint16_t *flags)
{
	mtx_assert(&swp->lock, MA_NOTOWNED);

	if (swp->cfg.atomic) {
		mtx_lock_spin(&swp->lock);
		*flags = swp->flags;
	} else {
		mtx_lock(&swp->lock);
		while (swp->flags & DPAA2_SWP_LOCKED)
			cv_wait(&swp->cv, &swp->lock);
		*flags = swp->flags;
		swp->flags |= DPAA2_SWP_LOCKED;
		mtx_unlock(&swp->lock);
	}
}

void
dpaa2_swp_unlock(dpaa2_swp_t swp)
{
	if (swp->cfg.atomic) {
		mtx_unlock_spin(&swp->lock);
	} else {
		mtx_lock(&swp->lock);
		swp->flags &= ~DPAA2_SWP_LOCKED;
		cv_signal(&swp->cv);
		mtx_unlock(&swp->lock);
	}
}

uint32_t
dpaa2_swp_set_cfg(uint8_t max_fill, uint8_t wn, uint8_t est, uint8_t rpm,
    uint8_t dcm, uint8_t epm, int sd, int sp, int se, int dp, int de, int ep)
{
	return (
	    max_fill	<< DPAA2_SWP_CFG_DQRR_MF_SHIFT |
	    est		<< DPAA2_SWP_CFG_EST_SHIFT |
	    wn		<< DPAA2_SWP_CFG_WN_SHIFT |
	    rpm		<< DPAA2_SWP_CFG_RPM_SHIFT |
	    dcm		<< DPAA2_SWP_CFG_DCM_SHIFT |
	    epm		<< DPAA2_SWP_CFG_EPM_SHIFT |
	    sd		<< DPAA2_SWP_CFG_SD_SHIFT |
	    sp		<< DPAA2_SWP_CFG_SP_SHIFT |
	    se		<< DPAA2_SWP_CFG_SE_SHIFT |
	    dp		<< DPAA2_SWP_CFG_DP_SHIFT |
	    de		<< DPAA2_SWP_CFG_DE_SHIFT |
	    ep		<< DPAA2_SWP_CFG_EP_SHIFT
	);
}

/* Read/write registers of a software portal. */

void
dpaa2_swp_write_reg(dpaa2_swp_t swp, uint32_t offset, uint32_t val)
{
	bus_write_4(swp->cinh_map, offset, val);
}

uint32_t
dpaa2_swp_read_reg(dpaa2_swp_t swp, uint32_t offset)
{
	return (bus_read_4(swp->cinh_map, offset));
}

/* Helper routines. */

/**
 * @brief Clear enqueue descriptor.
 */
void
dpaa2_swp_clear_ed(dpaa2_eq_desc_t *ed)
{
	memset(ed, 0, sizeof(*ed));
}

/**
 * @brief Set enqueue descriptor without Order Point Record ID.
 *
 * ed:			Enqueue descriptor.
 * response_always:	Enqueue with response always (1); FD from a rejected
 * 			enqueue will be returned on a FQ (0).
 */
void
dpaa2_swp_set_ed_norp(dpaa2_eq_desc_t *ed, int response_always)
{
	ed->verb &= ~(1 << ENQ_CMD_ORP_ENABLE_SHIFT);
	if (response_always)
		ed->verb |= ENQ_CMD_RESPONSE_ALWAYS;
	else
		ed->verb |= ENQ_CMD_REJECTS_TO_FQ;
}

/**
 * @brief Set FQ of the enqueue descriptor.
 */
void
dpaa2_swp_set_ed_fq(dpaa2_eq_desc_t *ed, uint32_t fqid)
{
	ed->verb &= ~(1 << ENQ_CMD_TARGET_TYPE_SHIFT);
	ed->tgtid = fqid;
}

/**
 * @brief Enable interrupts for a software portal.
 */
void
dpaa2_swp_set_intr_trigger(dpaa2_swp_t swp, uint32_t mask)
{
	if (swp)
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_IER, mask);
	else
		printf("%s failed\n", __func__);
}

/**
 * @brief Return the value in the SWP_IER register.
 */
uint32_t
dpaa2_swp_get_intr_trigger(dpaa2_swp_t swp)
{
	if (swp)
		return dpaa2_swp_read_reg(swp, DPAA2_SWP_CINH_IER);
	else
		printf("%s failed\n", __func__);

	return (0);
}

/**
 * @brief Return the value in the SWP_ISR register.
 */
uint32_t
dpaa2_swp_read_intr_status(dpaa2_swp_t swp)
{
	if (swp)
		return dpaa2_swp_read_reg(swp, DPAA2_SWP_CINH_ISR);
	else
		printf("%s failed\n", __func__);

	return (0);
}

/**
 * @brief Clear SWP_ISR register according to the given mask.
 */
void
dpaa2_swp_clear_intr_status(dpaa2_swp_t swp, uint32_t mask)
{
	if (swp)
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_ISR, mask);
	else
		printf("%s failed\n", __func__);
}

/**
 * @brief Enable or disable push dequeue.
 *
 * swp:		the software portal object
 * chan_idx:	the channel index (0 to 15)
 * en:		enable or disable push dequeue
 */
void
dpaa2_swp_set_push_dequeue(dpaa2_swp_t swp, uint8_t chan_idx, bool en)
{
	uint16_t dqsrc;

	if (chan_idx > 15) {
		printf("%s: channel index should be <= 15: chan_idx=%d\n",
		    __func__, chan_idx);
		return;
	}

	if (swp) {
		if (en)
			swp->sdq |= 1 << chan_idx;
		else
			swp->sdq &= ~(1 << chan_idx);
		/*
		 * Read make the complete src map. If no channels are enabled
		 * the SDQCR must be 0 or else QMan will assert errors.
		 */
		dqsrc = (swp->sdq >> DPAA2_SDQCR_SRC_SHIFT) &
		    DPAA2_SDQCR_SRC_MASK;
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_SDQCR, dqsrc != 0
		    ? swp->sdq : 0);
	} else
		printf("%s failed\n", __func__);
}

/**
 * @brief Configure the channel data availability notification (CDAN)
 * in a particular WQ channel.
 */
int
dpaa2_swp_conf_wq_channel(dpaa2_swp_t swp, uint16_t chan_id, uint8_t we_mask,
    bool cdan_en, uint64_t ctx)
{
	/* NOTE: 64 bytes command. */
	struct __packed {
		uint8_t		verb;
		uint8_t		result; /* in response only! */
		uint16_t	chan_id;
		uint8_t		we;
		uint8_t		ctrl;
		uint16_t	_reserved2;
		uint64_t	ctx;
		uint8_t		_reserved3[48];
	} cmd;
	struct __packed {
		uint8_t		verb;
		uint8_t		result;
		uint16_t	chan_id;
		uint8_t		_reserved[60];
	} rsp;
	int error;

	memset(&cmd, 0, sizeof(cmd));
	cmd.chan_id = chan_id;
	cmd.we = we_mask;
	cmd.ctrl = cdan_en ? 1 : 0;
	cmd.ctx = ctx;

	error = exec_command(swp, (dpaa2_swp_cmd_t) &cmd, (dpaa2_swp_rsp_t) &rsp,
	    CMDID_SWP_WQCHAN_CONFIGURE);
	if (error) {
		printf("%s: WQ channel configuration failed: error=%d\n",
		    __func__, error);
		return (EIO);
	}

	if (rsp.result != QBMAN_CMD_RC_OK) {
		printf("%s: WQ channel configuration error: channel_id=%d, "
		    "result=0x%02x\n", __func__, chan_id, rsp.result);
		return (EIO);
	}

	return (0);
}

/*
 * Internal functions.
 */

/**
 * @internal
 * @brief Issue a command to enqueue a frame using one enqueue descriptor.
 *
 * swp:		Software portal used to send this command to.
 * ed:		Enqueue command descriptor.
 * fd:		Frame descriptor to enqueue.
 */
static int
swp_enq_direct(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed, const dpaa2_fd_t *fd)
{
	/* TBD */
	return (0);
}

/**
 * @internal
 * @brief Issue a command to enqueue a frame using one enqueue descriptor.
 *
 * swp:		Software portal used to send this command to.
 * ed:		Enqueue command descriptor.
 * fd:		Frame descriptor to enqueue.
 */
static int
swp_enq_memback(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed, const dpaa2_fd_t *fd)
{
	uint32_t flags = 0;
	int rc = swp_enq_mult_memback(swp, ed, fd, &flags, 1);

	return (rc >= 0 ? 0 : EBUSY);
}

/**
 * @internal
 * @brief Issue a command to enqueue frames using one enqueue descriptor.
 *
 * swp:		Software portal used to send this command to.
 * ed:		Enqueue command descriptor.
 * fd:		Frame descriptor to enqueue.
 * flags:	Table pointer of QBMAN_ENQUEUE_FLAG_DCA flags, not used if NULL.
 * frames_n:	Number of FDs to enqueue.
 */
static int
swp_enq_mult_direct(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n)
{
	/* TBD */
	return (0);
}

/**
 * @internal
 * @brief Issue a command to enqueue frames using one enqueue descriptor.
 *
 * swp:		Software portal used to send this command to.
 * ed:		Enqueue command descriptor.
 * fd:		Frame descriptor to enqueue.
 * flags:	Table pointer of QBMAN_ENQUEUE_FLAG_DCA flags, not used if NULL.
 * frames_n:	Number of FDs to enqueue.
 *
 * NOTE: Enqueue command is 64 bytes long: 32 bytes of the enqueue descriptor +
 *       32 bytes of the frame descriptor.
 */
static int
swp_enq_mult_memback(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n)
{
	const uint8_t  *ed_pdat8 =  (const uint8_t *) ed;
	const uint32_t *ed_pdat32 = (const uint32_t *) ed;
	const uint64_t *ed_pdat64 = (const uint64_t *) ed;
	const uint64_t *fd_pdat64 = (const uint64_t *) fd;
	uint32_t eqcr_ci; /* consumer index */
	uint32_t eqcr_pi; /* producer index */
	uint32_t half_mask, full_mask;
	uint16_t swp_flags;
	int num_enq = 0;
	uint32_t val;

	dpaa2_swp_lock(swp, &swp_flags);

	half_mask = swp->eqcr.pi_ci_mask >> 1;
	full_mask = swp->eqcr.pi_ci_mask;

	if (!swp->eqcr.available) {
		val = bus_read_4(swp->cena_map, DPAA2_SWP_CENA_EQCR_CI_MEMBACK);
		eqcr_ci = swp->eqcr.ci;
		swp->eqcr.ci = val & full_mask;
		swp->eqcr.available = cyc_diff(swp->eqcr.pi_ring_size,
		    eqcr_ci, swp->eqcr.ci);

		if (!swp->eqcr.available) {
			dpaa2_swp_unlock(swp);
			return (0);
		}
	}

	eqcr_pi = swp->eqcr.pi;
	num_enq = swp->eqcr.available < frames_n
	    ? swp->eqcr.available : frames_n;
	swp->eqcr.available -= num_enq;

	/* Fill in the EQCR ring. */
	for (int i = 0; i < num_enq; i++) {
		/* Write enq. desc. without the VERB, DCA, SEQNUM and OPRID. */
		for (int j = 1; j <= 3; j++)
			bus_write_8(swp->cena_map,
			    DPAA2_SWP_CENA_EQCR(eqcr_pi & half_mask) +
			    sizeof(uint64_t) * j, ed_pdat64[j]);
		/* Write OPRID. */
		bus_write_4(swp->cena_map,
		    DPAA2_SWP_CENA_EQCR(eqcr_pi & half_mask) + sizeof(uint32_t),
		    ed_pdat32[1]);
		/* Write DCA and SEQNUM without VERB byte. */
		for (int j = 1; j <= 3; j++)
			bus_write_1(swp->cena_map,
			    DPAA2_SWP_CENA_EQCR(eqcr_pi & half_mask) +
			    sizeof(uint8_t) * j, ed_pdat8[j]);

		/* Write frame descriptor. */
		for (int j = 0; j <= 3; j++)
			bus_write_8(swp->cena_map,
			    DPAA2_SWP_CENA_EQCR(eqcr_pi & half_mask) +
			    ENQ_DESC_FD_OFFSET +
			    sizeof(uint64_t) * j, fd_pdat64[j]);
		eqcr_pi++;
	}

	/* Write the VERB byte of enqueue descriptor. */
	eqcr_pi = swp->eqcr.pi;
	for (int i = 0; i < num_enq; i++) {
		bus_write_1(swp->cena_map,
		    DPAA2_SWP_CENA_EQCR(eqcr_pi & half_mask),
		    ed_pdat8[0] | swp->eqcr.pi_vb);

		if (flags && (flags[i] & ENQ_FLAG_DCA)) {
			/* Update DCA byte. */
			bus_write_1(swp->cena_map,
			    DPAA2_SWP_CENA_EQCR(eqcr_pi & half_mask) + 1,
			    (1 << ENQ_CMD_DCA_EN_SHIFT) |
			    (flags[i] & ENQ_DCA_IDXMASK));
		}
		eqcr_pi++;
		if (!(eqcr_pi & half_mask))
			swp->eqcr.pi_vb ^= DPAA2_SWP_VALID_BIT;
	}
	swp->eqcr.pi = eqcr_pi & full_mask;

	bus_barrier(swp->cena_map, 0, rman_get_size(swp->cena_res),
	    BUS_SPACE_BARRIER_WRITE);
	bus_barrier(swp->cinh_map, 0, rman_get_size(swp->cinh_res),
	    BUS_SPACE_BARRIER_WRITE);

	dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_EQCR_PI, 
	    DPAA2_SWP_RT_MODE | swp->eqcr.pi | swp->eqcr.pi_vb);

	dpaa2_swp_unlock(swp);

	return (num_enq);
}

/**
 * @internal
 */
static uint8_t
cyc_diff(const uint8_t ringsize, const uint8_t first, const uint8_t last)
{
	/* 'first' is included, 'last' is excluded */
	if (first <= last)
		return (last - first);
	else
		return (2 * ringsize) - (first - last);
}

/**
 * @internal
 */
static int
exec_command(dpaa2_swp_t swp, dpaa2_swp_cmd_t cmd, dpaa2_swp_rsp_t rsp,
    const uint8_t cmdid)
{
	struct __packed with_verb {
		uint8_t	verb;
		uint8_t	_reserved[63];
	} *r;
	uint16_t flags;
	int error;

	if (!swp || !cmd)
		return (EINVAL);

	dpaa2_swp_lock(swp, &flags);
	if (flags & DPAA2_SWP_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_swp_unlock(swp);
		return (ENOENT);
	}

	/* Send a command to QBMan and wait for the result. */
	send_command(swp, cmd, cmdid);
	error = wait_for_command(swp, cmd, rsp);
	if (error) {
		dpaa2_swp_unlock(swp);
		return (error);
	}
	dpaa2_swp_unlock(swp);

	r = (struct with_verb *) rsp;
	KASSERT((r->verb & CMD_VERB_MASK) == cmdid,
	    ("wrong VERB byte in response: resp=0x%02x, expected=0x%02x",
	    r->verb, cmdid));

	return (0);
}

/**
 * @internal
 */
static void
send_command(dpaa2_swp_t swp, dpaa2_swp_cmd_t cmd, const uint8_t cmdid)
{
	const uint8_t *cmd_pdat8 = (const uint8_t *) cmd->params;
	const uint32_t *cmd_pdat32 = (const uint32_t *) cmd->params;
	const uint32_t offset = swp->cfg.mem_backed ? DPAA2_SWP_CENA_CR_MEM
	    : DPAA2_SWP_CENA_CR;
	struct resource_map *map = swp->cfg.writes_cinh ? swp->cinh_map
	    : swp->cena_map;

	/* Write command bytes (without VERB byte). */
	for (uint32_t i = 1; i < DPAA2_SWP_CMD_PARAMS_N; i++)
		bus_write_8(map, offset + sizeof(uint64_t) * i, cmd->params[i]);
	bus_write_4(map, offset + 4, cmd_pdat32[1]);
	for (uint32_t i = 1; i <= 3; i++)
		bus_write_1(map, offset + i, cmd_pdat8[i]);

	/* Write VERB byte and trigger command execution. */
	if (swp->cfg.mem_backed) {
		bus_write_1(map, offset, cmdid | swp->mr.valid_bit);
		wmb();
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_CR_RT,
		    DPAA2_SWP_RT_MODE);
	} else {
		wmb();
		bus_write_1(map, offset, cmdid | swp->mc.valid_bit);
	}
}

/**
 * @internal
 */
static int
wait_for_command(dpaa2_swp_t swp, dpaa2_swp_cmd_t cmd, dpaa2_swp_rsp_t rsp)
{
	const uint32_t attempts = swp->cfg.atomic ? CMD_SPIN_ATTEMPTS
	    : CMD_SLEEP_ATTEMPTS;
	struct resource_map *map = swp->cena_map;
	uint32_t i, offset;
	uint8_t verb;
	int rc;

	/* Wait for QBMan to complete writing a response. */
	dsb(osh);

	/* Wait for a command response from QBMan. */
	for (i = 1; i <= attempts; i++) {
		if (swp->cfg.mem_backed) {
			/* Command response to be read from the only RR. */
			offset = DPAA2_SWP_CENA_RR_MEM;

			verb = bus_read_1(map, offset);
			if (swp->mr.valid_bit != (verb & DPAA2_SWP_VALID_BIT))
				goto wait;
			if (!(verb & ~DPAA2_SWP_VALID_BIT))
				goto wait;
			swp->mr.valid_bit ^= DPAA2_SWP_VALID_BIT;
		} else {
			/* Command response to be read from RR0/RR1. */
			offset = DPAA2_SWP_CENA_RR(swp->mc.valid_bit);

			verb = bus_read_1(map, offset);
			if (!(verb & ~DPAA2_SWP_VALID_BIT))
				goto wait;
			swp->mc.valid_bit ^= DPAA2_SWP_VALID_BIT;
		}
		break;
 wait:
		if (swp->cfg.atomic)
			DELAY(CMD_SPIN_TIMEOUT);
		else
			pause("dpaa2", CMD_SLEEP_TIMEOUT);
	}
	/* Return an error on expired timeout. */
	rc = i > attempts ? ETIMEDOUT : 0;

	/* Read command response. */
	for (i = 0; i < DPAA2_SWP_RSP_PARAMS_N; i++)
		rsp->params[i] = bus_read_8(map, offset + i * sizeof(uint64_t));

	return (rc);
}
