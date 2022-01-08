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
#include <machine/atomic.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_swp.h"
#include "dpaa2_mc.h"

#define CMD_SLEEP_TIMEOUT		1u	/* ms */
#define CMD_SLEEP_ATTEMPTS		2000u	/* max. 2000 ms */
#define CMD_SPIN_TIMEOUT		100u	/* us */
#define CMD_SPIN_ATTEMPTS		20000u	/* max. 2000 ms */

#define CMD_VERB_MASK			0x7Fu

/* Shifts in the VERB byte of the enqueue command descriptor. */
#define ENQ_CMD_ORP_ENABLE_SHIFT	2
#define ENQ_CMD_IRQ_ON_DISPATCH_SHIFT	3
#define ENQ_CMD_TARGET_TYPE_SHIFT	4
#define ENQ_CMD_DCA_EN_SHIFT		7
/* VERB byte options of the enqueue command descriptor. */
#define ENQ_CMD_EMPTY			0u
#define ENQ_CMD_RESPONSE_ALWAYS		1u
#define ENQ_CMD_REJECTS_TO_FQ		2u

#define ENQ_DESC_FD_OFFSET		32u

#define ENQ_DCA_IDXMASK			0x0Fu
#define ENQ_FLAG_DCA			(1ull << 31)

/* QBMan portal command codes. */
#define CMDID_SWP_MC_ACQUIRE		0x30
#define CMDID_SWP_WQCHAN_CONFIGURE	0x46

/* QBMan portal command result codes. */
#define QBMAN_CMD_RC_OK			0xF0

/* SDQCR attribute codes */
#define QB_SDQCR_FC_SHIFT 		29u
#define QB_SDQCR_FC_MASK		0x1u
#define QB_SDQCR_DCT_SHIFT		24u
#define QB_SDQCR_DCT_MASK		0x3u
#define QB_SDQCR_TOK_SHIFT		16u
#define QB_SDQCR_TOK_MASK		0xFFu
#define QB_SDQCR_SRC_SHIFT		0u
#define QB_SDQCR_SRC_MASK		0xFFFFu

/* Opaque token for static dequeues. */
#define QMAN_SDQCR_TOKEN		0xBBu

/* Shifts in the VERB byte of the volatile dequeue command. */
#define QB_VDQCR_VERB_DCT_SHIFT		0
#define QB_VDQCR_VERB_DT_SHIFT		2
#define QB_VDQCR_VERB_RLS_SHIFT		4
#define QB_VDQCR_VERB_WAE_SHIFT		5

/* Opaque token for static dequeues. */
#define QMAN_VDQCR_TOKEN		0xCCu

/* Maximum timeout period for the DQRR interrupt. */
#define DQRR_MAX_ITP			4096u
#define DQRR_PI_MASK			0x0Fu

/* Release Array Allocation register helpers. */
#define RAR_IDX(rar)			((rar) & 0x7u)
#define RAR_VB(rar)			((rar) & 0x80u)
#define RAR_SUCCESS(rar)		((rar) & 0x100u)

MALLOC_DEFINE(M_DPAA2_SWP, "dpaa2_swp", "DPAA2 QBMan Software Portal");

enum qbman_sdqcr_dct {
	qbman_sdqcr_dct_null = 0,
	qbman_sdqcr_dct_prio_ics,
	qbman_sdqcr_dct_active_ics,
	qbman_sdqcr_dct_active
};

enum qbman_sdqcr_fc {
	qbman_sdqcr_fc_one = 0,
	qbman_sdqcr_fc_up_to_3 = 1
};

/* Forward declarations. */

static int swp_enq_direct(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd);
static int swp_enq_memback(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd);
static int swp_enq_mult_direct(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd, uint32_t *flags, int frames_n);
static int swp_enq_mult_memback(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd, uint32_t *flags, int frames_n);

static int cyc_diff(uint8_t ring_sz, uint8_t first, uint8_t last);

/* Routines to execute software portal commands. */

static int exec_mgmt_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd,
    struct dpaa2_swp_rsp *rsp, uint8_t cmdid);
static int exec_br_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd,
    uint32_t buf_num);
static int exec_vdc_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd);

/* Management Commands helpers. */

static int send_mgmt_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd,
    uint8_t cmdid);
static int wait_for_mgmt_response(struct dpaa2_swp *swp,
    struct dpaa2_swp_rsp *rsp);

/* Atomic access routines. */

#define	atomic_inc_and_test(v) (atomic_add_return(1, (v)) == 0)
#define	atomic_dec_and_test(v) (atomic_sub_return(1, (v)) == 0)

static inline int atomic_add_return(int i, atomic_t *v);
static inline int atomic_sub_return(int i, atomic_t *v);
static inline int atomic_xchg(atomic_t *v, int i);
static inline int atomic_inc(atomic_t *v);
static inline int atomic_dec(atomic_t *v);

/* Management routines. */

int
dpaa2_swp_init_portal(struct dpaa2_swp **swp, struct dpaa2_swp_desc *desc,
    uint16_t flags, bool atomic)
{
	struct dpaa2_swp *p;
	uint32_t reg, mask_size, eqcr_pi; /* EQCR producer index */

	if (!swp || !desc)
		return (DPAA2_SWP_STAT_EINVAL);

	p = malloc(sizeof(struct dpaa2_swp), M_DPAA2_SWP,
	    flags & DPAA2_SWP_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO)
	    : (M_WAITOK | M_ZERO));
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

	/* Static Dequeue Command Register configuration. */
	p->sdq = 0;
	p->sdq |= qbman_sdqcr_dct_prio_ics << QB_SDQCR_DCT_SHIFT;
	p->sdq |= qbman_sdqcr_fc_up_to_3 << QB_SDQCR_FC_SHIFT;
	p->sdq |= QMAN_SDQCR_TOKEN << QB_SDQCR_TOK_SHIFT;

	/* Volatile Dequeue Command configuration. */
	atomic_xchg(&p->vdq.avail, 1);
	p->vdq.valid_bit = DPAA2_SWP_VALID_BIT;

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
		reg &= ~(1 << DPAA2_SWP_CFG_CPBS_SHIFT); /* QMan-backed mode */
	} else {
		bus_set_region_4(p->cena_map, 0, 0,
		    rman_get_size(p->cena_res) / 4);

		reg = dpaa2_swp_set_cfg(
		    p->dqrr.ring_size, /* max. entries QMan writes to DQRR */					/* DQRR_MF */
		    1, /* writes enabled in the CINH memory only */						/* WN */
		    1, /* EQCR_CI stashing threshold */								/* EST */
		    3, /* RPM: RCR in array mode */								/* RPM */
		    2, /* DCM: Discrete consumption ack */							/* DCM */
		    0, /* EPM: EQCR in ring mode (FIFO) */							/* EPM */
		    1, /* Dequeued frame data, annotation, and FQ context stashing drop enable */		/* SD */
		    1, /* Dequeued frame data, annotation, and FQ context stashing priority */			/* SP */
		    1, /* Dequeued frame data, annotation, and FQ context stashing enable */			/* SE */
		    1, /* Dequeue response ring (DQRR) entry stashing priority */				/* DP */
		    0, /* Dequeue response ring (DQRR) entry, or cacheable portal area, stashing enable. */	/* DE */
		    0  /* EQCR_CI stashing priority */								/* EP */
		);
		/* TODO: Switch to memory-backed mode. */
		reg &= ~(1 << DPAA2_SWP_CFG_CPBS_SHIFT); /* QMan-backed mode */
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

	/* Initialize the portal with an IRQ threshold and timeout of 0us. */
	dpaa2_swp_set_irq_coalescing(p, p->dqrr.ring_size - 1, 0);

	*swp = p;

	return (0);
}

void
dpaa2_swp_free_portal(struct dpaa2_swp *swp)
{
	uint16_t flags;

	if (swp != NULL) {
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
dpaa2_swp_lock(struct dpaa2_swp *swp, uint16_t *flags)
{
	if (swp != NULL && flags != NULL) {
		mtx_assert(&swp->lock, MA_NOTOWNED);

		if (swp->cfg.atomic) {
			mtx_lock_spin(&swp->lock);
			*flags = swp->flags;
			swp->flags |= DPAA2_SWP_LOCKED;
		} else {
			mtx_lock(&swp->lock);
			while (swp->flags & DPAA2_SWP_LOCKED)
				cv_wait(&swp->cv, &swp->lock);
			*flags = swp->flags;
			swp->flags |= DPAA2_SWP_LOCKED;
			mtx_unlock(&swp->lock);
		}
	}
}

void
dpaa2_swp_unlock(struct dpaa2_swp *swp)
{
	if (swp != NULL) {
		if (swp->cfg.atomic) {
			swp->flags &= ~DPAA2_SWP_LOCKED;
			mtx_unlock_spin(&swp->lock);
		} else {
			mtx_lock(&swp->lock);
			swp->flags &= ~DPAA2_SWP_LOCKED;
			cv_signal(&swp->cv);
			mtx_unlock(&swp->lock);
		}
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
dpaa2_swp_write_reg(struct dpaa2_swp *swp, uint32_t o, uint32_t v)
{
	bus_write_4(swp->cinh_map, o, v);
}

uint32_t
dpaa2_swp_read_reg(struct dpaa2_swp *swp, uint32_t o)
{
	return (bus_read_4(swp->cinh_map, o));
}

/* Helper routines. */

/**
 * @brief Set enqueue descriptor without Order Point Record ID.
 *
 * ed:		Enqueue descriptor.
 * resp_always:	Enqueue with response always (1); FD from a rejected enqueue
 *		will be returned on a FQ (0).
 */
void
dpaa2_swp_set_ed_norp(struct dpaa2_eq_desc *ed, bool resp_always)
{
	ed->verb &= ~(1 << ENQ_CMD_ORP_ENABLE_SHIFT);
	if (resp_always)
		ed->verb |= ENQ_CMD_RESPONSE_ALWAYS;
	else
		ed->verb |= ENQ_CMD_REJECTS_TO_FQ;
}

/**
 * @brief Set FQ of the enqueue descriptor.
 */
void
dpaa2_swp_set_ed_fq(struct dpaa2_eq_desc *ed, uint32_t fqid)
{
	ed->verb &= ~(1 << ENQ_CMD_TARGET_TYPE_SHIFT);
	ed->tgtid = fqid;
}

/**
 * @brief Enable interrupts for a software portal.
 */
void
dpaa2_swp_set_intr_trigger(struct dpaa2_swp *swp, uint32_t mask)
{
	if (swp != NULL)
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_IER, mask);
}

/**
 * @brief Return the value in the SWP_IER register.
 */
uint32_t
dpaa2_swp_get_intr_trigger(struct dpaa2_swp *swp)
{
	if (swp != NULL)
		return dpaa2_swp_read_reg(swp, DPAA2_SWP_CINH_IER);
	return (0);
}

/**
 * @brief Return the value in the SWP_ISR register.
 */
uint32_t
dpaa2_swp_read_intr_status(struct dpaa2_swp *swp)
{
	if (swp != NULL)
		return dpaa2_swp_read_reg(swp, DPAA2_SWP_CINH_ISR);
	return (0);
}

/**
 * @brief Clear SWP_ISR register according to the given mask.
 */
void
dpaa2_swp_clear_intr_status(struct dpaa2_swp *swp, uint32_t mask)
{
	if (swp != NULL)
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_ISR, mask);
}

/**
 * @brief Enable or disable push dequeue.
 *
 * swp:		the software portal object
 * chan_idx:	the channel index (0 to 15)
 * en:		enable or disable push dequeue
 */
void
dpaa2_swp_set_push_dequeue(struct dpaa2_swp *swp, uint8_t chan_idx, bool en)
{
	uint16_t dqsrc;

	if (swp != NULL) {
		if (chan_idx > 15u) {
			device_printf(swp->desc->dpio_dev, "channel index "
			    "should be <= 15: chan_idx=%d\n", chan_idx);
			return;
		}

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
	}
}

/**
 * @brief Set new IRQ coalescing values.
 *
 * swp:		The software portal object.
 * threshold:	Threshold for DQRR interrupt generation. The DQRR interrupt
 *		asserts when the ring contains greater than "threshold" entries.
 * holdoff:	DQRR interrupt holdoff (timeout) period in us.
 */
int dpaa2_swp_set_irq_coalescing(struct dpaa2_swp *swp, uint32_t threshold,
    uint32_t holdoff)
{
	uint32_t itp; /* Interrupt Timeout Period */

	if (swp == NULL)
		return (EINVAL);
	
	/*
	 * Convert "holdoff" value from us to 256 QBMAN clock cycles
	 * increments. This depends on the QBMAN internal frequency.
	 */
	itp = (holdoff * 1000u) / swp->desc->swp_cycles_ratio;
	if (itp > DQRR_MAX_ITP) {
		itp = DQRR_MAX_ITP;
		device_printf(swp->desc->dpio_dev, "DQRR IRQ holdoff must be <= "
		    "%u\n", (swp->desc->swp_cycles_ratio * DQRR_MAX_ITP) / 1000u);
	}

	if (threshold >= swp->dqrr.ring_size) {
		threshold = swp->dqrr.ring_size - 1;
		device_printf(swp->desc->dpio_dev, "DQRR IRQ threshold must be "
		    "<= %u\n", swp->dqrr.ring_size - 1);
	}

	swp->dqrr.irq_threshold = threshold;
	swp->dqrr.irq_itp = itp;

	dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_DQRR_ITR, threshold);
	dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_ITPR, itp);

	return 0;
}

/*
 * Software portal commands.
 */

/**
 * @brief Configure the channel data availability notification (CDAN)
 * in a particular WQ channel.
 */
int
dpaa2_swp_conf_wq_channel(struct dpaa2_swp *swp, uint16_t chan_id,
    uint8_t we_mask, bool cdan_en, uint64_t ctx)
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
	} cmd = {0};
	struct __packed {
		uint8_t		verb;
		uint8_t		result;
		uint16_t	chan_id;
		uint8_t		_reserved[60];
	} rsp;
	int error;

	if (swp == NULL)
		return (EINVAL);

	cmd.chan_id = chan_id;
	cmd.we = we_mask;
	cmd.ctrl = cdan_en ? 1u : 0u;
	cmd.ctx = ctx;	

	error = exec_mgmt_command(swp, (struct dpaa2_swp_cmd *) &cmd,
	    (struct dpaa2_swp_rsp *) &rsp, CMDID_SWP_WQCHAN_CONFIGURE);
	if (error) {
		device_printf(swp->desc->dpio_dev, "WQ channel configuration "
		    "failed: error=%d\n", error);
		return (EIO);
	}

	if (rsp.result != QBMAN_CMD_RC_OK) {
		device_printf(swp->desc->dpio_dev, "WQ channel configuration "
		    "error: channel_id=%d, result=0x%02x\n", chan_id,
		    rsp.result);
		return (EIO);
	}

	return (0);
}

int
dpaa2_swp_release_bufs(struct dpaa2_swp *swp, uint16_t bpid, bus_addr_t *buf,
    uint32_t buf_num)
{
	/* NOTE: 64 bytes command. */
	struct __packed {
		uint8_t		verb;
		uint8_t		_reserved1;
		uint16_t	bpid;
		uint32_t	_reserved2;
		uint64_t	buf[DPAA2_SWP_BUFS_PER_CMD];
	} cmd = {0};
	int error;

	if (swp == NULL || buf == NULL || buf_num == 0u ||
	    buf_num > DPAA2_SWP_BUFS_PER_CMD)
		return (EINVAL);

	for (uint32_t i = 0; i < buf_num; i++)
		cmd.buf[i] = buf[i];
	cmd.bpid = bpid;
	cmd.verb |= 1 << 5; /* Switch release buffer command to valid. */

	error = exec_br_command(swp, (struct dpaa2_swp_cmd *) &cmd, buf_num);
	if (error) {
		device_printf(swp->desc->dpio_dev, "buffers release command "
		    "failed\n");
		return (error);
	}

	return (0);
}

int
dpaa2_swp_dqrr_next(struct dpaa2_swp *swp, struct dpaa2_dq *dq, uint32_t *idx)
{
	struct resource_map *map;
	struct dpaa2_swp_rsp *rsp = (struct dpaa2_swp_rsp *) dq;
	uint32_t verb, offset, pi; /* producer index */
	uint16_t flags;

	if (swp == NULL || dq == NULL)
		return (EINVAL);

	dpaa2_swp_lock(swp, &flags);
	if (flags & DPAA2_SWP_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_swp_unlock(swp);
		return (ENOENT);
	}

	map = swp->cena_map;
	offset = swp->cfg.mem_backed
	    ? DPAA2_SWP_CENA_DQRR_MEM(swp->dqrr.next_idx)
	    : DPAA2_SWP_CENA_DQRR(swp->dqrr.next_idx);

	/*
	 * Before using valid-bit to detect if something is there, we have to
	 * handle the case of the DQRR reset bug...
	 */
	if (swp->dqrr.reset_bug) {
		/*
		 * We pick up new entries by cache-inhibited producer index,
		 * which means that a non-coherent mapping would require us to
		 * invalidate and read *only* once that PI has indicated that
		 * there's an entry here. The first trip around the DQRR ring
		 * will be much less efficient than all subsequent trips around
		 * it...
		 */
		pi = dpaa2_swp_read_reg(swp, DPAA2_SWP_CINH_DQPI) & DQRR_PI_MASK;

		/* There are new entries if pi != next_idx */
		if (pi == swp->dqrr.next_idx) {
			dpaa2_swp_unlock(swp);
			return (ENOENT);
		}

		/*
		 * If next_idx is/was the last ring index, and 'pi' is
		 * different, we can disable the workaround as all the ring
		 * entries have now been DMA'd to so valid-bit checking is
		 * repaired.
		 *
		 * NOTE: This logic needs to be based on next_idx (which
		 *	 increments one at a time), rather than on pi (which
		 *	 can burst and wrap-around between our snapshots of it).
		 */
		if (swp->dqrr.next_idx == (swp->dqrr.ring_size - 1))
			swp->dqrr.reset_bug = 0;
	}

	/* Read dequeue response message. */
	for (int i = 0; i < DPAA2_SWP_RSP_PARAMS_N; i++)
		rsp->params[i] = bus_read_8(map, offset + i * sizeof(uint64_t));
	verb = dq->common.verb;

	/*
	 * If the valid-bit isn't of the expected polarity, nothing there. Note,
	 * in the DQRR reset bug workaround, we shouldn't need to skip these
	 * check, because we've already determined that a new entry is available
	 * and we've invalidated the cacheline before reading it, so the
	 * valid-bit behaviour is repaired and should tell us what we already
	 * knew from reading PI.
	 */
	if ((verb & DPAA2_SWP_VALID_BIT) != swp->dqrr.valid_bit) {
		dpaa2_swp_unlock(swp);
		return (ENOENT);
	}

	/* Return index of the current entry (if requested). */
	if (idx != NULL)
		*idx = swp->dqrr.next_idx;

	/*
	 * There's something there. Move "next_idx" attention to the next ring
	 * entry before returning what we found.
	 */
	swp->dqrr.next_idx++;
	swp->dqrr.next_idx &= swp->dqrr.ring_size - 1; /* wrap around */
	if (!swp->dqrr.next_idx)
		swp->dqrr.valid_bit ^= DPAA2_SWP_VALID_BIT;

	/*
	 * If this is the final response to a volatile dequeue command
	 * indicate that the VDQ is available.
	 */
	/* flags = dq->fdr.stat; */
	/* resp_verb = verb & QBMAN_RESULT_MASK; */
	/* if ((response_verb == QBMAN_RESULT_DQ) && */
	/*     (flags & DPAA2_DQ_STAT_VOLATILE) && */
	/*     (flags & DPAA2_DQ_STAT_EXPIRED)) */
	/* 	atomic_inc(&s->vdq.available); */

	dpaa2_swp_unlock(swp);
	return (0);
}

int
dpaa2_swp_pull(struct dpaa2_swp *swp, uint16_t chan_id, bus_addr_t buf,
    uint32_t frames_n)
{
	/* NOTE: 64 bytes command. */
	struct __packed {
		uint8_t		verb;
		uint8_t		numf;
		uint8_t		tok;
		uint8_t		_reserved;
		uint32_t	dq_src;
		uint64_t	rsp_addr;
		uint64_t	_reserved1[6];
	} cmd = {0};
	int error, dequeues = -1;

	if (swp == NULL || frames_n == 0u || frames_n > 16u)
		return (EINVAL);

	cmd.numf = frames_n - 1;
	cmd.tok = QMAN_VDQCR_TOKEN;
	cmd.dq_src = chan_id;
	cmd.rsp_addr = (uint64_t) buf;

	cmd.verb |= 1 << QB_VDQCR_VERB_RLS_SHIFT;
	cmd.verb |= 1 << QB_VDQCR_VERB_WAE_SHIFT;
	cmd.verb |= 1 << QB_VDQCR_VERB_DCT_SHIFT;
	cmd.verb |= 0 << QB_VDQCR_VERB_DT_SHIFT; /* pull from channel */

	/* Retry while portal is busy */
	do {
		error = exec_vdc_command(swp, (struct dpaa2_swp_cmd *) &cmd);
		dequeues++;
		cpu_spinwait();
	} while (error == EBUSY && dequeues < DPAA2_SWP_BUSY_RETRIES);

	if (error) {
		device_printf(swp->desc->dpio_dev, "volatile dequeue command "
		    "failed\n");
		return (error);
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
swp_enq_direct(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd)
{
	/* TBD */
	return (ENODEV);
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
swp_enq_memback(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd)
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
swp_enq_mult_direct(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd, uint32_t *flags, int frames_n)
{
	/* TBD */
	return (ENODEV);
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
 * NOTE: Enqueue command (64 bytes): 32 (descriptor) + 32 (frame descriptor).
 */
static int
swp_enq_mult_memback(struct dpaa2_swp *swp, struct dpaa2_eq_desc *ed,
    struct dpaa2_fd *fd, uint32_t *flags, int frames_n)
{
	const uint8_t  *ed_pdat8 =  (const uint8_t *) ed;
	const uint32_t *ed_pdat32 = (const uint32_t *) ed;
	const uint64_t *ed_pdat64 = (const uint64_t *) ed;
	const uint64_t *fd_pdat64 = (const uint64_t *) fd;
	uint32_t eqcr_ci, eqcr_pi; /* consumer/producer index */
	uint32_t half_mask, full_mask, val;
	uint16_t swp_flags;
	int num_enq = 0;

	if (swp == NULL || ed == NULL || fd == NULL || flags == NULL ||
	    frames_n == 0)
		return (EINVAL);

	dpaa2_swp_lock(swp, &swp_flags);
	if (swp_flags & DPAA2_SWP_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_swp_unlock(swp);
		return (ENOENT);
	}

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
static int
cyc_diff(uint8_t ringsize, uint8_t first, uint8_t last)
{
	/* 'first' is included, 'last' is excluded */
	return ((first <= last)
	    ? (last - first) : ((2 * ringsize) - (first - last)));
}

/**
 * @internal
 * @brief Execute Buffer Release Command (BRC).
 */
static int
exec_br_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd,
    uint32_t buf_num)
{
	struct __packed with_verb {
		uint8_t	verb;
		uint8_t	_reserved[63];
	} *c;
	const uint8_t *cmd_pdat8 = (const uint8_t *) cmd->params;
	const uint32_t *cmd_pdat32 = (const uint32_t *) cmd->params;
	struct resource_map *map;
	uint32_t offset, rar; /* Release Array Allocation register */
	uint16_t flags;

	if (!swp || !cmd)
		return (EINVAL);

	dpaa2_swp_lock(swp, &flags);
	if (flags & DPAA2_SWP_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_swp_unlock(swp);
		return (ENOENT);
	}

	rar = dpaa2_swp_read_reg(swp, DPAA2_SWP_CINH_RAR);
	if (!RAR_SUCCESS(rar)) {
		dpaa2_swp_unlock(swp);
		return (EBUSY);
	}

	map = swp->cfg.writes_cinh ? swp->cinh_map : swp->cena_map;
	offset = swp->cfg.mem_backed
	    ? DPAA2_SWP_CENA_RCR_MEM(RAR_IDX(rar))
	    : DPAA2_SWP_CENA_RCR(RAR_IDX(rar));
	c = (struct with_verb *) cmd;

	/* Write command bytes (without VERB byte). */
	for (uint32_t i = 1; i < DPAA2_SWP_CMD_PARAMS_N; i++)
		bus_write_8(map, offset + sizeof(uint64_t) * i, cmd->params[i]);
	bus_write_4(map, offset + 4, cmd_pdat32[1]);
	for (uint32_t i = 1; i <= 3; i++)
		bus_write_1(map, offset + i, cmd_pdat8[i]);

	/* Write VERB byte and trigger command execution. */
	if (swp->cfg.mem_backed) {
		bus_write_1(map, offset, c->verb | RAR_VB(rar) | buf_num);
		wmb();
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_RCR_AM_RT +
		    RAR_IDX(rar) * 4, DPAA2_SWP_RT_MODE);
	} else {
		wmb();
		bus_write_1(map, offset, c->verb | RAR_VB(rar) | buf_num);
	}

	dpaa2_swp_unlock(swp);

	return (0);
}

/**
 * @internal
 * @brief Execute Volatile Dequeue Command (VDC).
 *
 * This command will be executed by QBMan only once in order to deliver requested
 * number of frames (1-16 or 1-32 depending on QBMan version) to the driver via
 * DQRR or arbitrary DMA-mapped memory.
 *
 * NOTE: There is a counterpart to the volatile dequeue command called static
 *	 dequeue command (SDQC) which is executed periodically all the time the
 *	 command is present in the SDQCR register.
 */
static int
exec_vdc_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd)
{
	struct __packed with_verb {
		uint8_t	verb;
		uint8_t	_reserved[63];
	} *c;
	const uint8_t *cmd_pdat8 = (const uint8_t *) cmd->params;
	const uint32_t *cmd_pdat32 = (const uint32_t *) cmd->params;
	struct resource_map *map;
	uint32_t offset;
	uint16_t flags;

	if (!swp || !cmd)
		return (EINVAL);

	if (!atomic_dec_and_test(&swp->vdq.avail)) {
		atomic_inc(&swp->vdq.avail);
		return (EBUSY);
	}

	dpaa2_swp_lock(swp, &flags);
	if (flags & DPAA2_SWP_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_swp_unlock(swp);
		return (ENOENT);
	}

	map = swp->cfg.writes_cinh ? swp->cinh_map : swp->cena_map;
	offset = swp->cfg.mem_backed
	    ? DPAA2_SWP_CENA_VDQCR_MEM
	    : DPAA2_SWP_CENA_VDQCR;
	c = (struct with_verb *) cmd;

	/* Write command bytes (without VERB byte). */
	for (uint32_t i = 1; i < DPAA2_SWP_CMD_PARAMS_N; i++)
		bus_write_8(map, offset + sizeof(uint64_t) * i, cmd->params[i]);
	bus_write_4(map, offset + 4, cmd_pdat32[1]);
	for (uint32_t i = 1; i <= 3; i++)
		bus_write_1(map, offset + i, cmd_pdat8[i]);

	/* Write VERB byte and trigger command execution. */
	if (swp->cfg.mem_backed) {
		bus_write_1(map, offset, c->verb | swp->vdq.valid_bit);
		swp->vdq.valid_bit ^= DPAA2_SWP_VALID_BIT;
		wmb();
		dpaa2_swp_write_reg(swp, DPAA2_SWP_CINH_VDQCR_RT,
		    DPAA2_SWP_RT_MODE);
	} else {
		wmb();
		bus_write_1(map, offset, c->verb | swp->vdq.valid_bit);
		swp->vdq.valid_bit ^= DPAA2_SWP_VALID_BIT;
	}

	dpaa2_swp_unlock(swp);

	return (0);
}

/**
 * @internal
 * @brief Execute a QBMan management command.
 */
static int
exec_mgmt_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd,
    struct dpaa2_swp_rsp *rsp, uint8_t cmdid)
{
	struct __packed with_verb {
		uint8_t	verb;
		uint8_t	_reserved[63];
	} *r;
	uint16_t flags;
	int error;

	if (swp == NULL || cmd == NULL || rsp == NULL)
		return (EINVAL);

	dpaa2_swp_lock(swp, &flags);
	if (flags & DPAA2_SWP_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_swp_unlock(swp);
		return (ENOENT);
	}

	/*
	 * Send a command to QBMan using Management Command register and wait
	 * for response from the Management Response registers.
	 */
	send_mgmt_command(swp, cmd, cmdid);
	error = wait_for_mgmt_response(swp, rsp);
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
static int
send_mgmt_command(struct dpaa2_swp *swp, struct dpaa2_swp_cmd *cmd,
    uint8_t cmdid)
{
	const uint8_t *cmd_pdat8 = (const uint8_t *) cmd->params;
	const uint32_t *cmd_pdat32 = (const uint32_t *) cmd->params;
	struct resource_map *map;
	uint32_t offset;

	map = swp->cfg.writes_cinh ? swp->cinh_map : swp->cena_map;
	offset = swp->cfg.mem_backed ? DPAA2_SWP_CENA_CR_MEM : DPAA2_SWP_CENA_CR;

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

	return (0);
}

/**
 * @internal
 */
static int
wait_for_mgmt_response(struct dpaa2_swp *swp, struct dpaa2_swp_rsp *rsp)
{
	const uint32_t attempts = swp->cfg.atomic ? CMD_SPIN_ATTEMPTS
	    : CMD_SLEEP_ATTEMPTS;
	struct resource_map *map = swp->cena_map;
	/* Management command response to be read from the only RR or RR0/RR1. */
	const uint32_t offset = swp->cfg.mem_backed ? DPAA2_SWP_CENA_RR_MEM
	    : DPAA2_SWP_CENA_RR(swp->mc.valid_bit);
	uint32_t i;
	uint8_t verb;
	int rc;

	/*
	 * Let's have a data synchronization barrier for the whole system to be
	 * sure that QBMan's command execution result is transferred to memory.
	 */
	dsb(sy);

	/* Wait for a command response from QBMan. */
	for (i = 1; i <= attempts; i++) {
		if (swp->cfg.mem_backed) {
			verb = bus_read_1(map, offset);
			if (swp->mr.valid_bit != (verb & DPAA2_SWP_VALID_BIT))
				goto wait;
			if (!(verb & ~DPAA2_SWP_VALID_BIT))
				goto wait;
			swp->mr.valid_bit ^= DPAA2_SWP_VALID_BIT;
		} else {
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

		/*
		 * Let's have a data synchronization barrier for the whole
		 * system to be sure that QBMan's command execution result is
		 * transferred to memory.
		 */
		dsb(sy);
	}
	/* Return an error on expired timeout. */
	rc = i > attempts ? ETIMEDOUT : 0;

	/* Read command response. */
	for (i = 0; i < DPAA2_SWP_RSP_PARAMS_N; i++)
		rsp->params[i] = bus_read_8(map, offset + i * sizeof(uint64_t));

	return (rc);
}

/**
 * @internal
 */
static inline int
atomic_add_return(int i, atomic_t *v)
{
	return i + atomic_fetchadd_int(&v->counter, i);
}

/**
 * @internal
 */
static inline int
atomic_sub_return(int i, atomic_t *v)
{
	return atomic_fetchadd_int(&v->counter, -i) - i;
}

/**
 * @internal
 */
static inline int
atomic_xchg(atomic_t *v, int i)
{
	return (atomic_swap_int(&v->counter, i));
}

/**
 * @internal
 */
static inline int
atomic_inc(atomic_t *v)
{
	return atomic_fetchadd_int(&v->counter, 1) + 1;
}

/**
 * @internal
 */
static inline int
atomic_dec(atomic_t *v)
{
	return atomic_fetchadd_int(&v->counter, -1) - 1;
}
