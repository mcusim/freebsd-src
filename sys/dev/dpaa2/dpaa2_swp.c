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

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_swp.h"
#include "dpaa2_mc.h"

/* Shifts in the VERB byte of the enqueue command descriptor. */
#define ENQ_CMD_ORP_ENABLE_SHIFT	2
#define ENQ_CMD_IRQ_ON_DISPATCH_SHIFT	3
#define ENQ_CMD_TARGET_TYPE_SHIFT	4
#define ENQ_CMD_DCA_EN_SHIFT		7
/* VERB byte options of the enqueue command descriptor. */
#define ENQ_CMD_EMPTY			(0u)
#define ENQ_CMD_RESPONSE_ALWAYS		(1u)
#define ENQ_CMD_REJECTS_TO_FQ		(2u)

MALLOC_DEFINE(M_DPAA2_SWP, "dpaa2_swp_memory", "DPAA2 QBMan Software Portal "
    "memory");

/* Forward declarations. */
static int	swp_enq_direct(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd);
static int	swp_enq_memback(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd);
static int	swp_enq_mult_direct(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n);
static int	swp_enq_mult_memback(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
		    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n);

int
dpaa2_swp_init_portal(dpaa2_swp_t *portal, dpaa2_swp_desc_t *desc,
    const uint16_t flags)
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

	mtx_init(&p->lock, "QBMan portal spin lock", NULL, MTX_SPIN);

	p->desc = desc;
	p->flags = flags;
	p->mc.valid_bit = DPAA2_SWP_VALID_BIT;
	if ((desc->swp_version & DPAA2_SWP_REV_MASK) >= DPAA2_SWP_REV_5000)
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
		reg = dpaa2_swp_set_cfg(p->dqrr.ring_size,
		    1, /* Writes Non-cacheable */
		    0, /* EQCR_CI stashing threshold */
		    3, /* RPM: RCR in array mode */
		    2, /* DCM: Discrete consumption ack */
		    2, /* EPM: EQCR in ring mode */
		    1, /* mem stashing drop enable enable */
		    1, /* mem stashing priority enable */
		    1, /* mem stashing enable */
		    1, /* dequeue stashing priority enable */
		    0, /* dequeue stashing enable enable */
		    0); /* EQCR_CI stashing priority enable */
	} else {
		bus_set_region_4(p->cena_map, 0, 0,
		    rman_get_size(p->cena_res) / 4);
		reg = dpaa2_swp_set_cfg(p->dqrr.ring_size,
		    1, /* Writes Non-cacheable */
		    1, /* EQCR_CI stashing threshold */
		    3, /* RPM: RCR in array mode */
		    2, /* DCM: Discrete consumption ack */
		    0, /* EPM: EQCR in ring mode */
		    1, /* mem stashing drop enable */
		    1, /* mem stashing priority enable */
		    1, /* mem stashing enable */
		    1, /* dequeue stashing priority enable */
		    0, /* dequeue stashing enable */
		    0); /* EQCR_CI stashing priority enable */
		reg |= 1 << DPAA2_SWP_CFG_CPBS_SHIFT | /* memory-backed mode */
		    1 << DPAA2_SWP_CFG_VPM_SHIFT |  /* VDQCR read triggered mode */
		    1 << DPAA2_SWP_CFG_CPM_SHIFT;   /* CR read triggered mode */
	}
	dpaa2_swp_write_reg(p, DPAA2_SWP_CINH_CFG, reg);
	reg = dpaa2_swp_read_reg(p, DPAA2_SWP_CINH_CFG);
	if (!reg) {
		free(p, M_DPAA2_SWP);
		return (DPAA2_SWP_STAT_PORTAL_DISABLED);
	}

	/* Enable read trigger mode. */
	if ((desc->swp_version & DPAA2_SWP_REV_MASK) >= DPAA2_SWP_REV_5000) {
		dpaa2_swp_write_reg(p, DPAA2_SWP_CINH_EQCR_PI,
		    DPAA2_SWP_RT_MODE);
		dpaa2_swp_write_reg(p, DPAA2_SWP_CINH_RCR_PI, DPAA2_SWP_RT_MODE);
	}

	/*
	 * Static Dequeue Command Register needs to be initialized to 0 when no
	 * channels are being dequeued from or else the QMan HW will indicate an
	 * error. The values that were calculated above will be applied when
	 * dequeues from a specific channel are enabled.
	 */
	dpaa2_swp_write_reg(p, DPAA2_SWP_CINH_SDQCR, 0);

	p->enqueue = swp_enq_direct;
	p->enqueue_mult = swp_enq_mult_direct;

	p->eqcr.pi_ring_size = 8;
	if ((desc->swp_version & DPAA2_SWP_REV_MASK) >= DPAA2_SWP_REV_5000) {
		p->eqcr.pi_ring_size = 32;

		p->enqueue = swp_enq_memback;
		p->enqueue_mult = swp_enq_mult_memback;
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
dpaa2_swp_free_portal(dpaa2_swp_t portal)
{
	if (portal)
		free(portal, M_DPAA2_SWP);
}

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
	if (respond_success)
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
 * @brief Issue a command to enqueue a frame using one enqueue descriptor.
 *
 * swp:		Software portal used to send this command to.
 * ed:		Enqueue command descriptor.
 * fd:		Frame descriptor to enqueue.
 */
int
dpaa2_swp_enq(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed, const dpaa2_fd_t *fd)
{
	return (swp->enqueue(swp, ed, fd));
}

/**
 * @brief Issue a command to enqueue frames using one enqueue descriptor.
 *
 * swp:		Software portal used to send this command to.
 * ed:		Enqueue command descriptor.
 * fd:		Frame descriptor to enqueue.
 * flags:	Table pointer of QBMAN_ENQUEUE_FLAG_DCA flags, not used if NULL.
 * frames_n:	Number of FDs to enqueue.
 */
int
dpaa2_swp_enq_mult(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n)
{
	return (swp->enqueue_mult(swp, ed, fd, flags, frames_n));
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
 */
static int
swp_enq_mult_memback(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n)
{
	/* TBD */
	return (0);
}
