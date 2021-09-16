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
 * QBMan software portal interface.
 *
 * Software portals are used by data path software executing on a processor core
 * to communicate with the Queue Manager (QMan) which acts as a central resource
 * in DPAA2, managing the queueing of data between multiple processor cores,
 * network interfaces, and hardware accelerators in a multicore SoC.
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

/* All QBMan command and result structures use this "valid bit" encoding */
#define QBMAN_VALID_BIT		((uint32_t)0x80)

/* Versions of the QBMan software portals. */
#define QBMAN_REV_4000		0x04000000
#define QBMAN_REV_4100		0x04010000
#define QBMAN_REV_4101		0x04010001
#define QBMAN_REV_5000		0x05000000

#define QBMAN_REV_MASK		0xFFFF0000

/* Register offsets in the cache-inhibited area */
#define QBMAN_CINH_SWP_CR	0x600 /* Management Command Register */
#define QBMAN_CINH_SWP_EQCR_PI	0x800 /* Enqueue Ring, Producer Index Register */
#define QBMAN_CINH_SWP_EQCR_CI	0x840 /* Enqueue Ring, Consumer Index Register */
#define QBMAN_CINH_SWP_CR_RT	0x900 /* CR Read Trigger register */
#define QBMAN_CINH_SWP_VDQCR_RT	0x940 /* VDQCR Read Trigger register */
#define QBMAN_CINH_SWP_EQCR_AM_RT 0x980
#define QBMAN_CINH_SWP_RCR_AM_RT  0x9C0
#define QBMAN_CINH_SWP_DQPI	0xA00
#define QBMAN_CINH_SWP_DCAP	0xAC0
#define QBMAN_CINH_SWP_SDQCR	0xB00 /* Static Dequeue Command Register */
#define QBMAN_CINH_SWP_EQCR_AM_RT2 0xB40
#define QBMAN_CINH_SWP_RCR_PI	0xC00 /* Release Ring, Producer Index */
#define QBMAN_CINH_SWP_RAR	0xCC0
#define QBMAN_CINH_SWP_ISR	0xE00
#define QBMAN_CINH_SWP_IER	0xE40
#define QBMAN_CINH_SWP_ISDR	0xE80
#define QBMAN_CINH_SWP_IIR	0xEC0
#define QBMAN_CINH_SWP_CFG	0xD00

/* CENA register offsets */
#define QBMAN_CENA_SWP_EQCR(n)	(0x000 + ((uint32_t)(n) << 6))
#define QBMAN_CENA_SWP_DQRR(n)	(0x200 + ((uint32_t)(n) << 6))
#define QBMAN_CENA_SWP_RCR(n)	(0x400 + ((uint32_t)(n) << 6))
#define QBMAN_CENA_SWP_CR	(0x600) /* Management Command Register */
#define QBMAN_CENA_SWP_RR(vb)	(0x700 + ((uint32_t)(vb) >> 1))
#define QBMAN_CENA_SWP_VDQCR	(0x780)
#define QBMAN_CENA_SWP_EQCR_CI	(0x840)

/* CENA register offsets in memory-backed mode */
#define QBMAN_CENA_SWP_DQRR_MEM(n)	(0x0800 + ((uint32_t)(n) << 6))
#define QBMAN_CENA_SWP_RCR_MEM(n)	(0x1400 + ((uint32_t)(n) << 6))
#define QBMAN_CENA_SWP_CR_MEM		(0x1600)
#define QBMAN_CENA_SWP_RR_MEM		(0x1680)
#define QBMAN_CENA_SWP_VDQCR_MEM	(0x1780)
#define QBMAN_CENA_SWP_EQCR_CI_MEMBACK	(0x1840)

/* Shifts in the portal's configuration register. */
#define SWP_CFG_DQRR_MF_SHIFT	20
#define SWP_CFG_EST_SHIFT	16
#define SWP_CFG_CPBS_SHIFT	15
#define SWP_CFG_WN_SHIFT	14
#define SWP_CFG_RPM_SHIFT	12
#define SWP_CFG_DCM_SHIFT	10
#define SWP_CFG_EPM_SHIFT	8
#define SWP_CFG_VPM_SHIFT	7
#define SWP_CFG_CPM_SHIFT	6
#define SWP_CFG_SD_SHIFT	5
#define SWP_CFG_SP_SHIFT	4
#define SWP_CFG_SE_SHIFT	3
#define SWP_CFG_DP_SHIFT	2
#define SWP_CFG_DE_SHIFT	1
#define SWP_CFG_EP_SHIFT	0

/*
 * Read trigger bit is used to trigger QMan to read an enqueue command from
 * memory, without having software perform a cache flush to force a write of the
 * command to QMan.
 *
 * NOTE: Implemented in QBMan 5.0 or above.
 */
#define QBMAN_RT_MODE		((uint32_t)0x100)

MALLOC_DEFINE(M_DPAA2_SWP, "dpaa2_swp_memory", "DPAA2 QBMan Software Portal "
    "memory");

/**
 * @brief Helper object to interact with the QBMan software portal.
 *
 * res:		Unmapped cache-enabled and cache-inhibited parts of the portal.
 * map:		Mapped cache-enabled and cache-inhibited parts of the portal.
 * desc:	Descriptor of the QBMan software portal.
 * lock:	Spinlock to guard an access to the portal.
 * flags:	Current state of the object.
 * dqrr:	Dequeue Response Ring is used to issue frame dequeue responses
 * 		from the QBMan to the driver.
 * eqcr:	Enqueue Command Ring is used to issue frame enqueue commands
 *		from the driver to the QBMan.
 */
struct dpaa2_swp {
	struct resource 	*cena_res;
	struct resource_map	*cena_map;
	struct resource		*cinh_res;
	struct resource_map	*cinh_map;

	const dpaa2_swp_desc_t	*desc;
	struct mtx		 lock;
	uint16_t		 flags;

	struct {
		uint32_t	 next_idx;
		uint32_t	 valid_bit;
		uint8_t		 ring_size;
		bool		 reset_bug; /* dqrr reset workaround */
	} dqrr;

	struct {
		uint32_t	 pi;
		uint32_t	 pi_vb;
		uint32_t	 pi_ring_size;
		uint32_t	 pi_ci_mask;
		uint32_t	 ci;
		int		 available;
		uint32_t	 pend;
		uint32_t	 no_pfdr;
	} eqcr;
};

static inline void swp_write_reg(dpaa2_swp_t swp, uint32_t offset, uint32_t val);
static inline uint32_t swp_read_reg(dpaa2_swp_t swp, uint32_t offset);
static inline uint32_t swp_set_cfg(uint8_t max_fill, uint8_t wn, uint8_t est,
    uint8_t rpm, uint8_t dcm, uint8_t epm, int sd, int sp, int se, int dp,
    int de, int ep);

/*
 * Management routines.
 */

int
dpaa2_swp_init_portal(dpaa2_swp_t *portal, dpaa2_swp_desc_t *desc,
    const uint16_t flags)
{
	const int mflags = flags & DPAA2_SWP_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO) : (M_WAITOK | M_ZERO);
	dpaa2_swp_t p;
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

	p->cena_res = desc->cena_res;
	p->cena_map = desc->cena_map;
	p->cinh_res = desc->cinh_res;
	p->cinh_map = desc->cinh_map;

	/* Dequeue Response Ring configuration */
	p->dqrr.next_idx = 0;
	p->dqrr.valid_bit = QBMAN_VALID_BIT;
	if ((desc->swp_version & QBMAN_REV_MASK) < QBMAN_REV_4100) {
		p->dqrr.dqrr_size = 4;
		p->dqrr.reset_bug = 1;
	} else {
		p->dqrr.dqrr_size = 8;
		p->dqrr.reset_bug = 0;
	}

	if ((desc->swp_version & QBMAN_REV_MASK) < QBMAN_REV_5000) {
		reg = swp_set_cfg(p->dqrr.dqrr_size,
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
		bus_set_region_1(p->cena_map, 0, 0, 64 * 1024);
		reg = swp_set_cfg(p->dqrr.dqrr_size,
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
		reg |= 1 << SWP_CFG_CPBS_SHIFT | /* memory-backed mode */
		    1 << SWP_CFG_VPM_SHIFT |  /* VDQCR read triggered mode */
		    1 << SWP_CFG_CPM_SHIFT;   /* CR read triggered mode */
	}
	swp_write_reg(p, QBMAN_CINH_SWP_CFG, reg);
	reg = swp_read_reg(p, QBMAN_CINH_SWP_CFG);
	if (!reg) {
		free(p, M_DPAA2_SWP);
		return (DPAA2_SWP_STAT_PORTAL_DISABLED);
	}

	/* Enable read trigger mode. */
	if ((desc->swp_version & QBMAN_REV_MASK) >= QBMAN_REV_5000) {
		swp_write_reg(p, QBMAN_CINH_SWP_EQCR_PI, QBMAN_RT_MODE);
		swp_write_reg(p, QBMAN_CINH_SWP_RCR_PI, QBMAN_RT_MODE);
	}

	/*
	 * Static Dequeue Command Register needs to be initialized to 0 when no
	 * channels are being dequeued from or else the QMan HW will indicate an
	 * error. The values that were calculated above will be applied when
	 * dequeues from a specific channel are enabled.
	 */
	qbman_write_register(p, QBMAN_CINH_SWP_SDQCR, 0);

	p->eqcr.pi_ring_size = 8;
	if ((p->desc->qman_version & QMAN_REV_MASK) >= QMAN_REV_5000) {
		p->eqcr.pi_ring_size = 32;
		/* qbman_swp_enqueue_ptr = */
		/*     qbman_swp_enqueue_mem_back; */
		/* qbman_swp_enqueue_multiple_ptr = */
		/*     qbman_swp_enqueue_multiple_mem_back; */
		/* qbman_swp_enqueue_multiple_desc_ptr = */
		/*     qbman_swp_enqueue_multiple_desc_mem_back; */
		/* qbman_swp_pull_ptr = qbman_swp_pull_mem_back; */
		/* qbman_swp_dqrr_next_ptr = qbman_swp_dqrr_next_mem_back; */
		/* qbman_swp_release_ptr = qbman_swp_release_mem_back; */
	}

	for (mask_size = p->eqcr.pi_ring_size; mask_size > 0; mask_size >>= 1)
		p->eqcr.pi_ci_mask = (p->eqcr.pi_ci_mask << 1) + 1;

	eqcr_pi = qbman_read_register(p, QBMAN_CINH_SWP_EQCR_PI);
	p->eqcr.pi = eqcr_pi & p->eqcr.pi_ci_mask;
	p->eqcr.pi_vb = eqcr_pi & QBMAN_VALID_BIT;
	p->eqcr.ci = qbman_read_register(p, QBMAN_CINH_SWP_EQCR_CI)
	    & p->eqcr.pi_ci_mask;
	p->eqcr.available = p->eqcr.pi_ring_size;

	*portal = p;

	return (0);
}

void
dpaa2_swp_free_portal(dpaa2_swp_t portal)
{
	return;
}

static inline void
swp_write_reg(dpaa2_swp_t swp, uint32_t offset, uint32_t val)
{
	bus_write_4(swp->cinh_map, offset, val);
}

static inline uint32_t
swp_read_reg(dpaa2_swp_t swp, uint32_t offset)
{
	return (bus_read_4(swp->cinh_map, offset));
}

static inline uint32_t
swp_set_cfg(uint8_t max_fill, uint8_t wn, uint8_t est, uint8_t rpm, uint8_t dcm,
    uint8_t epm, int sd, int sp, int se, int dp, int de, int ep)
{
	return (
	    max_fill	<< SWP_CFG_DQRR_MF_SHIFT |
	    est		<< SWP_CFG_EST_SHIFT |
	    wn		<< SWP_CFG_WN_SHIFT |
	    rpm		<< SWP_CFG_RPM_SHIFT |
	    dcm		<< SWP_CFG_DCM_SHIFT |
	    epm		<< SWP_CFG_EPM_SHIFT |
	    sd		<< SWP_CFG_SD_SHIFT |
	    sp		<< SWP_CFG_SP_SHIFT |
	    se		<< SWP_CFG_SE_SHIFT |
	    dp		<< SWP_CFG_DP_SHIFT |
	    de		<< SWP_CFG_DE_SHIFT |
	    ep		<< SWP_CFG_EP_SHIFT
	);
}
