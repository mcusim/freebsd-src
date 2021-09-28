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

MALLOC_DEFINE(M_DPAA2_SWP, "dpaa2_swp_memory", "DPAA2 QBMan Software Portal "
    "memory");

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
		reg = swp_set_cfg(p->dqrr.ring_size,
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
		reg = swp_set_cfg(p->dqrr.ring_size,
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
	swp_write_reg(p, DPAA2_SWP_CINH_CFG, reg);
	reg = swp_read_reg(p, DPAA2_SWP_CINH_CFG);
	if (!reg) {
		free(p, M_DPAA2_SWP);
		return (DPAA2_SWP_STAT_PORTAL_DISABLED);
	}

	/* Enable read trigger mode. */
	if ((desc->swp_version & DPAA2_SWP_REV_MASK) >= DPAA2_SWP_REV_5000) {
		swp_write_reg(p, DPAA2_SWP_CINH_EQCR_PI, DPAA2_SWP_RT_MODE);
		swp_write_reg(p, DPAA2_SWP_CINH_RCR_PI, DPAA2_SWP_RT_MODE);
	}

	/*
	 * Static Dequeue Command Register needs to be initialized to 0 when no
	 * channels are being dequeued from or else the QMan HW will indicate an
	 * error. The values that were calculated above will be applied when
	 * dequeues from a specific channel are enabled.
	 */
	swp_write_reg(p, DPAA2_SWP_CINH_SDQCR, 0);

	p->eqcr.pi_ring_size = 8;
	if ((desc->swp_version & DPAA2_SWP_REV_MASK) >= DPAA2_SWP_REV_5000) {
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

	eqcr_pi = swp_read_reg(p, DPAA2_SWP_CINH_EQCR_PI);
	p->eqcr.pi = eqcr_pi & p->eqcr.pi_ci_mask;
	p->eqcr.pi_vb = eqcr_pi & DPAA2_SWP_VALID_BIT;
	p->eqcr.ci = swp_read_reg(p, DPAA2_SWP_CINH_EQCR_CI)
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
