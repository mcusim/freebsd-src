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

#ifndef	_DPAA2_SWP_H
#define	_DPAA2_SWP_H

/*
 * QBMan software portal interface.
 *
 * Software portals are used by data path software executing on a processor core
 * to communicate with the Queue Manager (QMan) which acts as a central resource
 * in DPAA2, managing the queueing of data between multiple processor cores,
 * network interfaces, and hardware accelerators in a multicore SoC. These
 * portals are memory mapped in the system.
 */

/* Portal flags. */
#define DPAA2_SWP_DEF		0x0u
#define DPAA2_SWP_NOWAIT_ALLOC	0x1u	/* Do not sleep during init */

/* Command return codes. */
#define DPAA2_SWP_STAT_OK		0x0
#define DPAA2_SWP_STAT_NO_MEMORY	0x9	/* No memory available */
#define DPAA2_SWP_STAT_PORTAL_DISABLED	0xFD	/* QBMan portal disabled */
#define DPAA2_SWP_STAT_EINVAL		0xFE	/* Invalid argument */
#define DPAA2_SWP_STAT_ERR		0xFF	/* General error */

/* Interrupt Enable Register bits. */
#define DPAA2_SWP_INTR_EQRI		0x01
#define DPAA2_SWP_INTR_EQDI		0x02
#define DPAA2_SWP_INTR_DQRI		0x04
#define DPAA2_SWP_INTR_RCRI		0x08
#define DPAA2_SWP_INTR_RCDI		0x10
#define DPAA2_SWP_INTR_VDCI		0x20

/*
 * Public types.
 */

/**
 * @brief Descriptor of the QBMan software portal.
 *
 * cena_res:	Unmapped cache-enabled part of the portal's I/O memory.
 * cena_map:	Mapped cache-enabled part of the portal's I/O memory.
 * cinh_res:	Unmapped cache-inhibited part of the portal's I/O memory.
 * cinh_map:	Mapped cache-inhibited part of the portal's I/O memory.
 *
 * dpio_dev:	Device associated with the DPIO object to manage this portal.
 * swp_version:	Hardware IP version of the software portal.
 * swp_id:	Software portal ID.
 * has_notif:	True if the notification mode is used.
 * has_8prio:	True for a channel with 8 priority WQs. Ignored unless
 *		"has_notif" is true.
 */
typedef struct {
	struct resource 	*cena_res;
	struct resource_map	*cena_map;
	struct resource		*cinh_res;
	struct resource_map	*cinh_map;

	device_t		 dpio_dev;
	uint32_t		 swp_version;
	uint16_t		 swp_id;
	bool			 has_notif;
	bool			 has_8prio;
} dpaa2_swp_desc_t;

/*
 * Opaque pointers.
 */
typedef struct dpaa2_swp *dpaa2_swp_t;

/*
 * Management routines.
 */
int	 dpaa2_swp_init_portal(dpaa2_swp_t *portal, dpaa2_swp_desc_t *desc,
	     const uint16_t flags);
void	 dpaa2_swp_free_portal(dpaa2_swp_t portal);

/*
 * Software portal routines.
 */
void	 dpaa2_swp_set_intr_trigger(dpaa2_swp_t p, uint32_t mask);
uint32_t dpaa2_swp_get_intr_trigger(dpaa2_swp_t p);
uint32_t dpaa2_swp_read_intr_status(dpaa2_swp_t p);
void	 dpaa2_swp_clear_intr_status(dpaa2_swp_t p, uint32_t mask);
void	 dpaa2_swp_set_push_dequeue(dpaa2_swp_t p, uint8_t chan_idx, bool en);


#endif /* _DPAA2_SWP_H */
