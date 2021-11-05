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

#include <sys/bus.h>

/*
 * QBMan software portal helper routines.
 */

/* All QBMan command and result structures use this "valid bit" encoding */
#define DPAA2_SWP_VALID_BIT		((uint32_t)0x80)

#define DPAA2_SWP_TIMEOUT		100000	/* us */
#define DPAA2_SWP_CMD_PARAMS_N		8u

/* Versions of the QBMan software portals. */
#define DPAA2_SWP_REV_4000		0x04000000
#define DPAA2_SWP_REV_4100		0x04010000
#define DPAA2_SWP_REV_4101		0x04010001
#define DPAA2_SWP_REV_5000		0x05000000

#define DPAA2_SWP_REV_MASK		0xFFFF0000

/* Register offsets in the cache-inhibited area */
#define DPAA2_SWP_CINH_CR		0x600 /* Management Command */
#define DPAA2_SWP_CINH_EQCR_PI		0x800 /* Enqueue Ring, Producer Index */
#define DPAA2_SWP_CINH_EQCR_CI		0x840 /* Enqueue Ring, Consumer Index */
#define DPAA2_SWP_CINH_CR_RT		0x900 /* CR Read Trigger */
#define DPAA2_SWP_CINH_VDQCR_RT		0x940 /* VDQCR Read Trigger */
#define DPAA2_SWP_CINH_EQCR_AM_RT	0x980
#define DPAA2_SWP_CINH_RCR_AM_RT	0x9C0
#define DPAA2_SWP_CINH_DQPI		0xA00
#define DPAA2_SWP_CINH_DCAP		0xAC0
#define DPAA2_SWP_CINH_SDQCR		0xB00 /* Static Dequeue Command */
#define DPAA2_SWP_CINH_EQCR_AM_RT2	0xB40
#define DPAA2_SWP_CINH_RCR_PI		0xC00 /* Release Ring, Producer Index */
#define DPAA2_SWP_CINH_RAR		0xCC0
#define DPAA2_SWP_CINH_ISR		0xE00
#define DPAA2_SWP_CINH_IER		0xE40
#define DPAA2_SWP_CINH_ISDR		0xE80
#define DPAA2_SWP_CINH_IIR		0xEC0
#define DPAA2_SWP_CINH_CFG		0xD00

/* CENA register offsets */
#define DPAA2_SWP_CENA_EQCR(n)		(0x000 + ((uint32_t)(n) << 6))
#define DPAA2_SWP_CENA_DQRR(n)		(0x200 + ((uint32_t)(n) << 6))
#define DPAA2_SWP_CENA_RCR(n)		(0x400 + ((uint32_t)(n) << 6))
#define DPAA2_SWP_CENA_CR		(0x600) /* Command Ring offset */
#define DPAA2_SWP_CENA_RR(vb)		(0x700 + ((uint32_t)(vb) >> 1))
#define DPAA2_SWP_CENA_VDQCR		(0x780)
#define DPAA2_SWP_CENA_EQCR_CI		(0x840)

/* CENA register offsets in memory-backed mode */
#define DPAA2_SWP_CENA_DQRR_MEM(n)	(0x0800 + ((uint32_t)(n) << 6))
#define DPAA2_SWP_CENA_RCR_MEM(n)	(0x1400 + ((uint32_t)(n) << 6))
#define DPAA2_SWP_CENA_CR_MEM		(0x1600) /* CR offset (memory backed) */
#define DPAA2_SWP_CENA_RR_MEM		(0x1680)
#define DPAA2_SWP_CENA_VDQCR_MEM	(0x1780)
#define DPAA2_SWP_CENA_EQCR_CI_MEMBACK	(0x1840)

/* Shifts in the portal's configuration register. */
#define DPAA2_SWP_CFG_DQRR_MF_SHIFT	20
#define DPAA2_SWP_CFG_EST_SHIFT		16
#define DPAA2_SWP_CFG_CPBS_SHIFT	15
#define DPAA2_SWP_CFG_WN_SHIFT		14
#define DPAA2_SWP_CFG_RPM_SHIFT		12
#define DPAA2_SWP_CFG_DCM_SHIFT		10
#define DPAA2_SWP_CFG_EPM_SHIFT		8
#define DPAA2_SWP_CFG_VPM_SHIFT		7
#define DPAA2_SWP_CFG_CPM_SHIFT		6
#define DPAA2_SWP_CFG_SD_SHIFT		5
#define DPAA2_SWP_CFG_SP_SHIFT		4
#define DPAA2_SWP_CFG_SE_SHIFT		3
#define DPAA2_SWP_CFG_DP_SHIFT		2
#define DPAA2_SWP_CFG_DE_SHIFT		1
#define DPAA2_SWP_CFG_EP_SHIFT		0

/* Static Dequeue Command Register attribute codes */
#define DPAA2_SDQCR_FC_SHIFT		29 /* Dequeue Command Frame Count */
#define DPAA2_SDQCR_FC_MASK		0x1
#define DPAA2_SDQCR_DCT_SHIFT		24 /* Dequeue Command Type */
#define DPAA2_SDQCR_DCT_MASK		0x3
#define DPAA2_SDQCR_TOK_SHIFT		16 /* Dequeue Command Token */
#define DPAA2_SDQCR_TOK_MASK		0xff
#define DPAA2_SDQCR_SRC_SHIFT		0  /* Dequeue Source */
#define DPAA2_SDQCR_SRC_MASK		0xffff

/*
 * Read trigger bit is used to trigger QMan to read a command from memory,
 * without having software perform a cache flush to force a write of the command
 * to QMan.
 *
 * NOTE: Implemented in QBMan 5.0 or above.
 */
#define DPAA2_SWP_RT_MODE		((uint32_t)0x100)

/*
 * Portal flags.
 *
 * TODO: Use the same flags for both MC and software portals.
 */
#define DPAA2_SWP_DEF			0x0u
#define DPAA2_SWP_NOWAIT_ALLOC		0x2u	/* Do not sleep during init */
#define DPAA2_SWP_LOCKED		0x4000u	/* Wait till portal's unlocked */
#define DPAA2_SWP_DESTROYED		0x8000u /* Terminate any operations */

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
 * @brief Enqueue command descriptor.
 *
 * NOTE: 32 bytes.
 */
typedef struct __packed {
	uint8_t		verb;
	uint8_t		dca;
	uint16_t	seqnum;
	uint16_t	orpid;
	uint16_t	reserved;
	uint32_t	tgtid;
	uint32_t	tag;
	uint16_t	qdbin;
	uint8_t		qpri;
	uint8_t		reserved1[3];
	uint8_t		wae;
	uint8_t		rspid;
	uint64_t	rsp_addr;
} dpaa2_eq_desc_t;

/**
 * @brief DPAA2 frame descriptor.
 *
 * addr:	Memory address of the start of the buffer holding the frame
 *		data or the buffer containing the scatter/gather list.
 * length:	Length of the frame data (in bytes).
 * bpid:	Buffer pool ID (14 bit + BMT bit + IVP bit)
 * off_fmt_sl:	Frame data offset, frame format and short-length fields.
 * frame_ctx:   This field allows the sender of a frame to communicate some
 *		out-of-band information to the receiver of the frame.
 * ctrl:	Control bits (DD, SC, DROPP, PTAC, ERR, etc.)
 * flow_ctx:	Frame flow context.
 *
 * NOTE: 32 bytes.
 */
typedef struct __packed {
	uint64_t	addr;
	uint32_t	length;
	uint16_t	bpid;
	uint16_t	off_fmt_sl;
	uint32_t	frame_ctx;
	uint32_t	ctrl;
	uint64_t	flow_ctx;
} dpaa2_fd_t;

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

/**
 * @brief Command object holds data to be written to the software portal.
 *
 * params:	Parts of the command to write to the software portal. Might keep
 *		command execution results.
 */
struct dpaa2_swp_cmd {
	uint64_t	params[DPAA2_SWP_CMD_PARAMS_N];
};

typedef struct dpaa2_swp *dpaa2_swp_t;
typedef struct dpaa2_swp_cmd *dpaa2_swp_cmd_t;

/**
 * @brief Helper object to interact with the QBMan software portal.
 *
 * res:		Unmapped cache-enabled and cache-inhibited parts of the portal.
 * map:		Mapped cache-enabled and cache-inhibited parts of the portal.
 * desc:	Descriptor of the QBMan software portal.
 * lock:	Lock to guard an access to the portal.
 * cv:		Conditional variable helps to wait for the helper object's state
 *		change.
 * flags:	Current state of the object.
 * sdq:		Push dequeues status.
 * mc:		Management commands data.
 * mr:		Management response data.
 * dqrr:	Dequeue Response Ring is used to issue frame dequeue responses
 * 		from the QBMan to the driver.
 * eqcr:	Enqueue Command Ring is used to issue frame enqueue commands
 *		from the driver to the QBMan.
 */
struct dpaa2_swp {
	struct resource		*cena_res;
	struct resource_map	*cena_map;
	struct resource		*cinh_res;
	struct resource_map	*cinh_map;

	int (*enq)(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
	    const dpaa2_fd_t *fd);
	int (*enq_mult)(dpaa2_swp_t swp, const dpaa2_eq_desc_t *ed,
	    const dpaa2_fd_t *fd, uint32_t *flags, int frames_n);

	struct mtx		 lock;
	struct cv		 cv;
	const dpaa2_swp_desc_t	*desc;
	uint16_t		 flags;
	uint32_t		 sdq;
	uint8_t			 atomic;

	struct {
		uint32_t	 valid_bit; /* 0x00 or 0x80 */
		bus_dma_tag_t	 tag;
		bus_dmamap_t	 map;
		uint8_t		*vaddr;
		bus_addr_t	 paddr;
		uint32_t	 size; /* 64 bytes */
	} mc;

	struct {
		uint32_t	 valid_bit; /* 0x00 or 0x80 */
	} mr;

	struct {
		uint32_t	 next_idx;
		uint32_t	 valid_bit;
		uint8_t		 ring_size;
		bool		 reset_bug; /* dqrr reset workaround */
	} dqrr;

	struct {
		uint32_t	 pi; /* producer index */
		uint32_t	 pi_vb; /* PI valid bits */
		uint32_t	 pi_ring_size;
		uint32_t	 pi_ci_mask;
		uint32_t	 ci;
		int		 available;
		uint32_t	 pend;
		uint32_t	 no_pfdr;
	} eqcr;
};

/* Management routines. */

int	 dpaa2_swp_init_portal(dpaa2_swp_t *swp, dpaa2_swp_desc_t *desc,
	     const uint16_t flags);
int	 dpaa2_swp_init_atomic(dpaa2_swp_t *swp, dpaa2_swp_desc_t *desc,
	     const uint16_t flags);
void	 dpaa2_swp_free_portal(dpaa2_swp_t swp);
void	 dpaa2_swp_lock(dpaa2_swp_t swp, uint16_t *flags);
void	 dpaa2_swp_unlock(dpaa2_swp_t swp);
uint32_t dpaa2_swp_set_cfg(uint8_t max_fill, uint8_t wn, uint8_t est,
	     uint8_t rpm, uint8_t dcm, uint8_t epm, int sd, int sp, int se,
	     int dp, int de, int ep);

/* Read/write registers of a software portal. */

void	 dpaa2_swp_write_reg(dpaa2_swp_t swp, uint32_t offset, uint32_t val);
uint32_t dpaa2_swp_read_reg(dpaa2_swp_t swp, uint32_t offset);

/* Helper routines. */

void	 dpaa2_swp_clear_ed(dpaa2_eq_desc_t *ed);
void	 dpaa2_swp_set_ed_norp(dpaa2_eq_desc_t *ed, int response_always);
void	 dpaa2_swp_set_ed_fq(dpaa2_eq_desc_t *ed, uint32_t fqid);
void	 dpaa2_swp_set_intr_trigger(dpaa2_swp_t swp, uint32_t mask);
uint32_t dpaa2_swp_get_intr_trigger(dpaa2_swp_t swp);
uint32_t dpaa2_swp_read_intr_status(dpaa2_swp_t swp);
void	 dpaa2_swp_clear_intr_status(dpaa2_swp_t swp, uint32_t mask);
void	 dpaa2_swp_set_push_dequeue(dpaa2_swp_t swp, uint8_t chan_idx, bool en);
int	 dpaa2_swp_cdan_set_ctx_enable(dpaa2_swp_t swp, uint16_t chan_id,
	     uint64_t ctx);
int	 dpaa2_swp_cdan_set(dpaa2_swp_t swp, uint16_t chan_id, uint8_t we_mask,
	     uint8_t cdan_en, uint64_t ctx);

#endif /* _DPAA2_SWP_H */
