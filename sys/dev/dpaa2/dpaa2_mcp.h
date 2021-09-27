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

#ifndef	_DPAA2_MCP_H
#define	_DPAA2_MCP_H

#include <sys/rman.h>
#include <sys/condvar.h>
#include <sys/mutex.h>

/*
 * DPAA2 MC command interface helper routines.
 */

/* Portal flags. */
#define DPAA2_PORTAL_DEF		0x0u
#define DPAA2_PORTAL_ATOMIC		0x1u	/* Use spinlock for a portal */
#define DPAA2_PORTAL_NOWAIT_ALLOC	0x2u	/* Do not sleep during init */
#define DPAA2_PORTAL_LOCKED		0x4000u	/* Wait till portal's unlocked */
#define DPAA2_PORTAL_DESTROYED		0x8000u /* Terminate any operations */

#define DPAA2_CMD_PARAMS_N		7u
/* Command flags. */
#define DPAA2_CMD_DEF			0x0u
#define DPAA2_CMD_HIGH_PRIO		0x80u	/* High priority command */
#define DPAA2_CMD_INTR_DIS		0x100u	/* Disable cmd finished intr */
#define DPAA2_CMD_NOWAIT_ALLOC		0x8000u	/* Do not sleep during init */

/* Command return codes. */
#define DPAA2_CMD_STAT_OK		0x0	/* Set by MC on success */
#define DPAA2_CMD_STAT_READY		0x1	/* Ready to be processed */
#define DPAA2_CMD_STAT_AUTH_ERR		0x3	/* Illegal object-portal-icid */
#define DPAA2_CMD_STAT_NO_PRIVILEGE	0x4	/* No privilege */
#define DPAA2_CMD_STAT_DMA_ERR		0x5	/* DMA or I/O error */
#define DPAA2_CMD_STAT_CONFIG_ERR	0x6	/* Invalid/conflicting params */
#define DPAA2_CMD_STAT_TIMEOUT		0x7	/* Command timed out */
#define DPAA2_CMD_STAT_NO_RESOURCE	0x8	/* No DPAA2 resources */
#define DPAA2_CMD_STAT_NO_MEMORY	0x9	/* No memory available */
#define DPAA2_CMD_STAT_BUSY		0xA	/* Device is busy */
#define DPAA2_CMD_STAT_UNSUPPORTED_OP	0xB	/* Unsupported operation */
#define DPAA2_CMD_STAT_INVALID_STATE	0xC	/* Invalid state */
#define DPAA2_CMD_STAT_EINVAL		0xFE	/* Invalid argument */
#define DPAA2_CMD_STAT_ERR		0xFF	/* General error */

/* Object's memory region flags. */
#define DPAA2_RC_REG_CACHEABLE		0x1	/* Cacheable memory mapping */

/*
 * Public types.
 */

enum dpaa2_rc_region_type {
	DPAA2_RC_REG_MC_PORTAL		= 0,
	DPAA2_RC_REG_QBMAN_PORTAL	= 1
};

enum dpaa2_io_chan_mode {
	DPAA2_IO_NO_CHANNEL		= 0,
	DPAA2_IO_LOCAL_CHANNEL		= 1
};

/**
 * @brief Helper object to send commands to the MC portal.
 *
 * res:			Unmapped portal's I/O memory.
 * map:			Mapped portal's I/O memory.
 * lock:		Lock to send a command to the portal and wait for the
 *			result.
 * cv:			Conditional variable helps to wait for the helper
 *			object's state change.
 * flags:		Current state of the object.
 * rc_api_major:	Major version of the DPRC API (cached).
 * rc_api_minor:	Minor version of the DPRC API (cached).
 */
struct dpaa2_mcp {
	struct resource *res;
	struct resource_map *map;
	struct mtx	lock;
	struct cv	cv;
	uint16_t	flags;
	uint16_t	rc_api_major;
	uint16_t	rc_api_minor;
};

/**
 * @brief Command object holds data to be written to the MC portal.
 *
 * header:	8 least significant bytes of the MC portal.
 * params:	Parameters to pass together with the command to MC. Might keep
 *		command execution results.
 */
struct dpaa2_cmd {
	uint64_t	header;
	uint64_t	params[DPAA2_CMD_PARAMS_N];
};

/**
 * @brief Helper object to access fields of the MC command header.
 *
 * srcid:	The SoC architected source ID of the submitter. This field is
 *		reserved and cannot be written by the driver.
 * flags_hw:	Bits from 8 to 15 of the command header. Most of them are
 *		reserved at the moment.
 * status:	Command ready/status. This field is used as the handshake field
 *		between MC and the driver. MC reports command completion with
 *		success/error codes in this field.
 * flags_sw:	...
 * token:	...
 * cmdid:	...
 */
struct __packed dpaa2_cmd_header {
	uint8_t		srcid;
	uint8_t		flags_hw;
	uint8_t		status;
	uint8_t		flags_sw;
	uint16_t	token;
	uint16_t	cmdid;
};

/**
 * @brief Information about DPAA2 object.
 *
 * id:		ID of a logical object resource.
 * vendor:	Object vendor identifier.
 * irq_count:	Number of interrupts supported by the object.
 * reg_count:	Number of mappable regions supported by the object.
 * state:	Object state (combination of states).
 * ver_major:	Major version of the object.
 * ver_minor:	Minor version of the object.
 * flags:	Object attributes flags.
 * type:	...
 * label:	...
 */
typedef struct {
	uint32_t	id;
	uint16_t	vendor;
	uint8_t		irq_count;
	uint8_t		reg_count;
	uint32_t	state;
	uint16_t	ver_major;
	uint16_t	ver_minor;
	uint16_t	flags;
	uint8_t		type[16];
	uint8_t		label[16];
} dpaa2_obj_t;

/**
 * @brief Attributes of the DPRC object.
 *
 * cont_id:	Container ID.
 * portal_id:	Container's portal ID.
 * options:	Container's options as set at container's creation.
 * icid:	Container's isolation context ID.
 */
typedef struct {
	uint32_t	cont_id;
	uint32_t	portal_id;
	uint32_t	options;
	uint16_t	icid;
} dpaa2_rc_attr_t;

/**
 * @brief Description of the object's memory region.
 *
 * base_paddr:	Region base physical address.
 * base_offset:	Region base offset.
 * size:	Region size (in bytes).
 * flags:	Region flags (cacheable, etc.)
 * type:	Type of a software portal this region belongs to.
 */
typedef struct {
	uint64_t	base_paddr;
	uint64_t	base_offset;
	uint32_t	size;
	uint32_t	flags;
	enum dpaa2_rc_region_type type;
} dpaa2_rc_obj_region_t;

/**
 * @brief Attributes of the DPIO object.
 *
 * swp_ce_paddr: Physical address of the software portal cache-enabled area.
 * swp_ci_paddr: Physical address of the software portal cache-inhibited area.
 * swp_version:	 Hardware IP version of the software portal.
 * id:		 DPIO object ID.
 * swp_id:	 Software portal ID.
 * priors_num:	 Number of priorities for the notification channel (1-8);
 *		 relevant only if channel mode is "local channel".
 * chan_mode:	 Notification channel mode.
 */
typedef struct {
	uint64_t	swp_ce_paddr;
	uint64_t	swp_ci_paddr;
	uint32_t	swp_version;
	uint32_t	id;
	uint16_t	swp_id;
	uint8_t		priors_num;
	enum dpaa2_io_chan_mode chan_mode;
} dpaa2_io_attr_t;

/**
 * @brief Attributes of the DPBP object.
 *
 * id:		 DPBP object ID.
 * bpid:	 Hardware buffer pool ID; should be used as an argument in
 *		 acquire/release operations on buffers.
 */
typedef struct {
	uint32_t	id;
	uint16_t	bpid;
} dpaa2_bp_attr_t;

typedef struct dpaa2_mcp *dpaa2_mcp_t;
typedef struct dpaa2_cmd *dpaa2_cmd_t;

int	dpaa2_mcp_init_portal(dpaa2_mcp_t *portal, struct resource *res,
	    struct resource_map *map, const uint16_t flags);
int	dpaa2_mcp_init_command(dpaa2_cmd_t *cmd, const uint16_t flags);
void	dpaa2_mcp_free_portal(dpaa2_mcp_t portal);
void	dpaa2_mcp_free_command(dpaa2_cmd_t cmd);
void	dpaa2_mcp_set_token(dpaa2_cmd_t cmd, const uint16_t token);
void	dpaa2_mcp_set_flags(dpaa2_cmd_t cmd, const uint16_t flags);

#endif /* _DPAA2_MCP_H */
