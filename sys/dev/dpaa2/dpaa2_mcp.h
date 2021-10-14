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

#include "dpaa2_types.h"

/*
 * DPAA2 MC command interface helper routines.
 */

#define DPAA2_PORTAL_TIMEOUT		100000	/* us */

/* Portal flags. */
#define DPAA2_PORTAL_DEF		0x0u
#define DPAA2_PORTAL_ATOMIC		0x1u	/* Use spinlock for a portal */
#define DPAA2_PORTAL_NOWAIT_ALLOC	0x2u	/* Do not sleep during init */
#define DPAA2_PORTAL_LOCKED		0x4000u	/* Wait till portal's unlocked */
#define DPAA2_PORTAL_DESTROYED		0x8000u /* Terminate any operations */

/* Command flags. */
#define DPAA2_CMD_DEF			0x0u
#define DPAA2_CMD_HIGH_PRIO		0x80u	/* High priority command */
#define DPAA2_CMD_INTR_DIS		0x100u	/* Disable cmd finished intr */
#define DPAA2_CMD_NOWAIT_ALLOC		0x8000u	/* Do not sleep during init */

/* DPAA2 command return codes. */
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
/* Driver-specific return codes. */
#define DPAA2_CMD_STAT_UNKNOWN_OBJ	0xFD	/* Unknown DPAA2 object. */
#define DPAA2_CMD_STAT_EINVAL		0xFE	/* Invalid argument */
#define DPAA2_CMD_STAT_ERR		0xFF	/* General error */

/* Object's memory region flags. */
#define DPAA2_RC_REG_CACHEABLE		0x1	/* Cacheable memory mapping */

#define DPAA2_HW_FLAG_HIGH_PRIO		0x80u
#define DPAA2_SW_FLAG_INTR_DIS		0x01u

#define DPAA2_CMD_PARAMS_N		7u
#define DPAA2_LABEL_SZ			16

/* DPNI link configuration options. */
#define DPAA2_NI_LINK_OPT_AUTONEG	((uint64_t) 0x01u)
#define DPAA2_NI_LINK_OPT_HALF_DUPLEX	((uint64_t) 0x02u)
#define DPAA2_NI_LINK_OPT_PAUSE		((uint64_t) 0x04u)
#define DPAA2_NI_LINK_OPT_ASYM_PAUSE	((uint64_t) 0x08u)
#define DPAA2_NI_LINK_OPT_PFC_PAUSE	((uint64_t) 0x10u)

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

enum dpaa2_ni_queue_type {
	DPAA2_NI_QUEUE_RX		= 0,
	DPAA2_NI_QUEUE_TX		= 1,
	DPAA2_NI_QUEUE_TX_CONF		= 2,
	DPAA2_NI_QUEUE_RX_ERR		= 3
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
 * rc_api_major:	Major version of the DPRC API.
 * rc_api_minor:	Minor version of the DPRC API.
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
	uint8_t		label[DPAA2_LABEL_SZ];
	enum dpaa2_dev_type type;
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

/**
 * @brief Attributes of the DPNI object.
 *
 * options:	 ...
 * wriop_ver:	 Revision of the underlying WRIOP hardware block.
 */
typedef struct {
	uint32_t		options;
	uint16_t		wriop_ver;
	struct {
		uint16_t	fs;
		uint8_t		mac;
		uint8_t		vlan;
		uint8_t		qos;
	} entries;
	struct {
		uint8_t		queues;
		uint8_t		rx_tcs;
		uint8_t		tx_tcs;
		uint8_t		channels;
		uint8_t		cgs;
	} num;
	struct {
		uint8_t		fs;
		uint8_t		qos;
	} key_size;
} dpaa2_ni_attr_t;

/**
 * @brief Buffer layout attributes.
 *
 * pd_size:		Size kept for private data (in bytes, max. 64)
 * fd_align:		Frame data alignment.
 * head_size:		Data head room.
 * tail_size:		Data tail room.
 * options:		...
 * pass_timestamp:	Timestamp is included in the buffer layout.
 * pass_parser_result:	Parsing results are included in the buffer layout.
 * pass_frame_status:	Frame status is included in the buffer layout.
 * pass_sw_opaque:	SW annotation is activated.
 * queue_type:		Type of a queue this configuration applies to.
 */
typedef struct {
	uint16_t	pd_size;
	uint16_t	fd_align;
	uint16_t	head_size;
	uint16_t	tail_size;
	uint16_t	options;
	bool		pass_timestamp;
	bool		pass_parser_result;
	bool		pass_frame_status;
	bool		pass_sw_opaque;
	enum dpaa2_ni_queue_type queue_type;
} dpaa2_ni_buf_layout_t;

/**
 * @brief Link configuration.
 *
 * options:	Mask of available options.
 * adv_speeds:	Speeds that are advertised for autoneg.
 * rate:	Rate in Mbps.
 */
typedef struct {
	uint64_t	options;
	uint64_t	adv_speeds;
	uint32_t	rate;
} dpaa2_ni_link_cfg_t;

/**
 * @brief DPAA2 endpoint descriptor.
 *
 * obj_id:	Endpoint object ID.
 * if_id:	Interface ID; for endpoints with multiple interfaces
 *		(DPSW, DPDMUX), 0 - otherwise.
 * type:	Endpoint object type, null-terminated string.
 */
typedef struct {
	uint32_t	obj_id;
	uint32_t	if_id;
	enum dpaa2_dev_type type;
} dpaa2_ep_desc_t;

typedef struct dpaa2_mcp *dpaa2_mcp_t;
typedef struct dpaa2_cmd *dpaa2_cmd_t;

int	dpaa2_mcp_init_portal(dpaa2_mcp_t *portal, struct resource *res,
	    struct resource_map *map, const uint16_t flags);
int	dpaa2_mcp_init_command(dpaa2_cmd_t *cmd, const uint16_t flags);
void	dpaa2_mcp_free_portal(dpaa2_mcp_t portal);
void	dpaa2_mcp_free_command(dpaa2_cmd_t cmd);
void	dpaa2_mcp_set_token(dpaa2_cmd_t cmd, const uint16_t token);
void	dpaa2_mcp_set_flags(dpaa2_cmd_t cmd, const uint16_t flags);
void	dpaa2_mcp_lock(dpaa2_mcp_t portal, uint16_t *flags);
void	dpaa2_mcp_unlock(dpaa2_mcp_t portal);

#endif /* _DPAA2_MCP_H */
