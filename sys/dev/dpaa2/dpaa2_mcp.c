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
 * MC command interface and the DPAA2 Management Complex Portal (DPMCP) driver.
 *
 * DPMCP is an optional object exported by MC to control the MC portal operation
 * mode (polling or interrupt-based).
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

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

#define PORTAL_TIMEOUT		100000	/* us */

#define CMD_PARAMS_N		7u
#define CMD_SLEEP_TIMEOUT	1u	/* ms */
#define CMD_SLEEP_ATTEMPTS	150u	/* max. 150 ms */
#define CMD_SPIN_TIMEOUT	10u	/* us */
#define CMD_SPIN_ATTEMPTS	15u	/* max. 150 us */

#define HW_FLAG_HIGH_PRIO	0x80u
#define SW_FLAG_INTR_DIS	0x01u

#define TYPE_LEN_MAX		16u
#define LABEL_LEN_MAX		16u

/* ------------------------- DPRC command IDs ------------------------------- */
#define CMD_RC_BASE_VERSION	1
#define CMD_RC_2ND_VERSION	2
#define CMD_RC_3RD_VERSION	3
#define CMD_RC_ID_OFFSET	4

#define CMD_RC(id)	(((id) << CMD_RC_ID_OFFSET) | CMD_RC_BASE_VERSION)
#define CMD_RC_V2(id)	(((id) << CMD_RC_ID_OFFSET) | CMD_RC_2ND_VERSION)
#define CMD_RC_V3(id)	(((id) << CMD_RC_ID_OFFSET) | CMD_RC_3RD_VERSION)

#define CMDID_RC_OPEN				CMD_RC(0x805)
#define CMDID_RC_CLOSE				CMD_RC(0x800)
#define CMDID_RC_GET_API_VERSION		CMD_RC(0xA05)
#define CMDID_RC_GET_ATTR			CMD_RC(0x004)
#define CMDID_RC_RESET_CONT			CMD_RC(0x005)
#define CMDID_RC_RESET_CONT_V2			CMD_RC_V2(0x005)
#define CMDID_RC_SET_IRQ			CMD_RC(0x010)
#define CMDID_RC_SET_IRQ_ENABLE			CMD_RC(0x012)
#define CMDID_RC_SET_IRQ_MASK			CMD_RC(0x014)
#define CMDID_RC_GET_IRQ_STATUS			CMD_RC(0x016)
#define CMDID_RC_CLEAR_IRQ_STATUS		CMD_RC(0x017)
#define CMDID_RC_GET_CONT_ID			CMD_RC(0x830)
#define CMDID_RC_GET_OBJ_COUNT			CMD_RC(0x159)
#define CMDID_RC_GET_OBJ			CMD_RC(0x15A)
#define CMDID_RC_GET_OBJ_DESC			CMD_RC(0x162)
#define CMDID_RC_GET_OBJ_REG			CMD_RC(0x15E)
#define CMDID_RC_GET_OBJ_REG_V2			CMD_RC_V2(0x15E)
#define CMDID_RC_GET_OBJ_REG_V3			CMD_RC_V3(0x15E)
#define CMDID_RC_SET_OBJ_IRQ			CMD_RC(0x15F)
#define CMDID_RC_GET_CONNECTION			CMD_RC(0x16C)

/* ------------------------- DPIO command IDs ------------------------------- */
#define CMD_IO_BASE_VERSION	1
#define CMD_IO_ID_OFFSET	4

#define CMD_IO(id)	(((id) << CMD_IO_ID_OFFSET) | CMD_IO_BASE_VERSION)

#define CMDID_IO_OPEN				CMD_IO(0x803)
#define CMDID_IO_CLOSE				CMD_IO(0x800)
#define CMDID_IO_ENABLE				CMD_IO(0x002)
#define CMDID_IO_DISABLE			CMD_IO(0x003)

/* ------------------------- DPNI command IDs ------------------------------- */
#define CMD_NI_BASE_VERSION	1
#define CMD_NI_ID_OFFSET	4

#define CMD_NI(id)	(((id) << CMD_NI_ID_OFFSET) | CMD_NI_BASE_VERSION)

#define CMDID_NI_OPEN				CMD_NI(0x801)
#define CMDID_NI_CLOSE				CMD_NI(0x800)

#define LOCK_PORTAL(portal, flags) do {					\
	if ((portal)->flags & DPAA2_PORTAL_ATOMIC) {			\
		mtx_lock_spin(&(portal)->lock);				\
		(flags) = (portal)->flags;				\
	} else {							\
		mtx_lock(&(portal)->lock);				\
		while ((portal)->flags & DPAA2_PORTAL_LOCKED)		\
			cv_wait(&(portal)->cv, &(portal)->lock);	\
		(flags) = (portal)->flags;				\
		(portal)->flags |= DPAA2_PORTAL_LOCKED;			\
		mtx_unlock(&(portal)->lock);				\
	}								\
} while (0)
#define UNLOCK_PORTAL(portal) do {				\
	if ((portal)->flags & DPAA2_PORTAL_ATOMIC) {		\
		mtx_unlock_spin(&(portal)->lock);		\
	} else {						\
		mtx_lock(&(portal)->lock);			\
		(portal)->flags &= ~DPAA2_PORTAL_LOCKED;	\
		cv_signal(&(portal)->cv);			\
		mtx_unlock(&(portal)->lock);			\
	}							\
} while (0)

MALLOC_DEFINE(M_DPAA2_MCP, "dpaa2_mcp_memory", "DPAA2 Management Complex Portal "
    "memory");

/**
 * @brief Helper object to send commands to the MC portal.
 *
 * res:			Unmapped portal's I/O memory.
 * map:			Mapped portal's I/O memory.
 * lock:		Lock to send a command to the portal and wait for the
 *			result.
 * cv:			Conditional variable helps to wait for the helper
 *			object's state change.
 * flags:		Current object state.
 * rc_api_major:	Major version of the DPRC API (cached).
 * rc_api_minor:	Minor version of the DPRC API (cached).
 */
struct dpaa2_mcp {
	struct resource		*res;
	struct resource_map	*map;
	struct mtx		 lock;
	struct cv		 cv;
	uint16_t		 flags;
	uint16_t		 rc_api_major;
	uint16_t		 rc_api_minor;
};

/**
 * @brief Command object holds data to be written to the MC portal.
 *
 * header:	8 least significant bytes of the MC portal.
 * params:	Parameters to pass together with the command to MC. Might keep
 *		command execution results.
 */
struct dpaa2_cmd {
	uint64_t		 header;
	uint64_t		 params[CMD_PARAMS_N];
};

/**
 * @brief Helper object to access fields of the MC command header.
 *
 * srcid:	The SoC architected source ID of the submitter. This field is
 *		reserved and cannot be written by a GPP processor.
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
	uint8_t			 srcid;
	uint8_t			 flags_hw;
	uint8_t			 status;
	uint8_t			 flags_sw;
	uint16_t		 token;
	uint16_t		 cmdid;
};

/**
 * @brief Helper object to access fields of the DPAA2 object information
 * response.
 */
struct __packed dpaa2_obj {
	uint32_t		 _reserved1;
	uint32_t		 id;
	uint16_t		 vendor;
	uint8_t			 irq_count;
	uint8_t			 reg_count;
	uint32_t		 state;
	uint16_t		 ver_major;
	uint16_t		 ver_minor;
	uint16_t		 flags;
	uint16_t		 _reserved2;
	uint8_t			 type[16];
	uint8_t			 label[16];
};

/**
 * @brief Helper object to access fields of the DPRC attributes response.
 */
struct __packed dpaa2_rc_attr {
	uint32_t		 cont_id;
	uint16_t		 icid;
	uint16_t		 _reserved1;
	uint32_t		 options;
	uint32_t		 portal_id;
};

static int exec_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    const uint16_t cmdid);
static void send_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);
static int wait_for_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);
static int set_irq_enable(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    const uint8_t irq_idx, const uint8_t enable, const uint16_t cmdid);

/*
 * Management routines.
 */

int
dpaa2_mcp_init_portal(dpaa2_mcp_t *portal, struct resource *res,
    struct resource_map *map, const uint16_t flags)
{
	const int mflags = flags & DPAA2_PORTAL_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO) : (M_WAITOK | M_ZERO);
	dpaa2_mcp_t p;

	if (!portal || !res || !map)
		return (EINVAL);

	p = malloc(sizeof(struct dpaa2_mcp), M_DPAA2_MCP, mflags);
	if (!p)
		return (ENOMEM);

	p->res = res;
	p->map = map;
	p->flags = flags;
	if (flags & DPAA2_PORTAL_ATOMIC) {
		/*
		 * NOTE: Do not initialize cv for atomic portal: it's not
		 * possible to sleep on it in case of a spin mutex.
		 */
		mtx_init(&p->lock, "MC portal spin lock", NULL, MTX_SPIN);
	} else {
		mtx_init(&p->lock, "MC portal sleep lock", NULL, MTX_DEF);
		cv_init(&p->cv, "MC portal cv");
	}
	/* Reset DPRC API version to cache later. */
	p->rc_api_major = 0;
	p->rc_api_minor = 0;

	*portal = p;

	return (0);
}

void
dpaa2_mcp_free_portal(dpaa2_mcp_t portal)
{
	uint16_t flags;

	if (portal) {
		if (portal->flags & DPAA2_PORTAL_ATOMIC) {
			mtx_destroy(&portal->lock);
			free(portal, M_DPAA2_MCP);
		} else {
			/*
			 * Signal all threads sleeping on portal's cv that it's
			 * going to be destroyed.
			 */
			LOCK_PORTAL(portal, flags);
			portal->flags |= DPAA2_PORTAL_DESTROYED;
			cv_signal(&portal->cv);
			UNLOCK_PORTAL(portal);

			/* Let threads stop using this portal. */
			DELAY(PORTAL_TIMEOUT);

			mtx_destroy(&portal->lock);
			cv_destroy(&portal->cv);
			free(portal, M_DPAA2_MCP);
		}
	}
}

int
dpaa2_mcp_init_command(dpaa2_cmd_t *cmd, const uint16_t flags)
{
	const int mflags = flags & DPAA2_CMD_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO) : (M_WAITOK | M_ZERO);
	dpaa2_cmd_t c;
	struct dpaa2_cmd_header *hdr;

	if (!cmd)
		return (EINVAL);

	c = malloc(sizeof(struct dpaa2_cmd), M_DPAA2_MCP, mflags);
	if (!c)
		return (ENOMEM);

	hdr = (struct dpaa2_cmd_header *) &c->header;
	hdr->srcid = 0;
	hdr->status = DPAA2_CMD_STAT_OK;
	hdr->token = 0;
	hdr->cmdid = 0;
	hdr->flags_hw = DPAA2_CMD_DEF;
	hdr->flags_sw = DPAA2_CMD_DEF;
	if (flags & DPAA2_CMD_HIGH_PRIO)
		hdr->flags_hw |= HW_FLAG_HIGH_PRIO;
	if (flags & DPAA2_CMD_INTR_DIS)
		hdr->flags_sw |= SW_FLAG_INTR_DIS;
	for (uint32_t i = 0; i < CMD_PARAMS_N; i++)
		c->params[i] = 0;
	*cmd = c;

	return (0);
}

void
dpaa2_mcp_free_command(dpaa2_cmd_t cmd)
{
	if (cmd)
		free(cmd, M_DPAA2_MCP);
}

void
dpaa2_mcp_set_token(dpaa2_cmd_t cmd, const uint16_t token)
{
	struct dpaa2_cmd_header *hdr;

	if (cmd) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		hdr->token = token;
	}
}

void
dpaa2_mcp_set_flags(dpaa2_cmd_t cmd, const uint16_t flags)
{
	struct dpaa2_cmd_header *hdr;

	if (cmd) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		hdr->flags_hw = DPAA2_CMD_DEF;
		hdr->flags_sw = DPAA2_CMD_DEF;

		if (flags & DPAA2_CMD_HIGH_PRIO)
			hdr->flags_hw |= HW_FLAG_HIGH_PRIO;
		if (flags & DPAA2_CMD_INTR_DIS)
			hdr->flags_sw |= SW_FLAG_INTR_DIS;
	}
}

/*
 * Data Path Management (DPMNG) commands.
 */

int
dpaa2_cmd_mng_get_version(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint32_t *major,
    uint32_t *minor, uint32_t *rev)
{
	int error;

	if (!portal || !cmd || !major || !minor || !rev)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(portal, cmd, 0x8311);
	if (!error) {
		*major = cmd->params[0] >> 32;
		*minor = cmd->params[1] & 0xFFFFFFFF;
		*rev = cmd->params[0] & 0xFFFFFFFF;
	}

	return (error);
}

int
dpaa2_cmd_mng_get_soc_version(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t *pvr, uint32_t *svr)
{
	int error;

	if (!portal || !cmd || !pvr || !svr)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(portal, cmd, 0x8321);
	if (!error) {
		*pvr = cmd->params[0] >> 32;
		*svr = cmd->params[0] & 0xFFFFFFFF;
	}

	return (error);
}

int
dpaa2_cmd_mng_get_container_id(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t *cont_id)
{
	int error;

	if (!portal || !cmd || !cont_id)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(portal, cmd, 0x8301);
	if (!error)
		*cont_id = cmd->params[0] & 0xFFFFFFFF;

	return (error);
}

/*
 * Data Path Resource Containter (DPRC) commands.
 */

int
dpaa2_cmd_rc_open(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint32_t cont_id,
    uint16_t *token)
{
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = cont_id;

	error = exec_command(portal, cmd, CMDID_RC_OPEN);
	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

int
dpaa2_cmd_rc_close(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(portal, cmd, CMDID_RC_CLOSE));
}

int
dpaa2_cmd_rc_get_obj_count(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t *obj_count)
{
	int error;

	if (!portal || !cmd || !obj_count)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(portal, cmd, CMDID_RC_GET_OBJ_COUNT);
	if (!error)
		*obj_count = (uint32_t)(cmd->params[0] >> 32);

	return (error);
}

int
dpaa2_cmd_rc_get_obj(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t obj_idx, dpaa2_obj_t *obj)
{
	struct dpaa2_obj *pobj;
	int error;

	if (!portal || !cmd || !obj)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = obj_idx;

	error = exec_command(portal, cmd, CMDID_RC_GET_OBJ);
	if (!error) {
		pobj = (struct dpaa2_obj *) &cmd->params[0];
		obj->id = pobj->id;
		obj->vendor = pobj->vendor;
		obj->irq_count = pobj->irq_count;
		obj->reg_count = pobj->reg_count;
		obj->state = pobj->state;
		obj->ver_major = pobj->ver_major;
		obj->ver_minor = pobj->ver_minor;
		obj->flags = pobj->flags;
		memcpy(obj->type, pobj->type, sizeof(pobj->type));
		memcpy(obj->label, pobj->label, sizeof(pobj->label));
	}

	return (error);
}

int
dpaa2_cmd_rc_get_obj_descriptor(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t obj_id, const char *type, dpaa2_obj_t *obj)
{
	struct __packed get_obj_desc_args {
		uint32_t	obj_id;
		uint32_t	_reserved1;
		uint8_t		type[16];
	} *args;
	struct dpaa2_obj *pobj;
	int error;

	if (!portal || !cmd || !type || !obj)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct get_obj_desc_args *) &cmd->params[0];
	args->obj_id = obj_id;
	memcpy(args->type, type, min(strlen(type) + 1, TYPE_LEN_MAX));

	error = exec_command(portal, cmd, CMDID_RC_GET_OBJ_DESC);
	if (!error) {
		pobj = (struct dpaa2_obj *) &cmd->params[0];
		obj->id = pobj->id;
		obj->vendor = pobj->vendor;
		obj->irq_count = pobj->irq_count;
		obj->reg_count = pobj->reg_count;
		obj->state = pobj->state;
		obj->ver_major = pobj->ver_major;
		obj->ver_minor = pobj->ver_minor;
		obj->flags = pobj->flags;
		memcpy(obj->type, pobj->type, sizeof(pobj->type));
		memcpy(obj->label, pobj->label, sizeof(pobj->label));
	}

	return (error);
}

int
dpaa2_cmd_rc_get_attributes(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    dpaa2_rc_attr_t *attr)
{
	struct dpaa2_rc_attr *pattr;
	int error;

	if (!portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(portal, cmd, CMDID_RC_GET_ATTR);
	if (!error) {
		pattr = (struct dpaa2_rc_attr *) &cmd->params[0];
		attr->cont_id = pattr->cont_id;
		attr->portal_id = pattr->portal_id;
		attr->options = pattr->options;
		attr->icid = pattr->icid;
	}

	return (error);
}

int
dpaa2_cmd_rc_get_obj_region(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t obj_id, uint8_t reg_idx, const char *type,
    dpaa2_rc_obj_region_t *reg)
{
	struct __packed obj_region_args {
		uint32_t	obj_id;
		uint16_t	_reserved1;
		uint8_t		reg_idx;
		uint8_t		_reserved2;
		uint64_t	_reserved3;
		uint64_t	_reserved4;
		uint8_t		type[16];
	} *args;
	struct __packed obj_region {
		uint64_t	_reserved1;
		uint64_t	base_offset;
		uint32_t	size;
		uint32_t	type;
		uint32_t	flags;
		uint32_t	_reserved2;
		uint64_t	base_paddr;
	} *resp;
	uint16_t cmdid, api_major, api_minor;
	int error;

	if (!portal || !cmd || !type || !reg)
		return (DPAA2_CMD_STAT_ERR);

	/*
	 * If the DPRC object version was not yet cached, cache it now.
	 * Otherwise use the already cached value.
	 */
	if (!portal->rc_api_major && !portal->rc_api_minor) {
		error = dpaa2_cmd_rc_get_api_version(portal, cmd, &api_major,
		    &api_minor);
		if (error)
			return (error);
		portal->rc_api_major = api_major;
		portal->rc_api_minor = api_minor;
	} else {
		api_major = portal->rc_api_major;
		api_minor = portal->rc_api_minor;
	}

	if (api_major > 6u || (api_major == 6u && api_minor >= 6u))
		/*
		 * MC API version 6.6 changed the size of the MC portals and
		 * software portals to 64K (as implemented by hardware).
		 */
		cmdid = CMDID_RC_GET_OBJ_REG_V3;
	else if (api_major == 6u && api_minor >= 3u)
		/*
		 * MC API version 6.3 introduced a new field to the region
		 * descriptor: base_address.
		 */
		cmdid = CMDID_RC_GET_OBJ_REG_V2;
	else
		cmdid = CMDID_RC_GET_OBJ_REG;

	args = (struct obj_region_args *) &cmd->params[0];
	args->obj_id = obj_id;
	args->reg_idx = reg_idx;
	memcpy(args->type, type, min(strlen(type) + 1, TYPE_LEN_MAX));

	error = exec_command(portal, cmd, cmdid);
	if (!error) {
		resp = (struct obj_region *) &cmd->params[0];
		reg->base_paddr = resp->base_paddr;
		reg->base_offset = resp->base_offset;
		reg->size = resp->size;
		reg->flags = resp->flags;
		reg->type = resp->type & 0xFu;
	}

	return (error);
}

int
dpaa2_cmd_rc_get_api_version(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint16_t *major, uint16_t *minor)
{
	struct __packed rc_api_version {
		uint16_t	major;
		uint16_t	minor;
	} *resp;
	int error;

	if (!portal || !cmd || !major || !minor)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(portal, cmd, CMDID_RC_GET_API_VERSION);
	if (!error) {
		resp = (struct rc_api_version *) &cmd->params[0];
		*major = resp->major;
		*minor = resp->minor;
	}

	return (error);
}

int
dpaa2_cmd_rc_set_irq_enable(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint8_t enable)
{
	return (set_irq_enable(portal, cmd, irq_idx, enable,
	    CMDID_RC_SET_IRQ_ENABLE));
}

int
dpaa2_cmd_rc_set_obj_irq(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint64_t addr, uint32_t data, uint32_t irq_usr, uint32_t obj_id,
    const char *type)
{
	struct __packed set_obj_irq_args {
		uint32_t	data;
		uint8_t		irq_idx;
		uint8_t		_reserved1[3];
		uint64_t	addr;
		uint32_t	irq_usr;
		uint32_t	obj_id;
		uint8_t		type[16];
	} *args;

	if (!portal || !cmd || !type)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct set_obj_irq_args *) &cmd->params[0];
	args->irq_idx = irq_idx;
	args->addr = addr;
	args->data = data;
	args->irq_usr = irq_usr;
	args->obj_id = obj_id;
	memcpy(args->type, type, min(strlen(type) + 1, TYPE_LEN_MAX));

	return (exec_command(portal, cmd, CMDID_RC_SET_OBJ_IRQ));
}

/*
 * Data Path Network Interface (DPNI) commands.
 */

int
dpaa2_cmd_ni_open(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, const uint32_t dpni_id,
    uint16_t *token)
{
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpni_id;
	error = exec_command(portal, cmd, CMDID_NI_OPEN);
 	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

int
dpaa2_cmd_ni_close(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(portal, cmd, CMDID_NI_CLOSE));
}

/*
 * Data Path I/O (DPIO) commands.
 */

int
dpaa2_cmd_io_open(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, const uint32_t dpio_id,
    uint16_t *token)
{
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpio_id;
	error = exec_command(portal, cmd, CMDID_IO_OPEN);
	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

int
dpaa2_cmd_io_close(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(portal, cmd, CMDID_IO_CLOSE));
}

int
dpaa2_cmd_io_enable(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(portal, cmd, CMDID_IO_ENABLE));
}

int
dpaa2_cmd_io_disable(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(portal, cmd, CMDID_IO_DISABLE));
}

/*
 * Internal functions.
 */

static int
set_irq_enable(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, const uint8_t irq_idx,
    const uint8_t enable, const uint16_t cmdid)
{
	struct __packed set_irq_enable_args {
		uint8_t		enable;
		uint8_t		_reserved1;
		uint16_t	_reserved2;
		uint8_t		irq_idx;
		uint8_t		_reserved3;
		uint16_t	_reserved4;
		uint64_t	_reserved5[6];
	} *args;

	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct set_irq_enable_args *) &cmd->params[0];
	args->irq_idx = irq_idx;
	args->enable = enable == 0u ? 0u : 1u;

	return (exec_command(portal, cmd, cmdid));
}

static int
exec_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint16_t cmdid)
{
	struct dpaa2_cmd_header *hdr;
	uint16_t flags;
	int error;

	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	/* Prepare a command for the MC hardware. */
	hdr = (struct dpaa2_cmd_header *) &cmd->header;
	hdr->cmdid = cmdid;
	hdr->status = DPAA2_CMD_STAT_READY;

	LOCK_PORTAL(portal, flags);
	if (flags & DPAA2_PORTAL_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		UNLOCK_PORTAL(portal);
		return (DPAA2_CMD_STAT_INVALID_STATE);
	}

	/* Send a command to MC and wait for the result. */
	send_command(portal, cmd);
	error = wait_for_command(portal, cmd);
	if (error) {
		UNLOCK_PORTAL(portal);
		return (DPAA2_CMD_STAT_ERR);
	}
	if (hdr->status != DPAA2_CMD_STAT_OK) {
		UNLOCK_PORTAL(portal);
		return (int)(hdr->status);
	}
	UNLOCK_PORTAL(portal);

	return (DPAA2_CMD_STAT_OK);
}

static void
send_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	/* Write command parameters. */
	for (uint32_t i = 1; i <= CMD_PARAMS_N; i++)
		bus_write_8(portal->map, sizeof(uint64_t) * i, cmd->params[i-1]);

	bus_barrier(portal->map, 0, sizeof(struct dpaa2_cmd),
	    BUS_SPACE_BARRIER_WRITE);

	/* Write command header to trigger execution. */
	bus_write_8(portal->map, 0, cmd->header);
}

static int
wait_for_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	const uint16_t atomic_portal = portal->flags & DPAA2_PORTAL_ATOMIC;
	const uint32_t attempts = atomic_portal ? CMD_SPIN_ATTEMPTS
	    : CMD_SLEEP_ATTEMPTS;

	struct dpaa2_cmd_header *hdr;
	uint64_t val;
	uint32_t i;

	/* Wait for a command execution result from the MC hardware. */
	for (i = 1; i <= attempts; i++) {
		val = bus_read_8(portal->map, 0);
		hdr = (struct dpaa2_cmd_header *) &val;
		if (hdr->status != DPAA2_CMD_STAT_READY)
			break;

		if (atomic_portal)
			DELAY(CMD_SPIN_TIMEOUT);
		else
			pause("mcp_pa", CMD_SLEEP_TIMEOUT);
	}

	/* Update command results. */
	cmd->header = val;
	for (i = 1; i <= CMD_PARAMS_N; i++)
		cmd->params[i-1] = bus_read_8(portal->map, i * sizeof(uint64_t));

	/* Return an error on expired timeout. */
	if (i > attempts)
		return (DPAA2_CMD_STAT_TIMEOUT);

	return (DPAA2_CMD_STAT_OK);
}
