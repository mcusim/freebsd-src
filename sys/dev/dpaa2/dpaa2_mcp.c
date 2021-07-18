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

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

#define PORTAL_TIMEOUT		100000	/* us */

#define CMD_PARAMS_N		7u
#define CMD_SLEEP_TIMEOUT	1u	/* ms */
#define CMD_SLEEP_ATTEMPTS	150u	/* max 150 ms */
#define CMD_SPIN_TIMEOUT	10u	/* us */
#define CMD_SPIN_ATTEMPTS	15u	/* max. 150 us */

#define HW_FLAG_HIGH_PRIO	0x80u
#define SW_FLAG_INTR_DIS	0x01u

#define TYPE_LEN_MAX		16u
#define LABEL_LEN_MAX		16u

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

/*
 * Helper object to send commands to the MC portal.
 *
 * res: Unmapped portal's I/O memory.
 * map: Mapped portal's I/O memory.
 * lock: Lock to send a command to the portal and wait for the result.
 * cv: Conditional variable helps to wait for the helper object's state change.
 * flags: Current object state.
 */
struct dpaa2_mcp {
	struct resource		*res;
	struct resource_map	*map;
	struct mtx		 lock;
	struct cv		 cv;
	uint16_t		 flags;
};

/*
 * Command object holds data to be written to the MC portal.
 *
 * header: 8 least significant bytes of the MC portal.
 * params: Parameters to pass together with the command to MC. Might keep
 *         command execution results.
 */
struct dpaa2_cmd {
	uint64_t		 header;
	uint64_t		 params[CMD_PARAMS_N];
};

/*
 * Helper object which allows to access fields of the MC command header.
 *
 * srcid: The SoC architected source ID of the submitter. This field is reserved
 *        and cannot be written by a GPP processor.
 * flags_hw: Bits from 8 to 15 of the command header. Most of them are reserved
 *           at the moment.
 * status: Command ready/status. This field is used as the handshake field
 *         between MC and the driver. MC reports command completion with
 *         success/error codes in this field.
 * flags_sw:
 * token:
 * cmdid:
 */
struct __packed dpaa2_cmd_header {
	uint8_t			 srcid;
	uint8_t			 flags_hw;
	uint8_t			 status;
	uint8_t			 flags_sw;
	uint16_t		 token;
	uint16_t		 cmdid;
};

/*
 * Helper object which allows to access fields of the DPAA2 object information
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

/*
 * Helper object which allows to access fields of the DPRC attributes response.
 */
struct __packed dpaa2_rc_attr {
	uint32_t		 cont_id;
	uint16_t		 icid;
	uint16_t		 _reserved1;
	uint32_t		 options;
	uint32_t		 portal_id;
};

static int exec_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint16_t cmdid);
static void send_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);
static int wait_for_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);

/*
 * Management routines.
 */

int
dpaa2_mcp_init_portal(dpaa2_mcp_t *portal, struct resource *res,
    struct resource_map *map, const uint16_t flags)
{
	dpaa2_mcp_t p;
	int mflags = M_WAITOK | M_ZERO;

	if (!portal || !res || !map)
		return (1);
	*portal = NULL;

	/* Prepare malloc flags. */
	if (flags & DPAA2_PORTAL_NOWAIT_ALLOC)
		mflags = M_NOWAIT | M_ZERO;

	p = malloc(sizeof(struct dpaa2_mcp), M_DPAA2_MCP, mflags);
	if (!p)
		return (1);

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
	dpaa2_cmd_t c;
	struct dpaa2_cmd_header *hdr;
	int mflags = M_WAITOK | M_ZERO;

	if (!cmd)
		return (1);
	*cmd = NULL;

	/* Prepare malloc flags. */
	if (flags & DPAA2_CMD_NOWAIT_ALLOC)
		mflags = M_NOWAIT | M_ZERO;

	c = malloc(sizeof(struct dpaa2_cmd), M_DPAA2_MCP, mflags);
	if (!c)
		return (1);

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
	int rc;

	if (!portal || !cmd || !major || !minor || !rev)
		return (DPAA2_CMD_STAT_ERR);

	rc = exec_command(portal, cmd, 0x8311);
	*major = cmd->params[0] >> 32;
	*minor = cmd->params[1] & 0xFFFFFFFF;
	*rev = cmd->params[0] & 0xFFFFFFFF;

	return (rc);
}

int
dpaa2_cmd_mng_get_soc_version(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t *pvr, uint32_t *svr)
{
	int rc;

	if (!portal || !cmd || !pvr || !svr)
		return (DPAA2_CMD_STAT_ERR);

	rc = exec_command(portal, cmd, 0x8321);
	*pvr = cmd->params[0] >> 32;
	*svr = cmd->params[0] & 0xFFFFFFFF;

	return (rc);
}

int
dpaa2_cmd_mng_get_container_id(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t *cont_id)
{
	int rc;

	if (!portal || !cmd || !cont_id)
		return (DPAA2_CMD_STAT_ERR);

	rc = exec_command(portal, cmd, 0x8301);
	*cont_id = cmd->params[0] & 0xFFFFFFFF;

	return (rc);
}

/*
 * Data Path Resource Containter (DPRC) commands.
 */

int
dpaa2_cmd_rc_open(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint32_t cont_id,
    uint16_t *token)
{
	struct dpaa2_cmd_header *hdr;
	int rc;

	if (!portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = cont_id;
	rc = exec_command(portal, cmd, 0x8051);
	hdr = (struct dpaa2_cmd_header *) &cmd->header;
	*token = hdr->token;

	return (rc);
}

int
dpaa2_cmd_rc_close(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(portal, cmd, 0x8001));
}

int
dpaa2_cmd_rc_get_obj_count(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t *obj_count)
{
	int rc;

	if (!portal || !cmd || !obj_count)
		return (DPAA2_CMD_STAT_ERR);

	rc = exec_command(portal, cmd, 0x1591);
	*obj_count = (uint32_t)(cmd->params[0] >> 32);

	return (rc);
}

int
dpaa2_cmd_rc_get_obj(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t obj_idx, dpaa2_obj_t *obj)
{
	struct dpaa2_obj *pobj;
	int rc;

	if (!portal || !cmd || !obj)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = obj_idx;
	rc = exec_command(portal, cmd, 0x15A1);

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

	return (rc);
}

int
dpaa2_cmd_rc_get_obj_descriptor(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    uint32_t obj_id, const char *type, dpaa2_obj_t *obj)
{
	struct __packed get_obj_desc_arg {
		uint32_t	obj_id;
		uint32_t	_reserved1;
		uint8_t		type[16];
	} *args;
	struct dpaa2_obj *pobj;
	int rc;

	if (!portal || !cmd || !type || !obj)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct get_obj_desc_arg *) &cmd->params[0];
	args->obj_id = obj_id;
	memcpy(args->type, type, min(strlen(type) + 1, TYPE_LEN_MAX));

	rc = exec_command(portal, cmd, 0x1621);

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

	return (rc);
}

int
dpaa2_cmd_rc_get_attributes(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
    dpaa2_rc_attr_t *attr)
{
	struct dpaa2_rc_attr *pattr;
	int rc;

	if (!portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_ERR);

	rc = exec_command(portal, cmd, 0x0041);

	pattr = (struct dpaa2_rc_attr *) &cmd->params[0];
	attr->cont_id = pattr->cont_id;
	attr->portal_id = pattr->portal_id;
	attr->options = pattr->options;
	attr->icid = pattr->icid;

	return (rc);
}

/*
 * Data Path Network Interface (DPNI) commands.
 */

int
dpaa2_cmd_ni_open(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint32_t dpni_id,
    uint16_t *token)
{
	struct dpaa2_cmd_header *hdr;
	int rc;

	if (!portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpni_id;
	rc = exec_command(portal, cmd, 0x8011);
	hdr = (struct dpaa2_cmd_header *) &cmd->header;
	*token = hdr->token;

	return (rc);
}

int
dpaa2_cmd_ni_close(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(portal, cmd, 0x8001));
}

static int
exec_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint16_t cmdid)
{
	struct dpaa2_cmd_header *hdr;
	uint16_t flags;
	int error;

	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	/* Prepare command for the MC hardware. */
	hdr = (struct dpaa2_cmd_header *) &cmd->header;
	hdr->cmdid = cmdid;
	hdr->status = DPAA2_CMD_STAT_READY;

	LOCK_PORTAL(portal, flags);
	if (flags & DPAA2_PORTAL_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		UNLOCK_PORTAL(portal);
		return (DPAA2_CMD_STAT_INVALID_STATE);
	}

	/* Send command to MC and wait for the result. */
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
		return (1);

	return (0);
}
