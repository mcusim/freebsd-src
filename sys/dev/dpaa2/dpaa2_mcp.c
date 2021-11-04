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
 * DPAA2 MC command interface helper routines.
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

MALLOC_DEFINE(M_DPAA2_MCP, "dpaa2_mcp_memory", "DPAA2 Management Complex Portal "
    "memory");

/* Forward declarations. */

static int	mcp_init_portal(dpaa2_mcp_t *portal, struct resource *res,
		    struct resource_map *map, const uint16_t flags,
		    const uint8_t atomic);

int
dpaa2_mcp_init_portal(dpaa2_mcp_t *mcp, struct resource *res,
    struct resource_map *map, const uint16_t flags)
{
	return (mcp_init_portal(mcp, res, map, flags, 0));
}

int
dpaa2_mcp_init_atomic(dpaa2_mcp_t *mcp, struct resource *res,
    struct resource_map *map, const uint16_t flags)
{
	return (mcp_init_portal(mcp, res, map, flags, 0xFF));
}

static int
mcp_init_portal(dpaa2_mcp_t *portal, struct resource *res,
    struct resource_map *map, const uint16_t flags, const uint8_t atomic)
{
	const int mflags = flags & DPAA2_PORTAL_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO) : (M_WAITOK | M_ZERO);
	dpaa2_mcp_t p;

	if (!portal || !res || !map)
		return (DPAA2_CMD_STAT_EINVAL);

	p = malloc(sizeof(struct dpaa2_mcp), M_DPAA2_MCP, mflags);
	if (!p)
		return (DPAA2_CMD_STAT_NO_MEMORY);

	if (atomic) {
		/*
		 * NOTE: Do not initialize cv for atomic portal: it's not
		 * possible to sleep on it in case of a spin mutex.
		 */
		mtx_init(&p->lock, "MC portal spin lock", NULL, MTX_SPIN);
	} else {
		mtx_init(&p->lock, "MC portal sleep lock", NULL, MTX_DEF);
		cv_init(&p->cv, "MC portal cv");
	}

	p->res = res;
	p->map = map;
	p->flags = flags;
	/* Reset DPRC API version to cache later. */
	p->rc_api_major = 0;
	p->rc_api_minor = 0;
	p->atomic = atomic;

	*portal = p;

	return (0);
}

void
dpaa2_mcp_free_portal(dpaa2_mcp_t portal)
{
	uint16_t flags;

	if (portal) {
		if (portal->atomic) {
			mtx_destroy(&portal->lock);
			free(portal, M_DPAA2_MCP);
		} else {
			/*
			 * Signal all threads sleeping on portal's cv that it's
			 * going to be destroyed.
			 */
			dpaa2_mcp_lock(portal, &flags);
			portal->flags |= DPAA2_PORTAL_DESTROYED;
			cv_signal(&portal->cv);
			dpaa2_mcp_unlock(portal);

			/* Let threads stop using this portal. */
			DELAY(DPAA2_PORTAL_TIMEOUT);

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
		return (DPAA2_CMD_STAT_EINVAL);

	c = malloc(sizeof(struct dpaa2_cmd), M_DPAA2_MCP, mflags);
	if (!c)
		return (DPAA2_CMD_STAT_NO_MEMORY);

	hdr = (struct dpaa2_cmd_header *) &c->header;
	hdr->srcid = 0;
	hdr->status = DPAA2_CMD_STAT_OK;
	hdr->token = 0;
	hdr->cmdid = 0;
	hdr->flags_hw = DPAA2_CMD_DEF;
	hdr->flags_sw = DPAA2_CMD_DEF;
	if (flags & DPAA2_CMD_HIGH_PRIO)
		hdr->flags_hw |= DPAA2_HW_FLAG_HIGH_PRIO;
	if (flags & DPAA2_CMD_INTR_DIS)
		hdr->flags_sw |= DPAA2_SW_FLAG_INTR_DIS;
	for (uint32_t i = 0; i < DPAA2_CMD_PARAMS_N; i++)
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

dpaa2_cmd_t
dpaa2_mcp_tk(dpaa2_cmd_t cmd, const uint16_t token)
{
	struct dpaa2_cmd_header *hdr;
	if (cmd) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		hdr->token = token;
	}
	return (cmd);
}

dpaa2_cmd_t
dpaa2_mcp_f(dpaa2_cmd_t cmd, const uint16_t flags)
{
	struct dpaa2_cmd_header *hdr;
	if (cmd) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		hdr->flags_hw = DPAA2_CMD_DEF;
		hdr->flags_sw = DPAA2_CMD_DEF;

		if (flags & DPAA2_CMD_HIGH_PRIO)
			hdr->flags_hw |= DPAA2_HW_FLAG_HIGH_PRIO;
		if (flags & DPAA2_CMD_INTR_DIS)
			hdr->flags_sw |= DPAA2_SW_FLAG_INTR_DIS;
	}
	return (cmd);
}

void
dpaa2_mcp_lock(dpaa2_mcp_t portal, uint16_t *flags)
{
	mtx_assert(&portal->lock, MA_NOTOWNED);

	if (portal->atomic) {
		mtx_lock_spin(&portal->lock);
		*flags = portal->flags;
	} else {
		mtx_lock(&portal->lock);
		while (portal->flags & DPAA2_PORTAL_LOCKED)
			cv_wait(&portal->cv, &portal->lock);
		*flags = portal->flags;
		portal->flags |= DPAA2_PORTAL_LOCKED;
		mtx_unlock(&portal->lock);
	}
}

void
dpaa2_mcp_unlock(dpaa2_mcp_t portal)
{
	if (portal->atomic) {
		mtx_unlock_spin(&portal->lock);
	} else {
		mtx_lock(&portal->lock);
		portal->flags &= ~DPAA2_PORTAL_LOCKED;
		cv_signal(&portal->cv);
		mtx_unlock(&portal->lock);
	}
}
