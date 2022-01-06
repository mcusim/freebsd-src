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

MALLOC_DEFINE(M_DPAA2_MCP, "dpaa2_mcp", "DPAA2 Management Complex Portal");

int
dpaa2_mcp_init_portal(struct dpaa2_mcp **mcp, struct resource *res,
    struct resource_map *map, uint16_t flags, bool atomic)
{
	const int mflags = flags & DPAA2_PORTAL_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO) : (M_WAITOK | M_ZERO);
	struct dpaa2_mcp *p;

	if (!mcp || !res || !map)
		return (DPAA2_CMD_STAT_EINVAL);

	p = malloc(sizeof(struct dpaa2_mcp), M_DPAA2_MCP, mflags);
	if (p == NULL)
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
	p->rc_api_major = 0; /* DPRC API version to be cached later. */
	p->rc_api_minor = 0;
	p->atomic = atomic;

	*mcp = p;

	return (0);
}

void
dpaa2_mcp_free_portal(struct dpaa2_mcp *mcp)
{
	uint16_t flags;

	if (mcp != NULL) {
		if (mcp->atomic) {
			dpaa2_mcp_lock(mcp, &flags);
			mcp->flags |= DPAA2_PORTAL_DESTROYED;
			dpaa2_mcp_unlock(mcp);

			/* Let threads stop using this portal. */
			DELAY(DPAA2_PORTAL_TIMEOUT);

			mtx_destroy(&mcp->lock);
		} else {
			/*
			 * Signal all threads sleeping on portal's cv that it's
			 * going to be destroyed.
			 */
			dpaa2_mcp_lock(mcp, &flags);
			mcp->flags |= DPAA2_PORTAL_DESTROYED;
			cv_signal(&mcp->cv);
			dpaa2_mcp_unlock(mcp);

			/* Let threads stop using this portal. */
			DELAY(DPAA2_PORTAL_TIMEOUT);

			mtx_destroy(&mcp->lock);
			cv_destroy(&mcp->cv);
		}
		free(mcp, M_DPAA2_MCP);
	}
}

int
dpaa2_mcp_init_command(struct dpaa2_cmd **cmd, uint16_t flags)
{
	const int mflags = flags & DPAA2_CMD_NOWAIT_ALLOC
	    ? (M_NOWAIT | M_ZERO) : (M_WAITOK | M_ZERO);
	struct dpaa2_cmd *c;
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
dpaa2_mcp_free_command(struct dpaa2_cmd *cmd)
{
	if (cmd != NULL)
		free(cmd, M_DPAA2_MCP);
}

struct dpaa2_cmd *
dpaa2_mcp_tk(struct dpaa2_cmd *cmd, uint16_t token)
{
	struct dpaa2_cmd_header *hdr;
	if (cmd != NULL) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		hdr->token = token;
	}
	return (cmd);
}

struct dpaa2_cmd *
dpaa2_mcp_f(struct dpaa2_cmd *cmd, uint16_t flags)
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
dpaa2_mcp_lock(struct dpaa2_mcp *mcp, uint16_t *flags)
{
	if (mcp != NULL && flags != NULL) {
		mtx_assert(&mcp->lock, MA_NOTOWNED);

		if (mcp->atomic) {
			mtx_lock_spin(&mcp->lock);
			*flags = mcp->flags;
			mcp->flags |= DPAA2_PORTAL_LOCKED;
		} else {
			mtx_lock(&mcp->lock);
			while (mcp->flags & DPAA2_PORTAL_LOCKED)
				cv_wait(&mcp->cv, &mcp->lock);
			*flags = mcp->flags;
			mcp->flags |= DPAA2_PORTAL_LOCKED;
			mtx_unlock(&mcp->lock);
		}
	}
}

void
dpaa2_mcp_unlock(struct dpaa2_mcp *mcp)
{
	if (mcp != NULL) {
		if (mcp->atomic) {
			mcp->flags &= ~DPAA2_PORTAL_LOCKED;
			mtx_unlock_spin(&mcp->lock);
		} else {
			mtx_lock(&mcp->lock);
			mcp->flags &= ~DPAA2_PORTAL_LOCKED;
			cv_signal(&mcp->cv);
			mtx_unlock(&mcp->lock);
		}
	}
}
