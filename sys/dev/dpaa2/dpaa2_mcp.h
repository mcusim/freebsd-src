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
#define DPAA2_CMD_STAT_ERR		0xFF	/* General error */

/*
 * Opaque pointers.
 */
typedef struct dpaa2_mcp *dpaa2_mcp_t;
typedef struct dpaa2_cmd *dpaa2_cmd_t;

/*
 * Management routines.
 */
int	dpaa2_mcp_init_portal(dpaa2_mcp_t *portal, struct resource *res,
		struct resource_map *map, const uint16_t flags);
void	dpaa2_mcp_free_portal(dpaa2_mcp_t portal);
int	dpaa2_mcp_init_command(dpaa2_cmd_t *cmd, const uint16_t flags);
void	dpaa2_mcp_free_command(dpaa2_cmd_t cmd);
void	dpaa2_mcp_set_token(dpaa2_cmd_t cmd, const uint16_t token);
void	dpaa2_mcp_set_flags(dpaa2_cmd_t cmd, const uint16_t flags);

/*
 * Data Path Management (DPMNG) commands.
 */
int	dpaa2_cmd_mng_get_version(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
		uint32_t *major, uint32_t *minor, uint32_t *rev);
int	dpaa2_cmd_mng_get_soc_version(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
		uint32_t *pvr, uint32_t *svr);
int	dpaa2_cmd_mng_get_container_id(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
		uint32_t *cont_id);

/*
 * Data Path Resource Containter (DPRC) commands.
 */
int	dpaa2_cmd_rc_open(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint32_t cont_id,
		uint16_t *token);
int	dpaa2_cmd_rc_close(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);

/*
 * Data Path Network Interface (DPNI) commands.
 */
int	dpaa2_cmd_ni_open(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint32_t dpni_id,
		uint16_t *token);
int	dpaa2_cmd_ni_close(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);

#endif /* _DPAA2_MCP_H */
