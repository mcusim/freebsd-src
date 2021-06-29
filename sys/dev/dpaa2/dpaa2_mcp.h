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

#define DPAA2_CMD_PARAMS_N	7

/*
 * Helper object to send commands to the MC portal.
 *
 * dev: Owner device on behalf of which memory was allocated for this object.
 * mcpdev: DPMCP device associated with this helper object (optional).
 * res: Unmapped portal I/O memory resource.
 * map: Mapped portal I/O memory resource.
 * lock: Lock to send a command to the portal and wait for the result.
 *
 * NOTE: The same object can be shared between MC, DPRC and DPMCP.
 */
typedef struct dpaa2_mcp {
	device_t		 dev;
	device_t		 mcpdev;
	struct resource		*res;
	struct resource_map	*map;
	struct mtx		 lock;
} dpaa2_mcp_t;

typedef struct dpaa2_cmd {
	uint64_t	header;
	uint64_t	params[DPAA2_CMD_PARAMS_N];
} dpaa2_cmd_t;

typedef struct dpaa2_cmd_header {
	uint8_t		srcid;
	uint8_t		flags_hw;
	uint8_t		status;
	uint8_t		flags_sw;
	uint16_t	token;
	uint16_t	cmdid;
} dpaa2_cmd_header_t;

typedef enum dpaa2_cmd_status {
	DPAA2_CMD_OK			= 0x0,	/* Set by MC on success */
	DPAA2_CMD_READY			= 0x1,	/* Ready to be processed */
	DPAA2_CMD_AUTH_ERR		= 0x3,	/* Illegal object-portal-icid */
	DPAA2_CMD_NO_PRIVILEGE		= 0x4,	/* No privilege */
	DPAA2_CMD_DMA_ERR		= 0x5,	/* DMA or I/O error */
	DPAA2_CMD_CONFIG_ERR		= 0x6,	/* Invalid/conflicting params */
	DPAA2_CMD_TIMEOUT		= 0x7,	/* Command timed out */
	DPAA2_CMD_NO_RESOURCE		= 0x8,	/* No DPAA2 resources */
	DPAA2_CMD_NO_MEMORY		= 0x9,	/* No memory available */
	DPAA2_CMD_BUSY			= 0xA,	/* Device is busy */
	DPAA2_CMD_UNSUPPORTED_OP	= 0xB,	/* Unsupported operation */
	DPAA2_CMD_INVALID_STATE		= 0xC	/* Invalid state */
} dpaa2_cmd_status_t;

int dpaa2_mcp_init_command(dpaa2_cmd_t *cmd);

#endif /* _DPAA2_MCP_H */
