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

#ifndef	_DPAA2_MCVAR_H
#define	_DPAA2_MCVAR_H

#define DPAA2_MCP_MEM_WIDTH	0x40 /* Expected minimal size of the portal. */

#define	BIT(x)			(1 << (x))

#define MC_REG_GCR1		0x00u
#define GCR1_P1_STOP		BIT(31)

#define MC_REG_GSR		0x08u
#define MC_REG_FAPR		0x28u

/*
 * Helper object to send commands to MC portal.
 *
 * @dev: DPMCP device associated with this helper object (optional).
 * @portal: Unmapped MC portal resource.
 * @mportal: Mapped MC portal resource.
 * @mutex: Lock to send MC portal commands and wait for the result (in polling
 *         mode).
 *
 * NOTE: The same object can be shared between MC, DPRC and DPMCP.
 */
typedef struct dpaa2_mcp {
	device_t		 dev;
	struct resource		*portal;
	struct resource_map	*mportal;
	struct mtx		 mutex;
} dpaa2_mcp_t;

/*
 * Software contexts.
 */

DECLARE_CLASS(dpaa2_mc_driver);

/*
 * Software context for the DPAA2 Management Complex (MC) driver.
 *
 * @dev: Device associated with this software context.
 * @node: OFW node associated with this device (optional).
 * @res: Unmapped MC command portal and MC control registers.
 * @map: Mapped MC command portal and MC control registers.
 * @mcp: Helper object to send commands to the root MC portal.
 */
struct dpaa2_mc_softc {
	struct resource_map	 map[2];
	struct resource 	*res[2];
	device_t		 dev;
	device_t		 rcdev;
	dpaa2_mcp_t		*mcp;
};

struct dpaa2_rc_softc {
	device_t	dev;
	int		unit;
};

int dpaa2_mc_attach(device_t dev);
int dpaa2_mc_detach(device_t dev);

#endif /* _DPAA2_MCVAR_H */
