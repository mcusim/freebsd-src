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

#ifndef	_DPAA2_TYPES_H
#define	_DPAA2_TYPES_H

enum dpaa2_dev_type {
	DPAA2_DEV_MC = 7500,	/* Management Complex (firmware bus) */
	DPAA2_DEV_RC,		/* Resource Container (firmware bus) */
	DPAA2_DEV_IO,		/* I/O object (to work with QBMan portal) */
	DPAA2_DEV_NI,		/* Network Interface */
	DPAA2_DEV_MCP,		/* MC portal (to configure MC portal) */
	DPAA2_DEV_BP,		/* Buffer Pool */
	DPAA2_DEV_CON,		/* Concentrator */
	DPAA2_DEV_MAC,		/* MAC object */

	DPAA2_DEV_NOTYPE	/* Shouldn't be assigned to any DPAA2 device. */
};

/* Convert DPAA2 type to/from string. */
const char		*dpaa2_ttos(enum dpaa2_dev_type type);
enum dpaa2_dev_type	 dpaa2_stot(const char *str);

#endif /* _DPAA2_TYPES_H */
