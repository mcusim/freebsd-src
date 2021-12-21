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

#ifndef	_DPAA2_IO_H
#define	_DPAA2_IO_H

#include <sys/rman.h>
#include <sys/bus.h>
#include <sys/queue.h>

#include "dpaa2_types.h"
#include "dpaa2_mcp.h"

/* Maximum number of MSIs supported by the DPIO objects. */
#define DPAA2_IO_MSI_COUNT	1

enum dpaa2_io_chan_mode {
	DPAA2_IO_NO_CHANNEL,
	DPAA2_IO_LOCAL_CHANNEL
};

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
 * @brief Context used by DPIO to configure data availability notifications
 * (CDAN) on a particular WQ channel.
 */
typedef struct dpaa2_io_notif_ctx {
	void (*cb)(struct dpaa2_io_notif_ctx *ctx);

	device_t	io_dev;
	uint64_t	qman_ctx;
	uint16_t	fq_chan_id;
	bool		cdan_en;
} dpaa2_io_notif_ctx_t;

/**
 * @brief Software context for the DPAA2 I/O driver.
 */
struct dpaa2_io_softc {
	device_t		 dev;
	dpaa2_swp_desc_t	 swp_desc;
	dpaa2_swp_t		 swp;
	dpaa2_io_attr_t		 attr;

	/* Help to send commands to MC. */
	dpaa2_cmd_t		 cmd;
	uint16_t		 rc_token;
	uint16_t		 io_token;

	struct resource 	*res[3];
	struct resource_map	 map[3];

	int			 irq_rid[DPAA2_IO_MSI_COUNT];
	struct resource		*irq_resource;
	void			*intr; /* interrupt handle */
};

#endif /* _DPAA2_IO_H */
