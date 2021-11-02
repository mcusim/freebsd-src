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

#ifndef	_DPAA2_NI_H
#define	_DPAA2_NI_H

#include <sys/rman.h>
#include <sys/bus.h>
#include <sys/queue.h>

#include <net/ethernet.h>

#include "dpaa2_types.h"
#include "dpaa2_mcp.h"

/* Maximum number of resources per DPNI. */
#define DPAA2_NI_MAX_RESOURCES	9

/* Maximum number of channels to distribute Rx and Tx conf traffic to GPPs. */
#define DPAA2_NI_MAX_CHANNELS	16

/**
 * @brief
 */
typedef struct {
	device_t		 dpio_dev;
	device_t		 dpcon_dev;
	uint16_t		 chan_id;
} dpaa2_ni_channel_t;

/**
 * @brief Software context for the DPAA2 Network Interface driver.
 *
 * dev:		Device associated with this software context.
 * api_major:	Major version of the DPNI API.
 * api_minor:	Minor version of the DPNI API.
 * rx_bufsz:	Size of a buffer to receive frames.
 * tx_data_off: ...
 * attr:	Attributes of the DPNI object.
 * mac:		Details about DPMAC connected to this DPNI object (if exists).
 * link_state:	Link state of the network interface.
 */
struct dpaa2_ni_softc {
	device_t		 dev;
	struct resource 	*res[DPAA2_NI_MAX_RESOURCES];
	uint16_t		 api_major;
	uint16_t		 api_minor;
	uint16_t		 rx_bufsz;
	uint16_t		 tx_data_off;
	dpaa2_ni_attr_t		 attr;

	/* For network interface and miibus. */
	struct ifnet		*ifp;
	struct mtx		 lock;
	device_t		 miibus;
	struct mii_data		*mii;
	struct callout		 mii_callout;
	int			 media_status;

	/* Channels for ingress traffic (Rx, Tx confirmation). */
	uint8_t			 num_chan;
	dpaa2_ni_channel_t	*channel[DPAA2_NI_MAX_CHANNELS];

	struct {
		bus_dma_tag_t	 dtag;
		bus_dmamap_t	 dmap;
		bus_addr_t	 buf_busaddr;
		uint8_t		*buf;
	} qos_kcfg; /* QoS table key configuration. */

	struct {
		uint32_t	 dpmac_id;
		uint8_t		 addr[ETHER_ADDR_LEN];
	} mac; /* Info about connected DPMAC (if exists) */

	struct {
		uint32_t	 rate;
		uint64_t	 options;
		bool		 up;
	} link_state;
};

extern struct resource_spec dpaa2_ni_spec[];

#endif /* _DPAA2_NI_H */
