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

#ifndef	_DPAA2_MAC_H
#define	_DPAA2_MAC_H

#include <sys/rman.h>
#include <sys/bus.h>
#include <sys/queue.h>

#include <net/ethernet.h>

#include "dpaa2_types.h"

/* DPMAC link configuration options. */
#define DPAA2_MAC_LINK_OPT_AUTONEG	((uint64_t) 0x01u)
#define DPAA2_MAC_LINK_OPT_HALF_DUPLEX	((uint64_t) 0x02u)
#define DPAA2_MAC_LINK_OPT_PAUSE	((uint64_t) 0x04u)
#define DPAA2_MAC_LINK_OPT_ASYM_PAUSE	((uint64_t) 0x08u)

enum dpaa2_mac_eth_if {
	DPAA2_MAC_ETH_IF_MII,
	DPAA2_MAC_ETH_IF_RMII,
	DPAA2_MAC_ETH_IF_SMII,
	DPAA2_MAC_ETH_IF_GMII,
	DPAA2_MAC_ETH_IF_RGMII,
	DPAA2_MAC_ETH_IF_SGMII,
	DPAA2_MAC_ETH_IF_QSGMII,
	DPAA2_MAC_ETH_IF_XAUI,
	DPAA2_MAC_ETH_IF_XFI,
	DPAA2_MAC_ETH_IF_CAUI,
	DPAA2_MAC_ETH_IF_1000BASEX,
	DPAA2_MAC_ETH_IF_USXGMII
};

enum dpaa2_mac_link_type {
	DPAA2_MAC_LINK_TYPE_NONE,
	DPAA2_MAC_LINK_TYPE_FIXED,
	DPAA2_MAC_LINK_TYPE_PHY,
	DPAA2_MAC_LINK_TYPE_BACKPLANE
};

/**
 * @brief Attributes of the DPMAC object.
 *
 * id:		DPMAC object ID.
 * max_rate:	Maximum supported rate (in Mbps).
 * eth_if:	Type of the Ethernet interface.
 * link_type:	Type of the link.
 */
typedef struct {
	uint32_t		 id;
	uint32_t		 max_rate;
	enum dpaa2_mac_eth_if	 eth_if;
	enum dpaa2_mac_link_type link_type;
} dpaa2_mac_attr_t;

/**
 * @brief Link state of the DPMAC object.
 */
typedef struct {
	uint64_t		 options;
	uint64_t		 supported;
	uint64_t		 advert;
	uint32_t		 rate;
	bool			 up;
	bool			 state_valid;
} dpaa2_mac_link_state_t;

/**
 * @brief Software context for the DPAA2 MAC driver.
 *
 * dev:		Device associated with this software context.
 * addr:	Physical address assigned to the DPMAC object.
 * attr:	Attributes of the DPMAC object.
 */
struct dpaa2_mac_softc {
	device_t		 dev;
	uint8_t			 addr[ETHER_ADDR_LEN];
	dpaa2_mac_attr_t	 attr;
};

#endif /* _DPAA2_MAC_H */
