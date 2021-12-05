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
 * The DPAA2 MAC driver.
 *
 * For every DPAA2 MAC, there is an MC object named DPMAC, for MDIO and link
 * state updates. The DPMAC virtualizes the MDIO interface, so each PHY driver
 * may see a private interface (removing the need for synchronization in GPP on
 * the multiplexed MDIO hardware).
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/mutex.h>

#include <vm/vm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mc.h"
#include "dpaa2_ni.h"
#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"
#include "dpaa2_swp_if.h"
#include "dpaa2_cmd_if.h"

/* Forward declarations. */

static const char *etf_if_to_str(enum dpaa2_mac_eth_if);
static const char *link_type_to_str(enum dpaa2_mac_link_type);

/*
 * Device interface.
 */

static int
dpaa2_mac_probe(device_t dev)
{
	/* DPIO device will be added by a parent resource container itself. */
	device_set_desc(dev, "DPAA2 MAC");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_mac_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_rc_softc *rcsc;
	struct dpaa2_mac_softc *sc;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	dpaa2_cmd_t cmd;
	uint16_t rc_token, mac_token;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	pdev = device_get_parent(dev);
	rcsc = device_get_softc(pdev);
	rcinfo = device_get_ivars(pdev);
	dinfo = device_get_ivars(dev);
	memset(sc->addr, 0, ETHER_ADDR_LEN);

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Open resource container and DPMAC object. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: error=%d\n", error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_MAC_OPEN(dev, cmd, dinfo->id, &mac_token);
	if (error) {
		device_printf(dev, "Failed to open DPMAC: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}

	error = DPAA2_CMD_MAC_GET_ATTRIBUTES(dev, cmd, &sc->attr);
	if (error) {
		device_printf(dev, "Failed to get DPMAC attributes: id=%d, "
		    "error=%d\n", dinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_MAC_GET_ADDR(dev, cmd, sc->addr);
	if (error)
		device_printf(dev, "Failed to get physical address: error=%d\n",
		    error);
	/*
	 * TODO: Enable debug output via sysctl.
	 */
	if (bootverbose) {
		device_printf(dev, "ether %6D\n", sc->addr, ":");
		device_printf(dev, "max_rate=%d, eth_if=%s, link_type=%s\n",
		    sc->attr.max_rate, etf_if_to_str(sc->attr.eth_if),
		    link_type_to_str(sc->attr.link_type));
	}

	/* Close the DPMAC object and the resource container. */
	error = DPAA2_CMD_MAC_CLOSE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to close DPMAC: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(cmd, rc_token));
	if (error) {
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
		goto err_free_cmd;
	}

	return (0);

 err_free_cmd:
	dpaa2_mcp_free_command(cmd);
 err_exit:
	return (ENXIO);
}

static int
dpaa2_mac_detach(device_t dev)
{
	return (0);
}

/*
 * Internal functions.
 */

static const char *
etf_if_to_str(enum dpaa2_mac_eth_if eth_if)
{
	switch (eth_if) {
	case DPAA2_MAC_ETH_IF_MII:
		return ("MII");
	case DPAA2_MAC_ETH_IF_RMII:
		return ("RMII");
	case DPAA2_MAC_ETH_IF_SMII:
		return ("SMII");
	case DPAA2_MAC_ETH_IF_GMII:
		return ("GMII");
	case DPAA2_MAC_ETH_IF_RGMII:
		return ("RGMII");
	case DPAA2_MAC_ETH_IF_SGMII:
		return ("SGMII");
	case DPAA2_MAC_ETH_IF_QSGMII:
		return ("QSGMII");
	case DPAA2_MAC_ETH_IF_XAUI:
		return ("XAUI");
	case DPAA2_MAC_ETH_IF_XFI:
		return ("XFI");
	case DPAA2_MAC_ETH_IF_CAUI:
		return ("CAUI");
	case DPAA2_MAC_ETH_IF_1000BASEX:
		return ("1000BASE-X");
	case DPAA2_MAC_ETH_IF_USXGMII:
		return ("USXGMII");
	default:
		return ("unknown");
	}
}

static const char *
link_type_to_str(enum dpaa2_mac_link_type link_type)
{
	switch (link_type) {
	case DPAA2_MAC_LINK_TYPE_NONE:
		return ("NONE");
	case DPAA2_MAC_LINK_TYPE_FIXED:
		return ("FIXED");
	case DPAA2_MAC_LINK_TYPE_PHY:
		return ("PHY");
	case DPAA2_MAC_LINK_TYPE_BACKPLANE:
		return ("BACKPLANE");
	default:
		return ("unknown");
	}
}

static device_method_t dpaa2_mac_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mac_probe),
	DEVMETHOD(device_attach,	dpaa2_mac_attach),
	DEVMETHOD(device_detach,	dpaa2_mac_detach),

	DEVMETHOD_END
};

static driver_t dpaa2_mac_driver = {
	"dpaa2_mac",
	dpaa2_mac_methods,
	sizeof(struct dpaa2_mac_softc),
};

static devclass_t dpaa2_mac_devclass;

DRIVER_MODULE(dpaa2_mac, dpaa2_rc, dpaa2_mac_driver, dpaa2_mac_devclass, 0, 0);
