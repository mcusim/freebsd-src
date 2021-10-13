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
 * The DPAA2 Network Interface (DPNI) driver.
 *
 * The DPNI object is a network interface that is configurable to support a wide
 * range of features from a very basic Ethernet interface up to a
 * high-functioning network interface. The DPNI supports features that are
 * expected by standard network stacks, from basic features to offloads.
 *
 * DPNIs work with Ethernet traffic, starting with the L2 header. Additional
 * functions are provided for standard network protocols (L2, L3, L4, etc.).
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/mbuf.h>

#include <vm/vm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/ethernet.h>
#include <net/bpf.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <dev/pci/pcivar.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "miibus_if.h"

#include "dpaa2_mc.h"
#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"
#include "dpaa2_swp_if.h"
#include "dpaa2_cmd_if.h"

#define WRIOP_VERSION(x, y, z)	((x) << 10 | (y) << 5 | (z) << 0)
#define ALIGN_DOWN(x, a)	((x) & ~((1 << (a)) - 1))

#define DPNI_VER_MAJOR		7U
#define DPNI_VER_MINOR		0U
#define DPNI_ENQ_FQID_VER_MAJOR	7U
#define DPNI_ENQ_FQID_VER_MINOR	9U

/*
 * Due to a limitation in WRIOP 1.0.0, the RX buffer data must be aligned
 * to 256B. For newer revisions, the requirement is only for 64B alignment.
 */
#define ETH_RX_BUF_ALIGN_REV1	256
#define ETH_RX_BUF_ALIGN	64

/*
 * We are accommodating a skb backpointer and some S/G info in the frame's
 * software annotation. The hardware options are either 0 or 64, so we choose
 * the latter.
 */
#define ETH_SWA_SIZE		64

/*
 * Hardware annotation area in RX/TX buffers.
 */
#define ETH_RX_HWA_SIZE		64

#define ETH_RX_BUF_RAW_SIZE	PAGE_SIZE
#define ETH_RX_BUF_TAILROOM	ALIGN(sizeof(struct mbuf))//, CACHE_LINE_SIZE)
#define ETH_RX_BUF_SIZE		(ETH_RX_BUF_RAW_SIZE - ETH_RX_BUF_TAILROOM)

/* DPNI buffer layout modification options */

/* Select to modify the time-stamp setting */
#define DPNI_BUF_LAYOUT_OPT_TIMESTAMP		0x00000001
/* Select to modify the parser-result setting; not applicable for Tx */
#define DPNI_BUF_LAYOUT_OPT_PARSER_RESULT	0x00000002
/* Select to modify the frame-status setting */
#define DPNI_BUF_LAYOUT_OPT_FRAME_STATUS	0x00000004
/* Select to modify the private-data-size setting */
#define DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE	0x00000008
/* Select to modify the data-alignment setting */
#define DPNI_BUF_LAYOUT_OPT_DATA_ALIGN		0x00000010
/* Select to modify the data-head-room setting */
#define DPNI_BUF_LAYOUT_OPT_DATA_HEAD_ROOM	0x00000020
/* Select to modify the data-tail-room setting */
#define DPNI_BUF_LAYOUT_OPT_DATA_TAIL_ROOM	0x00000040

static struct resource_spec dpaa2_ni_spec[] = {
	{ DPAA2_DEV_IO,  0, RF_ACTIVE | RF_UNMAPPED },
	{ DPAA2_DEV_BP,  1, RF_ACTIVE | RF_UNMAPPED },
	{ DPAA2_DEV_CON, 2, RF_ACTIVE | RF_UNMAPPED },

	RESOURCE_SPEC_END
};

/* Forward declarations. */
static int	set_buf_layout(device_t dev, dpaa2_cmd_t cmd);
static int	cmp_api_version(struct dpaa2_ni_softc *sc, const uint16_t major,
		    uint16_t minor);

/*
 * Device interface.
 */

static int
dpaa2_ni_probe(device_t dev)
{
	/* DPNI device will be added by a parent resource container itself. */
	device_set_desc(dev, "DPAA2 Network Interface");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_ni_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_ni_softc *sc;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	dpaa2_cmd_t cmd;
	dpaa2_ep_desc_t ep1_desc, ep2_desc;
	uint32_t link_stat;
	uint16_t rc_token, ni_token;
	uint8_t mac[6];
	int error;

 	sc = device_get_softc(dev);
	sc->dev = dev;
	pdev = device_get_parent(dev);
	rcinfo = device_get_ivars(pdev);
	dinfo = device_get_ivars(dev);

	error = bus_alloc_resources(sc->dev, dpaa2_ni_spec, sc->res);
	if (error) {
		device_printf(dev, "Failed to allocate resources: error=%d\n",
		    error);
		return (ENXIO);
	}

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Open resource container and DPNI object. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: id=%d, error=%d\n",
		    rcinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_NI_OPEN(dev, cmd, dinfo->id, &ni_token);
	if (error) {
		device_printf(dev, "Failed to open DPNI: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}

	/* Check if we can work with this DPNI object. */
	error = DPAA2_CMD_NI_GET_API_VERSION(dev, cmd, &sc->api_major,
	    &sc->api_minor);
	if (error) {
		device_printf(dev, "Failed to get DPNI API version: error=%d\n",
		    error);
		goto err_free_cmd;
	}
	if (cmp_api_version(sc, DPNI_VER_MAJOR, DPNI_VER_MINOR) < 0) {
		device_printf(dev, "DPNI API version %u.%u not supported, "
		    "need >= %u.%u\n", sc->api_major, sc->api_minor,
		    DPNI_VER_MAJOR, DPNI_VER_MINOR);
		goto err_free_cmd;
	}

	/* Reset the DPNI object. */
	error = DPAA2_CMD_NI_RESET(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to reset DPNI: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}

	/* Obtain attributes of the DPNI object. */
	error = DPAA2_CMD_NI_GET_ATTRIBUTES(dev, cmd, &sc->attr);
	if (error) {
		device_printf(dev, "Failed to obtain DPNI attributes: id=%d, "
		    "error=%d\n", dinfo->id, error);
		goto err_free_cmd;
	}

	/* Configure buffer layouts of the DPNI queues. */
	error = set_buf_layout(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to configure buffer layout: "
		    "error=%d\n", error);
		goto err_free_cmd;
	}

	ep1_desc.obj_id = dinfo->id;
	ep1_desc.if_id = 0; /* DPNI has an only endpoint */
	ep1_desc.type = dinfo->dtype;

	dpaa2_mcp_set_token(cmd, rc_token);
	error = DPAA2_CMD_RC_GET_CONN(dev, cmd, &ep1_desc, &ep2_desc,
	    &link_stat);
	if (error)
		device_printf(dev, "Failed to obtain an object DPNI is "
		    "connected to: error=%d\n", error);
	else {
		device_printf(dev, "Connected to: %s (id=%d)\n",
		    dpaa2_ttos(ep2_desc.type), ep2_desc.obj_id);

		if (ep2_desc.type == DPAA2_DEV_MAC) {
			/*
			 * This is the simplest case when DPNI is connected to
			 * DPMAC directly. Let's obtain physical address then.
			 */
			error = DPAA2_CMD_NI_GET_PORT_MAC_ADDR(dev, cmd, mac);
			if (error)
				device_printf(dev, "Failed to obtain a MAC "
				    "address of the connected DPMAC: error=%d\n",
				    error);
			else {
				device_printf(dev, "ether %.2x", mac[0]);
				for (int i = 1; i < 6; i++)
					printf(":%.2x", mac[i]);
				printf("\n");
			}
		}
	}

	/* Close the DPNI object and the resource container. */
	dpaa2_mcp_set_token(cmd, ni_token);
	error = DPAA2_CMD_NI_CLOSE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to close DPNI: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}
	dpaa2_mcp_set_token(cmd, rc_token);
	error = DPAA2_CMD_RC_CLOSE(dev, cmd);
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
dpaa2_ni_detach(device_t dev)
{
	return (0);
}

/*
 * MII interface.
 */

static int
dpaa2_ni_miibus_readreg(device_t dev, int phy, int reg)
{
	device_t pdev;
	struct dpaa2_ni_softc *sc;
	struct dpaa2_devinfo *rcinfo;
	dpaa2_cmd_t cmd;
	uint16_t rc_token, mac_token;
	uint16_t val = 0;
	int error;

	sc = device_get_softc(dev);
	pdev = device_get_parent(dev);
	rcinfo = device_get_ivars(pdev);

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		return (0);
	}

	/* Open resource container and DPMAC object. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: id=%d, error=%d\n",
		    rcinfo->id, error);
		goto free_cmd;
	}
	error = DPAA2_CMD_MAC_OPEN(dev, cmd, sc->mac.dpmac_id, &mac_token);
	if (error) {
		device_printf(dev, "Failed to open DPMAC: id=%d, error=%d\n",
		    sc->mac.dpmac_id, error);
		goto close_rc;
	}

	/* Read PHY register. */
	error = DPAA2_CMD_MAC_MDIO_READ(dev, cmd, phy, reg, &val);
	if (error) {
		device_printf(dev, "Failed to read PHY register: dpmac_id=%d, "
		    "phy=0x%x, reg=0x%x, error=%d\n", sc->mac.dpmac_id,
		    phy, reg, error);
		val = 0;
	}

	error = DPAA2_CMD_MAC_CLOSE(dev, cmd);
	if (error)
		device_printf(dev, "Failed to close DPMAC: id=%d, error=%d\n",
		    sc->mac.dpmac_id, error);
 close_rc:
	dpaa2_mcp_set_token(cmd, rc_token);
	error = DPAA2_CMD_RC_CLOSE(dev, cmd);
	if (error)
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
 free_cmd:
	dpaa2_mcp_free_command(cmd);
	return (val);
}

static int
dpaa2_ni_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	device_t pdev;
	struct dpaa2_ni_softc *sc;
	struct dpaa2_devinfo *rcinfo;
	dpaa2_cmd_t cmd;
	uint16_t rc_token, mac_token;
	int error;

	sc = device_get_softc(dev);
	pdev = device_get_parent(dev);
	rcinfo = device_get_ivars(pdev);

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		return (0);
	}

	/* Open resource container and DPMAC object. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: id=%d, error=%d\n",
		    rcinfo->id, error);
		goto free_cmd;
	}
	error = DPAA2_CMD_MAC_OPEN(dev, cmd, sc->mac.dpmac_id, &mac_token);
	if (error) {
		device_printf(dev, "Failed to open DPMAC: id=%d, error=%d\n",
		    sc->mac.dpmac_id, error);
		goto close_rc;
	}

	/* Write PHY register. */
	error = DPAA2_CMD_MAC_MDIO_WRITE(dev, cmd, phy, reg, val);
	if (error)
		device_printf(dev, "Failed to write PHY register: dpmac_id=%d, "
		    "phy=0x%x, reg=0x%x, error=%d\n", sc->mac.dpmac_id,
		    phy, reg, error);

	error = DPAA2_CMD_MAC_CLOSE(dev, cmd);
	if (error)
		device_printf(dev, "Failed to close DPMAC: id=%d, error=%d\n",
		    sc->mac.dpmac_id, error);
 close_rc:
	dpaa2_mcp_set_token(cmd, rc_token);
	error = DPAA2_CMD_RC_CLOSE(dev, cmd);
	if (error)
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
 free_cmd:
	dpaa2_mcp_free_command(cmd);
	return (0);
}

static void
dpaa2_ni_miibus_statchg(device_t dev)
{
	/* TBD */
}

/*
 * Internal functions.
 */

/**
 * @internal
 * @brief Configure buffer layouts of the different DPNI queues.
 */
static int
set_buf_layout(device_t dev, dpaa2_cmd_t cmd)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	dpaa2_ni_buf_layout_t buf_layout = {0};
	uint16_t rx_buf_align;
	int error;

	/*
	 * We need to check for WRIOP version 1.0.0, but depending on the MC
	 * version, this number is not always provided correctly on rev1.
	 * We need to check for both alternatives in this situation.
	 */
	if (sc->attr.wriop_ver == WRIOP_VERSION(0, 0, 0) ||
	    sc->attr.wriop_ver == WRIOP_VERSION(1, 0, 0))
		rx_buf_align = ETH_RX_BUF_ALIGN_REV1;
	else
		rx_buf_align = ETH_RX_BUF_ALIGN;

	/*
	 * We need to ensure that the buffer size seen by WRIOP is a multiple
	 * of 64 or 256 bytes depending on the WRIOP version.
	 */
	sc->rx_bufsz = ALIGN_DOWN(ETH_RX_BUF_SIZE, rx_buf_align);

	/* TX buffer layout */
	buf_layout.queue_type = DPAA2_NI_QUEUE_TX;
	buf_layout.pd_size = ETH_SWA_SIZE;
	buf_layout.pass_timestamp = true;
	buf_layout.pass_frame_status = true;
	buf_layout.options =
	    DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE |
	    DPNI_BUF_LAYOUT_OPT_TIMESTAMP |
	    DPNI_BUF_LAYOUT_OPT_FRAME_STATUS;
	error = DPAA2_CMD_NI_SET_BUF_LAYOUT(dev, cmd, &buf_layout);
	if (error) {
		device_printf(dev, "Failed to set TX buffer layout\n");
		return (error);
	}

	/* TX-confirmation buffer layout */
	buf_layout.queue_type = DPAA2_NI_QUEUE_TX_CONF;
	buf_layout.options =
	    DPNI_BUF_LAYOUT_OPT_TIMESTAMP |
	    DPNI_BUF_LAYOUT_OPT_FRAME_STATUS;
	error = DPAA2_CMD_NI_SET_BUF_LAYOUT(dev, cmd, &buf_layout);
	if (error) {
		device_printf(dev, "Failed to set TX_CONF buffer layout\n");
		return (error);
	}

	/*
	 * Now that we've set our TX buffer layout, retrieve the minimum
	 * required TX data offset.
	 */
	error = DPAA2_CMD_NI_GET_TX_DATA_OFF(dev, cmd, &sc->tx_data_off);
	if (error) {
		device_printf(dev, "Failed to obtain TX data offset\n");
		return (error);
	}

	if (bootverbose)
		device_printf(dev, "TX data offset=%d\n", sc->tx_data_off);
	if ((sc->tx_data_off % 64) != 0)
		device_printf(dev, "TX data offset (%d) not a multiple of 64B\n",
		    sc->tx_data_off);

	/* RX buffer */
	/*
	 * Extra headroom space requested to hardware, in order to make sure
	 * there's no realloc'ing in forwarding scenarios.
	 */
	buf_layout.queue_type = DPAA2_NI_QUEUE_RX;
	buf_layout.head_size = sc->tx_data_off - ETH_RX_HWA_SIZE;
	buf_layout.fd_align = rx_buf_align;
	buf_layout.pass_frame_status = true;
	buf_layout.pass_parser_result = true;
	buf_layout.pass_timestamp = true;
	buf_layout.pd_size = 0;
	buf_layout.options =
	    DPNI_BUF_LAYOUT_OPT_DATA_HEAD_ROOM |
	    DPNI_BUF_LAYOUT_OPT_DATA_ALIGN |
	    DPNI_BUF_LAYOUT_OPT_FRAME_STATUS |
	    DPNI_BUF_LAYOUT_OPT_PARSER_RESULT |
	    DPNI_BUF_LAYOUT_OPT_TIMESTAMP;
	error = DPAA2_CMD_NI_SET_BUF_LAYOUT(dev, cmd, &buf_layout);
	if (error) {
		device_printf(dev, "Failed to set RX buffer layout\n");
		return (error);
	}

	return (0);
}

static int
cmp_api_version(struct dpaa2_ni_softc *sc, const uint16_t major, uint16_t minor)
{
	if (sc->api_major == major)
		return sc->api_minor - minor;
	return sc->api_major - major;
}

static device_method_t dpaa2_ni_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_ni_probe),
	DEVMETHOD(device_attach,	dpaa2_ni_attach),
	DEVMETHOD(device_detach,	dpaa2_ni_detach),

	/* MII bus interface */
	DEVMETHOD(miibus_readreg,	dpaa2_ni_miibus_readreg),
	DEVMETHOD(miibus_writereg,	dpaa2_ni_miibus_writereg),
	DEVMETHOD(miibus_statchg,	dpaa2_ni_miibus_statchg),

	DEVMETHOD_END
};

static driver_t dpaa2_ni_driver = {
	"dpaa2_ni",
	dpaa2_ni_methods,
	sizeof(struct dpaa2_ni_softc),
};

static devclass_t dpaa2_ni_devclass;

DRIVER_MODULE(dpaa2_ni, dpaa2_rc, dpaa2_ni_driver, dpaa2_ni_devclass, 0, 0);
DRIVER_MODULE(miibus, dpaa2_ni, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(dpaa2_ni, miibus, 1, 1, 1);
