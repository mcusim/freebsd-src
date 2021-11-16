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
#include <sys/sockio.h>
#include <sys/sysctl.h>
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
#include "dpaa2_ni.h"
#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"
#include "dpaa2_swp_if.h"
#include "dpaa2_cmd_if.h"

#define WRIOP_VERSION(x, y, z)	((x) << 10 | (y) << 5 | (z) << 0)
#define ALIGN_DOWN(x, a)	((x) & ~((1 << (a)) - 1))

#define DPNI_LOCK(sc) do {			\
	mtx_assert(&(sc)->lock, MA_NOTOWNED);	\
	mtx_lock(&(sc)->lock);			\
} while (0)
#define	DPNI_UNLOCK(sc)		mtx_unlock(&(sc)->lock)

/*
 * Minimally supported version of the DPNI API.
 */
#define DPNI_VER_MAJOR		7U
#define DPNI_VER_MINOR		0U

#define DPNI_ENQ_FQID_VER_MAJOR	7U
#define DPNI_ENQ_FQID_VER_MINOR	9U

/*
 * Due to a limitation in WRIOP 1.0.0, the RX buffer data must be aligned
 * to 256 bytes. For newer revisions, the requirement is only for 64B alignment.
 */
#define ETH_RX_BUF_ALIGN_REV1	256
#define ETH_RX_BUF_ALIGN	64

/*
 * Frame's software annotation. The hardware options are either 0 or 64.
 */
#define ETH_SWA_SIZE		64

/*
 * Hardware annotation area in RX/TX buffers.
 */
#define ETH_RX_HWA_SIZE		64

/*
 * Rx buffer configuration.
 */
#define ETH_RX_BUF_RAW_SIZE	PAGE_SIZE
#define ETH_RX_BUF_TAILROOM	ALIGN(sizeof(struct mbuf))//, CACHE_LINE_SIZE)
#define ETH_RX_BUF_SIZE		(ETH_RX_BUF_RAW_SIZE - ETH_RX_BUF_TAILROOM)

/*
 * Size of a buffer to keep a QoS table key configuration.
 */
#define ETH_QOS_KCFG_BUF_SIZE	256

/*
 * DPNI buffer layout options.
 */
#define DPNI_BUF_LAYOUT_OPT_TIMESTAMP		0x00000001
#define DPNI_BUF_LAYOUT_OPT_PARSER_RESULT	0x00000002
#define DPNI_BUF_LAYOUT_OPT_FRAME_STATUS	0x00000004
#define DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE	0x00000008
#define DPNI_BUF_LAYOUT_OPT_DATA_ALIGN		0x00000010
#define DPNI_BUF_LAYOUT_OPT_DATA_HEAD_ROOM	0x00000020
#define DPNI_BUF_LAYOUT_OPT_DATA_TAIL_ROOM	0x00000040

/*
 * Enables TCAM for Flow Steering and QoS look-ups.
 */
#define DPNI_OPT_HAS_KEY_MASKING		0x000010

MALLOC_DEFINE(M_DPAA2_NI, "dpaa2_ni", "DPAA2 Network Interface");

/*
 * Macros to calculate DPAA2 resource IDs.
 */
/* DPIO resources */
#define IO_RID_OFF		(0u)
#define IO_RID(rid)		((rid) + IO_RID_OFF)
#define IO_RES_NUM		(4u)
/* DPBP resources */
#define BP_RID_OFF		(IO_RID_OFF + IO_RES_NUM)
#define BP_RID(rid)		((rid) + BP_RID_OFF)
#define BP_RES_NUM		(1u)
/* DPCON resources */
#define CON_RID_OFF		(BP_RID_OFF + BP_RES_NUM)
#define CON_RID(rid)		((rid) + CON_RID_OFF)
#define CON_RES_NUM		(4u)

struct resource_spec dpaa2_ni_spec[] = {
	/*
	 * DPIO resources.
	 *
	 * NOTE: One per running core. While DPIOs are the source of data
	 * availability interrupts, the DPCONs are used to identify the network
	 * interface that has produced ingress data to that core.
	 */
	{ DPAA2_DEV_IO,  IO_RID(0),   RF_ACTIVE | RF_SHAREABLE },
	{ DPAA2_DEV_IO,  IO_RID(1),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(2),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(3),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	/*
	 * DPBP resources.
	 *
	 * NOTE: One buffer pool per network interface.
	 */
	{ DPAA2_DEV_BP,  BP_RID(0),   RF_ACTIVE },
	/*
	 * DPCON resources.
	 *
	 * NOTE: One concentrator per core where Rx or Tx confirmation traffic
	 * to be distributed to. Since it is necessary to distinguish between
	 * traffic from different network interfaces arriving on the same core,
	 * the DPCONs must be private to the DPNIs.
	 */
	{ DPAA2_DEV_CON, CON_RID(0),  RF_ACTIVE },
	{ DPAA2_DEV_CON, CON_RID(1),  RF_ACTIVE | RF_OPTIONAL },
	{ DPAA2_DEV_CON, CON_RID(2),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(3),  RF_ACTIVE | RF_OPTIONAL },

	RESOURCE_SPEC_END
};

/* Forward declarations. */

static int	setup_dpni(device_t dev, dpaa2_cmd_t cmd, uint16_t rc_token);
static int	setup_channels(device_t dev, dpaa2_cmd_t cmd, uint16_t rc_token);
static int	setup_fqs(device_t dev, dpaa2_cmd_t cmd, uint16_t rc_token);

static int	set_buf_layout(device_t dev, dpaa2_cmd_t cmd);
static int	set_pause_frame(device_t dev, dpaa2_cmd_t cmd);
static int	set_qos_table(device_t dev, dpaa2_cmd_t cmd);

static int	dpni_ifmedia_change(struct ifnet *ifp);
static void	dpni_ifmedia_status(struct ifnet *ifp, struct ifmediareq *ifmr);
static void	dpni_ifmedia_tick(void *arg);

static void	dpni_if_init(void *arg);
static void	dpni_if_start(struct ifnet *ifp);
static int	dpni_if_ioctl(struct ifnet *ifp, u_long command, caddr_t data);

static uint8_t	calc_channels_num(struct dpaa2_ni_softc *sc);
static int	cmp_api_version(struct dpaa2_ni_softc *sc, const uint16_t major,
		    uint16_t minor);

/* Callbacks. */

static void	dpni_cdan_cb(dpaa2_io_notif_ctx_t *ctx);
static void	dpni_qos_kcfg_dmamap_cb(void *arg, bus_dma_segment_t *segs,
		    int nseg, int error);

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
	struct ifnet *ifp;
	uint16_t rc_token;
	int error;

 	sc = device_get_softc(dev);
	sc->dev = dev;
	pdev = device_get_parent(dev);
	rcinfo = device_get_ivars(pdev);
	dinfo = device_get_ivars(dev);

	sc->ifp = NULL;
	sc->miibus = NULL;
	sc->mii = NULL;
	sc->media_status = 0;
	sc->mac.dpmac_id = 0;
	memset(sc->mac.addr, 0, ETHER_ADDR_LEN);

	error = bus_alloc_resources(sc->dev, dpaa2_ni_spec, sc->res);
	if (error) {
		device_printf(dev, "Failed to allocate resources: error=%d\n",
		    error);
		return (ENXIO);
	}

	mtx_init(&sc->lock, device_get_nameunit(dev), "dpni lock", MTX_DEF);

	/* Allocate network interface */
	ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "Failed to allocate network interface\n");
		return (ENXIO);
	}

	sc->ifp = ifp;

	if_initname(ifp, device_get_name(sc->dev), device_get_unit(sc->dev));
	ifp->if_softc = sc;
	ifp->if_flags = IFF_SIMPLEX | IFF_MULTICAST | IFF_BROADCAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU | IFCAP_HWCSUM;
	ifp->if_capenable = ifp->if_capabilities;

	ifp->if_init =	dpni_if_init;
	ifp->if_start = dpni_if_start;
	ifp->if_ioctl = dpni_if_ioctl;

	ifp->if_snd.ifq_drv_maxlen = 64; /* arbitrary length for now */
	IFQ_SET_MAXLEN(&ifp->if_snd, ifp->if_snd.ifq_drv_maxlen);
	IFQ_SET_READY(&ifp->if_snd);

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Open resource container. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: id=%d, error=%d\n",
		    rcinfo->id, error);
		goto err_free_cmd;
	}

	/* Setup network interface object. */
	error = setup_dpni(dev, cmd, rc_token);
	if (error)
		goto err_free_cmd;

	/* Configure QBMan channels. */
	error = setup_channels(dev, cmd, rc_token);
	if (error)
		goto err_free_cmd;

	/* Configure frame queues. */
	error = setup_fqs(dev, cmd, rc_token);
	if (error)
		goto err_free_cmd;

	/* Close resource container. */
	error = DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(cmd, rc_token));
	if (error) {
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
		goto err_free_cmd;
	}

	ether_ifattach(sc->ifp, sc->mac.addr);
	callout_init(&sc->mii_callout, 0);

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
	error = DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(cmd, rc_token));
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
	error = DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(cmd, rc_token));
	if (error)
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
 free_cmd:
	dpaa2_mcp_free_command(cmd);
	return (0);
}

/*
 * Internal functions.
 */

/**
 * @internal
 * @brief Configure DPAA2 network interface object.
 */
static int
setup_dpni(device_t dev, dpaa2_cmd_t cmd, uint16_t rc_token)
{
	struct dpaa2_ni_softc *sc;
	struct dpaa2_devinfo *dinfo;
	dpaa2_ep_desc_t ep1_desc, ep2_desc;
	uint16_t ni_token;
	uint32_t link;
	int error;

	sc = device_get_softc(dev);
	dinfo = device_get_ivars(dev);

	/* Open network interface object. */
	error = DPAA2_CMD_NI_OPEN(dev, cmd, dinfo->id, &ni_token);
	if (error) {
		device_printf(dev, "Failed to open DPNI: id=%d, error=%d\n",
		    dinfo->id, error);
		return (ENXIO);
	}

	/* Check if we can work with this DPNI object. */
	error = DPAA2_CMD_NI_GET_API_VERSION(dev, cmd, &sc->api_major,
	    &sc->api_minor);
	if (error) {
		device_printf(dev, "Failed to get DPNI API version: error=%d\n",
		    error);
		goto err_close_ni;
	}
	if (cmp_api_version(sc, DPNI_VER_MAJOR, DPNI_VER_MINOR) < 0) {
		device_printf(dev, "DPNI API version %u.%u not supported, "
		    "need >= %u.%u\n", sc->api_major, sc->api_minor,
		    DPNI_VER_MAJOR, DPNI_VER_MINOR);
		goto err_close_ni;
	}

	/* Reset the DPNI object. */
	error = DPAA2_CMD_NI_RESET(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to reset DPNI: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_ni;
	}

	/* Obtain attributes of the DPNI object. */
	error = DPAA2_CMD_NI_GET_ATTRIBUTES(dev, cmd, &sc->attr);
	if (error) {
		device_printf(dev, "Failed to obtain DPNI attributes: id=%d, "
		    "error=%d\n", dinfo->id, error);
		goto err_close_ni;
	}
	if (bootverbose)
		device_printf(dev,
		    "options=%#x queues=%d tx_channels=%d wriop_version=%#x\n"
		    "\t traffic classes: rx=%d tx=%d cgs_groups=%d\n"
		    "\t table entries: mac=%d vlan=%d qos=%d fs=%d\n"
		    "\t key sizes: qos=%d fs=%d\n",
		    sc->attr.options,
		    sc->attr.num.queues, sc->attr.num.channels,
		    sc->attr.wriop_ver,
		    sc->attr.num.rx_tcs + 1, sc->attr.num.tx_tcs + 8,
		    sc->attr.num.cgs,
		    sc->attr.entries.mac + 16, sc->attr.entries.vlan,
		    sc->attr.entries.qos + 64, sc->attr.entries.fs + 64,
		    sc->attr.key_size.qos, sc->attr.key_size.fs);

	/* Configure buffer layouts of the DPNI queues. */
	error = set_buf_layout(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to configure buffer layout: "
		    "error=%d\n", error);
		goto err_close_ni;
	}

	/* Attach miibus and PHY in case of DPNI<->DPMAC. */
	ep1_desc.obj_id = dinfo->id;
	ep1_desc.if_id = 0; /* DPNI has an only endpoint */
	ep1_desc.type = dinfo->dtype;

	error = DPAA2_CMD_RC_GET_CONN(dev, dpaa2_mcp_tk(cmd, rc_token),
	    &ep1_desc, &ep2_desc, &link);
	if (error)
		device_printf(dev, "Failed to obtain an object DPNI is "
		    "connected to: error=%d\n", error);
	else {
		/*
		 * NOTE: For the DPAA2 link to be up, both objects must be in
		 * enabled state.
		 */
		device_printf(dev, "connected to %s (id=%d), DPAA2 link %s\n",
		    dpaa2_ttos(ep2_desc.type), ep2_desc.obj_id,
		    link ? "up" : "down");

		if (ep2_desc.type == DPAA2_DEV_MAC) {
			/*
			 * This is the simplest case when DPNI is connected to
			 * DPMAC directly. Let's attach miibus then.
			 */
			sc->mac.dpmac_id = ep2_desc.obj_id;

			error = DPAA2_CMD_NI_GET_PORT_MAC_ADDR(dev,
			    dpaa2_mcp_tk(cmd, ni_token), sc->mac.addr);
			if (error)
				device_printf(dev, "Failed to obtain a MAC "
				    "address of the connected DPMAC: error=%d\n",
				    error);
			else {
				device_printf(dev, "ether %6D\n", sc->mac.addr,
				    ":");

				error = mii_attach(dev, &sc->miibus, sc->ifp,
				    dpni_ifmedia_change, dpni_ifmedia_status,
				    BMSR_DEFCAPMASK, MII_PHY_ANY, 0, 0);
				if (error)
					device_printf(dev, "Failed to attach "
					    "miibus: error=%d\n", error);
				else
					sc->mii = device_get_softc(sc->miibus);
			}
		}
	}

	/* Select mode to enqueue frames. */
	/* ... TBD ... */

	/*
	 * Update link configuration to enable Rx/Tx pause frames support.
	 *
	 * NOTE: MC may generate an interrupt to the DPMAC and request changes
	 *       in link configuration. It might be necessary to attach miibus
	 *       and PHY before this point.
	 */
	error = set_pause_frame(dev, dpaa2_mcp_tk(cmd, ni_token));
	if (error) {
		device_printf(dev, "Failed to configure Rx/Tx pause frames: "
		    "error=%d\n", error);
		goto err_close_ni;
	}

	/* Configure ingress traffic classification. */
	error = set_qos_table(dev, dpaa2_mcp_tk(cmd, ni_token));
	if (error)
		device_printf(dev, "Failed to configure QoS table: error=%d\n",
		    error);

	error = DPAA2_CMD_NI_CLOSE(dev, dpaa2_mcp_tk(cmd, ni_token));
	if (error) {
		device_printf(dev, "Failed to close DPNI: id=%d, error=%d\n",
		    dinfo->id, error);
		return (ENXIO);
	}

	return (0);

 err_close_ni:
	error = DPAA2_CMD_NI_CLOSE(dev, dpaa2_mcp_tk(cmd, ni_token));
	if (error)
		device_printf(dev, "Failed to close DPNI: id=%d, error=%d\n",
		    dinfo->id, error);

	return (ENXIO);
}

/**
 * @internal
 * @brief Сonfigure QBMan channels and register data availability notifications.
 */
static int
setup_channels(device_t dev, dpaa2_cmd_t cmd, uint16_t rc_token)
{
	device_t io_dev, con_dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_io_softc *iosc;
	struct dpaa2_con_softc *consc;
	struct dpaa2_devinfo *io_info;
	struct dpaa2_devinfo *con_info;
	dpaa2_ni_channel_t *channel;
	dpaa2_io_notif_ctx_t *ctx;
	dpaa2_con_notif_cfg_t notif_cfg;
	uint16_t con_token;
	int error;

	sc->num_chan = calc_channels_num(sc);

	/* Allocate no more channels then DPNI queues. */
	sc->num_chan = sc->num_chan > sc->attr.num.queues
	    ? sc->attr.num.queues : sc->num_chan;

	for (uint32_t i = 0; i < sc->num_chan; i++) {
		channel = malloc(sizeof(dpaa2_ni_channel_t), M_DPAA2_NI,
		    M_WAITOK | M_ZERO);
		if (!channel) {
			device_printf(dev, "Failed to allocate a channel\n");
			return (ENOMEM);
		}
		sc->channel[i] = channel;

		io_dev =  (device_t) rman_get_start(sc->res[IO_RID(i)]);
		con_dev = (device_t) rman_get_start(sc->res[CON_RID(i)]);
		iosc = device_get_softc(io_dev);
		consc = device_get_softc(con_dev);
		io_info = device_get_ivars(io_dev);
		con_info = device_get_ivars(con_dev);

		channel->io_dev = io_dev;
		channel->con_dev = con_dev;
		channel->id = consc->attr.chan_id;

		/* Setup WQ channel notification context. */
		ctx = &channel->ctx;
		ctx->cb = dpni_cdan_cb;
		ctx->qman_ctx = (uint64_t) ctx;
		ctx->cdan_en = true;
		ctx->fq_chan_id = channel->id;
		ctx->io_dev = channel->io_dev;

		/* Register the new notification context. */
		error = DPAA2_SWP_CONF_WQ_CHANNEL(channel->io_dev, ctx);
		if (error) {
			device_printf(dev, "Failed to register notification: "
			    "error=%d\n", error);
			return (ENXIO);
		}

		/* Open data path concentrator object. */
		error = DPAA2_CMD_CON_OPEN(dev, dpaa2_mcp_tk(cmd, rc_token),
		    con_info->id, &con_token);
		if (error) {
			device_printf(dev, "Failed to open DPCON: id=%d, "
			    "error=%d\n", con_info->id, error);
			return (ENXIO);
		}

		/* Register DPCON notification with MC. */
		notif_cfg.dpio_id = io_info->id;
		notif_cfg.prior = 0;
		notif_cfg.qman_ctx = ctx->qman_ctx;
		error = DPAA2_CMD_CON_SET_NOTIF(dev, cmd, &notif_cfg);
		if (error) {
			device_printf(dev, "Failed to set DPCON notification: "
			    "id=%d, error=%d\n", con_info->id, error);
			goto err_close_con;
		}

		/* Close data path concentrator object. */
		error = DPAA2_CMD_CON_CLOSE(dev, dpaa2_mcp_tk(cmd, con_token));
		if (error)
			device_printf(dev, "Failed to close DPCON: id=%d, "
			    "error=%d\n", con_info->id, error);

		if (bootverbose)
			device_printf(dev, "channel: dpio_id=%d dpcon_id=%d "
			    "channel_id=%d\n", io_info->id, con_info->id,
			    channel->id);
	}

	/* TODO: De-allocate redundant DPIOs or DPCONs if exist. */
	return (0);
 err_close_con:
	error = DPAA2_CMD_CON_CLOSE(dev, dpaa2_mcp_tk(cmd, con_token));
	if (error)
		device_printf(dev, "Failed to close DPCON: id=%d, error=%d\n",
		    con_info->id, error);

	return (ENXIO);
}

/**
 * @internal
 * @brief Сonfigure frame queues.
 */
static int
setup_fqs(device_t dev, dpaa2_cmd_t cmd, uint16_t rc_token)
{
	return (0);
}

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

/**
 * @internal
 * @brief Enable Rx/Tx pause frames.
 *
 * NOTE: DPNI stops sending when a pause frame is received (Rx frame) or DPNI
 *       itself generates pause frames (Tx frame).
 */
static int
set_pause_frame(device_t dev, dpaa2_cmd_t cmd)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	dpaa2_ni_link_cfg_t link_cfg = {0};
	int error;

	error = DPAA2_CMD_NI_GET_LINK_CFG(dev, cmd, &link_cfg);
	if (error) {
		device_printf(dev, "Failed to obtain link configuration: "
		    "error=%d\n", error);
		return (error);
	}

	/* Enable both Rx and Tx pause frames by default. */
	link_cfg.options |= DPAA2_NI_LINK_OPT_PAUSE;
	link_cfg.options &= ~DPAA2_NI_LINK_OPT_ASYM_PAUSE;

	error = DPAA2_CMD_NI_SET_LINK_CFG(dev, cmd, &link_cfg);
	if (error) {
		device_printf(dev, "Failed to set link configuration: "
		    "error=%d\n", error);
		return (error);
	}

	sc->link_state.options = link_cfg.options;

	return (0);
}

/**
 * @internal
 * @brief Configure QoS table to determine the traffic class for the received
 * frame.
 */
static int
set_qos_table(device_t dev, dpaa2_cmd_t cmd)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	dpaa2_ni_qos_table_t tbl;
	int error;

	if (sc->attr.num.rx_tcs == 1 ||
	    !(sc->attr.options & DPNI_OPT_HAS_KEY_MASKING)) {
		if (bootverbose)
			device_printf(dev, "VLAN-based QoS classification is "
			    "not supported\n");
		return (0);
	}

	/*
	 * Allocate a buffer visible to the device to hold the QoS table key
	 * configuration.
	 */

	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    PAGE_SIZE, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* low restricted addr */
	    BUS_SPACE_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    ETH_QOS_KCFG_BUF_SIZE, 1,	/* maxsize, nsegments */
	    ETH_QOS_KCFG_BUF_SIZE, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->qos_kcfg.dtag);
	if (error) {
		device_printf(dev, "Failed to create a DMA tag for QoS key "
		    "configuration buffer\n");
		return (error);
	}

	error = bus_dmamem_alloc(sc->qos_kcfg.dtag, (void **) &sc->qos_kcfg.buf,
	    BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->qos_kcfg.dmap);
	if (error) {
		device_printf(dev, "Failed to allocate a buffer for QoS key "
		    "configuration\n");
		return (error);
	}

	error = bus_dmamap_load(sc->qos_kcfg.dtag, sc->qos_kcfg.dmap,
	    sc->qos_kcfg.buf, ETH_QOS_KCFG_BUF_SIZE, dpni_qos_kcfg_dmamap_cb,
	    &sc->qos_kcfg.buf_busaddr, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(dev, "Failed to map QoS key configuration buffer "
		    "into bus space\n");
		return (error);
	}

	tbl.default_tc = 0;
	tbl.discard_on_miss = false;
	tbl.keep_entries = false;
	tbl.kcfg_busaddr = sc->qos_kcfg.buf_busaddr;
	error = DPAA2_CMD_NI_SET_QOS_TABLE(dev, cmd, &tbl);
	if (error) {
		device_printf(dev, "Failed to set QoS table\n");
		return (error);
	}

	error = DPAA2_CMD_NI_CLEAR_QOS_TABLE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to clear QoS table\n");
		return (error);
	}

	return (0);
}

/**
 * @internal
 * @brief Callback function to process media change request.
 */
static int
dpni_ifmedia_change(struct ifnet *ifp)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;

	DPNI_LOCK(sc);

	if (sc->mii) {
		mii_mediachg(sc->mii);
		sc->media_status = sc->mii->mii_media.ifm_media;
	}

	DPNI_UNLOCK(sc);

	return (0);
}

/**
 * @internal
 * @brief Callback function to process media status request.
 */
static void
dpni_ifmedia_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;

	DPNI_LOCK(sc);

	if (sc->mii) {
		mii_pollstat(sc->mii);
		ifmr->ifm_active = sc->mii->mii_media_active;
		ifmr->ifm_status = sc->mii->mii_media_status;
	}

	DPNI_UNLOCK(sc);
}

/**
 * @internal
 * @brief Callout function to check and update media status.
 */
static void
dpni_ifmedia_tick(void *arg)
{
	struct dpaa2_ni_softc *sc = (struct dpaa2_ni_softc *) arg;

	/* Check for media type change */
	if (sc->mii) {
		mii_tick(sc->mii);
		if (sc->media_status != sc->mii->mii_media.ifm_media) {
			printf("%s: media type changed (ifm_media=%x)\n",
			    __func__, sc->mii->mii_media.ifm_media);
			dpni_ifmedia_change(sc->ifp);
		}
	}

	/* Schedule another timeout one second from now */
	callout_reset(&sc->mii_callout, hz, dpni_ifmedia_tick, sc);
}

/**
 * @internal
 */
static void
dpni_if_init(void *arg)
{
	struct dpaa2_ni_softc *sc = (struct dpaa2_ni_softc *) arg;
	struct ifnet *ifp = sc->ifp;

	DPNI_LOCK(sc);

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0) {
		DPNI_UNLOCK(sc);
		return;
	}

	if (sc->mii)
		mii_mediachg(sc->mii);
	callout_reset(&sc->mii_callout, hz, dpni_ifmedia_tick, sc);
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	DPNI_UNLOCK(sc);
}

/**
 * @internal
 */
static void
dpni_if_start(struct ifnet *ifp)
{
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return;
	/* ... enqueue frames here ... */
}

/**
 * @internal
 */
static int
dpni_if_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;
	struct ifreq *ifr = (struct ifreq *) data;
	uint32_t changed = 0;
	int error = 0;

	switch (command) {
	case SIOCSIFCAP:
		changed = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (changed & IFCAP_HWCSUM) {
			if ((ifr->ifr_reqcap & changed) & IFCAP_HWCSUM)
				ifp->if_capenable |= IFCAP_HWCSUM;
			else
				ifp->if_capenable &= ~IFCAP_HWCSUM;
		}
		break;
	case SIOCSIFFLAGS:
		/* TBD */
		break;
	case SIOCADDMULTI:
		/* TBD */
		break;
	case SIOCDELMULTI:
		/* TBD */
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		if (sc->mii)
			error = ifmedia_ioctl(ifp, ifr, &sc->mii->mii_media,
			    command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
	}

	return (error);
}

/**
 * @internal
 * @brief Channel data availability notification (CDAN) callback.
 */
static void
dpni_cdan_cb(dpaa2_io_notif_ctx_t *ctx)
{
	/* TBD */
}

/**
 * @internal
 * @brief Callback function to obtain an address of the QoS table key
 * configuration buffer in the device visible address space.
 */
static void
dpni_qos_kcfg_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error)
		return;
	/*
	 * Only one 256-bytes long segment has been requested for QoS key
	 * configuration buffer.
	 */
	*(bus_addr_t *) arg = segs[0].ds_addr;
}

/**
 * @internal
 */
static int
cmp_api_version(struct dpaa2_ni_softc *sc, const uint16_t major, uint16_t minor)
{
	if (sc->api_major == major)
		return sc->api_minor - minor;
	return sc->api_major - major;
}

/**
 * @internal
 * @brief
 */
static uint8_t
calc_channels_num(struct dpaa2_ni_softc *sc)
{
	uint8_t i, num_chan;

	/* # of allocated DPIOs */
	for (i = 0; i < IO_RES_NUM; i++)
		if (!sc->res[IO_RID(i)])
			break;
	num_chan = i;

	/* # of allocated DPCONs */
	for (i = 0; i < CON_RES_NUM; i++)
		if (!sc->res[CON_RID(i)])
			break;
	num_chan = i < num_chan ? i : num_chan;

	return (num_chan > DPAA2_NI_MAX_CHANNELS
	    ? DPAA2_NI_MAX_CHANNELS : num_chan);
}

static device_method_t dpaa2_ni_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_ni_probe),
	DEVMETHOD(device_attach,	dpaa2_ni_attach),
	DEVMETHOD(device_detach,	dpaa2_ni_detach),

	/* MII bus interface */
	DEVMETHOD(miibus_readreg,	dpaa2_ni_miibus_readreg),
	DEVMETHOD(miibus_writereg,	dpaa2_ni_miibus_writereg),
	/* DEVMETHOD(miibus_statchg,	dpaa2_ni_miibus_statchg), */

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
