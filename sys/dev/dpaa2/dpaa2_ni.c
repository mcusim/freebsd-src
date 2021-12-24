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
#include <dev/mdio/mdio.h>

#include "pcib_if.h"
#include "pci_if.h"
#include "miibus_if.h"
#include "mdio_if.h"

#include "dpaa2_mc.h"
#include "dpaa2_mc_if.h"
#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"
#include "dpaa2_swp_if.h"
#include "dpaa2_cmd_if.h"
#include "dpaa2_ni.h"

#define BIT(x)			(1ul << (x))
#define WRIOP_VERSION(x, y, z)	((x) << 10 | (y) << 5 | (z) << 0)
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))

#define	ALIGN_UP(x, y)		roundup2((x), (y))
#define	ALIGN_DOWN(x, y)	rounddown2((x), (y))
#define CACHE_LINE_ALIGN(x)	ALIGN_UP((x), CACHE_LINE_SIZE)

#define DPNI_LOCK(sc) do {			\
	mtx_assert(&(sc)->lock, MA_NOTOWNED);	\
	mtx_lock(&(sc)->lock);			\
} while (0)
#define	DPNI_UNLOCK(sc)		mtx_unlock(&(sc)->lock)

/* Maximum acceptable frame length and MTU. */
#define DPAA2_ETH_MFL		(10 * 1024)
#define DPAA2_ETH_HDR_AND_VLAN	(ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN)
#define DPAA2_ETH_MTU		(DPAA2_ETH_MFL - DPAA2_ETH_HDR_AND_VLAN)

/* Index of the only DPNI IRQ. */
#define DPNI_IRQ_INDEX		0

/* DPNI IRQ statuses. */
#define DPNI_IRQ_LINK_CHANGED	0x00000001 /* link state changed */
#define DPNI_IRQ_EP_CHANGED	0x00000002 /* DPAA2 endpoint dis/connected */

/* Minimally supported version of the DPNI API. */
#define DPNI_VER_MAJOR		7U
#define DPNI_VER_MINOR		0U

/* RX buffer data alignment. */
#define ETH_RX_BUF_ALIGN_V1	256 /* limitation of the WRIOP v1.0.0 */
#define ETH_RX_BUF_ALIGN	64

/* Frame's software annotation. The hardware options are either 0 or 64. */
#define ETH_SWA_SIZE		64

/* Hardware annotation area in RX/TX buffers. */
#define ETH_RX_HWA_SIZE		64

/* Rx buffer configuration. */
#define ETH_RX_BUF_RAW_SIZE	PAGE_SIZE
#define ETH_RX_BUF_TAILROOM	CACHE_LINE_ALIGN(sizeof(struct mbuf))
#define ETH_RX_BUF_SIZE		(ETH_RX_BUF_RAW_SIZE - ETH_RX_BUF_TAILROOM)

/* Size of a buffer to keep a QoS table key configuration. */
#define ETH_QOS_KCFG_BUF_SIZE	256

/* Required by struct dpni_rx_tc_dist_cfg::key_cfg_iova */
#define DPAA2_CLASSIFIER_DMA_SIZE 256

/* Buffers layout options. */
#define DPNI_BUF_LAYOUT_OPT_TIMESTAMP		0x00000001
#define DPNI_BUF_LAYOUT_OPT_PARSER_RESULT	0x00000002
#define DPNI_BUF_LAYOUT_OPT_FRAME_STATUS	0x00000004
#define DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE	0x00000008
#define DPNI_BUF_LAYOUT_OPT_DATA_ALIGN		0x00000010
#define DPNI_BUF_LAYOUT_OPT_DATA_HEAD_ROOM	0x00000020
#define DPNI_BUF_LAYOUT_OPT_DATA_TAIL_ROOM	0x00000040

/* Enables TCAM for Flow Steering and QoS look-ups. */
#define DPNI_OPT_HAS_KEY_MASKING		0x000010

/* Unique IDs for the supported Rx classification header fields */
#define DPAA2_ETH_DIST_ETHDST			BIT(0)
#define DPAA2_ETH_DIST_ETHSRC			BIT(1)
#define DPAA2_ETH_DIST_ETHTYPE			BIT(2)
#define DPAA2_ETH_DIST_VLAN			BIT(3)
#define DPAA2_ETH_DIST_IPSRC			BIT(4)
#define DPAA2_ETH_DIST_IPDST			BIT(5)
#define DPAA2_ETH_DIST_IPPROTO			BIT(6)
#define DPAA2_ETH_DIST_L4SRC			BIT(7)
#define DPAA2_ETH_DIST_L4DST			BIT(8)
#define DPAA2_ETH_DIST_ALL			(~0ULL)

/* L3-L4 network traffic flow hash options */
#define	RXH_L2DA	(1 << 1)
#define	RXH_VLAN	(1 << 2)
#define	RXH_L3_PROTO	(1 << 3)
#define	RXH_IP_SRC	(1 << 4)
#define	RXH_IP_DST	(1 << 5)
#define	RXH_L4_B_0_1	(1 << 6) /* src port in case of TCP/UDP/SCTP */
#define	RXH_L4_B_2_3	(1 << 7) /* dst port in case of TCP/UDP/SCTP */
#define	RXH_DISCARD	(1 << 31)

/* Default Rx hash options, set during attaching */
#define DPAA2_RXH_DEFAULT	(RXH_L3_PROTO | RXH_IP_SRC | RXH_IP_DST | \
				 RXH_L4_B_0_1 | RXH_L4_B_2_3)

MALLOC_DEFINE(M_DPAA2_NI, "dpaa2_ni", "DPAA2 Network Interface");

/* for DPIO resources */
#define IO_RID_OFF		(0u)
#define IO_RID(rid)		((rid) + IO_RID_OFF)
#define IO_RES_NUM		(8u)
/* for DPBP resources */
#define BP_RID_OFF		(IO_RID_OFF + IO_RES_NUM)
#define BP_RID(rid)		((rid) + BP_RID_OFF)
#define BP_RES_NUM		(1u)
/* for DPCON resources */
#define CON_RID_OFF		(BP_RID_OFF + BP_RES_NUM)
#define CON_RID(rid)		((rid) + CON_RID_OFF)
#define CON_RES_NUM		(8u)
/* for DPMCP resources */
#define MCP_RID_OFF		(CON_RID_OFF + CON_RES_NUM)
#define MCP_RID(rid)		((rid) + MCP_RID_OFF)
#define MCP_RES_NUM		(1u)

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
	{ DPAA2_DEV_IO,  IO_RID(4),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(5),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(6),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(7),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
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
 	{ DPAA2_DEV_CON, CON_RID(4),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(5),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(6),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(7),  RF_ACTIVE | RF_OPTIONAL },
	/*
	 * DPMCP resources.
	 *
	 * NOTE: One per DPNI. MC command portals (MCPs) are used to send
	 * commands to, and receive responses from, the MC firmware.
	 */
	/* { DPAA2_DEV_MCP, MCP_RID(0), RF_ACTIVE }, */

	RESOURCE_SPEC_END
};

/* Supported header fields for Rx hash distribution key */
static const struct dpaa2_eth_dist_fields dist_fields[] = {
	{
		/* L2 header */
		.rxnfc_field = RXH_L2DA,
		.cls_prot = NET_PROT_ETH,
		.cls_field = NH_FLD_ETH_DA,
		.id = DPAA2_ETH_DIST_ETHDST,
		.size = 6,
	}, {
		.cls_prot = NET_PROT_ETH,
		.cls_field = NH_FLD_ETH_SA,
		.id = DPAA2_ETH_DIST_ETHSRC,
		.size = 6,
	}, {
		/* This is the last ethertype field parsed:
		 * depending on frame format, it can be the MAC ethertype
		 * or the VLAN etype.
		 */
		.cls_prot = NET_PROT_ETH,
		.cls_field = NH_FLD_ETH_TYPE,
		.id = DPAA2_ETH_DIST_ETHTYPE,
		.size = 2,
	}, {
		/* VLAN header */
		.rxnfc_field = RXH_VLAN,
		.cls_prot = NET_PROT_VLAN,
		.cls_field = NH_FLD_VLAN_TCI,
		.id = DPAA2_ETH_DIST_VLAN,
		.size = 2,
	}, {
		/* IP header */
		.rxnfc_field = RXH_IP_SRC,
		.cls_prot = NET_PROT_IP,
		.cls_field = NH_FLD_IP_SRC,
		.id = DPAA2_ETH_DIST_IPSRC,
		.size = 4,
	}, {
		.rxnfc_field = RXH_IP_DST,
		.cls_prot = NET_PROT_IP,
		.cls_field = NH_FLD_IP_DST,
		.id = DPAA2_ETH_DIST_IPDST,
		.size = 4,
	}, {
		.rxnfc_field = RXH_L3_PROTO,
		.cls_prot = NET_PROT_IP,
		.cls_field = NH_FLD_IP_PROTO,
		.id = DPAA2_ETH_DIST_IPPROTO,
		.size = 1,
	}, {
		/* Using UDP ports, this is functionally equivalent to raw
		 * byte pairs from L4 header.
		 */
		.rxnfc_field = RXH_L4_B_0_1,
		.cls_prot = NET_PROT_UDP,
		.cls_field = NH_FLD_UDP_PORT_SRC,
		.id = DPAA2_ETH_DIST_L4SRC,
		.size = 2,
	}, {
		.rxnfc_field = RXH_L4_B_2_3,
		.cls_prot = NET_PROT_UDP,
		.cls_field = NH_FLD_UDP_PORT_DST,
		.id = DPAA2_ETH_DIST_L4DST,
		.size = 2,
	},
};

/* Forward declarations. */

static int	setup_dpni(device_t dev);
static int	setup_channels(device_t dev);
static int	setup_frame_queues(device_t dev);
static int	setup_dpni_binding(device_t dev);
static int	setup_dpni_irqs(device_t dev);
static int	setup_rx_distribution(device_t dev);
static int	setup_rx_flow(device_t, dpaa2_cmd_t, dpaa2_ni_fq_t *);
static int	setup_tx_flow(device_t, dpaa2_cmd_t, dpaa2_ni_fq_t *);
static int	setup_rx_err_flow(device_t, dpaa2_cmd_t, dpaa2_ni_fq_t *);
static int	setup_msi(struct dpaa2_ni_softc *);
static int	setup_if_caps(struct dpaa2_ni_softc *);
static int	setup_if_flags(struct dpaa2_ni_softc *);

static int	set_buf_layout(device_t, dpaa2_cmd_t);
static int	set_pause_frame(device_t, dpaa2_cmd_t);
static int	set_qos_table(device_t, dpaa2_cmd_t);
static int	set_mac_addr(device_t, dpaa2_cmd_t, uint16_t, uint16_t);

static uint8_t	calc_channels_num(struct dpaa2_ni_softc *sc);
static int	cmp_api_version(struct dpaa2_ni_softc *sc, const uint16_t major,
		    uint16_t minor);
static void	print_statistics(struct dpaa2_ni_softc *);
static int	seed_buf_pool(struct dpaa2_ni_softc *, dpaa2_ni_channel_t *);
static int	dpni_prepare_key_cfg(const struct dpkg_profile_cfg *cfg,
		    uint8_t *key_cfg_buf);
static int	dpaa2_eth_set_hash(struct dpaa2_ni_softc *sc, uint64_t flags);
static int	dpaa2_eth_set_dist_key(struct dpaa2_ni_softc *sc,
		    enum dpaa2_ni_dist_mode type, uint64_t flags);

/* Callbacks. */

static void	dpni_if_init(void *arg);
static void	dpni_if_start(struct ifnet *ifp);
static int	dpni_if_ioctl(struct ifnet *ifp, u_long command, caddr_t data);

static int	dpni_ifmedia_change(struct ifnet *ifp);
static void	dpni_ifmedia_status(struct ifnet *ifp, struct ifmediareq *ifmr);
static void	dpni_ifmedia_tick(void *arg);

static void	dpni_cdan_cb(dpaa2_io_notif_ctx_t *ctx);
static void	dpni_single_seg_dmamap_cb(void *arg, bus_dma_segment_t *segs,
		    int nseg, int error);

static void	dpni_consume_tx_conf(device_t dev, dpaa2_ni_channel_t *channel,
		    struct dpaa2_ni_fq *fq, const dpaa2_fd_t *fd);
static void	dpni_consume_rx(device_t dev, dpaa2_ni_channel_t *channel,
		    struct dpaa2_ni_fq *fq, const dpaa2_fd_t *fd);
static void	dpni_consume_rx_err(device_t dev, dpaa2_ni_channel_t *channel,
		    struct dpaa2_ni_fq *fq, const dpaa2_fd_t *fd);

/* ISRs */

static void	dpni_msi_intr(void *arg);

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
	device_t pdev = device_get_parent(dev);
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(pdev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(dev);
	struct ifnet *ifp;
	int error;

	sc->dev = dev;
	sc->ifp = NULL;
	sc->miibus = NULL;
	sc->mii = NULL;
	sc->media_status = 0;
	sc->if_flags = 0;
	sc->link_state = LINK_STATE_UNKNOWN;

	sc->qos_kcfg.dtag = NULL;
	sc->qos_kcfg.dmap = NULL;
	sc->qos_kcfg.buf_pa = 0;
	sc->qos_kcfg.buf_va = NULL;

	sc->mac.dpmac_id = 0;
	sc->mac.phy_dev = NULL;
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

	if_initname(ifp, DPAA2_NI_IFNAME, device_get_unit(sc->dev));
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
	error = dpaa2_mcp_init_command(&sc->cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Open resource container and network interface object. */
	error = DPAA2_CMD_RC_OPEN(dev, sc->cmd, rcinfo->id, &sc->rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: id=%d, error=%d\n",
		    rcinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_NI_OPEN(dev, dpaa2_mcp_tk(sc->cmd, sc->rc_token),
	    dinfo->id, &sc->ni_token);
	if (error) {
		device_printf(dev, "Failed to open DPNI: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_rc;
	}

	error = setup_dpni(dev);
	if (error) {
		device_printf(dev, "Failed to setup DPNI: error=%d\n", error);
		goto err_close_ni;
	}
	error = setup_channels(dev);
	if (error) {
		device_printf(dev, "Failed to setup QBMan channels: error=%d\n",
		    error);
		goto err_close_ni;
	}
	error = setup_frame_queues(dev);
	if (error) {
		device_printf(dev, "Failed to setup frame queues: error=%d\n",
		    error);
		goto err_close_ni;
	}
	error = setup_dpni_binding(dev);
	if (error) {
		device_printf(dev, "Failed to bind DPNI: error=%d\n", error);
		goto err_close_ni;
	}
	error = setup_dpni_irqs(dev);
	if (error) {
		device_printf(dev, "Failed to setup IRQs: error=%d\n", error);
		goto err_close_ni;
	}

	ether_ifattach(sc->ifp, sc->mac.addr);
	callout_init(&sc->mii_callout, 0);

	return (0);

err_close_ni:
	DPAA2_CMD_NI_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->ni_token));
err_close_rc:
	DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->rc_token));
err_free_cmd:
	dpaa2_mcp_free_command(sc->cmd);
err_exit:
	return (ENXIO);
}

static int
dpaa2_ni_detach(device_t dev)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);

	DPAA2_CMD_NI_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->ni_token));
	DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->rc_token));
	dpaa2_mcp_free_command(sc->cmd);

	sc->cmd = NULL;
	sc->ni_token = 0;
	sc->rc_token = 0;

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
setup_dpni(device_t dev)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(dev);
	dpaa2_ep_desc_t ep1_desc, ep2_desc;
	uint8_t eth_bca[ETHER_ADDR_LEN];
	dpaa2_cmd_t cmd = sc->cmd;
	uint16_t rc_token = sc->rc_token;
	uint16_t ni_token = sc->ni_token;
	uint32_t link;
	int error;

	/* Check if we can work with this DPNI object. */
	error = DPAA2_CMD_NI_GET_API_VERSION(dev, dpaa2_mcp_tk(cmd, ni_token),
	    &sc->api_major, &sc->api_minor);
	if (error) {
		device_printf(dev, "Failed to get DPNI API version\n");
		return (error);
	}
	if (cmp_api_version(sc, DPNI_VER_MAJOR, DPNI_VER_MINOR) < 0) {
		device_printf(dev, "DPNI API version %u.%u not supported, "
		    "need >= %u.%u\n", sc->api_major, sc->api_minor,
		    DPNI_VER_MAJOR, DPNI_VER_MINOR);
		error = ENODEV;
		return (error);
	}

	/* Reset the DPNI object. */
	error = DPAA2_CMD_NI_RESET(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to reset DPNI: id=%d\n", dinfo->id);
		return (error);
	}

	/* Obtain attributes of the DPNI object. */
	error = DPAA2_CMD_NI_GET_ATTRIBUTES(dev, cmd, &sc->attr);
	if (error) {
		device_printf(dev, "Failed to obtain DPNI attributes: id=%d\n",
		    dinfo->id);
		return (error);
	}
	if (bootverbose) {
		device_printf(dev, "options=0x%#x queues=%d tx_channels=%d "
		    "wriop_version=%#x\n", sc->attr.options, sc->attr.num.queues,
		    sc->attr.num.channels, sc->attr.wriop_ver);
		device_printf(dev, "\ttraffic classes: rx=%d tx=%d "
		    "cgs_groups=%d\n", sc->attr.num.rx_tcs, sc->attr.num.tx_tcs,
		    sc->attr.num.cgs);
		device_printf(dev, "\ttable entries: mac=%d vlan=%d qos=%d "
		    "fs=%d\n", sc->attr.entries.mac, sc->attr.entries.vlan,
		    sc->attr.entries.qos, sc->attr.entries.fs);
		device_printf(dev, "\tkey sizes: qos=%d fs=%d\n",
		    sc->attr.key_size.qos, sc->attr.key_size.fs);
	}

	/* Configure buffer layouts of the DPNI queues. */
	error = set_buf_layout(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to configure buffer layout\n");
		return (error);
	}

	/* Attach miibus and PHY in case of DPNI<->DPMAC. */
	ep1_desc.obj_id = dinfo->id;
	ep1_desc.if_id = 0; /* DPNI has the only endpoint */
	ep1_desc.type = dinfo->dtype;

	error = DPAA2_CMD_RC_GET_CONN(dev, dpaa2_mcp_tk(cmd, rc_token),
	    &ep1_desc, &ep2_desc, &link);
	if (error)
		device_printf(dev, "Failed to obtain an object DPNI is "
		    "connected to: error=%d\n", error);
	else {
		device_printf(dev, "connected to %s (id=%d)\n",
		    dpaa2_ttos(ep2_desc.type), ep2_desc.obj_id);

		if (ep2_desc.type == DPAA2_DEV_MAC) {
			/*
			 * This is the simplest case when DPNI is connected to
			 * DPMAC directly. Let's attach mdio/miibus then.
			 */
			sc->mac.dpmac_id = ep2_desc.obj_id;

			error = set_mac_addr(dev, cmd, rc_token, ni_token);
			if (error)
				device_printf(dev, "Failed to set MAC address: "
				    "error=%d\n", error);

			error = DPAA2_MC_GET_PHY_DEV(dev, &sc->mac.phy_dev,
			    sc->mac.dpmac_id);
			if (error == 0) {
#if 0
				device_printf(dev, "MAC PHY device is '%s'\n",
				    device_get_nameunit(sc->mac.phy_dev));
#endif
				error = mii_attach(sc->mac.phy_dev,
				    &sc->miibus, sc->ifp,
				    dpni_ifmedia_change, dpni_ifmedia_status,
				    BMSR_DEFCAPMASK, MII_PHY_ANY, 0, 0);
				if (error != 0)
					device_printf(dev, "Failed to attach "
					    "miibus: error=%d\n", error);
				else
					sc->mii = device_get_softc(sc->miibus);
			} else
				device_printf(dev, "Failed to obtain PHY "
				    "device: error=%d, dpmac_id=%d, "
				    "mac_addr=%6D\n", error, sc->mac.dpmac_id,
				    sc->mac.addr, ":");
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
		device_printf(dev, "Failed to configure Rx/Tx pause frames:\n");
		return (error);
	}

	/* Configure ingress traffic classification. */
	error = set_qos_table(dev, dpaa2_mcp_tk(cmd, ni_token));
	if (error)
		device_printf(dev, "Failed to configure QoS table: error=%d\n",
		    error);

	/* Add broadcast physical address to the MAC filtering table. */
	memset(eth_bca, 0xff, ETHER_ADDR_LEN);
	error = DPAA2_CMD_NI_ADD_MAC_ADDR(dev, cmd, eth_bca);
	if (error) {
		device_printf(dev, "Failed to add broadcast physical address to "
		    "the MAC filtering table\n");
		return (error);
	}

	/* Set the maximum allowed length for received frames. */
	error = DPAA2_CMD_NI_SET_MFL(dev, cmd, DPAA2_ETH_MFL);
	if (error) {
		device_printf(dev, "Failed to set maximum length for received "
		    "frames\n");
		return (error);
	}

	return (0);
}

/**
 * @internal
 * @brief Сonfigure QBMan channels and register data availability notifications.
 */
static int
setup_channels(device_t dev)
{
	device_t io_dev, con_dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_io_softc *iosc;
	struct dpaa2_con_softc *consc;
	struct dpaa2_devinfo *io_info, *con_info;
	dpaa2_ni_channel_t *channel;
	dpaa2_io_notif_ctx_t *ctx;
	dpaa2_con_notif_cfg_t notif_cfg;
	dpaa2_cmd_t cmd = sc->cmd;
	uint16_t con_token, rc_token = sc->rc_token;
	int error;

	/* Calculate a number of channels based on the allocated resources. */
	sc->num_chan = calc_channels_num(sc);

	/* Allocate no more channels than DPNI queues. */
	sc->num_chan = sc->num_chan > sc->attr.num.queues
	    ? sc->attr.num.queues : sc->num_chan;

	if (bootverbose)
		device_printf(dev, "channels=%d\n", sc->num_chan);

	/* DMA tag to allocate buffers for buffer pool. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    sc->rx_buf_align, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* low restricted addr */
	    BUS_SPACE_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    ETH_RX_BUF_RAW_SIZE, 1,	/* maxsize, nsegments */
	    ETH_RX_BUF_RAW_SIZE, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->bp_dtag);
	if (error) {
		device_printf(dev, "Failed to create buffer pool DMA tag\n");
		return (error);
	}

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
		channel->buf_num = 0;

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
			device_printf(dev, "Failed to register notification\n");
			return (error);
		}

		/* Open data path concentrator object. */
		error = DPAA2_CMD_CON_OPEN(dev, dpaa2_mcp_tk(cmd, rc_token),
		    con_info->id, &con_token);
		if (error) {
			device_printf(dev, "Failed to open DPCON: id=%d\n",
			    con_info->id);
			return (error);
		}

		/* Register DPCON notification with MC. */
		notif_cfg.dpio_id = io_info->id;
		notif_cfg.prior = 0;
		notif_cfg.qman_ctx = ctx->qman_ctx;
		error = DPAA2_CMD_CON_SET_NOTIF(dev, cmd, &notif_cfg);
		if (error) {
			device_printf(dev, "Failed to set DPCON notification: "
			    "id=%d\n", con_info->id);
			DPAA2_CMD_CON_CLOSE(dev, dpaa2_mcp_tk(cmd, con_token));
			return (error);
		}

		/* Close data path concentrator object. */
		error = DPAA2_CMD_CON_CLOSE(dev, dpaa2_mcp_tk(cmd, con_token));
		if (error)
			device_printf(dev, "Failed to close DPCON: id=%d, "
			    "error=%d\n", con_info->id, error);

		/* Allocate and map buffers for the buffer pool. */
		error = seed_buf_pool(sc, channel);
		if (error) {
			device_printf(dev, "Failed to seed buffer pool.\n");
			return (error);
		}

		if (bootverbose)
			device_printf(dev, "channel: dpio_id=%d dpcon_id=%d "
			    "chan_id=%d, priorities=%d\n", io_info->id,
			    con_info->id, channel->id, consc->attr.prior_num);
	}
	/* TODO: De-allocate redundant DPIOs or DPCONs if exist. */
	return (0);
}

/**
 * @internal
 * @brief Сonfigure frame queues.
 */
static int
setup_frame_queues(device_t dev)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	uint32_t txc_fqs, rx_fqs, rx_err_fqs;
	int i, j;

	sc->num_fqs = 0;

	/* Tx-conf queues. */
	for (i = 0; i < sc->num_chan; i++) {
		sc->fq[sc->num_fqs].type = DPAA2_NI_QUEUE_TX_CONF;
		sc->fq[sc->num_fqs].flowid = (uint16_t) i;
		sc->fq[sc->num_fqs].consume = dpni_consume_tx_conf;
		sc->num_fqs++;
	}
	txc_fqs = sc->num_chan;

	/* Rx queues. */
	for (j = 0; j < sc->attr.num.rx_tcs; j++) {
		for (i = 0; i < sc->num_chan; i++) {
			sc->fq[sc->num_fqs].type = DPAA2_NI_QUEUE_RX;
			sc->fq[sc->num_fqs].tc = (uint8_t) j;
			sc->fq[sc->num_fqs].flowid = (uint16_t) i;
			sc->fq[sc->num_fqs].consume = dpni_consume_rx;
			sc->num_fqs++;
		}
	}
	rx_fqs = sc->attr.num.rx_tcs * sc->num_chan;

	/* There is exactly one Rx error queue per DPNI. */
	sc->fq[sc->num_fqs].type = DPAA2_NI_QUEUE_RX_ERR;
	sc->fq[sc->num_fqs].tc = 0; /* ignored */
	sc->fq[sc->num_fqs].flowid = 0; /* ignored */
	sc->fq[sc->num_fqs].consume = dpni_consume_rx_err;
	sc->num_fqs++;
	rx_err_fqs = 1;

	/* Assign a channel for each FQ. */
	for (i = 0, j = 0; i < sc->num_fqs; i++) {
		j = (j == sc->num_chan) ? 0 : j;
		sc->fq[i].channel = sc->channel[j++];
	}

	if (bootverbose)
		device_printf(dev, "frame queues: tx_conf=%d rx=%d rx_err=%d\n",
		    txc_fqs, rx_fqs, rx_err_fqs);

	return (0);
}

/**
 * @internal
 * @brief Bind DPNI to DPBPs, DPIOs, frame queues and channels.
 */
static int
setup_dpni_binding(device_t dev)
{
	device_t bp_dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *bp_info;
	dpaa2_cmd_t cmd = sc->cmd;
	uint16_t ni_token = sc->ni_token;
	dpaa2_ni_pools_cfg_t pools_cfg;
	dpaa2_ni_err_cfg_t err_cfg;
	int error;

	bp_dev = (device_t) rman_get_start(sc->res[BP_RID(0)]);
	bp_info = device_get_ivars(bp_dev);

	/* Configure buffers pool. */
	pools_cfg.pools_num = 1;
	pools_cfg.pools[0].bp_obj_id = bp_info->id;
	pools_cfg.pools[0].backup_flag = 0;
	pools_cfg.pools[0].buf_sz = sc->rx_bufsz;
	error = DPAA2_CMD_NI_SET_POOLS(dev, dpaa2_mcp_tk(cmd, ni_token),
	    &pools_cfg);
	if (error) {
		device_printf(dev, "Failed to set buffer pools\n");
		return (error);
	}

	/* Setup ingress traffic distribution. */
	error = setup_rx_distribution(dev);
	if (error && error != EOPNOTSUPP) {
		device_printf(dev, "Failed to setup ingress traffic "
		    "distribution\n");
		return (error);
	}
	if (bootverbose && error == EOPNOTSUPP)
		device_printf(dev, "Ingress traffic distribution not "
		    "supported\n");

	/* Configure handling of error frames. */
	err_cfg.err_mask = DPAA2_NI_FAS_RX_ERR_MASK;
	err_cfg.set_err_fas = false;
	err_cfg.action = DPAA2_NI_ERR_DISCARD;
	error = DPAA2_CMD_NI_SET_ERR_BEHAVIOR(dev, cmd, &err_cfg);
	if (error) {
		device_printf(dev, "Failed to set errors behavior\n");
		return (error);
	}

	/* Configure Rx and Tx-confirmation queues to generate CDANs. */
	for (int i = 0; i < sc->num_fqs; i++) {
		switch (sc->fq[i].type) {
		case DPAA2_NI_QUEUE_RX:
			error = setup_rx_flow(dev, cmd, &sc->fq[i]);
			break;
		case DPAA2_NI_QUEUE_TX_CONF:
			error = setup_tx_flow(dev, cmd, &sc->fq[i]);
			break;
		case DPAA2_NI_QUEUE_RX_ERR:
			error = setup_rx_err_flow(dev, cmd, &sc->fq[i]);
			break;
		default:
			device_printf(dev, "Invalid FQ type %d\n",
			    sc->fq[i].type);
			return (EINVAL);
		}
		if (error)
			return (error);
	}

	/*
	 * Get the Queuing Destination ID (QDID) that should be used for frame
	 * enqueue operations.
	 */
	error = DPAA2_CMD_NI_GET_QDID(dev, cmd, DPAA2_NI_QUEUE_TX, &sc->tx_qdid);
	if (error) {
		device_printf(dev, "Failed to get Tx queuing destination ID\n");
		return (error);
	}

	return (0);
}

/**
 * @internal
 * @brief Setup ingress traffic distribution.
 *
 * NOTE: Ingress traffic distribution is valid only when DPNI_OPT_NO_FS option
 *	 hasn't been set for DPNI and a number of DPNI queues > 1.
 */
static int
setup_rx_distribution(device_t dev)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	int error;

	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    PAGE_SIZE, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* low restricted addr */
	    BUS_SPACE_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    DPAA2_CLASSIFIER_DMA_SIZE, 1, /* maxsize, nsegments */
	    DPAA2_CLASSIFIER_DMA_SIZE, 0, /* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rx_dist_kcfg.dtag);
	if (error) {
		device_printf(dev, "Failed to create a DMA tag for Rx traffic "
		    "distribution key configuration buffer\n");
		return (error);
	}

	/*
	 * Have the interface implicitly distribute traffic based on the default
	 * hash key.
	 */
	return (dpaa2_eth_set_hash(sc, DPAA2_RXH_DEFAULT));
}

/**
 * @internal
 */
static int
setup_rx_flow(device_t dev, dpaa2_cmd_t cmd, dpaa2_ni_fq_t *fq)
{
	struct dpaa2_devinfo *con_info;
	dpaa2_ni_queue_cfg_t queue_cfg = {0};
	int error;

	/* Obtain DPCON associated with the FQ's channel. */
	con_info = device_get_ivars(fq->channel->con_dev);

	queue_cfg.type = DPAA2_NI_QUEUE_RX;
	queue_cfg.tc = fq->tc;
	queue_cfg.idx = fq->flowid;
	error = DPAA2_CMD_NI_GET_QUEUE(dev, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "Failed to obtain Rx queue configuration: "
		    "tc=%d, flowid=%d\n", queue_cfg.tc, queue_cfg.idx);
		return (error);
	}

	fq->fqid = queue_cfg.fqid;

	if (bootverbose)
		device_printf(dev, "Rx queue: tc=%d, flowid=%d, fqid=%d, "
		    "dpcon_id=%d\n", queue_cfg.tc, queue_cfg.idx, queue_cfg.fqid,
		    con_info->id);

	queue_cfg.dest_id = con_info->id;
	queue_cfg.dest_type = DPAA2_NI_DEST_DPCON;
	queue_cfg.priority = 0;
	queue_cfg.user_ctx = (uint64_t)(uintmax_t) fq;
	queue_cfg.options = DPAA2_NI_QUEUE_OPT_USER_CTX |
	    DPAA2_NI_QUEUE_OPT_DEST;
	error = DPAA2_CMD_NI_SET_QUEUE(dev, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "Failed to update Rx queue configuration: "
		    "tc=%d, flowid=%d\n", queue_cfg.tc, queue_cfg.idx);
		return (error);
	}

	return (0);
}

/**
 * @internal
 */
static int
setup_tx_flow(device_t dev, dpaa2_cmd_t cmd, dpaa2_ni_fq_t *fq)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *con_info;
	dpaa2_ni_queue_cfg_t queue_cfg = {0};
	int error;

	/* Obtain DPCON associated with the FQ's channel. */
	con_info = device_get_ivars(fq->channel->con_dev);

	for (int i = 0; i < sc->attr.num.tx_tcs; i++) {
		queue_cfg.type = DPAA2_NI_QUEUE_TX;
		queue_cfg.tc = i;
		queue_cfg.idx = fq->flowid;
		error = DPAA2_CMD_NI_GET_QUEUE(dev, cmd, &queue_cfg);
		if (error) {
			device_printf(dev, "Failed to obtain Tx queue "
			    "configuration: tc=%d, flowid=%d\n", queue_cfg.tc,
			    queue_cfg.idx);
			return (error);
		}
		fq->tx_fqid[i] = queue_cfg.fqid;

		if (bootverbose)
			device_printf(dev, "Tx queue: tc=%d, flowid=%d, fqid=%d, "
			    "dpcon_id=%d\n", queue_cfg.tc, queue_cfg.idx,
			    queue_cfg.fqid, con_info->id);
	}

	/* All Tx queues which belong to the same flowid have the same qdbin. */
	fq->tx_qdbin = queue_cfg.qdbin;

	queue_cfg.type = DPAA2_NI_QUEUE_TX_CONF;
	queue_cfg.tc = 0; /* ignored for TxConf queue */
	queue_cfg.idx = fq->flowid;
	error = DPAA2_CMD_NI_GET_QUEUE(dev, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "Failed to obtain TxConf queue "
		    "configuration: tc=%d, flowid=%d\n", queue_cfg.tc,
		    queue_cfg.idx);
		return (error);
	}

	fq->fqid = queue_cfg.fqid;

	if (bootverbose)
		device_printf(dev, "TxConf queue: tc=%d, flowid=%d, fqid=%d, "
		    "dpcon_id=%d\n", queue_cfg.tc, queue_cfg.idx,
		    queue_cfg.fqid, con_info->id);

	queue_cfg.dest_id = con_info->id;
	queue_cfg.dest_type = DPAA2_NI_DEST_DPCON;
	queue_cfg.priority = 0;
	queue_cfg.user_ctx = (uint64_t)(uintmax_t) fq;
	queue_cfg.options = DPAA2_NI_QUEUE_OPT_USER_CTX |
	    DPAA2_NI_QUEUE_OPT_DEST;
	error = DPAA2_CMD_NI_SET_QUEUE(dev, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "Failed to update TxConf queue "
		    "configuration: tc=%d, flowid=%d\n", queue_cfg.tc,
		    queue_cfg.idx);
		return (error);
	}

	return (0);
}

/**
 * @internal
 */
static int
setup_rx_err_flow(device_t dev, dpaa2_cmd_t cmd, dpaa2_ni_fq_t *fq)
{
	struct dpaa2_devinfo *con_info;
	dpaa2_ni_queue_cfg_t queue_cfg = {0};
	int error;

	/* Obtain DPCON associated with the FQ's channel. */
	con_info = device_get_ivars(fq->channel->con_dev);

	queue_cfg.type = DPAA2_NI_QUEUE_RX_ERR;
	queue_cfg.tc = fq->tc; /* ignored */
	queue_cfg.idx = fq->flowid; /* ignored */
	error = DPAA2_CMD_NI_GET_QUEUE(dev, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "Failed to obtain RxErr queue "
		    "configuration\n");
		return (error);
	}

	fq->fqid = queue_cfg.fqid;

	if (bootverbose)
		device_printf(dev, "RxErr queue: tc=%d, flowid=%d, fqid=%d, "
		    "dpcon_id=%d\n", queue_cfg.tc, queue_cfg.idx,
		    queue_cfg.fqid, con_info->id);

	queue_cfg.dest_id = con_info->id;
	queue_cfg.dest_type = DPAA2_NI_DEST_DPCON;
	queue_cfg.priority = 0;
	queue_cfg.user_ctx = (uint64_t)(uintmax_t) fq;
	queue_cfg.options = DPAA2_NI_QUEUE_OPT_USER_CTX |
	    DPAA2_NI_QUEUE_OPT_DEST;
	error = DPAA2_CMD_NI_SET_QUEUE(dev, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "Failed to update RxErr queue "
		    "configuration\n");
		return (error);
	}

	return (0);
}

/**
 * @internal
 * @brief Configure DPNI object to generate interrupts.
 */
static int
setup_dpni_irqs(device_t dev)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	dpaa2_cmd_t cmd = sc->cmd;
	uint16_t ni_token = sc->ni_token;
	int error;

	/* Configure IRQs. */
	error = setup_msi(sc);
	if (error) {
		device_printf(dev, "Failed to allocate MSI\n");
		return (error);
	}
	if ((sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid[0], RF_ACTIVE | RF_SHAREABLE)) == NULL) {
		device_printf(dev, "Failed to allocate IRQ resource\n");
		return (ENXIO);
	}
	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, dpni_msi_intr, sc, &sc->intr)) {
		device_printf(dev, "Failed to setup IRQ resource\n");
		return (ENXIO);
	}

	/* Configure DPNI to generate interrupts. */
	error = DPAA2_CMD_NI_SET_IRQ_MASK(dev, dpaa2_mcp_tk(cmd, ni_token),
	    DPNI_IRQ_INDEX, DPNI_IRQ_LINK_CHANGED | DPNI_IRQ_EP_CHANGED);
	if (error) {
		device_printf(dev, "Failed to set DPNI IRQ mask\n");
		return (error);
	}

	/* Enable IRQ. */
	error = DPAA2_CMD_NI_SET_IRQ_ENABLE(dev, cmd, DPNI_IRQ_INDEX, true);
	if (error) {
		device_printf(dev, "Failed to enable DPNI IRQ\n");
		return (error);
	}

	return (0);
}

/**
 * @internal
 * @brief Allocate MSI interrupts for DPNI.
 */
static int
setup_msi(struct dpaa2_ni_softc *sc)
{
	int val;

	val = pci_msi_count(sc->dev);
	if (val < DPAA2_NI_MSI_COUNT)
		device_printf(sc->dev, "MSI: actual=%d, expected=%d\n", val,
		    DPAA2_IO_MSI_COUNT);
	val = MIN(val, DPAA2_NI_MSI_COUNT);

	if (pci_alloc_msi(sc->dev, &val) != 0)
		return (EINVAL);

	for (int i = 0; i < val; i++)
		sc->irq_rid[i] = i + 1;

	return (0);
}

/**
 * @internal
 * @brief Update DPNI according to the updated interface capabilities.
 */
static int
setup_if_caps(struct dpaa2_ni_softc *sc)
{
	const bool en_rxcsum = sc->ifp->if_capenable & IFCAP_RXCSUM;
	const bool en_txcsum = sc->ifp->if_capenable & IFCAP_TXCSUM;
	device_t dev = sc->dev;
	int error;

	/* Setup checksums validation. */
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token), DPAA2_NI_OFL_RX_L3_CSUM, en_rxcsum);
	if (error) {
		device_printf(dev, "Failed to %s L3 checksum validation\n",
		    en_rxcsum ? "enable" : "disable");
		return (error);
	}
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, sc->cmd, DPAA2_NI_OFL_RX_L4_CSUM,
	    en_rxcsum);
	if (error) {
		device_printf(dev, "Failed to %s L4 checksum validation\n",
		    en_rxcsum ? "enable" : "disable");
		return (error);
	}

	/* Setup checksums generation. */
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, sc->cmd, DPAA2_NI_OFL_TX_L3_CSUM,
	    en_txcsum);
	if (error) {
		device_printf(dev, "Failed to %s L3 checksum generation\n",
		    en_txcsum ? "enable" : "disable");
		return (error);
	}
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, sc->cmd, DPAA2_NI_OFL_TX_L4_CSUM,
	    en_txcsum);
	if (error) {
		device_printf(dev, "Failed to %s L4 checksum generation\n",
		    en_txcsum ? "enable" : "disable");
		return (error);
	}

	return (0);
}

/**
 * @internal
 * @brief Update DPNI according to the updated interface flags.
 */
static int
setup_if_flags(struct dpaa2_ni_softc *sc)
{
	const bool en_promisc = sc->ifp->if_flags & IFF_PROMISC;
	const bool en_allmulti = sc->ifp->if_flags & IFF_ALLMULTI;
	device_t dev = sc->dev;
	int error;

	if (bootverbose)
		device_printf(sc->dev, "promisc=%s, allmulti=%s\n",
		    en_promisc ? "TRUE" : "FALSE",
		    en_allmulti ? "TRUE" : "FALSE");

	error = DPAA2_CMD_NI_SET_MULTI_PROMISC(dev, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token), en_promisc ? true : en_allmulti);
	if (error) {
		device_printf(dev, "Failed to %s multicast promiscuous mode\n",
		    en_allmulti ? "enable" : "disable");
		return (error);
	}

	error = DPAA2_CMD_NI_SET_UNI_PROMISC(dev, sc->cmd, en_promisc);
	if (error) {
		device_printf(dev, "Failed to %s unicast promiscuous mode\n",
		    en_promisc ? "enable" : "disable");
		return (error);
	}

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
	int error;

	/*
	 * Select RX buffer alignment. It's necessary to ensure that the buffer
	 * size seen by WRIOP is a multiple of 64 or 256 bytes depending on the
	 * WRIOP version.
	 */
	sc->rx_buf_align = (sc->attr.wriop_ver == WRIOP_VERSION(0, 0, 0) ||
	    sc->attr.wriop_ver == WRIOP_VERSION(1, 0, 0))
	    ? ETH_RX_BUF_ALIGN_V1 : ETH_RX_BUF_ALIGN;

	/*
	 * We need to ensure that the buffer size seen by WRIOP is a multiple
	 * of 64 or 256 bytes depending on the WRIOP version.
	 */
	sc->rx_bufsz = ALIGN_DOWN(ETH_RX_BUF_SIZE, sc->rx_buf_align);
	if (bootverbose)
		device_printf(dev, "RX buffer: size=%d, alignment=%d\n",
		    sc->rx_bufsz, sc->rx_buf_align);

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
		device_printf(dev, "TX data offset (%d) is not a multiplication "
		    "of 64 bytes\n", sc->tx_data_off);

	/* RX buffer */
	/*
	 * Extra headroom space requested to hardware, in order to make sure
	 * there's no realloc'ing in forwarding scenarios.
	 */
	buf_layout.queue_type = DPAA2_NI_QUEUE_RX;
	buf_layout.head_size = sc->tx_data_off - ETH_RX_HWA_SIZE;
	buf_layout.fd_align = sc->rx_buf_align;
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

	sc->link_options = link_cfg.options;

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
			device_printf(dev, "Ingress traffic classification is "
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

	error = bus_dmamem_alloc(sc->qos_kcfg.dtag,
	    (void **) &sc->qos_kcfg.buf_va, BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->qos_kcfg.dmap);
	if (error) {
		device_printf(dev, "Failed to allocate a buffer for QoS key "
		    "configuration\n");
		return (error);
	}

	error = bus_dmamap_load(sc->qos_kcfg.dtag, sc->qos_kcfg.dmap,
	    sc->qos_kcfg.buf_va, ETH_QOS_KCFG_BUF_SIZE,
	    dpni_single_seg_dmamap_cb, &sc->qos_kcfg.buf_pa, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(dev, "Failed to map QoS key configuration buffer "
		    "into bus space\n");
		return (error);
	}

	tbl.default_tc = 0;
	tbl.discard_on_miss = false;
	tbl.keep_entries = false;
	tbl.kcfg_busaddr = sc->qos_kcfg.buf_pa;
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

static int
set_mac_addr(device_t dev, dpaa2_cmd_t cmd, uint16_t rc_token, uint16_t ni_token)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct ifnet *ifp = sc->ifp;
	struct ether_addr rnd_mac_addr;
	uint8_t mac_addr[ETHER_ADDR_LEN];
	uint8_t dpni_mac_addr[ETHER_ADDR_LEN];
	int error;

	/*
	 * Get the MAC address associated with the physical port, if the DPNI is
	 * connected to a DPMAC directly associated with one of the physical
	 * ports.
	 */
	error = DPAA2_CMD_NI_GET_PORT_MAC_ADDR(dev, dpaa2_mcp_tk(cmd, ni_token),
	    mac_addr);
	if (error) {
		device_printf(dev, "Failed to obtain the MAC address "
		    "associated with the physical port\n");
		return (error);
	}

	/* Get primary MAC address from the DPNI attributes. */
	error = DPAA2_CMD_NI_GET_PRIM_MAC_ADDR(dev, cmd, dpni_mac_addr);
	if (error) {
		device_printf(dev, "Failed to obtain primary MAC address\n");
		return (error);
	}

	if (!ETHER_IS_ZERO(mac_addr)) {
		/* Set MAC address of the physical port as DPNI's primary one. */
		error = DPAA2_CMD_NI_SET_PRIM_MAC_ADDR(dev, cmd, mac_addr);
		if (error) {
			device_printf(dev, "Failed to set primary MAC "
			    "address\n");
			return (error);
		}
		for (int i = 0; i < ETHER_ADDR_LEN; i++)
			sc->mac.addr[i] = mac_addr[i];
	} else if (ETHER_IS_ZERO(dpni_mac_addr)) {
		/* Generate random MAC address as DPNI's primary one. */
		ether_gen_addr(ifp, &rnd_mac_addr);
		for (int i = 0; i < ETHER_ADDR_LEN; i++)
			mac_addr[i] = rnd_mac_addr.octet[i];

		error = DPAA2_CMD_NI_SET_PRIM_MAC_ADDR(dev, cmd, mac_addr);
		if (error) {
			device_printf(dev, "Failed to set random primary MAC "
			    "address\n");
			return (error);
		}
		for (int i = 0; i < ETHER_ADDR_LEN; i++)
			sc->mac.addr[i] = mac_addr[i];
	} else {
		for (int i = 0; i < ETHER_ADDR_LEN; i++)
			sc->mac.addr[i] = dpni_mac_addr[i];
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
	dpaa2_mac_link_state_t mac_link = {0};
	uint16_t mac_token;
	int link_state = ifp->if_link_state;
	int error;

	DPNI_LOCK(sc);
	if (sc->mii) {
		mii_pollstat(sc->mii);
		ifmr->ifm_active = sc->mii->mii_media_active;
		ifmr->ifm_status = sc->mii->mii_media_status;
	}
	DPNI_UNLOCK(sc);

	if (link_state != sc->link_state) {
		sc->link_state = link_state;

		error = DPAA2_CMD_MAC_OPEN(sc->dev, dpaa2_mcp_tk(sc->cmd,
		    sc->rc_token), sc->mac.dpmac_id, &mac_token);
		if (error) {
			device_printf(sc->dev, "Failed to open DPMAC: id=%d, "
			    "error=%d\n", sc->mac.dpmac_id, error);
			goto err_exit;
		}

		if (link_state == LINK_STATE_UP ||
		    link_state == LINK_STATE_DOWN) {
			/* Update DPMAC link state. */
			mac_link.supported = sc->mii->mii_media.ifm_media;
			mac_link.advert = sc->mii->mii_media.ifm_media;
			mac_link.rate = 1000;
			mac_link.options =
			    DPAA2_MAC_LINK_OPT_AUTONEG |
			    DPAA2_MAC_LINK_OPT_PAUSE;
			mac_link.up = link_state == LINK_STATE_UP ? true : false;
			mac_link.state_valid = true;

			/* Inform DPMAC about link state. */
			error = DPAA2_CMD_MAC_SET_LINK_STATE(sc->dev, sc->cmd,
			    &mac_link);
			if (error) {
				device_printf(sc->dev, "Failed to set DPMAC "
				    "link state: id=%d, error=%d\n",
				    sc->mac.dpmac_id, error);
				goto err_close_mac;
			}
		}
		DPAA2_CMD_MAC_CLOSE(sc->dev, dpaa2_mcp_tk(sc->cmd, mac_token));
	}
	return;

 err_close_mac:
	DPAA2_CMD_MAC_CLOSE(sc->dev, dpaa2_mcp_tk(sc->cmd, mac_token));
 err_exit:
	return;
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
	device_t dev = sc->dev;
	int error;

	DPNI_LOCK(sc);
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0) {
		DPNI_UNLOCK(sc);
		return;
	}
	DPNI_UNLOCK(sc);

	error = DPAA2_CMD_NI_ENABLE(dev, dpaa2_mcp_tk(sc->cmd, sc->ni_token));
	if (error)
		device_printf(dev, "Failed to enable DPNI: error=%d\n", error);

	DPNI_LOCK(sc);
	if (sc->mii)
		mii_mediachg(sc->mii);
	callout_reset(&sc->mii_callout, hz, dpni_ifmedia_tick, sc);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	DPNI_UNLOCK(sc);

	return;
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
dpni_if_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;
	struct ifreq *ifr = (struct ifreq *) data;
	uint32_t changed = 0;
	int rc = 0;

	switch (cmd) {
	case SIOCSIFMTU:
		if (ifr->ifr_mtu < ETHERMIN || ifr->ifr_mtu > DPAA2_ETH_MTU)
			return (EINVAL);

		/* TODO: Update max. frame length according to the new MTU. */
		ifp->if_mtu = ifr->ifr_mtu;
		break;
	case SIOCSIFCAP:
		changed = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (changed & IFCAP_HWCSUM) {
			if ((ifr->ifr_reqcap & changed) & IFCAP_HWCSUM)
				ifp->if_capenable |= IFCAP_HWCSUM;
			else
				ifp->if_capenable &= ~IFCAP_HWCSUM;
		}
		rc = setup_if_caps(sc);
		if (rc) {
			printf("%s: Failed to update iface capabilities: "
			    "error=%d\n", __func__, rc);
			rc = ENXIO;
		}
		break;
	case SIOCSIFFLAGS:
		DPNI_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				changed = ifp->if_flags ^ sc->if_flags;

				if (bootverbose)
					device_printf(sc->dev, "SIOCSIFFLAGS: "
					    "up and running (changed=0x%x)\n",
					    changed);

				if (changed & IFF_PROMISC ||
				    changed & IFF_ALLMULTI) {
					rc = setup_if_flags(sc);
				}
				print_statistics(sc);
			} else {
				DPNI_UNLOCK(sc);
				if (bootverbose) {
					printf("%s: SIOCSIFFLAGS: starting up\n",
					    __func__);
				}
				dpni_if_init(sc);
				DPNI_LOCK(sc);
			}
		} else if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			if (bootverbose)
				printf("%s: SIOCSIFFLAGS: shutting down\n",
				    __func__);
			/* dpni_if_stop(sc); */
		}

		sc->if_flags = ifp->if_flags;
		DPNI_UNLOCK(sc);
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
			rc= ifmedia_ioctl(ifp, ifr, &sc->mii->mii_media, cmd);
		break;
	default:
		rc = ether_ioctl(ifp, cmd, data);
	}

	return (rc);
}

/**
 * @internal
 */
static void
dpni_msi_intr(void *arg)
{
	struct dpaa2_ni_softc *sc = (struct dpaa2_ni_softc *) arg;
	uint32_t status = ~0u; /* clear all IRQ status bits */
	int error;

	error = DPAA2_CMD_NI_GET_IRQ_STATUS(sc->dev, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token), DPNI_IRQ_INDEX, &status);
	if (error) {
		device_printf(sc->dev, "Failed to obtain IRQ status: error=%d\n",
		    error);
		return;
	}

	if (status & DPNI_IRQ_LINK_CHANGED)
		printf("%s: link state changed\n", __func__);

	if (status & DPNI_IRQ_EP_CHANGED)
		printf("%s: endpoint changed\n", __func__);
}

/**
 * @internal
 * @brief Channel data availability notification (CDAN) callback.
 */
static void
dpni_cdan_cb(dpaa2_io_notif_ctx_t *ctx)
{
	/* TBD */
	printf("%s: invoked\n", __func__);
}

/**
 * @internal
 * @brief Callback to obtain a physical address of the only DMA mapped segment.
 */
static void
dpni_single_seg_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg,
    int error)
{
	if (error)
		return;
	*(bus_addr_t *) arg = segs[0].ds_addr;
}

/**
 * @internal
 */
static void
dpni_consume_tx_conf(device_t dev, dpaa2_ni_channel_t *channel,
    struct dpaa2_ni_fq *fq, const dpaa2_fd_t *fd)
{
	/* TBD */
	printf("%s: invoked\n", __func__);
}

/**
 * @internal
 */
static void
dpni_consume_rx(device_t dev, dpaa2_ni_channel_t *channel,
    struct dpaa2_ni_fq *fq, const dpaa2_fd_t *fd)
{
	/* TBD */
	printf("%s: invoked\n", __func__);
}

/**
 * @internal
 */
static void
dpni_consume_rx_err(device_t dev, dpaa2_ni_channel_t *channel,
    struct dpaa2_ni_fq *fq, const dpaa2_fd_t *fd)
{
	/* TBD */
	printf("%s: invoked\n", __func__);
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

	/* Number of the allocated DPIOs. */
	for (i = 0; i < IO_RES_NUM; i++)
		if (!sc->res[IO_RID(i)])
			break;
	num_chan = i;

	/* Number of the allocated DPCONs. */
	for (i = 0; i < CON_RES_NUM; i++)
		if (!sc->res[CON_RID(i)])
			break;
	num_chan = i < num_chan ? i : num_chan;

	return (num_chan > DPAA2_NI_MAX_CHANNELS
	    ? DPAA2_NI_MAX_CHANNELS : num_chan);
}

/**
 * @internal
 * @brief Allocate buffers visible to QBMan and release them to the buffer pool.
 *
 * NOTE: DMA tag for the given channel should be created.
 */
static int
seed_buf_pool(struct dpaa2_ni_softc *sc, dpaa2_ni_channel_t *channel)
{
	device_t bp_dev;
	struct dpaa2_bp_softc *bpsc;
	dpaa2_ni_buf_t *buf;
	bus_addr_t paddr[DPAA2_SWP_BUFS_PER_CMD];
	int error, bufn;

	/* There's only one buffer pool for now. */
	bp_dev = (device_t) rman_get_start(sc->res[BP_RID(0)]);
	bpsc = device_get_softc(bp_dev);

	for (int i = 0; i < DPAA2_NI_BUFS_PER_CHAN; i += DPAA2_SWP_BUFS_PER_CMD) {
		/* Allocate enough buffers to release in one QBMan command. */
		for (int j = bufn = 0; j < DPAA2_SWP_BUFS_PER_CMD; j++) {
			buf = &channel->buf[i + j];

			error = bus_dmamem_alloc(sc->bp_dtag, &buf->vaddr,
			    BUS_DMA_ZERO | BUS_DMA_COHERENT, &buf->dmap);
			if (error) {
				device_printf(sc->dev, "Failed to allocate a "
				    "buffer for buffer pool\n");
				return (error);
			}

			error = bus_dmamap_load(sc->bp_dtag, buf->dmap,
			    buf->vaddr, ETH_RX_BUF_RAW_SIZE,
			    dpni_single_seg_dmamap_cb, &buf->paddr,
			    BUS_DMA_NOWAIT);
			if (error) {
				device_printf(sc->dev, "Failed to map a "
				    "buffer for buffer pool\n");
				return (error);
			}
			paddr[bufn] = buf->paddr;
			bufn++;
		}

		/* Release buffer to QBMan buffer pool. */
		error = DPAA2_SWP_RELEASE_BUFS(channel->io_dev, bpsc->attr.bpid,
		    paddr, bufn);
		if (error) {
			device_printf(sc->dev, "Failed to release buffers to "
			    "the buffer pool\n");
			return (error);
		}
	}

	return (0);
}

/**
 * @internal
 * @brief Print statistics of the network interface.
 */
static void
print_statistics(struct dpaa2_ni_softc *sc)
{
	device_t dev = sc->dev;
	uint64_t cnt[DPAA2_NI_STAT_COUNTERS];
	int error, pages = 3;

	for (int i = 0; i < pages; i++) {
		error = DPAA2_CMD_NI_GET_STATISTICS(dev,
		    dpaa2_mcp_tk(sc->cmd, sc->ni_token), i, 0, cnt);
		if (error) {
			device_printf(dev, "Failed to get statistics: page=%d, "
			    "error=%d\n", i, error);
			continue;
		}

		switch (i) {
		case 0:
			device_printf(dev, "INGRESS_ALL_FRAMES=%lu\n", cnt[0]);
			device_printf(dev, "INGRESS_ALL_BYTES=%lu\n", cnt[1]);
			device_printf(dev, "INGRESS_MULTICAST_FRAMES=%lu\n", cnt[2]);
			break;
		case 1:
			device_printf(dev, "EGRESS_ALL_FRAMES=%lu\n", cnt[0]);
			device_printf(dev, "EGRESS_ALL_BYTES=%lu\n", cnt[1]);
			device_printf(dev, "EGRESS_MULTICAST_FRAMES=%lu\n", cnt[2]);
			break;
		case 2:
			device_printf(dev, "INGRESS_FILTERED_FRAMES=%lu\n", cnt[0]);
			device_printf(dev, "INGRESS_DISCARDED_FRAMES=%lu\n", cnt[1]);
			device_printf(dev, "INGRESS_NOBUFFER_DISCARDS=%lu\n", cnt[2]);
			break;
		default:
			/* Other pages aren't interesting at the moment. */
			break;
		}
	}
}

/**
 * @internal
 */
static int
dpaa2_eth_set_hash(struct dpaa2_ni_softc *sc, uint64_t flags)
{
	uint64_t key = 0;
	int i;

	if (!(sc->attr.num.queues > 1))
		return (EOPNOTSUPP);

	for (i = 0; i < ARRAY_SIZE(dist_fields); i++)
		if (dist_fields[i].rxnfc_field & flags)
			key |= dist_fields[i].id;

	return dpaa2_eth_set_dist_key(sc, DPAA2_NI_DIST_MODE_HASH, key);
}

/**
 * @internal
 * @brief Set Rx distribution (hash or flow classification) key flags is a
 * combination of RXH_ bits.
 */
static int
dpaa2_eth_set_dist_key(struct dpaa2_ni_softc *sc, enum dpaa2_ni_dist_mode type,
    uint64_t flags)
{
	device_t dev = sc->dev;
	struct dpkg_profile_cfg cls_cfg;
	uint32_t rx_hash_fields = 0;
	int i, err = 0;

	memset(&cls_cfg, 0, sizeof(cls_cfg));

	/* Configure extracts according to the given flags. */
	for (i = 0; i < ARRAY_SIZE(dist_fields); i++) {
		struct dpkg_extract *key =
		    &cls_cfg.extracts[cls_cfg.num_extracts];

		if (!(flags & dist_fields[i].id))
			continue;
		if (type == DPAA2_NI_DIST_MODE_HASH)
			rx_hash_fields |= dist_fields[i].rxnfc_field;

		if (cls_cfg.num_extracts >= DPKG_MAX_NUM_OF_EXTRACTS) {
			device_printf(dev, "Failed to add key extraction rule "
			    "(too many rules?)\n");
			return (E2BIG);
		}

		key->type = DPKG_EXTRACT_FROM_HDR;
		key->extract.from_hdr.prot = dist_fields[i].cls_prot;
		key->extract.from_hdr.type = DPKG_FULL_FIELD;
		key->extract.from_hdr.field = dist_fields[i].cls_field;
		cls_cfg.num_extracts++;
	}

	err = bus_dmamem_alloc(sc->rx_dist_kcfg.dtag, &sc->rx_dist_kcfg.vaddr,
	    BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->rx_dist_kcfg.dmap);
	if (err) {
		device_printf(dev, "Failed to allocate a buffer for Rx traffic "
		    "distribution key configuration\n");
		return (err);
	}

	err = dpni_prepare_key_cfg(&cls_cfg, (uint8_t *) sc->rx_dist_kcfg.vaddr);
	if (err) {
		device_printf(dev, "dpni_prepare_key_cfg error %d\n", err);
		return (err);
	}

	/* Prepare for setting the Rx dist. */
	err = bus_dmamap_load(sc->rx_dist_kcfg.dtag, sc->rx_dist_kcfg.dmap,
	    sc->rx_dist_kcfg.vaddr, DPAA2_CLASSIFIER_DMA_SIZE,
	    dpni_single_seg_dmamap_cb, &sc->rx_dist_kcfg.paddr, BUS_DMA_NOWAIT);
	if (err) {
		device_printf(sc->dev, "Failed to map a buffer for Rx traffic "
		    "distribution key configuration\n");
		return (err);
	}

	if (type == DPAA2_NI_DIST_MODE_HASH) {
		err = DPAA2_CMD_NI_SET_RX_TC_DIST(dev, dpaa2_mcp_tk(sc->cmd,
		    sc->ni_token), sc->attr.num.queues, 0,
		    DPAA2_NI_DIST_MODE_HASH, sc->rx_dist_kcfg.paddr);
		if (err)
			device_printf(dev, "Failed to set distribution mode "
			    "and size for the traffic class\n");
	}

	return err;
}

/**
 * dpni_prepare_key_cfg() - function prepare extract parameters
 * @cfg: defining a full Key Generation profile (rule)
 * @key_cfg_buf: Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before the following functions:
 *	- dpni_set_rx_tc_dist()
 *	- dpni_set_qos_table()
 *
 * Return:	'0' on Success; Error code otherwise.
 */
static int
dpni_prepare_key_cfg(const struct dpkg_profile_cfg *cfg, uint8_t *key_cfg_buf)
{
	struct dpni_ext_set_rx_tc_dist *dpni_ext;
	struct dpni_dist_extract *extr;
	int i, j;

	if (cfg->num_extracts > DPKG_MAX_NUM_OF_EXTRACTS)
		return (EINVAL);

	dpni_ext = (struct dpni_ext_set_rx_tc_dist *)key_cfg_buf;
	dpni_ext->num_extracts = cfg->num_extracts;

	for (i = 0; i < cfg->num_extracts; i++) {
		extr = &dpni_ext->extracts[i];

		switch (cfg->extracts[i].type) {
		case DPKG_EXTRACT_FROM_HDR:
			extr->prot = cfg->extracts[i].extract.from_hdr.prot;
			extr->efh_type =
			    cfg->extracts[i].extract.from_hdr.type & 0x0Fu;
			extr->size = cfg->extracts[i].extract.from_hdr.size;
			extr->offset = cfg->extracts[i].extract.from_hdr.offset;
			extr->field = cfg->extracts[i].extract.from_hdr.field;
			extr->hdr_index =
				cfg->extracts[i].extract.from_hdr.hdr_index;
			break;
		case DPKG_EXTRACT_FROM_DATA:
			extr->size = cfg->extracts[i].extract.from_data.size;
			extr->offset =
				cfg->extracts[i].extract.from_data.offset;
			break;
		case DPKG_EXTRACT_FROM_PARSE:
			extr->size = cfg->extracts[i].extract.from_parse.size;
			extr->offset =
				cfg->extracts[i].extract.from_parse.offset;
			break;
		default:
			return (EINVAL);
		}

		extr->num_of_byte_masks = cfg->extracts[i].num_of_byte_masks;
		extr->extract_type = cfg->extracts[i].type & 0x0Fu;

		for (j = 0; j < DPKG_NUM_OF_MASKS; j++) {
			extr->masks[j].mask = cfg->extracts[i].masks[j].mask;
			extr->masks[j].offset =
				cfg->extracts[i].masks[j].offset;
		}
	}

	return 0;
}

static device_method_t dpaa2_ni_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_ni_probe),
	DEVMETHOD(device_attach,	dpaa2_ni_attach),
	DEVMETHOD(device_detach,	dpaa2_ni_detach),

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
