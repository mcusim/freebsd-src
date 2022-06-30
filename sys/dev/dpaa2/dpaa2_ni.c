/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021-2022 Dmitry Salychev <dsl@mcusim.org>
 * Copyright (c) 2022 Mathew McBride
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
#include <sys/systm.h>
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
#include <sys/taskqueue.h>
#include <sys/sysctl.h>
#include <sys/buf_ring.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/atomic.h>

#include <net/ethernet.h>
#include <net/bpf.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

#include <dev/pci/pcivar.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mdio/mdio.h>

#include "pcib_if.h"
#include "pci_if.h"
#include "miibus_if.h"
#include "mdio_if.h"
#include "memac_mdio_if.h"

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

/* Frame Dequeue Response status bits. */
#define IS_NULL_RESPONSE(stat)	((((stat) >> 4) & 1) == 0)

#define	ALIGN_UP(x, y)		roundup2((x), (y))
#define	ALIGN_DOWN(x, y)	rounddown2((x), (y))
#define CACHE_LINE_ALIGN(x)	ALIGN_UP((x), CACHE_LINE_SIZE)

#define DPNI_LOCK(sc) do {			\
	mtx_assert(&(sc)->lock, MA_NOTOWNED);	\
	mtx_lock(&(sc)->lock);			\
} while (0)
#define	DPNI_UNLOCK(sc)		mtx_unlock(&(sc)->lock)

#define DPAA2_TX_RING(sc, chan, tc)			\
(&(sc)->channels[(chan)]->txc_queue.tx_rings[(tc)])

/* Handy wrapper over an atomic operation. */
#define ATOMIC_XCHG(a, val)	(atomic_swap_int(&(a)->counter, (val)))

#define DPNI_IRQ_INDEX		0 /* Index of the only DPNI IRQ. */
#define DPNI_IRQ_LINK_CHANGED	1 /* Link state changed */
#define DPNI_IRQ_EP_CHANGED	2 /* DPAA2 endpoint dis/connected */

/* Default maximum frame length. */
#define DPAA2_ETH_MFL		(ETHER_MAX_LEN - ETHER_CRC_LEN)

/* Minimally supported version of the DPNI API. */
#define DPNI_VER_MAJOR		7
#define DPNI_VER_MINOR		0

/* Rx/Tx buffers configuration. */
#define BUF_ALIGN_V1		256 /* WRIOP v1.0.0 limitation */
#define BUF_ALIGN		64
#define BUF_SWA_SIZE		64  /* SW annotation size */
#define BUF_RX_HWA_SIZE		64  /* HW annotation size */
#define BUF_TX_HWA_SIZE		128 /* HW annotation size */
#define BUF_SIZE		(MJUM9BYTES)
#define	BUF_MAXADDR_49BIT	0x1FFFFFFFFFFFFul
#define	BUF_MAXADDR		(BUS_SPACE_MAXADDR)

#define DPAA2_TX_BUFRING_SZ	(4096u)
#define DPAA2_TX_SEGLIMIT	(16u)
#define DPAA2_TX_SEG_SZ		(4096u)
#define DPAA2_TX_SEGS_MAXSZ	(DPAA2_TX_SEGLIMIT * DPAA2_TX_SEG_SZ)
#define DPAA2_TX_SGT_SZ		(512u) /* bytes */

/* Size of a buffer to keep a QoS table key configuration. */
#define ETH_QOS_KCFG_BUF_SIZE	256

/* Required by struct dpni_rx_tc_dist_cfg::key_cfg_iova */
#define DPAA2_CLASSIFIER_DMA_SIZE 256

/* Channel storage buffer configuration. */
#define ETH_STORE_FRAMES	16
#define ETH_STORE_SIZE		((ETH_STORE_FRAMES + 1) * sizeof(struct dpaa2_dq))
#define ETH_STORE_ALIGN		64

/* Buffers layout options. */
#define BUF_LOPT_TIMESTAMP	0x1
#define BUF_LOPT_PARSER_RESULT	0x2
#define BUF_LOPT_FRAME_STATUS	0x4
#define BUF_LOPT_PRIV_DATA_SZ	0x8
#define BUF_LOPT_DATA_ALIGN	0x10
#define BUF_LOPT_DATA_HEAD_ROOM	0x20
#define BUF_LOPT_DATA_TAIL_ROOM	0x40

#define DPAA2_NI_BUF_ADDR_MASK	(0x1FFFFFFFFFFFFul) /* 49-bit addresses max. */
#define DPAA2_NI_BUF_CHAN_MASK	(0xFu)
#define DPAA2_NI_BUF_CHAN_SHIFT	(60)
#define DPAA2_NI_BUF_IDX_MASK	(0x7FFu)
#define DPAA2_NI_BUF_IDX_SHIFT	(49)
#define DPAA2_NI_TX_IDX_MASK	(0x7u)
#define DPAA2_NI_TX_IDX_SHIFT	(57)
#define DPAA2_NI_TXBUF_IDX_MASK	(0xFFu)
#define DPAA2_NI_TXBUF_IDX_SHIFT (49)

#define DPAA2_NI_FD_FMT_MASK	(0x3u)
#define DPAA2_NI_FD_FMT_SHIFT	(12)
#define DPAA2_NI_FD_ERR_MASK	(0xFFu)
#define DPAA2_NI_FD_ERR_SHIFT	(0)
#define DPAA2_NI_FD_SL_MASK	(0x1u)
#define DPAA2_NI_FD_SL_SHIFT	(14)
#define DPAA2_NI_FD_LEN_MASK	(0x3FFFFu)
#define DPAA2_NI_FD_OFFSET_MASK (0x0FFFu)

/* Enables TCAM for Flow Steering and QoS look-ups. */
#define DPNI_OPT_HAS_KEY_MASKING 0x10

/* Unique IDs for the supported Rx classification header fields. */
#define DPAA2_ETH_DIST_ETHDST	BIT(0)
#define DPAA2_ETH_DIST_ETHSRC	BIT(1)
#define DPAA2_ETH_DIST_ETHTYPE	BIT(2)
#define DPAA2_ETH_DIST_VLAN	BIT(3)
#define DPAA2_ETH_DIST_IPSRC	BIT(4)
#define DPAA2_ETH_DIST_IPDST	BIT(5)
#define DPAA2_ETH_DIST_IPPROTO	BIT(6)
#define DPAA2_ETH_DIST_L4SRC	BIT(7)
#define DPAA2_ETH_DIST_L4DST	BIT(8)
#define DPAA2_ETH_DIST_ALL	(~0ULL)

/* L3-L4 network traffic flow hash options. */
#define	RXH_L2DA		(1 << 1)
#define	RXH_VLAN		(1 << 2)
#define	RXH_L3_PROTO		(1 << 3)
#define	RXH_IP_SRC		(1 << 4)
#define	RXH_IP_DST		(1 << 5)
#define	RXH_L4_B_0_1		(1 << 6) /* src port in case of TCP/UDP/SCTP */
#define	RXH_L4_B_2_3		(1 << 7) /* dst port in case of TCP/UDP/SCTP */
#define	RXH_DISCARD		(1 << 31)

/* Default Rx hash options, set during attaching. */
#define DPAA2_RXH_DEFAULT	(RXH_L3_PROTO | RXH_IP_SRC | RXH_IP_DST | \
				 RXH_L4_B_0_1 | RXH_L4_B_2_3)

MALLOC_DEFINE(M_DPAA2_NI, "dpaa2_ni", "DPAA2 Network Interface");

/* DPAA2 Network Interface resource specification. */
struct resource_spec dpaa2_ni_spec[] = {
	/*
	 * DPMCP resources.
	 *
	 * NOTE: MC command portals (MCPs) are used to send commands to, and
	 *	 receive responses from, the MC firmware. One portal per DPNI.
	 */
#define MCP_RES_NUM	(1u)
#define MCP_RID_OFF	(0u)
#define MCP_RID(rid)	((rid) + MCP_RID_OFF)
	/* --- */
	{ DPAA2_DEV_MCP, MCP_RID(0),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	/*
	 * DPIO resources (software portals).
	 *
	 * NOTE: One per running core. While DPIOs are the source of data
	 *	 availability interrupts, the DPCONs are used to identify the
	 *	 network interface that has produced ingress data to that core.
	 */
#define IO_RES_NUM	(16u)
#define IO_RID_OFF	(MCP_RID_OFF + MCP_RES_NUM)
#define IO_RID(rid)	((rid) + IO_RID_OFF)
	/* --- */
	{ DPAA2_DEV_IO,  IO_RID(0),    RF_ACTIVE | RF_SHAREABLE },
	{ DPAA2_DEV_IO,  IO_RID(1),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(2),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(3),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(4),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(5),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(6),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(7),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(8),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(9),    RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(10),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(11),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(12),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(13),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(14),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	{ DPAA2_DEV_IO,  IO_RID(15),   RF_ACTIVE | RF_SHAREABLE | RF_OPTIONAL },
	/*
	 * DPBP resources (buffer pools).
	 *
	 * NOTE: One per network interface.
	 */
#define BP_RES_NUM	(1u)
#define BP_RID_OFF	(IO_RID_OFF + IO_RES_NUM)
#define BP_RID(rid)	((rid) + BP_RID_OFF)
	/* --- */
	{ DPAA2_DEV_BP,  BP_RID(0),   RF_ACTIVE },
	/*
	 * DPCON resources (channels).
	 *
	 * NOTE: One DPCON per core where Rx or Tx confirmation traffic to be
	 *	 distributed to.
	 * NOTE: Since it is necessary to distinguish between traffic from
	 *	 different network interfaces arriving on the same core, the
	 *	 DPCONs must be private to the DPNIs.
	 */
#define CON_RES_NUM	(16u)
#define CON_RID_OFF	(BP_RID_OFF + BP_RES_NUM)
#define CON_RID(rid)	((rid) + CON_RID_OFF)
	/* --- */
	{ DPAA2_DEV_CON, CON_RID(0),   RF_ACTIVE },
	{ DPAA2_DEV_CON, CON_RID(1),   RF_ACTIVE | RF_OPTIONAL },
	{ DPAA2_DEV_CON, CON_RID(2),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(3),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(4),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(5),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(6),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(7),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(8),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(9),   RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(10),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(11),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(12),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(13),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(14),  RF_ACTIVE | RF_OPTIONAL },
 	{ DPAA2_DEV_CON, CON_RID(15),  RF_ACTIVE | RF_OPTIONAL },
	/* --- */
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

static struct dpni_stat {
	int	 page;
	int	 cnt;
	char	*name;
	char	*desc;
} dpni_stat_sysctls[DPAA2_NI_STAT_SYSCTLS] = {
	/* PAGE, COUNTER, NAME, DESCRIPTION */
	{  0, 0, "in_all_frames",	"All accepted ingress frames" },
	{  0, 1, "in_all_bytes",	"Bytes in all accepted ingress frames" },
	{  0, 2, "in_multi_frames",	"Multicast accepted ingress frames" },
	{  1, 0, "eg_all_frames",	"All egress frames transmitted" },
	{  1, 1, "eg_all_bytes",	"Bytes in all frames transmitted" },
	{  1, 2, "eg_multi_frames",	"Multicast egress frames transmitted" },
	{  2, 0, "in_filtered_frames",	"All ingress frames discarded due to "
	   				"filtering" },
	{  2, 1, "in_discarded_frames",	"All frames discarded due to errors" },
	{  2, 2, "in_nobuf_discards",	"Discards on ingress side due to buffer "
	   				"depletion in DPNI buffer pools" },
};

/* Device interface */
static int dpaa2_ni_probe(device_t);
static int dpaa2_ni_attach(device_t);
static int dpaa2_ni_detach(device_t);

/* DPAA2 network interface setup and configuration */
static int dpaa2_ni_setup(device_t);
static int dpaa2_ni_setup_channels(device_t);
static int dpaa2_ni_setup_fq(device_t, struct dpaa2_ni_channel *,
    enum dpaa2_ni_queue_type);
static int dpaa2_ni_bind(device_t);
static int dpaa2_ni_setup_rx_dist(device_t);
static int dpaa2_ni_setup_irqs(device_t);
static int dpaa2_ni_setup_msi(struct dpaa2_ni_softc *);
static int dpaa2_ni_setup_if_caps(struct dpaa2_ni_softc *);
static int dpaa2_ni_setup_if_flags(struct dpaa2_ni_softc *);
static int dpaa2_ni_setup_sysctls(struct dpaa2_ni_softc *);
static int dpaa2_ni_setup_dma(struct dpaa2_ni_softc *);

/* Tx/Rx flow configuration */
static int dpaa2_ni_setup_rx_flow(device_t, struct dpaa2_cmd *,
    struct dpaa2_ni_fq *);
static int dpaa2_ni_setup_tx_flow(device_t, struct dpaa2_cmd *,
    struct dpaa2_ni_fq *);
static int dpaa2_ni_setup_rx_err_flow(device_t, struct dpaa2_cmd *,
    struct dpaa2_ni_fq *);

/* Configuration subroutines */
static int dpaa2_ni_set_buf_layout(device_t, struct dpaa2_cmd *);
static int dpaa2_ni_set_pause_frame(device_t, struct dpaa2_cmd *);
static int dpaa2_ni_set_qos_table(device_t, struct dpaa2_cmd *);
static int dpaa2_ni_set_mac_addr(device_t, struct dpaa2_cmd *, uint16_t,
    uint16_t);
static int dpaa2_ni_set_hash(device_t, uint64_t);
static int dpaa2_ni_set_dist_key(device_t, enum dpaa2_ni_dist_mode, uint64_t);

/* Buffers and buffer pools */
static int dpaa2_ni_seed_buf_pool(struct dpaa2_ni_softc *,
    struct dpaa2_ni_channel *);
static int dpaa2_ni_seed_buf(struct dpaa2_ni_softc *, struct dpaa2_ni_channel *,
    struct dpaa2_ni_buf *, int);
static int dpaa2_ni_seed_chan_storage(struct dpaa2_ni_softc *,
    struct dpaa2_ni_channel *);

/* Frame descriptor routines */
static int dpaa2_ni_build_fd(struct dpaa2_ni_softc *, struct dpaa2_ni_tx_ring *,
    struct dpaa2_ni_buf *, bus_dma_segment_t *, int, struct dpaa2_fd *);
static int dpaa2_ni_fd_err(struct dpaa2_fd *);
static uint32_t dpaa2_ni_fd_data_len(struct dpaa2_fd *);
static int dpaa2_ni_fd_chan_idx(struct dpaa2_fd *);
static int dpaa2_ni_fd_buf_idx(struct dpaa2_fd *);
static int dpaa2_ni_fd_tx_idx(struct dpaa2_fd *);
static int dpaa2_ni_fd_txbuf_idx(struct dpaa2_fd *);
static int dpaa2_ni_fd_format(struct dpaa2_fd *);
static bool dpaa2_ni_fd_short_len(struct dpaa2_fd *);
static int dpaa2_ni_fd_offset(struct dpaa2_fd *);

/* Various subroutines */
static int dpaa2_ni_cmp_api_version(struct dpaa2_ni_softc *, uint16_t, uint16_t);
static int dpaa2_ni_prepare_key_cfg(struct dpkg_profile_cfg *, uint8_t *);
static int dpaa2_ni_chan_storage_next(struct dpaa2_ni_channel *,
    struct dpaa2_dq **);

/* Network interface routines */
static void dpaa2_ni_init(void *);
static int  dpaa2_ni_transmit(struct ifnet *, struct mbuf *);
static void dpaa2_ni_qflush(struct ifnet *);
static int  dpaa2_ni_ioctl(struct ifnet *, u_long, caddr_t);
static int  dpaa2_ni_update_mac_filters(struct ifnet *);
static u_int dpaa2_ni_add_maddr(void *, struct sockaddr_dl *, u_int);

/* Interrupt handlers */
static void dpaa2_ni_intr(void *);

/* MII handlers */
static void dpaa2_ni_miibus_statchg(device_t);
static int  dpaa2_ni_media_change(struct ifnet *);
static void dpaa2_ni_media_status(struct ifnet *, struct ifmediareq *);
static void dpaa2_ni_media_tick(void *);

/* DMA mapping callback */
static void dpaa2_ni_dmamap_cb(void *, bus_dma_segment_t *, int, int);

/* Tx/Rx tasks. */
static void dpaa2_ni_poll_task(void *, int);
static void dpaa2_ni_tx_task(void *, int);

/* Tx/Rx subroutines */
static int  dpaa2_ni_consume_frames(struct dpaa2_ni_channel *,
    struct dpaa2_ni_fq **, uint32_t *);
static int  dpaa2_ni_rx(struct dpaa2_ni_channel *, struct dpaa2_ni_fq *,
    struct dpaa2_fd *);
static int  dpaa2_ni_rx_err(struct dpaa2_ni_channel *, struct dpaa2_ni_fq *,
    struct dpaa2_fd *);
static int  dpaa2_ni_tx_conf(struct dpaa2_ni_channel *, struct dpaa2_ni_fq *,
    struct dpaa2_fd *);

/* sysctl(9) */
static int dpaa2_ni_collect_stats(SYSCTL_HANDLER_ARGS);

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
	device_t child = dev;
	device_t mcp_dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(pdev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(dev);
	struct dpaa2_devinfo *mcp_dinfo;
	struct ifnet *ifp;
	int error;

	sc->dev = dev;
	sc->ifp = NULL;
	sc->miibus = NULL;
	sc->mii = NULL;
	sc->media_status = 0;
	sc->if_flags = 0;
	sc->link_state = LINK_STATE_UNKNOWN;
	sc->buf_align = 0;

	/* For debug purposes only! */
	sc->rx_anomaly_frames = 0;
	sc->rx_single_buf_frames = 0;
	sc->rx_sg_buf_frames = 0;
	sc->rx_enq_rej_frames = 0;
	sc->rx_ieoi_err_frames = 0;
	sc->tx_single_buf_frames = 0;
	sc->tx_sg_frames = 0;

	sc->bp_dmat = NULL;
	sc->st_dmat = NULL;
	sc->rxd_dmat = NULL;
	sc->qos_dmat = NULL;

	sc->qos_kcfg.dmap = NULL;
	sc->qos_kcfg.paddr = 0;
	sc->qos_kcfg.vaddr = NULL;

	sc->rxd_kcfg.dmap = NULL;
	sc->rxd_kcfg.paddr = 0;
	sc->rxd_kcfg.vaddr = NULL;

	sc->mac.dpmac_id = 0;
	sc->mac.phy_dev = NULL;
	memset(sc->mac.addr, 0, ETHER_ADDR_LEN);

	error = bus_alloc_resources(sc->dev, dpaa2_ni_spec, sc->res);
	if (error) {
		device_printf(dev, "%s: failed to allocate resources: "
		    "error=%d\n", __func__, error);
		return (ENXIO);
	}

	/* Obtain MC portal. */
	mcp_dev = (device_t) rman_get_start(sc->res[MCP_RID(0)]);
	mcp_dinfo = device_get_ivars(mcp_dev);
	dinfo->portal = mcp_dinfo->portal;

	mtx_init(&sc->lock, device_get_nameunit(dev), "dpaa2_ni", MTX_DEF);

	/* Allocate network interface */
	ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "%s: failed to allocate network interface\n",
		    __func__);
		return (ENXIO);
	}
	sc->ifp = ifp;
	if_initname(ifp, DPAA2_NI_IFNAME, device_get_unit(sc->dev));

	ifp->if_softc = sc;
	ifp->if_flags = IFF_SIMPLEX | IFF_MULTICAST | IFF_BROADCAST;
	ifp->if_init = dpaa2_ni_init;
	ifp->if_ioctl = dpaa2_ni_ioctl;
	ifp->if_transmit = dpaa2_ni_transmit;
	ifp->if_qflush = dpaa2_ni_qflush;

	ifp->if_capabilities = IFCAP_VLAN_MTU | IFCAP_HWCSUM | IFCAP_JUMBO_MTU;
	ifp->if_capenable = ifp->if_capabilities;

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&sc->cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "%s: failed to allocate dpaa2_cmd: "
		    "error=%d\n", __func__, error);
		goto err_exit;
	}

	/* Open resource container and network interface object. */
	error = DPAA2_CMD_RC_OPEN(dev, child, sc->cmd, rcinfo->id,
	    &sc->rc_token);
	if (error) {
		device_printf(dev, "%s: failed to open resource container: "
		    "id=%d, error=%d\n", __func__, rcinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_NI_OPEN(dev, child, dpaa2_mcp_tk(sc->cmd,
	    sc->rc_token), dinfo->id, &sc->ni_token);
	if (error) {
		device_printf(dev, "%s: failed to open network interface: "
		    "id=%d, error=%d\n", __func__, dinfo->id, error);
		goto err_close_rc;
	}

	/* Create a private taskqueue thread for handling driver events. */
	sc->tq = taskqueue_create("dpaa2_ni_taskq", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->tq);
	if (sc->tq == NULL) {
		device_printf(dev, "%s: failed to allocate task queue\n",
		    __func__);
		goto err_close_ni;
	}
	taskqueue_start_threads(&sc->tq, 1, PI_NET, "%s events",
	    device_get_nameunit(dev));

	error = dpaa2_ni_setup(dev);
	if (error) {
		device_printf(dev, "%s: failed to setup DPNI: error=%d\n",
		    __func__, error);
		goto err_close_ni;
	}
	error = dpaa2_ni_setup_channels(dev);
	if (error) {
		device_printf(dev, "%s: failed to setup QBMan channels: "
		    "error=%d\n", __func__, error);
		goto err_close_ni;
	}
	error = dpaa2_ni_bind(dev);
	if (error) {
		device_printf(dev, "%s: failed to bind DPNI: error=%d\n",
		    __func__, error);
		goto err_close_ni;
	}
	error = dpaa2_ni_setup_irqs(dev);
	if (error) {
		device_printf(dev, "%s: failed to setup IRQs: error=%d\n",
		    __func__, error);
		goto err_close_ni;
	}
	error = dpaa2_ni_setup_sysctls(sc);
	if (error) {
		device_printf(dev, "%s: failed to setup sysctls: error=%d\n",
		    __func__, error);
		goto err_close_ni;
	}

	ether_ifattach(sc->ifp, sc->mac.addr);
	callout_init(&sc->mii_callout, 0);

	return (0);

err_close_ni:
	DPAA2_CMD_NI_CLOSE(dev, child, dpaa2_mcp_tk(sc->cmd, sc->ni_token));
err_close_rc:
	DPAA2_CMD_RC_CLOSE(dev, child, dpaa2_mcp_tk(sc->cmd, sc->rc_token));
err_free_cmd:
	dpaa2_mcp_free_command(sc->cmd);
err_exit:
	return (ENXIO);
}

static void
dpaa2_ni_fixed_media_status(if_t ifp, struct ifmediareq* ifmr)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;

	DPNI_LOCK(sc);
	ifmr->ifm_count = 0;
	ifmr->ifm_mask = 0;
	ifmr->ifm_status = IFM_AVALID | IFM_ACTIVE;
	ifmr->ifm_current = ifmr->ifm_active =
	    sc->fixed_ifmedia.ifm_cur->ifm_media;

	/*
	 * In non-PHY usecases, we need to signal link state up, otherwise
	 * certain things requiring a link event (e.g async DHCP client) from
	 * devd do not happen.
	 */
	if (ifp->if_link_state == LINK_STATE_UNKNOWN) {
		if_link_state_change(ifp, LINK_STATE_UP);
	}

	/*
	 * TODO: Check the status of the link partner (DPMAC, DPNI or other) and
	 * reset if down. This is different to the DPAA2_MAC_LINK_TYPE_PHY as
	 * the MC firmware sets the status, instead of us telling the MC what
	 * it is.
	 */
	DPNI_UNLOCK(sc);

	return;
}

static void
dpaa2_ni_setup_fixed_link(struct dpaa2_ni_softc *sc)
{
	/*
	 * FIXME: When the DPNI is connected to a DPMAC, we can get the
	 * 'apparent' speed from it.
	 */
	sc->fixed_link = true;

	ifmedia_init(&sc->fixed_ifmedia, 0, dpaa2_ni_media_change,
		     dpaa2_ni_fixed_media_status);
	ifmedia_add(&sc->fixed_ifmedia, IFM_ETHER | IFM_1000_T, 0, NULL);
	ifmedia_set(&sc->fixed_ifmedia, IFM_ETHER | IFM_1000_T);
}

static int
dpaa2_ni_detach(device_t dev)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);

	DPAA2_CMD_NI_CLOSE(dev, child, dpaa2_mcp_tk(sc->cmd, sc->ni_token));
	DPAA2_CMD_RC_CLOSE(dev, child, dpaa2_mcp_tk(sc->cmd, sc->rc_token));
	dpaa2_mcp_free_command(sc->cmd);

	sc->cmd = NULL;
	sc->ni_token = 0;
	sc->rc_token = 0;

	return (0);
}

/**
 * @brief Configure DPAA2 network interface object.
 */
static int
dpaa2_ni_setup(device_t dev)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(dev);
	struct dpaa2_ep_desc ep1_desc, ep2_desc; /* endpoint descriptors */
	struct dpaa2_cmd *cmd = sc->cmd;
	uint8_t eth_bca[ETHER_ADDR_LEN]; /* broadcast physical address */
	uint16_t rc_token = sc->rc_token;
	uint16_t ni_token = sc->ni_token;
	uint16_t mac_token;
	struct dpaa2_mac_attr attr;
	enum dpaa2_mac_link_type link_type;
	uint32_t link;
	int error;

	/* Check if we can work with this DPNI object. */
	error = DPAA2_CMD_NI_GET_API_VERSION(dev, child, dpaa2_mcp_tk(cmd,
	    ni_token), &sc->api_major, &sc->api_minor);
	if (error) {
		device_printf(dev, "%s: failed to get DPNI API version\n",
		    __func__);
		return (error);
	}
	if (dpaa2_ni_cmp_api_version(sc, DPNI_VER_MAJOR, DPNI_VER_MINOR) < 0) {
		device_printf(dev, "%s: DPNI API version %u.%u not supported, "
		    "need >= %u.%u\n", __func__, sc->api_major, sc->api_minor,
		    DPNI_VER_MAJOR, DPNI_VER_MINOR);
		error = ENODEV;
		return (error);
	}

	/* Reset the DPNI object. */
	error = DPAA2_CMD_NI_RESET(dev, child, cmd);
	if (error) {
		device_printf(dev, "%s: failed to reset DPNI: id=%d\n",
		    __func__, dinfo->id);
		return (error);
	}

	/* Obtain attributes of the DPNI object. */
	error = DPAA2_CMD_NI_GET_ATTRIBUTES(dev, child, cmd, &sc->attr);
	if (error) {
		device_printf(dev, "%s: failed to obtain DPNI attributes: "
		    "id=%d\n", __func__, dinfo->id);
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
	error = dpaa2_ni_set_buf_layout(dev, cmd);
	if (error) {
		device_printf(dev, "%s: failed to configure buffer layout\n",
		    __func__);
		return (error);
	}

	/* Configure DMA resources. */
	error = dpaa2_ni_setup_dma(sc);
	if (error) {
		device_printf(dev, "%s: failed to setup DMA\n", __func__);
		return (error);
	}

	/* Setup link between DPNI and an object it's connected to. */
	ep1_desc.obj_id = dinfo->id;
	ep1_desc.if_id = 0; /* DPNI has the only endpoint */
	ep1_desc.type = dinfo->dtype;

	error = DPAA2_CMD_RC_GET_CONN(dev, child, dpaa2_mcp_tk(cmd, rc_token),
	    &ep1_desc, &ep2_desc, &link);
	if (error)
		device_printf(dev, "%s: failed to obtain an object DPNI is "
		    "connected to: error=%d\n", __func__, error);
	else {
		device_printf(dev, "connected to %s (id=%d)\n",
		    dpaa2_ttos(ep2_desc.type), ep2_desc.obj_id);

		error = dpaa2_ni_set_mac_addr(dev, cmd, rc_token, ni_token);
		if (error)
			device_printf(dev, "%s: failed to set MAC "
				      "address: error=%d\n", __func__, error);

		if (ep2_desc.type == DPAA2_DEV_MAC) {
			/*
			 * This is the simplest case when DPNI is connected to
			 * DPMAC directly.
			 */
			sc->mac.dpmac_id = ep2_desc.obj_id;

			link_type = DPAA2_MAC_LINK_TYPE_NONE;

			/*
			 * Need to determine if DPMAC type is PHY (attached to
			 * conventional MII PHY) or FIXED (usually SFP/SerDes,
			 * link state managed by MC firmware).
			 */
			error = DPAA2_CMD_MAC_OPEN(sc->dev, child,
			    dpaa2_mcp_tk(sc->cmd, sc->rc_token),
			    sc->mac.dpmac_id, &mac_token);
			/*
			 * Under VFIO, the DPMAC might be sitting in another
			 * container (DPRC) we don't have access to.
			 * Assume DPAA2_MAC_LINK_TYPE_FIXED if this is
			 * the case.
			 */
			if (error) {
				device_printf(dev, "%s: failed to open "
				    "connected DPMAC: %d (assuming in other DPRC)\n", __func__,
				    sc->mac.dpmac_id);
				link_type = DPAA2_MAC_LINK_TYPE_FIXED;
			} else {
				error = DPAA2_CMD_MAC_GET_ATTRIBUTES(dev, child,
				    sc->cmd, &attr);
				if (error)
					device_printf(dev, "%s: failed to get "
					    "DPMAC attributes: id=%d, "
					    "error=%d\n", __func__, dinfo->id,
					    error);
				else
					link_type = attr.link_type;
			}
			DPAA2_CMD_MAC_CLOSE(dev, child, dpaa2_mcp_tk(sc->cmd,
			    mac_token));

			if (link_type == DPAA2_MAC_LINK_TYPE_FIXED) {
				device_printf(dev, "connected DPMAC is in FIXED "
				    "mode\n");
				dpaa2_ni_setup_fixed_link(sc);
			} else if (link_type == DPAA2_MAC_LINK_TYPE_PHY) {
				device_printf(dev, "connected DPMAC is in PHY "
				    "mode\n");
				error = DPAA2_MC_GET_PHY_DEV(dev,
				    &sc->mac.phy_dev, sc->mac.dpmac_id);
				if (error == 0) {
					error = MEMAC_MDIO_SET_NI_DEV(
					    sc->mac.phy_dev, dev);
					if (error != 0)
						device_printf(dev, "%s: failed "
						    "to set dpni dev on memac "
						    "mdio dev %s: error=%d\n",
						    __func__,
						    device_get_nameunit(
						    sc->mac.phy_dev), error);
				}
				if (error == 0) {
					error = MEMAC_MDIO_GET_PHY_LOC(
					    sc->mac.phy_dev, &sc->mac.phy_loc);
					if (error == ENODEV)
						error = 0;
					if (error != 0)
						device_printf(dev, "%s: failed "
						    "to get phy location from "
						    "memac mdio dev %s: error=%d\n",
						    __func__, device_get_nameunit(
						    sc->mac.phy_dev), error);
				}
				if (error == 0) {
					error = mii_attach(sc->mac.phy_dev,
					    &sc->miibus, sc->ifp,
					    dpaa2_ni_media_change,
					    dpaa2_ni_media_status,
					    BMSR_DEFCAPMASK, sc->mac.phy_loc,
					    MII_OFFSET_ANY, 0);
					if (error != 0)
						device_printf(dev, "%s: failed "
						    "to attach to miibus: "
						    "error=%d\n",
						    __func__, error);
				}
				if (error == 0)
					sc->mii = device_get_softc(sc->miibus);
			} else {
				device_printf(dev, "%s: DPMAC link type is not "
				    "supported\n", __func__);
			}
		} else if (ep2_desc.type == DPAA2_DEV_NI ||
			   ep2_desc.type == DPAA2_DEV_MUX ||
			   ep2_desc.type == DPAA2_DEV_SW) {
			dpaa2_ni_setup_fixed_link(sc);
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
	error = dpaa2_ni_set_pause_frame(dev, dpaa2_mcp_tk(cmd, ni_token));
	if (error) {
		device_printf(dev, "%s: failed to configure Rx/Tx pause "
		    "frames\n", __func__);
		return (error);
	}

	/* Configure ingress traffic classification. */
	error = dpaa2_ni_set_qos_table(dev, dpaa2_mcp_tk(cmd, ni_token));
	if (error)
		device_printf(dev, "%s: failed to configure QoS table: "
		    "error=%d\n", __func__, error);

	/* Add broadcast physical address to the MAC filtering table. */
	memset(eth_bca, 0xff, ETHER_ADDR_LEN);
	error = DPAA2_CMD_NI_ADD_MAC_ADDR(dev, child, cmd, eth_bca);
	if (error) {
		device_printf(dev, "%s: failed to add broadcast physical "
		    "address to the MAC filtering table\n", __func__);
		return (error);
	}

	/* Set the maximum allowed length for received frames. */
	error = DPAA2_CMD_NI_SET_MFL(dev, child, cmd, DPAA2_ETH_MFL);
	if (error) {
		device_printf(dev, "%s: failed to set maximum length for "
		    "received frames\n", __func__);
		return (error);
	}

	return (0);
}

/**
 * @brief Сonfigure QBMan channels and register data availability notifications.
 */
static int
dpaa2_ni_setup_channels(device_t dev)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_con_softc *consc;
	struct dpaa2_devinfo *io_info, *con_info;
	device_t io_dev, con_dev, child = dev;
	struct dpaa2_ni_channel *channel;
	struct dpaa2_io_notif_ctx *ctx;
	struct dpaa2_con_notif_cfg notif_cfg;
	int error;
	struct sysctl_ctx_list *sysctl_ctx;
	struct sysctl_oid *node;
	struct sysctl_oid_list *parent;
	uint32_t i, num_chan;

	/* Calculate number of the channels based on the allocated resources. */
	for (i = 0; i < IO_RES_NUM; i++)
		if (!sc->res[IO_RID(i)])
			break;
	num_chan = i;
	for (i = 0; i < CON_RES_NUM; i++)
		if (!sc->res[CON_RID(i)])
			break;
	num_chan = i < num_chan ? i : num_chan;

	/* Limit maximum channels. */
	sc->chan_n = num_chan > DPAA2_NI_MAX_CHANNELS
	    ? DPAA2_NI_MAX_CHANNELS : num_chan;

	/* Limit channels by number of the queues. */
	sc->chan_n = sc->chan_n > sc->attr.num.queues
	    ? sc->attr.num.queues : sc->chan_n;

	device_printf(dev, "channels=%d\n", sc->chan_n);

	sysctl_ctx = device_get_sysctl_ctx(sc->dev);
	parent = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	node = SYSCTL_ADD_NODE(sysctl_ctx, parent, OID_AUTO, "channels",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "DPNI Channels");
	parent = SYSCTL_CHILDREN(node);

	/* Setup channels for the portal. */
	for (uint32_t i = 0; i < sc->chan_n; i++) {
		/* Select software portal. */
		io_dev = (device_t) rman_get_start(sc->res[IO_RID(i)]);
		io_info = device_get_ivars(io_dev);

		/* Select DPCON (channel). */
		con_dev = (device_t) rman_get_start(sc->res[CON_RID(i)]);
		consc = device_get_softc(con_dev);
		con_info = device_get_ivars(con_dev);

		/* Enable selected channel. */
		error = DPAA2_CMD_CON_ENABLE(dev, child, dpaa2_mcp_tk(consc->cmd,
		    consc->con_token));
		if (error) {
			device_printf(dev, "%s: failed to enable channel: "
			    "dpcon_id=%d, chan_id=%d\n", __func__, con_info->id,
			    consc->attr.chan_id);
			return (error);
		}

		channel = malloc(sizeof(struct dpaa2_ni_channel), M_DPAA2_NI,
		    M_WAITOK | M_ZERO);
		if (!channel) {
			device_printf(dev, "%s: failed to allocate a channel\n",
			    __func__);
			return (ENOMEM);
		}

		sc->channels[i] = channel;

		channel->id = consc->attr.chan_id;
		channel->flowid = i;
		channel->ni_dev = dev;
		channel->io_dev = io_dev;
		channel->con_dev = con_dev;
		channel->buf_num = 0;
		channel->recycled_n = 0;

		/* For debug purposes only! */
		channel->tx_frames = 0;
		channel->tx_dropped = 0;

		/* None of the frame queues for this channel configured yet. */
		channel->rxq_n = 0;

		/* Setup WQ channel notification context. */
		ctx = &channel->ctx;
		ctx->qman_ctx = (uint64_t) ctx;
		ctx->cdan_en = true;
		ctx->fq_chan_id = channel->id;
		ctx->io_dev = channel->io_dev;
		ctx->channel = channel;

		/* Task to poll frames when CDAN is received. */
		TASK_INIT(&channel->poll_task, 0, dpaa2_ni_poll_task, channel);
		ctx->tq = sc->tq;
		ctx->notif_task = &channel->poll_task;

		/* Register the new notification context. */
		error = DPAA2_SWP_CONF_WQ_CHANNEL(channel->io_dev, ctx);
		if (error) {
			device_printf(dev, "%s: failed to register notification "
			    "context\n", __func__);
			return (error);
		}

		/* Register DPCON notification with Management Complex. */
		notif_cfg.dpio_id = io_info->id;
		notif_cfg.prior = 0;
		notif_cfg.qman_ctx = ctx->qman_ctx;
		error = DPAA2_CMD_CON_SET_NOTIF(dev, child, dpaa2_mcp_tk(
		    consc->cmd, consc->con_token), &notif_cfg);
		if (error) {
			device_printf(dev, "%s: failed to set DPCON "
			    "notification: dpcon_id=%d, chan_id=%d\n", __func__,
			    con_info->id, consc->attr.chan_id);
			return (error);
		}

		/* Allocate buffers and channel storage. */
		error = dpaa2_ni_seed_buf_pool(sc, channel);
		if (error) {
			device_printf(dev, "%s: failed to seed buffer pool\n",
			    __func__);
			return (error);
		}
		error = dpaa2_ni_seed_chan_storage(sc, channel);
		if (error) {
			device_printf(dev, "%s: failed to seed channel "
			    "storage\n", __func__);
			return (error);
		}

		/* Prepare queues for this channel. */
		error = dpaa2_ni_setup_fq(dev, channel, DPAA2_NI_QUEUE_TX_CONF);
		if (error) {
			device_printf(dev, "%s: failed to prepare TxConf "
			    "queue: error=%d\n", __func__, error);
			return (error);
		}
		error = dpaa2_ni_setup_fq(dev, channel, DPAA2_NI_QUEUE_RX);
		if (error) {
			device_printf(dev, "%s: failed to prepare Rx queue: "
			    "error=%d\n", __func__, error);
			return (error);
		}

		if (bootverbose)
			device_printf(dev, "channel: dpio_id=%d "
			    "dpcon_id=%d chan_id=%d, priorities=%d\n",
			    io_info->id, con_info->id, channel->id,
			    consc->attr.prior_num);
	}

	/* There is exactly one Rx error queue per DPNI. */
	error = dpaa2_ni_setup_fq(dev, sc->channels[0], DPAA2_NI_QUEUE_RX_ERR);
	if (error) {
		device_printf(dev, "%s: failed to prepare RxError queue: "
		    "error=%d\n", __func__, error);
		return (error);
	}

	return (0);
}

/**
 * @brief Performs an initial configuration of the frame queues.
 */
static int
dpaa2_ni_setup_fq(device_t dev, struct dpaa2_ni_channel *chan,
    enum dpaa2_ni_queue_type queue_type)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_ni_fq *fq;

	switch (queue_type) {
	case DPAA2_NI_QUEUE_TX_CONF:
		/* One queue per channel. */
		fq = &chan->txc_queue;

		fq->consume = dpaa2_ni_tx_conf;
		fq->chan = chan;
		fq->flowid = chan->flowid;
		fq->tc = 0; /* ignored */
		fq->type = queue_type;

		break;
	case DPAA2_NI_QUEUE_RX:
		KASSERT(sc->attr.num.rx_tcs <= DPAA2_NI_MAX_TCS,
		    ("too many Rx traffic classes: rx_tcs=%d\n",
		    sc->attr.num.rx_tcs));

		chan->rxq_n = 0;

		/* One queue per Rx traffic class within a channel. */
		for (int i = 0; i < sc->attr.num.rx_tcs; i++) {
			fq = &chan->rx_queues[i];

			fq->consume = dpaa2_ni_rx;
			fq->chan = chan;
			fq->flowid = chan->flowid;
			fq->tc = (uint8_t) i;
			fq->type = queue_type;

			chan->rxq_n++;
		}
		break;
	case DPAA2_NI_QUEUE_RX_ERR:
		/* One queue per network interface. */
		fq = &sc->rxe_queue;

		fq->consume = dpaa2_ni_rx_err;
		fq->chan = chan;
		fq->flowid = 0; /* ignored */
		fq->tc = 0; /* ignored */
		fq->type = queue_type;
		break;
	default:
		device_printf(dev, "%s: unexpected frame queue type: %d\n",
		    __func__, queue_type);
		return (EINVAL);
	}

	return (0);
}

/**
 * @brief Bind DPNI to DPBPs, DPIOs, frame queues and channels.
 */
static int
dpaa2_ni_bind(device_t dev)
{
	device_t bp_dev, child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *bp_info;
	struct dpaa2_cmd *cmd = sc->cmd;
	struct dpaa2_ni_pools_cfg pools_cfg;
	struct dpaa2_ni_err_cfg err_cfg;
	struct dpaa2_ni_channel *chan;
	uint16_t ni_token = sc->ni_token;
	int error;

	/* Select buffer pool (only one available at the moment). */
	bp_dev = (device_t) rman_get_start(sc->res[BP_RID(0)]);
	bp_info = device_get_ivars(bp_dev);

	/* Configure buffers pool. */
	pools_cfg.pools_num = 1;
	pools_cfg.pools[0].bp_obj_id = bp_info->id;
	pools_cfg.pools[0].backup_flag = 0;
	pools_cfg.pools[0].buf_sz = sc->buf_sz;
	error = DPAA2_CMD_NI_SET_POOLS(dev, child, dpaa2_mcp_tk(cmd, ni_token),
	    &pools_cfg);
	if (error) {
		device_printf(dev, "%s: failed to set buffer pools\n", __func__);
		return (error);
	}

	/* Setup ingress traffic distribution. */
	error = dpaa2_ni_setup_rx_dist(dev);
	if (error && error != EOPNOTSUPP) {
		device_printf(dev, "%s: failed to setup ingress traffic "
		    "distribution\n", __func__);
		return (error);
	}
	if (bootverbose && error == EOPNOTSUPP)
		device_printf(dev, "Ingress traffic distribution not "
		    "supported\n");

	/* Configure handling of error frames. */
	err_cfg.err_mask = DPAA2_NI_FAS_RX_ERR_MASK;
	err_cfg.set_err_fas = false;
	err_cfg.action = DPAA2_NI_ERR_DISCARD;
	error = DPAA2_CMD_NI_SET_ERR_BEHAVIOR(dev, child, cmd, &err_cfg);
	if (error) {
		device_printf(dev, "%s: failed to set errors behavior\n",
		    __func__);
		return (error);
	}

	/* Configure channel queues to generate CDANs. */
	for (uint32_t i = 0; i < sc->chan_n; i++) {
		chan = sc->channels[i];

		/* Setup Rx flows. */
		for (uint32_t j = 0; j < chan->rxq_n; j++) {
			error = dpaa2_ni_setup_rx_flow(dev, cmd,
			    &chan->rx_queues[j]);
			if (error) {
				device_printf(dev, "%s: failed to setup Rx "
				    "flow: error=%d\n", __func__, error);
				return (error);
			}
		}

		/* Setup Tx flow. */
		error = dpaa2_ni_setup_tx_flow(dev, cmd, &chan->txc_queue);
		if (error) {
			device_printf(dev, "%s: failed to setup Tx "
			    "flow: error=%d\n", __func__, error);
			return (error);
		}
	}

	/* Configure RxError queue to generate CDAN. */
	error = dpaa2_ni_setup_rx_err_flow(dev, cmd, &sc->rxe_queue);
	if (error) {
		device_printf(dev, "%s: failed to setup RxError flow: "
		    "error=%d\n", __func__, error);
		return (error);
	}

	/*
	 * Get the Queuing Destination ID (QDID) that should be used for frame
	 * enqueue operations.
	 */
	error = DPAA2_CMD_NI_GET_QDID(dev, child, cmd, DPAA2_NI_QUEUE_TX,
	    &sc->tx_qdid);
	if (error) {
		device_printf(dev, "%s: failed to get Tx queuing destination "
		    "ID\n", __func__);
		return (error);
	}

	return (0);
}

/**
 * @brief Setup ingress traffic distribution.
 *
 * NOTE: Ingress traffic distribution is valid only when DPNI_OPT_NO_FS option
 *	 hasn't been set for DPNI and a number of DPNI queues > 1.
 */
static int
dpaa2_ni_setup_rx_dist(device_t dev)
{
	/*
	 * Have the interface implicitly distribute traffic based on the default
	 * hash key.
	 */
	return (dpaa2_ni_set_hash(dev, DPAA2_RXH_DEFAULT));
}

static int
dpaa2_ni_setup_rx_flow(device_t dev, struct dpaa2_cmd *cmd,
    struct dpaa2_ni_fq *fq)
{
	device_t child = dev;
	struct dpaa2_devinfo *con_info;
	struct dpaa2_ni_queue_cfg queue_cfg = {0};
	int error;

	/* Obtain DPCON associated with the FQ's channel. */
	con_info = device_get_ivars(fq->chan->con_dev);

	queue_cfg.type = DPAA2_NI_QUEUE_RX;
	queue_cfg.tc = fq->tc;
	queue_cfg.idx = fq->flowid;
	error = DPAA2_CMD_NI_GET_QUEUE(dev, child, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "%s: failed to obtain Rx queue "
		    "configuration: tc=%d, flowid=%d\n", __func__, queue_cfg.tc,
		    queue_cfg.idx);
		return (error);
	}

	fq->fqid = queue_cfg.fqid;

	queue_cfg.dest_id = con_info->id;
	queue_cfg.dest_type = DPAA2_NI_DEST_DPCON;
	queue_cfg.priority = 1;
	queue_cfg.user_ctx = (uint64_t)(uintmax_t) fq;
	queue_cfg.options =
	    DPAA2_NI_QUEUE_OPT_USER_CTX |
	    DPAA2_NI_QUEUE_OPT_DEST;
	error = DPAA2_CMD_NI_SET_QUEUE(dev, child, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "%s: failed to update Rx queue "
		    "configuration: tc=%d, flowid=%d\n", __func__, queue_cfg.tc,
		    queue_cfg.idx);
		return (error);
	}

	return (0);
}

static int
dpaa2_ni_setup_tx_flow(device_t dev, struct dpaa2_cmd *cmd,
    struct dpaa2_ni_fq *fq)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *con_info;
	struct dpaa2_ni_queue_cfg queue_cfg = {0};
	struct dpaa2_ni_tx_ring *tx;
	uint32_t tx_rings_n = 0;
	int error;

	/* Obtain DPCON associated with the FQ's channel. */
	con_info = device_get_ivars(fq->chan->con_dev);

	KASSERT(sc->attr.num.tx_tcs <= DPAA2_NI_MAX_TCS,
	    ("%s: too many Tx traffic classes: tx_tcs=%d\n", __func__,
	    sc->attr.num.tx_tcs));
	KASSERT(DPAA2_NI_BUFS_PER_TX <= DPAA2_NI_MAX_BPTX,
	    ("%s: too many Tx buffers (%d): max=%d\n", __func__,
	    DPAA2_NI_BUFS_PER_TX, DPAA2_NI_MAX_BPTX));

	/* Setup Tx rings. */
	for (int i = 0; i < sc->attr.num.tx_tcs; i++) {
		queue_cfg.type = DPAA2_NI_QUEUE_TX;
		queue_cfg.tc = i;
		queue_cfg.idx = fq->flowid;

		error = DPAA2_CMD_NI_GET_QUEUE(dev, child, cmd, &queue_cfg);
		if (error) {
			device_printf(dev, "%s: failed to obtain Tx queue "
			    "configuration: tc=%d, flowid=%d\n", __func__,
			    queue_cfg.tc, queue_cfg.idx);
			return (error);
		}

		tx = &fq->tx_rings[i];
		tx->fq = fq;
		tx->fqid = queue_cfg.fqid;
		tx->txid = tx_rings_n;

		mtx_init(&tx->mbuf_lock, "dpaa2_tx_mbuf_br", NULL, MTX_DEF);
		mtx_init(&tx->idx_lock, "dpaa2_tx_idx_br", NULL, MTX_DEF);

		/* Allocate Tx ring buffers. */
		tx->mbuf_br = buf_ring_alloc(DPAA2_TX_BUFRING_SZ, M_DEVBUF,
		    M_NOWAIT, &tx->mbuf_lock);
		if (tx->mbuf_br == NULL) {
			device_printf(dev, "%s: failed to setup Tx ring buffer"
			    " (1) fqid=%d\n", __func__, tx->fqid);
			return (ENOMEM);
		}
		tx->idx_br = buf_ring_alloc(DPAA2_TX_BUFRING_SZ, M_DEVBUF,
		    M_NOWAIT, &tx->idx_lock);
		if (tx->idx_br == NULL) {
			device_printf(dev, "%s: failed to setup Tx ring buffer"
			    " (2) fqid=%d\n", __func__, tx->fqid);
			return (ENOMEM);
		}

		/* Configure Tx buffers. */
		for (uint64_t j = 0; j < DPAA2_NI_BUFS_PER_TX; j++) {
			error = bus_dmamap_create(sc->tx_dmat, 0,
			    &tx->buf[j].dmap);
			if (error) {
				device_printf(sc->dev, "%s: failed to create "
				    "Tx DMA map: error=%d\n", __func__, error);
				return (error);
			}

			/* Allocate a buffer to hold scatter/gather table. */
			error = bus_dmamem_alloc(sc->sgt_dmat,
			    &tx->buf[j].sgt_vaddr, BUS_DMA_ZERO |
			    BUS_DMA_COHERENT, &tx->buf[j].sgt_dmap);
			if (error != 0) {
				device_printf(sc->dev, "%s: failed to allocate "
				    "S/G table: error=%d\n", __func__, error);
				return (error);
			}
			tx->buf[j].sgt_paddr = 0;

			/* Add index of the Tx buffer to the ring. */
			buf_ring_enqueue(tx->idx_br, (void *) j);
		}

		/* Task to send mbufs from the Tx ring. */
		TASK_INIT(&tx->task, 0, dpaa2_ni_tx_task, tx);

		/* Create a taskqueue for Tx ring to transmit mbufs. */
		tx->taskq = taskqueue_create_fast("dpaa2_ni_tx_taskq",
		    M_WAITOK, taskqueue_thread_enqueue, &tx->taskq);
		if (tx->taskq == NULL) {
			device_printf(dev, "%s: failed to allocate Tx ring "
			    "taskqueue\n", __func__);
			return (ENOMEM);
		}
		taskqueue_start_threads(&tx->taskq, 1, PI_NET,
		    "%s tx_taskq(fqid=%d)", device_get_nameunit(dev), tx->fqid);

		tx_rings_n++;
	}

	/* All Tx queues which belong to the same flowid have the same qdbin. */
	fq->tx_qdbin = queue_cfg.qdbin;

	queue_cfg.type = DPAA2_NI_QUEUE_TX_CONF;
	queue_cfg.tc = 0; /* ignored for TxConf queue */
	queue_cfg.idx = fq->flowid;
	error = DPAA2_CMD_NI_GET_QUEUE(dev, child, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "%s: failed to obtain TxConf queue "
		    "configuration: tc=%d, flowid=%d\n", __func__, queue_cfg.tc,
		    queue_cfg.idx);
		return (error);
	}

	fq->fqid = queue_cfg.fqid;

	queue_cfg.dest_id = con_info->id;
	queue_cfg.dest_type = DPAA2_NI_DEST_DPCON;
	queue_cfg.priority = 0;
	queue_cfg.user_ctx = (uint64_t)(uintmax_t) fq;
	queue_cfg.options =
	    DPAA2_NI_QUEUE_OPT_USER_CTX |
	    DPAA2_NI_QUEUE_OPT_DEST;
	error = DPAA2_CMD_NI_SET_QUEUE(dev, child, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "%s: failed to update TxConf queue "
		    "configuration: tc=%d, flowid=%d\n", __func__, queue_cfg.tc,
		    queue_cfg.idx);
		return (error);
	}

	return (0);
}

static int
dpaa2_ni_setup_rx_err_flow(device_t dev, struct dpaa2_cmd *cmd,
    struct dpaa2_ni_fq *fq)
{
	device_t child = dev;
	struct dpaa2_devinfo *con_info;
	struct dpaa2_ni_queue_cfg queue_cfg = {0};
	int error;

	/* Obtain DPCON associated with the FQ's channel. */
	con_info = device_get_ivars(fq->chan->con_dev);

	queue_cfg.type = DPAA2_NI_QUEUE_RX_ERR;
	queue_cfg.tc = fq->tc; /* ignored */
	queue_cfg.idx = fq->flowid; /* ignored */
	error = DPAA2_CMD_NI_GET_QUEUE(dev, child, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "%s: failed to obtain RxErr queue "
		    "configuration\n", __func__);
		return (error);
	}

	fq->fqid = queue_cfg.fqid;

	queue_cfg.dest_id = con_info->id;
	queue_cfg.dest_type = DPAA2_NI_DEST_DPCON;
	queue_cfg.priority = 1;
	queue_cfg.user_ctx = (uint64_t)(uintmax_t) fq;
	queue_cfg.options =
	    DPAA2_NI_QUEUE_OPT_USER_CTX |
	    DPAA2_NI_QUEUE_OPT_DEST;
	error = DPAA2_CMD_NI_SET_QUEUE(dev, child, cmd, &queue_cfg);
	if (error) {
		device_printf(dev, "%s: failed to update RxErr queue "
		    "configuration\n", __func__);
		return (error);
	}

	return (0);
}

/**
 * @brief Configure DPNI object to generate interrupts.
 */
static int
dpaa2_ni_setup_irqs(device_t dev)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_cmd *cmd = sc->cmd;
	uint16_t ni_token = sc->ni_token;
	int error;

	/* Configure IRQs. */
	error = dpaa2_ni_setup_msi(sc);
	if (error) {
		device_printf(dev, "%s: failed to allocate MSI\n", __func__);
		return (error);
	}
	if ((sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid[0], RF_ACTIVE | RF_SHAREABLE)) == NULL) {
		device_printf(dev, "%s: failed to allocate IRQ resource\n",
		    __func__);
		return (ENXIO);
	}
	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, dpaa2_ni_intr, sc, &sc->intr)) {
		device_printf(dev, "%s: failed to setup IRQ resource\n",
		    __func__);
		return (ENXIO);
	}

	/* Configure DPNI to generate interrupts. */
	error = DPAA2_CMD_NI_SET_IRQ_MASK(dev, child, dpaa2_mcp_tk(cmd,
	    ni_token), DPNI_IRQ_INDEX,
	    DPNI_IRQ_LINK_CHANGED | DPNI_IRQ_EP_CHANGED);
	if (error) {
		device_printf(dev, "%s: failed to set DPNI IRQ mask\n",
		    __func__);
		return (error);
	}

	/* Enable IRQ. */
	error = DPAA2_CMD_NI_SET_IRQ_ENABLE(dev, child, cmd, DPNI_IRQ_INDEX,
	    true);
	if (error) {
		device_printf(dev, "%s: failed to enable DPNI IRQ\n", __func__);
		return (error);
	}

	return (0);
}

/**
 * @brief Allocate MSI interrupts for DPNI.
 */
static int
dpaa2_ni_setup_msi(struct dpaa2_ni_softc *sc)
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
 * @brief Update DPNI according to the updated interface capabilities.
 */
static int
dpaa2_ni_setup_if_caps(struct dpaa2_ni_softc *sc)
{
	const bool en_rxcsum = sc->ifp->if_capenable & IFCAP_RXCSUM;
	const bool en_txcsum = sc->ifp->if_capenable & IFCAP_TXCSUM;
	device_t dev = sc->dev;
	device_t child = dev;
	int error;

	/* Setup checksums validation. */
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, child, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token), DPAA2_NI_OFL_RX_L3_CSUM, en_rxcsum);
	if (error) {
		device_printf(dev, "%s: failed to %s L3 checksum validation\n",
		    __func__, en_rxcsum ? "enable" : "disable");
		return (error);
	}
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, child, sc->cmd,
	    DPAA2_NI_OFL_RX_L4_CSUM, en_rxcsum);
	if (error) {
		device_printf(dev, "%s: failed to %s L4 checksum validation\n",
		    __func__, en_rxcsum ? "enable" : "disable");
		return (error);
	}

	/* Setup checksums generation. */
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, child, sc->cmd,
	    DPAA2_NI_OFL_TX_L3_CSUM, en_txcsum);
	if (error) {
		device_printf(dev, "%s: failed to %s L3 checksum generation\n",
		    __func__, en_txcsum ? "enable" : "disable");
		return (error);
	}
	error = DPAA2_CMD_NI_SET_OFFLOAD(dev, child, sc->cmd,
	    DPAA2_NI_OFL_TX_L4_CSUM, en_txcsum);
	if (error) {
		device_printf(dev, "%s: failed to %s L4 checksum generation\n",
		    __func__, en_txcsum ? "enable" : "disable");
		return (error);
	}

	return (0);
}

/**
 * @brief Update DPNI according to the updated interface flags.
 */
static int
dpaa2_ni_setup_if_flags(struct dpaa2_ni_softc *sc)
{
	const bool en_promisc = sc->ifp->if_flags & IFF_PROMISC;
	const bool en_allmulti = sc->ifp->if_flags & IFF_ALLMULTI;
	device_t dev = sc->dev;
	device_t child = dev;
	int error;

	error = DPAA2_CMD_NI_SET_MULTI_PROMISC(dev, child, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token), en_promisc ? true : en_allmulti);
	if (error) {
		device_printf(dev, "%s: failed to %s multicast promiscuous "
		    "mode\n", __func__, en_allmulti ? "enable" : "disable");
		return (error);
	}

	error = DPAA2_CMD_NI_SET_UNI_PROMISC(dev, child, sc->cmd, en_promisc);
	if (error) {
		device_printf(dev, "%s: failed to %s unicast promiscuous mode\n",
		    __func__, en_promisc ? "enable" : "disable");
		return (error);
	}

	return (0);
}

static int
dpaa2_ni_setup_sysctls(struct dpaa2_ni_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *node, *node2;
	struct sysctl_oid_list *parent, *parent2;
	char cbuf[128];
	int i;

	ctx = device_get_sysctl_ctx(sc->dev);
	parent = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	/* Add DPNI statistics. */
	node = SYSCTL_ADD_NODE(ctx, parent, OID_AUTO, "stats",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "DPNI Statistics");
	parent = SYSCTL_CHILDREN(node);
	for (i = 0; i < DPAA2_NI_STAT_SYSCTLS; ++i) {
		SYSCTL_ADD_PROC(ctx, parent, i, dpni_stat_sysctls[i].name,
		    CTLTYPE_U64 | CTLFLAG_RD, sc, 0, dpaa2_ni_collect_stats,
		    "IU", dpni_stat_sysctls[i].desc);
	}
	SYSCTL_ADD_UQUAD(ctx, parent, OID_AUTO, "rx_anomaly_frames",
	    CTLFLAG_RD, &sc->rx_anomaly_frames,
	    "Rx frames in the buffers outside of the buffer pools");
	SYSCTL_ADD_UQUAD(ctx, parent, OID_AUTO, "rx_single_buf_frames",
	    CTLFLAG_RD, &sc->rx_single_buf_frames,
	    "Rx frames in single buffers");
	SYSCTL_ADD_UQUAD(ctx, parent, OID_AUTO, "rx_sg_buf_frames",
	    CTLFLAG_RD, &sc->rx_sg_buf_frames,
	    "Rx frames in scatter/gather list");
	SYSCTL_ADD_UQUAD(ctx, parent, OID_AUTO, "rx_enq_rej_frames",
	    CTLFLAG_RD, &sc->rx_enq_rej_frames,
	    "Enqueue rejected by QMan");
	SYSCTL_ADD_UQUAD(ctx, parent, OID_AUTO, "rx_ieoi_err_frames",
	    CTLFLAG_RD, &sc->rx_ieoi_err_frames,
	    "QMan IEOI error");
	SYSCTL_ADD_UQUAD(ctx, parent, OID_AUTO, "tx_single_buf_frames",
	    CTLFLAG_RD, &sc->tx_single_buf_frames,
	    "Tx single buffer frames");
	SYSCTL_ADD_UQUAD(ctx, parent, OID_AUTO, "tx_sg_frames",
	    CTLFLAG_RD, &sc->tx_sg_frames,
	    "Tx S/G frames");

 	parent = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	/* Add channels statistics. */
	node = SYSCTL_ADD_NODE(ctx, parent, OID_AUTO, "channels",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "DPNI Channels");
	parent = SYSCTL_CHILDREN(node);
	for (int i = 0; i < sc->chan_n; i++) {
		snprintf(cbuf, sizeof(cbuf), "%d", i);

		node2 = SYSCTL_ADD_NODE(ctx, parent, OID_AUTO, cbuf,
		    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "DPNI Channel");
		parent2 = SYSCTL_CHILDREN(node2);

		SYSCTL_ADD_UQUAD(ctx, parent2, OID_AUTO, "tx_frames",
		    CTLFLAG_RD, &sc->channels[i]->tx_frames,
		    "Tx frames counter");
		SYSCTL_ADD_UQUAD(ctx, parent2, OID_AUTO, "tx_dropped",
		    CTLFLAG_RD, &sc->channels[i]->tx_dropped,
		    "Tx dropped counter");
	}

	return (0);
}

static int
dpaa2_ni_setup_dma(struct dpaa2_ni_softc *sc)
{
	device_t dev = sc->dev;
	int error;

	KASSERT((sc->buf_align == BUF_ALIGN) || (sc->buf_align == BUF_ALIGN_V1),
	    ("unexpected buffer alignment: %d\n", sc->buf_align));

	/*
	 * DMA tag to allocate buffers for buffer pool.
	 *
	 * NOTE: QBMan supports DMA addresses up to 49-bits maximum.
	 *	 Bits 63-49 are not used by QBMan.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    sc->buf_align, 0,		/* alignment, boundary */
	    BUF_MAXADDR_49BIT,		/* low restricted addr */
	    BUF_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    BUF_SIZE, 1,		/* maxsize, nsegments */
	    BUF_SIZE, 0,		/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->bp_dmat);
	if (error) {
		device_printf(dev, "%s: failed to create DMA tag for buffer "
		    "pool\n", __func__);
		return (error);
	}

	/* DMA tag to map Tx mbufs. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    sc->buf_align, 0,		/* alignment, boundary */
	    BUF_MAXADDR_49BIT,		/* low restricted addr */
	    BUF_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    DPAA2_TX_SEGS_MAXSZ,	/* maxsize */
	    DPAA2_TX_SEGLIMIT,		/* nsegments */
	    DPAA2_TX_SEG_SZ, 0,		/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->tx_dmat);
	if (error) {
		device_printf(dev, "%s: failed to create DMA tag for Tx "
		    "buffers\n", __func__);
		return (error);
	}

	/* DMA tag to allocate channel storage. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    ETH_STORE_ALIGN, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* low restricted addr */
	    BUS_SPACE_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    ETH_STORE_SIZE, 1,		/* maxsize, nsegments */
	    ETH_STORE_SIZE, 0,		/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->st_dmat);
	if (error) {
		device_printf(dev, "%s: failed to create DMA tag for channel "
		    "storage\n", __func__);
		return (error);
	}

	/* DMA tag for Rx distribution key. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    PAGE_SIZE, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* low restricted addr */
	    BUS_SPACE_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    DPAA2_CLASSIFIER_DMA_SIZE, 1, /* maxsize, nsegments */
	    DPAA2_CLASSIFIER_DMA_SIZE, 0, /* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rxd_dmat);
	if (error) {
		device_printf(dev, "%s: failed to create DMA tag for Rx "
		    "distribution key\n", __func__);
		return (error);
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    PAGE_SIZE, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* low restricted addr */
	    BUS_SPACE_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    ETH_QOS_KCFG_BUF_SIZE, 1,	/* maxsize, nsegments */
	    ETH_QOS_KCFG_BUF_SIZE, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->qos_dmat);
	if (error) {
		device_printf(dev, "%s: failed to create DMA tag for QoS key\n",
		    __func__);
		return (error);
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    PAGE_SIZE, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* low restricted addr */
	    BUS_SPACE_MAXADDR,		/* high restricted addr */
	    NULL, NULL,			/* filter, filterarg */
	    DPAA2_TX_SGT_SZ, 1,		/* maxsize, nsegments */
	    DPAA2_TX_SGT_SZ, 0,		/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->sgt_dmat);
	if (error) {
		device_printf(dev, "%s: failed to create DMA tag for S/G "
		    "tables\n", __func__);
		return (error);
	}

	return (0);
}

/**
 * @brief Configure buffer layouts of the different DPNI queues.
 */
static int
dpaa2_ni_set_buf_layout(device_t dev, struct dpaa2_cmd *cmd)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_ni_buf_layout buf_layout = {0};
	int error;

	/*
	 * Select Rx/Tx buffer alignment. It's necessary to ensure that the
	 * buffer size seen by WRIOP is a multiple of 64 or 256 bytes depending
	 * on the WRIOP version.
	 */
	sc->buf_align = (sc->attr.wriop_ver == WRIOP_VERSION(0, 0, 0) ||
	    sc->attr.wriop_ver == WRIOP_VERSION(1, 0, 0))
	    ? BUF_ALIGN_V1 : BUF_ALIGN;

	/*
	 * We need to ensure that the buffer size seen by WRIOP is a multiple
	 * of 64 or 256 bytes depending on the WRIOP version.
	 */
	sc->buf_sz = ALIGN_DOWN(BUF_SIZE, sc->buf_align);

	if (bootverbose)
		device_printf(dev, "Rx/Tx buffers: size=%d, alignment=%d\n",
		    sc->buf_sz, sc->buf_align);

	/*
	 *    Frame Descriptor       Tx buffer layout
	 *
	 *                ADDR -> |---------------------|
	 *                        | SW FRAME ANNOTATION | BUF_SWA_SIZE bytes
	 *                        |---------------------|
	 *                        | HW FRAME ANNOTATION | BUF_TX_HWA_SIZE bytes
	 *                        |---------------------|
	 *                        |    DATA HEADROOM    |
	 *       ADDR + OFFSET -> |---------------------|
	 *                        |                     |
	 *                        |                     |
	 *                        |     FRAME DATA      |
	 *                        |                     |
	 *                        |                     |
	 *                        |---------------------|
	 *                        |    DATA TAILROOM    |
	 *                        |---------------------|
	 *
	 * NOTE: It's for a single buffer frame only.
	 */
	buf_layout.queue_type = DPAA2_NI_QUEUE_TX;
	buf_layout.pd_size = BUF_SWA_SIZE;
	buf_layout.pass_timestamp = true;
	buf_layout.pass_frame_status = true;
	buf_layout.options =
	    BUF_LOPT_PRIV_DATA_SZ |
	    BUF_LOPT_TIMESTAMP | /* requires 128 bytes in HWA */
	    BUF_LOPT_FRAME_STATUS;
	error = DPAA2_CMD_NI_SET_BUF_LAYOUT(dev, child, cmd, &buf_layout);
	if (error) {
		device_printf(dev, "%s: failed to set Tx buffer layout\n",
		    __func__);
		return (error);
	}

	/* Tx-confirmation buffer layout */
	buf_layout.queue_type = DPAA2_NI_QUEUE_TX_CONF;
	buf_layout.options =
	    BUF_LOPT_TIMESTAMP |
	    BUF_LOPT_FRAME_STATUS;
	error = DPAA2_CMD_NI_SET_BUF_LAYOUT(dev, child, cmd, &buf_layout);
	if (error) {
		device_printf(dev, "%s: failed to set TxConf buffer layout\n",
		    __func__);
		return (error);
	}

	/*
	 * Driver should reserve the amount of space indicated by this command
	 * as headroom in all Tx frames.
	 */
	error = DPAA2_CMD_NI_GET_TX_DATA_OFF(dev, child, cmd, &sc->tx_data_off);
	if (error) {
		device_printf(dev, "%s: failed to obtain Tx data offset\n",
		    __func__);
		return (error);
	}

	if (bootverbose)
		device_printf(dev, "Tx data offset=%d\n", sc->tx_data_off);
	if ((sc->tx_data_off % 64) != 0)
		device_printf(dev, "Tx data offset (%d) is not a multiplication "
		    "of 64 bytes\n", sc->tx_data_off);

	/*
	 *    Frame Descriptor       Rx buffer layout
	 *
	 *                ADDR -> |---------------------|
	 *                        | SW FRAME ANNOTATION | 0 bytes
	 *                        |---------------------|
	 *                        | HW FRAME ANNOTATION | BUF_RX_HWA_SIZE bytes
	 *                        |---------------------|
	 *                        |    DATA HEADROOM    | OFFSET-BUF_RX_HWA_SIZE
	 *       ADDR + OFFSET -> |---------------------|
	 *                        |                     |
	 *                        |                     |
	 *                        |     FRAME DATA      |
	 *                        |                     |
	 *                        |                     |
	 *                        |---------------------|
	 *                        |    DATA TAILROOM    | 0 bytes
	 *                        |---------------------|
	 *
	 * NOTE: It's for a single buffer frame only.
	 */
	buf_layout.queue_type = DPAA2_NI_QUEUE_RX;
	buf_layout.pd_size = 0;
	buf_layout.fd_align = sc->buf_align;
	buf_layout.head_size = sc->tx_data_off - BUF_RX_HWA_SIZE;
	buf_layout.tail_size = 0;
	buf_layout.pass_frame_status = true;
	buf_layout.pass_parser_result = true;
	buf_layout.pass_timestamp = true;
	buf_layout.options =
	    BUF_LOPT_PRIV_DATA_SZ |
	    BUF_LOPT_DATA_ALIGN |
	    BUF_LOPT_DATA_HEAD_ROOM |
	    BUF_LOPT_DATA_TAIL_ROOM |
	    BUF_LOPT_FRAME_STATUS |
	    BUF_LOPT_PARSER_RESULT |
	    BUF_LOPT_TIMESTAMP;
	error = DPAA2_CMD_NI_SET_BUF_LAYOUT(dev, child, cmd, &buf_layout);
	if (error) {
		device_printf(dev, "%s: failed to set Rx buffer layout\n",
		    __func__);
		return (error);
	}

	return (0);
}

/**
 * @brief Enable Rx/Tx pause frames.
 *
 * NOTE: DPNI stops sending when a pause frame is received (Rx frame) or DPNI
 *       itself generates pause frames (Tx frame).
 */
static int
dpaa2_ni_set_pause_frame(device_t dev, struct dpaa2_cmd *cmd)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_ni_link_cfg link_cfg = {0};
	int error;

	error = DPAA2_CMD_NI_GET_LINK_CFG(dev, child, cmd, &link_cfg);
	if (error) {
		device_printf(dev, "%s: failed to obtain link configuration: "
		    "error=%d\n", __func__, error);
		return (error);
	}

	/* Enable both Rx and Tx pause frames by default. */
	link_cfg.options |= DPAA2_NI_LINK_OPT_PAUSE;
	link_cfg.options &= ~DPAA2_NI_LINK_OPT_ASYM_PAUSE;

	error = DPAA2_CMD_NI_SET_LINK_CFG(dev, child, cmd, &link_cfg);
	if (error) {
		device_printf(dev, "%s: failed to set link configuration: "
		    "error=%d\n", __func__, error);
		return (error);
	}

	sc->link_options = link_cfg.options;

	return (0);
}

/**
 * @brief Configure QoS table to determine the traffic class for the received
 * frame.
 */
static int
dpaa2_ni_set_qos_table(device_t dev, struct dpaa2_cmd *cmd)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpaa2_ni_qos_table tbl;
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

	error = bus_dmamem_alloc(sc->qos_dmat, &sc->qos_kcfg.vaddr,
	    BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->qos_kcfg.dmap);
	if (error) {
		device_printf(dev, "%s: failed to allocate a buffer for QoS key "
		    "configuration\n", __func__);
		return (error);
	}

	error = bus_dmamap_load(sc->qos_dmat, sc->qos_kcfg.dmap,
	    sc->qos_kcfg.vaddr, ETH_QOS_KCFG_BUF_SIZE, dpaa2_ni_dmamap_cb,
	    &sc->qos_kcfg.paddr, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(dev, "%s: failed to map QoS key configuration "
		    "buffer into bus space\n", __func__);
		return (error);
	}

	tbl.default_tc = 0;
	tbl.discard_on_miss = false;
	tbl.keep_entries = false;
	tbl.kcfg_busaddr = sc->qos_kcfg.paddr;
	error = DPAA2_CMD_NI_SET_QOS_TABLE(dev, child, cmd, &tbl);
	if (error) {
		device_printf(dev, "%s: failed to set QoS table\n", __func__);
		return (error);
	}

	error = DPAA2_CMD_NI_CLEAR_QOS_TABLE(dev, child, cmd);
	if (error) {
		device_printf(dev, "%s: failed to clear QoS table\n", __func__);
		return (error);
	}

	return (0);
}

static int
dpaa2_ni_set_mac_addr(device_t dev, struct dpaa2_cmd *cmd, uint16_t rc_token,
    uint16_t ni_token)
{
	device_t child = dev;
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
	error = DPAA2_CMD_NI_GET_PORT_MAC_ADDR(dev, child, dpaa2_mcp_tk(cmd,
	    ni_token), mac_addr);
	if (error) {
		device_printf(dev, "%s: failed to obtain the MAC address "
		    "associated with the physical port\n", __func__);
		return (error);
	}

	/* Get primary MAC address from the DPNI attributes. */
	error = DPAA2_CMD_NI_GET_PRIM_MAC_ADDR(dev, child, cmd, dpni_mac_addr);
	if (error) {
		device_printf(dev, "%s: failed to obtain primary MAC address\n",
		    __func__);
		return (error);
	}

	if (!ETHER_IS_ZERO(mac_addr)) {
		/* Set MAC address of the physical port as DPNI's primary one. */
		error = DPAA2_CMD_NI_SET_PRIM_MAC_ADDR(dev, child, cmd,
		    mac_addr);
		if (error) {
			device_printf(dev, "%s: failed to set primary MAC "
			    "address\n", __func__);
			return (error);
		}
		for (int i = 0; i < ETHER_ADDR_LEN; i++)
			sc->mac.addr[i] = mac_addr[i];
	} else if (ETHER_IS_ZERO(dpni_mac_addr)) {
		/* Generate random MAC address as DPNI's primary one. */
		ether_gen_addr(ifp, &rnd_mac_addr);
		for (int i = 0; i < ETHER_ADDR_LEN; i++)
			mac_addr[i] = rnd_mac_addr.octet[i];

		error = DPAA2_CMD_NI_SET_PRIM_MAC_ADDR(dev, child, cmd,
		    mac_addr);
		if (error) {
			device_printf(dev, "%s: failed to set random primary "
			    "MAC address\n", __func__);
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

static void
dpaa2_ni_miibus_statchg(device_t dev)
{
	struct dpaa2_ni_softc *sc;
	device_t child;
	struct dpaa2_mac_link_state mac_link = { 0 };
	uint16_t mac_token;
	int error, link_state;

	sc = device_get_softc(dev);
	if (sc->fixed_link || sc->mii == NULL)
		return;

	/*
	 * Note: ifp link state will only be changed AFTER we are called so we
	 * cannot rely on ifp->if_linkstate here.
	 */
	if (sc->mii->mii_media_status & IFM_AVALID) {
		if (sc->mii->mii_media_status & IFM_ACTIVE)
			link_state = LINK_STATE_UP;
		else
			link_state = LINK_STATE_DOWN;
	} else
		link_state = LINK_STATE_UNKNOWN;

	if (link_state != sc->link_state) {

		sc->link_state = link_state;

		child = sc->dev;
		error = DPAA2_CMD_MAC_OPEN(sc->dev, child, dpaa2_mcp_tk(sc->cmd,
		    sc->rc_token), sc->mac.dpmac_id, &mac_token);
		if (error) {
			device_printf(sc->dev, "%s: failed to open DPMAC: "
			    "id=%d, error=%d\n", __func__, sc->mac.dpmac_id,
			    error);
			return;
		}

		if (link_state == LINK_STATE_UP ||
		    link_state == LINK_STATE_DOWN) {
			/* Update DPMAC link state. */
			mac_link.supported = sc->mii->mii_media.ifm_media;
			mac_link.advert = sc->mii->mii_media.ifm_media;
			mac_link.rate = 1000; /* TODO: Where to get from? */	/* ifmedia_baudrate? */
			mac_link.options =
			    DPAA2_MAC_LINK_OPT_AUTONEG |
			    DPAA2_MAC_LINK_OPT_PAUSE;
			mac_link.up = (link_state == LINK_STATE_UP) ? true : false;
			mac_link.state_valid = true;

			/* Inform DPMAC about link state. */
			error = DPAA2_CMD_MAC_SET_LINK_STATE(sc->dev, child,
			    sc->cmd, &mac_link);
			if (error)
				device_printf(sc->dev, "%s: failed to set DPMAC "
				    "link state: id=%d, error=%d\n", __func__,
				    sc->mac.dpmac_id, error);
		}
		DPAA2_CMD_MAC_CLOSE(sc->dev, child, dpaa2_mcp_tk(sc->cmd,
		    mac_token));
	}
}

/**
 * @brief Callback function to process media change request.
 */
static int
dpaa2_ni_media_change(struct ifnet *ifp)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;

	DPNI_LOCK(sc);
	if (sc->mii) {
		mii_mediachg(sc->mii);
		sc->media_status = sc->mii->mii_media.ifm_media;
	} else if (sc->fixed_link) {
		if_printf(ifp, "%s: can't change media in fixed mode\n",
		    __func__);
	}
	DPNI_UNLOCK(sc);

	return (0);
}

/**
 * @brief Callback function to process media status request.
 */
static void
dpaa2_ni_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
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
 * @brief Callout function to check and update media status.
 */
static void
dpaa2_ni_media_tick(void *arg)
{
	struct dpaa2_ni_softc *sc = (struct dpaa2_ni_softc *) arg;

	/* Check for media type change */
	if (sc->mii) {
		mii_tick(sc->mii);
		if (sc->media_status != sc->mii->mii_media.ifm_media) {
			printf("%s: media type changed (ifm_media=%x)\n",
			    __func__, sc->mii->mii_media.ifm_media);
			dpaa2_ni_media_change(sc->ifp);
		}
	}

	/* Schedule another timeout one second from now */
	callout_reset(&sc->mii_callout, hz, dpaa2_ni_media_tick, sc);
}

static void
dpaa2_ni_init(void *arg)
{
	struct dpaa2_ni_softc *sc = (struct dpaa2_ni_softc *) arg;
	struct ifnet *ifp = sc->ifp;
	device_t dev = sc->dev;
	device_t child = dev;
	int error;

	DPNI_LOCK(sc);
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0) {
		DPNI_UNLOCK(sc);
		return;
	}
	DPNI_UNLOCK(sc);

	error = DPAA2_CMD_NI_ENABLE(dev, child, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token));
	if (error)
		device_printf(dev, "%s: failed to enable DPNI: error=%d\n",
		    __func__, error);

	DPNI_LOCK(sc);
	if (sc->mii)
		mii_mediachg(sc->mii);
	callout_reset(&sc->mii_callout, hz, dpaa2_ni_media_tick, sc);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	DPNI_UNLOCK(sc);

	/* Force link-state update to initilize things. */
	dpaa2_ni_miibus_statchg(dev);

	return;
}

static int
dpaa2_ni_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;
	struct dpaa2_ni_tx_ring *tx;
	int error, chan;

	if (__predict_false(!(ifp->if_drv_flags & IFF_DRV_RUNNING)))
		return (0);

	if (__predict_true(M_HASHTYPE_GET(m) != M_HASHTYPE_NONE))
		/* Select channel based on the mbuf's flowid. */
		chan = m->m_pkthdr.flowid % sc->chan_n;
	else
		chan = 0;

	/* TODO: Select Tx ring based on traffic class. */
	tx = DPAA2_TX_RING(sc, chan, 0);

	/* Enqueue and schedule taskqueue. */
	error = drbr_enqueue(ifp, tx->mbuf_br, m);
	if (__predict_false(error != 0)) {
		device_printf(sc->dev, "%s: drbr_enqueue() failed\n", __func__);
		return (error);
	}
	taskqueue_enqueue(tx->taskq, &tx->task);

	return (0);
}

static void
dpaa2_ni_qflush(struct ifnet *ifp)
{
	/* TODO: Find a way to drain Tx queues in QBMan. */
	if_qflush(ifp);
}

static int
dpaa2_ni_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;
	struct ifreq *ifr = (struct ifreq *) data;
	device_t dev, child;
	uint32_t changed = 0;
	int mtu, error, rc = 0;

	dev = child = sc->dev;

	switch (cmd) {
	case SIOCSIFMTU:
		DPNI_LOCK(sc);
		mtu = ifr->ifr_mtu;
		if (mtu < ETHERMIN || mtu > ETHERMTU_JUMBO) {
			DPNI_UNLOCK(sc);
			return (EINVAL);
		}
		ifp->if_mtu = mtu;
		DPNI_UNLOCK(sc);

		/* Update maximum frame length. */
		error = DPAA2_CMD_NI_SET_MFL(dev, child, dpaa2_mcp_tk(sc->cmd,
		    sc->ni_token), mtu + ETHER_HDR_LEN);
		if (error) {
			device_printf(dev, "%s: failed to update maximum frame "
			    "length: error=%d\n", __func__, error);
			return (error);
		}
		break;
	case SIOCSIFCAP:
		changed = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (changed & IFCAP_HWCSUM) {
			if ((ifr->ifr_reqcap & changed) & IFCAP_HWCSUM)
				ifp->if_capenable |= IFCAP_HWCSUM;
			else
				ifp->if_capenable &= ~IFCAP_HWCSUM;
		}
		rc = dpaa2_ni_setup_if_caps(sc);
		if (rc) {
			printf("%s: failed to update iface capabilities: "
			    "error=%d\n", __func__, rc);
			rc = ENXIO;
		}
		break;
	case SIOCSIFFLAGS:
		DPNI_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				changed = ifp->if_flags ^ sc->if_flags;
				if (changed & IFF_PROMISC ||
				    changed & IFF_ALLMULTI) {
					rc = dpaa2_ni_setup_if_flags(sc);
				}
			} else {
				DPNI_UNLOCK(sc);
				dpaa2_ni_init(sc);
				DPNI_LOCK(sc);
			}
		} else if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			/* dpni_if_stop(sc); */
		}

		sc->if_flags = ifp->if_flags;
		DPNI_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		DPNI_LOCK(sc);
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			DPNI_UNLOCK(sc);
			rc = dpaa2_ni_update_mac_filters(ifp);
			if (rc)
				device_printf(dev, "%s: failed to update MAC "
				    "filters: error=%d\n", __func__, rc);
			DPNI_LOCK(sc);
		}
		DPNI_UNLOCK(sc);
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		if (sc->mii)
			rc = ifmedia_ioctl(ifp, ifr, &sc->mii->mii_media, cmd);
		else if(sc->fixed_link) {
			rc = ifmedia_ioctl(ifp, ifr, &sc->fixed_ifmedia, cmd);
		}
		break;
	default:
		rc = ether_ioctl(ifp, cmd, data);
	}

	return (rc);
}

static int
dpaa2_ni_update_mac_filters(struct ifnet *ifp)
{
	struct dpaa2_ni_softc *sc = ifp->if_softc;
	struct dpaa2_ni_mcaddr_ctx ctx;
	device_t dev, child;
	int error;

	dev = child = sc->dev;

	/* Remove all multicast MAC filters. */
	error = DPAA2_CMD_NI_CLEAR_MAC_FILTERS(dev, child, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token), false, true);
	if (error) {
		device_printf(dev, "%s: failed to clear multicast MAC filters: "
		    "error=%d\n", __func__, error);
		return (error);
	}

	ctx.ifp = ifp;
	ctx.error = 0;
	ctx.nent = 0;

	if_foreach_llmaddr(ifp, dpaa2_ni_add_maddr, &ctx);

	return (ctx.error);
}

static u_int
dpaa2_ni_add_maddr(void *arg, struct sockaddr_dl *sdl, u_int cnt)
{
	struct dpaa2_ni_mcaddr_ctx *ctx = arg;
	struct dpaa2_ni_softc *sc = ctx->ifp->if_softc;
	device_t dev, child;

	dev = child = sc->dev;

	if (ctx->error != 0)
		return (0);

	if (ETHER_IS_MULTICAST(LLADDR(sdl))) {
		ctx->error = DPAA2_CMD_NI_ADD_MAC_ADDR(dev, child, dpaa2_mcp_tk(
		    sc->cmd, sc->ni_token), LLADDR(sdl));
		if (ctx->error != 0) {
			device_printf(dev, "%s: can't add more then %d MAC "
			    "addresses, switching to the multicast promiscuous "
			    "mode\n", __func__, ctx->nent);

			/* Enable multicast promiscuous mode. */
			DPNI_LOCK(sc);
			ctx->ifp->if_flags |= IFF_ALLMULTI;
			sc->if_flags |= IFF_ALLMULTI;
			ctx->error = dpaa2_ni_setup_if_flags(sc);
			DPNI_UNLOCK(sc);

			return (0);
		}
		ctx->nent++;
	}

	return (1);
}

static void
dpaa2_ni_intr(void *arg)
{
	struct dpaa2_ni_softc *sc = (struct dpaa2_ni_softc *) arg;
	device_t child = sc->dev;
	uint32_t status = ~0u; /* clear all IRQ status bits */
	int error;

	error = DPAA2_CMD_NI_GET_IRQ_STATUS(sc->dev, child, dpaa2_mcp_tk(sc->cmd,
	    sc->ni_token), DPNI_IRQ_INDEX, &status);
	if (error)
		device_printf(sc->dev, "%s: failed to obtain IRQ status: "
		    "error=%d\n", __func__, error);
}

/**
 * @brief Callback to obtain a physical address of the only DMA segment mapped.
 */
static void
dpaa2_ni_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error == 0) {
		KASSERT(nseg == 1, ("too many segments: nseg=%d\n", nseg));
		*(bus_addr_t *) arg = segs[0].ds_addr;
	}
}

/**
 * @brief Task to poll frames from a specific channel when CDAN is received.
 */
static void
dpaa2_ni_poll_task(void *arg, int count)
{
	struct dpaa2_ni_channel *chan = (struct dpaa2_ni_channel *) arg;
	struct dpaa2_ni_softc *sc = device_get_softc(chan->ni_dev);
	struct dpaa2_io_softc *iosc = device_get_softc(chan->io_dev);
	struct dpaa2_swp *swp = iosc->swp;
	struct dpaa2_ni_fq *fq;
	int error, consumed = 0;

	do {
		error = dpaa2_swp_pull(swp, chan->id, chan->store.paddr,
		    ETH_STORE_FRAMES);
		if (error) {
			device_printf(chan->ni_dev, "%s: failed to pull frames: "
			    "chan_id=%d, error=%d\n", __func__, chan->id, error);
			break;
		}

		/* Let's sync DQRR response from QBMan. */
		bus_dmamap_sync(sc->st_dmat, chan->store.dmap,
		    BUS_DMASYNC_POSTREAD);

		error = dpaa2_ni_consume_frames(chan, &fq, &consumed);
		if (error == ENOENT)
			break;
		if (error == ETIMEDOUT)
			device_printf(chan->ni_dev, "%s: timeout to consume "
			    "frames: chan_id=%d\n", __func__, chan->id);
	} while (true);

	/* Re-arm channel to generate CDAN. */
	error = DPAA2_SWP_CONF_WQ_CHANNEL(chan->io_dev, &chan->ctx);
	if (error)
		device_printf(chan->ni_dev, "%s: failed to rearm: chan_id=%d, "
		    "error=%d\n", __func__, chan->id, error);
}

/**
 * @brief Task to transmit mbufs.
 */
static void
dpaa2_ni_tx_task(void *arg, int count)
{
	struct dpaa2_ni_tx_ring *tx = (struct dpaa2_ni_tx_ring *) arg;
	struct dpaa2_ni_fq *fq = tx->fq;
	struct dpaa2_ni_channel	*chan = fq->chan;
	struct dpaa2_ni_softc *sc = device_get_softc(chan->ni_dev);
	struct dpaa2_ni_buf *txb;
	struct dpaa2_fd fd;
	struct mbuf *m, *m_d;
	bus_dma_segment_t txsegs[DPAA2_TX_SEGLIMIT];
	uint64_t idx;
	void *pidx;
	int error, rc, txnsegs;

	while ((m = drbr_peek(sc->ifp, tx->mbuf_br)) != NULL) {
		/* Obtain an index of a Tx buffer. */
		pidx = buf_ring_dequeue_mc(tx->idx_br);
		if (__predict_false(pidx == NULL)) {
			/* TODO: Limit number of attempts. */
			continue;
		} else {
			idx = (uint64_t) pidx;
			txb = &tx->buf[idx];
			txb->m = m;
			txb->idx = idx;
			txb->sgt_paddr = 0;
		}

		/* Load mbuf to transmit. */
		error = bus_dmamap_load_mbuf_sg(sc->tx_dmat, txb->dmap, m,
		    txsegs, &txnsegs, BUS_DMA_NOWAIT);
		if (__predict_false(error != 0)) {
			/* Too many fragments, trying to defragment... */
			m_d = m_collapse(m, M_NOWAIT, DPAA2_TX_SEGLIMIT);
			if (m_d == NULL) {
				device_printf(sc->dev, "%s: mbuf "
				    "defragmentation failed\n", __func__);
				fq->chan->tx_dropped++;
				goto loop_err;
			}

			txb->m = m = m_d;
			error = bus_dmamap_load_mbuf_sg(sc->tx_dmat, txb->dmap,
			    m, txsegs, &txnsegs, BUS_DMA_NOWAIT);
			if (__predict_false(error != 0)) {
				device_printf(sc->dev, "%s: failed to load "
				    "mbuf: error=%d\n", __func__, error);
				fq->chan->tx_dropped++;
				goto loop_err;
			}
		}

		/* Build frame descriptor. */
		error = dpaa2_ni_build_fd(sc, tx, txb, txsegs, txnsegs, &fd);
		if (__predict_false(error != 0)) {
			device_printf(sc->dev, "%s: failed to build frame "
			    "descriptor: error=%d\n", __func__, error);
			fq->chan->tx_dropped++;
			goto loop_err_unload;
		}

		/* TODO: Enqueue more frames in a single command. */
		for (int i = 0; i < DPAA2_NI_ENQUEUE_RETRIES; i++) {
			rc = DPAA2_SWP_ENQ_MULTIPLE_FQ(fq->chan->io_dev,
			    tx->fqid, &fd, 1);
			if (rc == 1) /* TODO: Return error codes instead. */
				break;
		}

		bus_dmamap_sync(sc->tx_dmat, txb->dmap, BUS_DMASYNC_PREWRITE);
		bus_dmamap_sync(sc->sgt_dmat, txb->sgt_dmap,
		    BUS_DMASYNC_PREWRITE);

		if (rc != 1) {
			fq->chan->tx_dropped++;
			goto loop_err_unload;
		} else {
			fq->chan->tx_frames++;
		}
		goto loop_next;

loop_err_unload:
		bus_dmamap_unload(sc->tx_dmat, txb->dmap);
		if (txb->sgt_paddr != 0)
			bus_dmamap_unload(sc->sgt_dmat, txb->sgt_dmap);
loop_err:
		m_freem(txb->m);
		buf_ring_enqueue(tx->idx_br, pidx);
loop_next:
		drbr_advance(sc->ifp, tx->mbuf_br);
	}
}

static int
dpaa2_ni_consume_frames(struct dpaa2_ni_channel *chan, struct dpaa2_ni_fq **src,
    uint32_t *consumed)
{
	struct dpaa2_ni_fq *fq = NULL;
	struct dpaa2_dq *dq;
	struct dpaa2_fd *fd;
	int frames = 0, retries = 0;
	int rc;

	do {
		rc = dpaa2_ni_chan_storage_next(chan, &dq);
		if (rc == EAGAIN) {
			if (retries >= DPAA2_SWP_BUSY_RETRIES) {
				rc = ETIMEDOUT;
				break;
			}
			retries++;
			continue;
		} else if (rc == EINPROGRESS) {
			if (dq != NULL && !IS_NULL_RESPONSE(dq->fdr.desc.stat)) {
				fd = &dq->fdr.fd;
				fq = (struct dpaa2_ni_fq *) dq->fdr.desc.fqd_ctx;
				fq->consume(chan, fq, fd);
				frames++;
			}
		} else if (rc == EALREADY || rc == ENOENT) {
			if (dq != NULL && !IS_NULL_RESPONSE(dq->fdr.desc.stat)) {
				fd = &dq->fdr.fd;
				fq = (struct dpaa2_ni_fq *) dq->fdr.desc.fqd_ctx;
				fq->consume(chan, fq, fd);
				frames++;
			}
			break;
		} else {
			KASSERT(1 == 0, ("%s: should not reach here", __func__));
		}
		retries = 0;
	} while (true);

	KASSERT(chan->store_idx < chan->store_sz,
	    ("channel store idx >= size: store_idx=%d, store_sz=%d",
	    chan->store_idx, chan->store_sz));

	/*
	 * A dequeue operation pulls frames from a single queue into the store.
	 * Return the frame queue and a number of consumed frames as an output.
	 */
	if (src != NULL)
		*src = fq;
	if (consumed != NULL)
		*consumed = frames;

	return (rc);
}

/**
 * @brief Main routine to receive frames.
 */
static int
dpaa2_ni_rx(struct dpaa2_ni_channel *chan, struct dpaa2_ni_fq *fq,
    struct dpaa2_fd *fd)
{
	struct dpaa2_ni_softc *sc = device_get_softc(chan->ni_dev);
	struct dpaa2_bp_softc *bpsc;
	struct dpaa2_ni_channel	*buf_chan;
	struct dpaa2_ni_buf *buf;
	struct ifnet *ifp = sc->ifp;
	struct mbuf *m;
	device_t bp_dev;
	bus_addr_t paddr = (bus_addr_t) fd->addr;
	bus_addr_t released[DPAA2_SWP_BUFS_PER_CMD];
	void *buf_data;
	int chan_idx, buf_idx, buf_len;
	int error, released_n = 0;

	/*
	 * Get channel and buffer indexes from the ADDR_TOK (not used by QBMan)
	 * bits of the physical address.
	 */
	chan_idx = dpaa2_ni_fd_chan_idx(fd);
	buf_idx = dpaa2_ni_fd_buf_idx(fd);
	buf_chan = sc->channels[chan_idx];
	buf = &buf_chan->buf[buf_idx];

	KASSERT(paddr == buf->paddr, ("%s: unexpected frame buffer: "
	    "fd_addr(%#jx) != buf_paddr(%#jx)", __func__, paddr, buf->paddr));
	if (paddr != buf->paddr) {
		sc->rx_anomaly_frames++;
		return (0);
	}

	/* Update statistics. */
	switch (dpaa2_ni_fd_err(fd)) {
	case 1: /* Enqueue rejected by QMan */
		sc->rx_enq_rej_frames++;
		break;
	case 2: /* QMan IEOI error */
		sc->rx_ieoi_err_frames++;
		break;
	default:
		break;
	}
	switch (dpaa2_ni_fd_format(fd)) {
	case DPAA2_FD_SINGLE:
		sc->rx_single_buf_frames++;
		break;
	case DPAA2_FD_SG:
		sc->rx_sg_buf_frames++;
		break;
	default:
		break;
	}

	m = buf->m;
	buf->m = NULL;
	bus_dmamap_sync(sc->bp_dmat, buf->dmap,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(sc->bp_dmat, buf->dmap);

	buf_len = dpaa2_ni_fd_data_len(fd);
	buf_data = (uint8_t *)buf->vaddr + dpaa2_ni_fd_offset(fd);

	/* Prefetch mbuf data. */
	__builtin_prefetch(buf_data);

	/* Write value to mbuf (avoid reading). */
	m->m_data = buf_data;
	m->m_len = buf_len;
	m->m_pkthdr.len = buf_len;
	m->m_pkthdr.rcvif = ifp;

	(*ifp->if_input)(ifp, m);

	/* Keep the buffer to be recycled. */
	chan->recycled[chan->recycled_n++] = paddr;

	/* Re-seed and release recycled buffers back to the pool. */
	if (chan->recycled_n == DPAA2_SWP_BUFS_PER_CMD) {
		for (int i = 0; i < chan->recycled_n; i++) {
			paddr = chan->recycled[i];

			/* Parse ADDR_TOK of the recycled buffer. */
			chan_idx = (paddr >> DPAA2_NI_BUF_CHAN_SHIFT)
			    & DPAA2_NI_BUF_CHAN_MASK;
			buf_idx = (paddr >> DPAA2_NI_BUF_IDX_SHIFT)
			    & DPAA2_NI_BUF_IDX_MASK;
			buf_chan = sc->channels[chan_idx];
			buf = &buf_chan->buf[buf_idx];

			/* Seed recycled buffer. */
			error = dpaa2_ni_seed_buf(sc, buf_chan, buf, buf_idx);
			KASSERT(error == 0, ("Failed to seed recycled buffer: "
			    "error=%d", error));
			if (__predict_false(error != 0)) {
				device_printf(sc->dev, "%s: failed to seed "
				    "recycled buffer: error=%d\n", __func__,
				    error);
				continue;
			}

			/* Prepare buffer to be released in a single command. */
			released[released_n++] = buf->paddr;
		}

		/* There's only one buffer pool for now. */
		bp_dev = (device_t) rman_get_start(sc->res[BP_RID(0)]);
		bpsc = device_get_softc(bp_dev);

		error = DPAA2_SWP_RELEASE_BUFS(chan->io_dev, bpsc->attr.bpid,
		    released, released_n);
		if (__predict_false(error != 0)) {
			device_printf(sc->dev, "%s: failed to release buffers "
			    "to the pool: error=%d\n", __func__, error);
			return (error);
		}

		/* Be ready to recycle the next portion of the buffers. */
		chan->recycled_n = 0;
	}

	return (0);
}

/**
 * @brief Main routine to receive Rx error frames.
 */
static int
dpaa2_ni_rx_err(struct dpaa2_ni_channel *chan, struct dpaa2_ni_fq *fq,
    struct dpaa2_fd *fd)
{
	device_t bp_dev;
	struct dpaa2_ni_softc *sc = device_get_softc(chan->ni_dev);
	struct dpaa2_bp_softc *bpsc;
	struct dpaa2_ni_channel	*buf_chan;
	struct dpaa2_ni_buf *buf;
	bus_addr_t paddr = (bus_addr_t) fd->addr;
	int chan_idx, buf_idx;
	int error;
	
	/*
	 * Get channel and buffer indexes from the ADDR_TOK (not used by QBMan)
	 * bits of the physical address.
	 */
	chan_idx = dpaa2_ni_fd_chan_idx(fd);
	buf_idx = dpaa2_ni_fd_buf_idx(fd);
	buf_chan = sc->channels[chan_idx];
	buf = &buf_chan->buf[buf_idx];

	KASSERT(paddr == buf->paddr, ("%s: unexpected frame buffer: "
	    "fd_addr(%#jx) != buf_paddr(%#jx)", __func__, paddr, buf->paddr));
	if (paddr != buf->paddr) {
		sc->rx_anomaly_frames++;
		return (0);
	}

	/* There's only one buffer pool for now. */
	bp_dev = (device_t) rman_get_start(sc->res[BP_RID(0)]);
	bpsc = device_get_softc(bp_dev);

	/* Release buffer to QBMan buffer pool. */
	error = DPAA2_SWP_RELEASE_BUFS(chan->io_dev, bpsc->attr.bpid, &paddr, 1);
	if (error) {
		device_printf(sc->dev, "%s: failed to release frame buffer to "
		    "the pool: error=%d\n", __func__, error);
		return (error);
	}

	return (0);
}

/**
 * @brief Main routine to receive Tx confirmation frames.
 */
static int
dpaa2_ni_tx_conf(struct dpaa2_ni_channel *chan, struct dpaa2_ni_fq *fq,
    struct dpaa2_fd *fd)
{
	struct dpaa2_ni_softc *sc = device_get_softc(chan->ni_dev);
	struct dpaa2_ni_channel	*buf_chan;
	struct dpaa2_ni_tx_ring *tx;
	struct dpaa2_ni_buf *txb;
	bus_addr_t paddr = (bus_addr_t) (fd->addr & BUF_MAXADDR_49BIT);
	uint64_t buf_idx;
	int chan_idx, tx_idx;

	/*
	 * Get channel, Tx ring and buffer indexes from the ADDR_TOK bits
	 * (not used by QBMan) of the physical address.
	 */
	chan_idx = dpaa2_ni_fd_chan_idx(fd);
	tx_idx = dpaa2_ni_fd_tx_idx(fd);
	buf_idx = (uint64_t) dpaa2_ni_fd_txbuf_idx(fd);

	KASSERT(tx_idx < DPAA2_NI_MAX_TCS, ("%s: invalid Tx ring index",
	    __func__));
	KASSERT(buf_idx < DPAA2_NI_BUFS_PER_TX, ("%s: invalid Tx buffer index",
	    __func__));

	buf_chan = sc->channels[chan_idx];
	tx = &buf_chan->txc_queue.tx_rings[tx_idx];
	txb = &tx->buf[buf_idx];

	KASSERT(paddr == txb->paddr, ("%s: unexpected frame buffer: "
	    "fd_addr(%#jx) != txb_paddr(%#jx)", __func__, fd->addr, txb->paddr));
	if (paddr != txb->paddr) {
		sc->rx_anomaly_frames++;
		return (0);
	}

	bus_dmamap_unload(sc->tx_dmat, txb->dmap);
	if (txb->sgt_paddr != 0)
		bus_dmamap_unload(sc->sgt_dmat, txb->sgt_dmap);
	m_freem(txb->m);

	/* Return Tx buffer index back to the ring. */
	buf_ring_enqueue(tx->idx_br, (void *) buf_idx);

	return (0);
}

/**
 * @brief Compare versions of the DPAA2 network interface API.
 */
static int
dpaa2_ni_cmp_api_version(struct dpaa2_ni_softc *sc, uint16_t major,
    uint16_t minor)
{
	if (sc->api_major == major)
		return sc->api_minor - minor;
	return sc->api_major - major;
}

/**
 * @brief Allocate buffers visible to QBMan and release them to the buffer pool.
 */
static int
dpaa2_ni_seed_buf_pool(struct dpaa2_ni_softc *sc, struct dpaa2_ni_channel *chan)
{
	device_t bp_dev;
	struct dpaa2_bp_softc *bpsc;
	struct dpaa2_ni_buf *buf;
	bus_addr_t paddr[DPAA2_SWP_BUFS_PER_CMD];
	int i, j, error, bufn, chan_bufn, buf_idx;
	bool stop = false;

	/* DMA tag for buffer pool must already be created. */
	if (sc->bp_dmat == NULL)
		return (ENXIO);

	/* There's only one buffer pool for now. */
	bp_dev = (device_t) rman_get_start(sc->res[BP_RID(0)]);
	bpsc = device_get_softc(bp_dev);

	KASSERT(DPAA2_NI_BUFS_PER_CHAN <= DPAA2_NI_MAX_BPC,
	    ("%s: too many buffers (%d) per channel: max=%d", __func__,
	    DPAA2_NI_BUFS_PER_CHAN, DPAA2_NI_MAX_BPC));

	chan_bufn = 0;

	for (i = 0; i < DPAA2_NI_BUFS_PER_CHAN; i += DPAA2_SWP_BUFS_PER_CMD) {
		bufn = 0;
		/* Allocate enough buffers to release in one QBMan command. */
		for (j = 0; j < DPAA2_SWP_BUFS_PER_CMD; j++) {
			buf_idx = i + j;
			if (buf_idx >= DPAA2_NI_BUFS_PER_CHAN) {
				/* Release the last buffers to the pool. */
				stop = true;
				break;
			}

			buf = &chan->buf[buf_idx];
			buf->dmap = NULL;
			buf->m = NULL;
			buf->paddr = 0;
			buf->vaddr = NULL;

			error = dpaa2_ni_seed_buf(sc, chan, buf, buf_idx);
			if (error) {
				/* Release some buffers to the pool at least. */
				stop = true;
				break;
			}
			paddr[bufn] = buf->paddr;
			bufn++;
			chan_bufn++;
		}

		/* Release buffer back to the buffer pool. */
		error = DPAA2_SWP_RELEASE_BUFS(chan->io_dev, bpsc->attr.bpid,
		    paddr, bufn);
		if (error) {
			device_printf(sc->dev, "%s: failed to release buffers "
			    "to the buffer pool\n", __func__);
			return (error);
		}

		if (stop)
			break; /* Stop seeding buffers after error. */
	}
	chan->buf_num = chan_bufn;

	return (0);
}

/**
 * @brief Prepares a buffer to be released to the buffer pool.
 */
static int
dpaa2_ni_seed_buf(struct dpaa2_ni_softc *sc, struct dpaa2_ni_channel *chan,
    struct dpaa2_ni_buf *buf, int buf_idx)
{
	struct mbuf *m;
	bus_dmamap_t dmap;
	bus_dma_segment_t segs;
	int error, nsegs;

	/* Create a DMA map for the giving buffer if it doesn't exist yet. */
	if (buf->dmap == NULL) {
		error = bus_dmamap_create(sc->bp_dmat, 0, &dmap);
		if (error) {
			device_printf(sc->dev, "%s: failed to create DMA map "
			    "for buffer: buf_idx=%d, error=%d\n", __func__,
			    buf_idx, error);
			return (error);
		}
		buf->dmap = dmap;
	}

	/* Allocate mbuf if needed. */
	if (buf->m == NULL) {
		m = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, BUF_SIZE);
		if (__predict_false(m == NULL)) {
			device_printf(sc->dev, "%s: failed to allocate mbuf for "
			    "buffer\n", __func__);
			return (ENOMEM);
		}
		m->m_len = m->m_ext.ext_size;
		m->m_pkthdr.len = m->m_ext.ext_size;
		buf->m = m;
	} else
		m = buf->m;

	error = bus_dmamap_load_mbuf_sg(sc->bp_dmat, buf->dmap,
	    m, &segs, &nsegs, BUS_DMA_NOWAIT);
	KASSERT(nsegs == 1, ("too many segments: nsegs=%d", nsegs));
	KASSERT(error == 0, ("failed to map mbuf: error=%d", error));
	if (__predict_false(error != 0 || nsegs != 1)) {
		device_printf(sc->dev, "%s: failed to map mbuf: error=%d, "
		    "nsegs=%d\n", __func__, error, nsegs);
		bus_dmamap_unload(sc->bp_dmat, buf->dmap);
		m_freem(m);
		return (error);
	}
	buf->paddr = segs.ds_addr;
	buf->vaddr = m->m_data;

	/*
	 * Write channel and buffer indices to the ADDR_TOK (bits 63-49) which
	 * is not used by QBMan and is supposed to assist in physical to virtual
	 * address translation.
	 *
	 * NOTE: "lowaddr" and "highaddr" of the window which cannot be accessed
	 * 	 by QBMan must be configured in the DMA tag accordingly.
	 */
	buf->paddr =
	    ((uint64_t)(chan->flowid & DPAA2_NI_BUF_CHAN_MASK) <<
		DPAA2_NI_BUF_CHAN_SHIFT) |
	    ((uint64_t)(buf_idx & DPAA2_NI_BUF_IDX_MASK) <<
		DPAA2_NI_BUF_IDX_SHIFT) |
	    (buf->paddr & DPAA2_NI_BUF_ADDR_MASK);

	return (0);
}

/**
 * @brief Allocate channel storage visible to QBMan.
 */
static int
dpaa2_ni_seed_chan_storage(struct dpaa2_ni_softc *sc,
    struct dpaa2_ni_channel *chan)
{
	struct dpaa2_ni_sbuf *store = &chan->store;
	int error;

	/* DMA tag for channel storage must already be created. */
	if (sc->st_dmat == NULL)
		return (ENXIO);

	error = bus_dmamem_alloc(sc->st_dmat, (void *) &store->vaddr,
	    BUS_DMA_ZERO | BUS_DMA_COHERENT, &store->dmap);
	if (error) {
		device_printf(sc->dev, "%s: failed to allocate channel "
		    "storage\n", __func__);
		return (error);
	}

	error = bus_dmamap_load(sc->st_dmat, store->dmap, (void *) store->vaddr,
	    ETH_STORE_SIZE, dpaa2_ni_dmamap_cb, &store->paddr, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(sc->dev, "%s: failed to map channel storage\n",
		    __func__);
		return (error);
	}

	chan->store_sz = ETH_STORE_FRAMES;
	chan->store_idx = 0;

	return (0);
}

/*
 * @brief Build a DPAA2 frame descriptor.
 */
static int
dpaa2_ni_build_fd(struct dpaa2_ni_softc *sc, struct dpaa2_ni_tx_ring *tx,
    struct dpaa2_ni_buf *txb, bus_dma_segment_t *txsegs, int txnsegs,
    struct dpaa2_fd *fd)
{
	struct dpaa2_ni_channel	*chan = tx->fq->chan;
	struct dpaa2_sg_entry *sgt;
	int i, error;

	KASSERT(txnsegs <= DPAA2_TX_SEGLIMIT, ("%s: too many segments, "
	    "txnsegs (%d) > %d", __func__, txnsegs, DPAA2_TX_SEGLIMIT));
	KASSERT(txb->sgt_vaddr != NULL, ("%s: S/G table not allocated?",
	    __func__));

	/* Reset frame descriptor fields. */
	memset(fd, 0, sizeof(*fd));

	if (__predict_true(txnsegs <= DPAA2_TX_SEGLIMIT)) {
		/* Populate S/G table. */
		sgt = (struct dpaa2_sg_entry *) txb->sgt_vaddr + sc->tx_data_off;
		for (i = 0; i < txnsegs; i++) {
			sgt[i].addr = (uint64_t) txsegs[i].ds_addr;
			sgt[i].len = (uint32_t) txsegs[i].ds_len;
			sgt[i].offset_fmt = 0u;
		}
		sgt[i-1].offset_fmt |= 0x8000u; /* set final entry flag */

		/* Load S/G table. */
		error = bus_dmamap_load(sc->sgt_dmat, txb->sgt_dmap,
		    txb->sgt_vaddr, DPAA2_TX_SGT_SZ, dpaa2_ni_dmamap_cb,
		    &txb->sgt_paddr, BUS_DMA_NOWAIT);
		if (error != 0) {
			device_printf(sc->dev, "%s: failed to map S/G table: "
			    "error=%d\n", __func__, error);
			return (error);
		}
		txb->paddr = txb->sgt_paddr;
		txb->vaddr = txb->sgt_vaddr;
		sc->tx_sg_frames++; /* for sysctl(9) */
	} else {
		return (EINVAL);
	}

	fd->addr =
	    ((uint64_t)(chan->flowid & DPAA2_NI_BUF_CHAN_MASK) <<
		DPAA2_NI_BUF_CHAN_SHIFT) |
	    ((uint64_t)(tx->txid & DPAA2_NI_TX_IDX_MASK) <<
		DPAA2_NI_TX_IDX_SHIFT) |
	    ((uint64_t)(txb->idx & DPAA2_NI_TXBUF_IDX_MASK) <<
		DPAA2_NI_TXBUF_IDX_SHIFT) |
	    (txb->paddr & DPAA2_NI_BUF_ADDR_MASK);

	fd->data_length = (uint32_t) txb->m->m_pkthdr.len;
	fd->bpid_ivp_bmt = 0;
	fd->offset_fmt_sl = 0x2000u | sc->tx_data_off;
	fd->ctrl = 0x00800000u;

	return (0);
}

static int
dpaa2_ni_fd_err(struct dpaa2_fd *fd)
{
	return ((fd->ctrl >> DPAA2_NI_FD_ERR_SHIFT) & DPAA2_NI_FD_ERR_MASK);
}

static uint32_t
dpaa2_ni_fd_data_len(struct dpaa2_fd *fd)
{
	if (dpaa2_ni_fd_short_len(fd))
		return (fd->data_length & DPAA2_NI_FD_LEN_MASK);

	return (fd->data_length);
}

static int
dpaa2_ni_fd_chan_idx(struct dpaa2_fd *fd)
{
	return ((((bus_addr_t) fd->addr) >> DPAA2_NI_BUF_CHAN_SHIFT) &
	    DPAA2_NI_BUF_CHAN_MASK);
}

static int
dpaa2_ni_fd_buf_idx(struct dpaa2_fd *fd)
{
	return ((((bus_addr_t) fd->addr) >> DPAA2_NI_BUF_IDX_SHIFT) &
	    DPAA2_NI_BUF_IDX_MASK);
}

static int
dpaa2_ni_fd_tx_idx(struct dpaa2_fd *fd)
{
	return ((((bus_addr_t) fd->addr) >> DPAA2_NI_TX_IDX_SHIFT) &
	    DPAA2_NI_TX_IDX_MASK);
}

static int
dpaa2_ni_fd_txbuf_idx(struct dpaa2_fd *fd)
{
	return ((((bus_addr_t) fd->addr) >> DPAA2_NI_TXBUF_IDX_SHIFT) &
	    DPAA2_NI_TXBUF_IDX_MASK);
}

static int
dpaa2_ni_fd_format(struct dpaa2_fd *fd)
{
	return ((enum dpaa2_fd_format)((fd->offset_fmt_sl >>
	    DPAA2_NI_FD_FMT_SHIFT) & DPAA2_NI_FD_FMT_MASK));
}

static bool
dpaa2_ni_fd_short_len(struct dpaa2_fd *fd)
{
	return (((fd->offset_fmt_sl >> DPAA2_NI_FD_SL_SHIFT)
	    & DPAA2_NI_FD_SL_MASK) == 1);
}

static int
dpaa2_ni_fd_offset(struct dpaa2_fd *fd)
{
	return (fd->offset_fmt_sl & DPAA2_NI_FD_OFFSET_MASK);
}

/**
 * @brief Collect statistics of the network interface.
 */
static int
dpaa2_ni_collect_stats(SYSCTL_HANDLER_ARGS)
{
	struct dpaa2_ni_softc *sc = (struct dpaa2_ni_softc *) arg1;
	struct dpni_stat *stat = &dpni_stat_sysctls[oidp->oid_number];
	device_t child = sc->dev;
	uint64_t cnt[DPAA2_NI_STAT_COUNTERS];
	uint64_t result = 0;
	int error;

	error = DPAA2_CMD_NI_GET_STATISTICS(sc->dev, child,
	    dpaa2_mcp_tk(sc->cmd, sc->ni_token), stat->page, 0, cnt);
	if (!error)
		result = cnt[stat->cnt];

	return (sysctl_handle_64(oidp, &result, 0, req));
}

static int
dpaa2_ni_set_hash(device_t dev, uint64_t flags)
{
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	uint64_t key = 0;
	int i;

	if (!(sc->attr.num.queues > 1))
		return (EOPNOTSUPP);

	for (i = 0; i < ARRAY_SIZE(dist_fields); i++)
		if (dist_fields[i].rxnfc_field & flags)
			key |= dist_fields[i].id;

	return dpaa2_ni_set_dist_key(dev, DPAA2_NI_DIST_MODE_HASH, key);
}

/**
 * @brief Set Rx distribution (hash or flow classification) key flags is a
 * combination of RXH_ bits.
 */
static int
dpaa2_ni_set_dist_key(device_t dev, enum dpaa2_ni_dist_mode type, uint64_t flags)
{
	device_t child = dev;
	struct dpaa2_ni_softc *sc = device_get_softc(dev);
	struct dpkg_profile_cfg cls_cfg;
	struct dpkg_extract *key;
	int i, err = 0;

	/* DMA tag for Rx traffic distribution key must already be created. */
	if (sc->rxd_dmat == NULL) {
		device_printf(dev, "%s: DMA tag for Rx traffic distribution "
		    "key is NULL\n", __func__);
		return (ENXIO);
	}

	memset(&cls_cfg, 0, sizeof(cls_cfg));

	/* Configure extracts according to the given flags. */
	for (i = 0; i < ARRAY_SIZE(dist_fields); i++) {
		key = &cls_cfg.extracts[cls_cfg.num_extracts];

		if (!(flags & dist_fields[i].id))
			continue;

		if (cls_cfg.num_extracts >= DPKG_MAX_NUM_OF_EXTRACTS) {
			device_printf(dev, "%s: failed to add key extraction "
			    "rule\n", __func__);
			return (E2BIG);
		}

		key->type = DPKG_EXTRACT_FROM_HDR;
		key->extract.from_hdr.prot = dist_fields[i].cls_prot;
		key->extract.from_hdr.type = DPKG_FULL_FIELD;
		key->extract.from_hdr.field = dist_fields[i].cls_field;
		cls_cfg.num_extracts++;
	}

	err = bus_dmamem_alloc(sc->rxd_dmat, &sc->rxd_kcfg.vaddr,
	    BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->rxd_kcfg.dmap);
	if (err) {
		device_printf(dev, "%s: failed to allocate a buffer for Rx "
		    "traffic distribution key configuration\n", __func__);
		return (err);
	}

	err = dpaa2_ni_prepare_key_cfg(&cls_cfg, (uint8_t *) sc->rxd_kcfg.vaddr);
	if (err) {
		device_printf(dev, "%s: failed to prepare key configuration: "
		    "error=%d\n", __func__, err);
		return (err);
	}

	/* Prepare for setting the Rx dist. */
	err = bus_dmamap_load(sc->rxd_dmat, sc->rxd_kcfg.dmap,
	    sc->rxd_kcfg.vaddr, DPAA2_CLASSIFIER_DMA_SIZE, dpaa2_ni_dmamap_cb,
	    &sc->rxd_kcfg.paddr, BUS_DMA_NOWAIT);
	if (err) {
		device_printf(sc->dev, "%s: failed to map a buffer for Rx "
		    "traffic distribution key configuration\n", __func__);
		return (err);
	}

	if (type == DPAA2_NI_DIST_MODE_HASH) {
		err = DPAA2_CMD_NI_SET_RX_TC_DIST(dev, child, dpaa2_mcp_tk(
		    sc->cmd, sc->ni_token), sc->attr.num.queues, 0,
		    DPAA2_NI_DIST_MODE_HASH, sc->rxd_kcfg.paddr);
		if (err)
			device_printf(dev, "%s: failed to set distribution mode "
			    "and size for the traffic class\n", __func__);
	}

	return err;
}

/**
 * @brief Prepares extract parameters.
 *
 * cfg:		Defining a full Key Generation profile.
 * key_cfg_buf:	Zeroed 256 bytes of memory before mapping it to DMA.
 */
static int
dpaa2_ni_prepare_key_cfg(struct dpkg_profile_cfg *cfg, uint8_t *key_cfg_buf)
{
	struct dpni_ext_set_rx_tc_dist *dpni_ext;
	struct dpni_dist_extract *extr;
	int i, j;

	if (cfg->num_extracts > DPKG_MAX_NUM_OF_EXTRACTS)
		return (EINVAL);

	dpni_ext = (struct dpni_ext_set_rx_tc_dist *) key_cfg_buf;
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

	return (0);
}

/**
 * @brief Obtain the next dequeue response from the channel storage.
 */
static int
dpaa2_ni_chan_storage_next(struct dpaa2_ni_channel *chan, struct dpaa2_dq **dq)
{
	struct dpaa2_io_softc *iosc = device_get_softc(chan->io_dev);
	struct dpaa2_swp *swp = iosc->swp;
	struct dpaa2_dq *msg = &chan->store.vaddr[chan->store_idx];
	int rc = EAGAIN;

	if ((msg->fdr.desc.stat & DPAA2_DQ_STAT_VOLATILE) &&
	    (msg->fdr.desc.tok == DPAA2_SWP_VDQCR_TOKEN)) {
		/* Reset token. */
		msg->fdr.desc.tok = 0;
		/* Make VDQ command available again. */
		ATOMIC_XCHG(&swp->vdq.avail, 1);
	} else {
		return (rc); /* DQ response is not available yet. */
	}

	rc = EINPROGRESS;
	chan->store_idx++;

	if (msg->fdr.desc.stat & DPAA2_DQ_STAT_EXPIRED) {
		rc = EALREADY; /* VDQ command is expired */
		chan->store_idx = 0;
		if (!(msg->fdr.desc.stat & DPAA2_DQ_STAT_VALIDFRAME))
			msg = NULL; /* Null response, FD is invalid */
	}
	if (msg->fdr.desc.stat & DPAA2_DQ_STAT_FQEMPTY) {
		rc = ENOENT; /* FQ is empty */
		chan->store_idx = 0;
	}

	if (dq != NULL)
		*dq = msg;

	return (rc);
}

static device_method_t dpaa2_ni_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_ni_probe),
	DEVMETHOD(device_attach,	dpaa2_ni_attach),
	DEVMETHOD(device_detach,	dpaa2_ni_detach),

	/* mii via memac_mdio */
	DEVMETHOD(miibus_statchg,	dpaa2_ni_miibus_statchg),

	DEVMETHOD_END
};

static driver_t dpaa2_ni_driver = {
	"dpaa2_ni",
	dpaa2_ni_methods,
	sizeof(struct dpaa2_ni_softc),
};

DRIVER_MODULE(miibus, dpaa2_ni, miibus_driver, 0, 0);
DRIVER_MODULE(dpaa2_ni, dpaa2_rc, dpaa2_ni_driver, 0, 0);

MODULE_DEPEND(dpaa2_ni, miibus, 1, 1, 1);
MODULE_DEPEND(dpaa2_ni, memac_mdio, 1, 1, 1);
