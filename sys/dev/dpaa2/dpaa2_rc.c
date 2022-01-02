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
 * The DPAA2 Resource Container (DPRC) bus driver.
 *
 * DPRC holds all the resources and object information that a software context
 * (kernel, virtual machine, etc.) can access or use.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/lock.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"
#include "dpaa2_ni.h"
#include "dpaa2_mc_if.h"
#include "dpaa2_cmd_if.h"

#define CMD_SLEEP_TIMEOUT	1u	/* ms */
#define CMD_SLEEP_ATTEMPTS	150u	/* max. 150 ms */
#define CMD_SPIN_TIMEOUT	100u	/* us */
#define CMD_SPIN_ATTEMPTS	1500u	/* max. 150 ms */

#define TYPE_LEN_MAX		16u
#define LABEL_LEN_MAX		16u

/* Mark the end of the DPAA2-specific resource list. */
#define DPAA2_RESDESC_END	{ DPAA2_DEV_NOTYPE, DPAA2_DEV_NOTYPE }

/* ------------------------- MNG command IDs -------------------------------- */
#define CMD_MNG_BASE_VERSION	1
#define CMD_MNG_ID_OFFSET	4

#define CMD_MNG(id)	(((id) << CMD_MNG_ID_OFFSET) | CMD_MNG_BASE_VERSION)

#define CMDID_MNG_GET_VER			CMD_MNG(0x831)
#define CMDID_MNG_GET_SOC_VER			CMD_MNG(0x832)
#define CMDID_MNG_GET_CONT_ID			CMD_MNG(0x830)

/* ------------------------- DPRC command IDs ------------------------------- */
#define CMD_RC_BASE_VERSION	1
#define CMD_RC_2ND_VERSION	2
#define CMD_RC_3RD_VERSION	3
#define CMD_RC_ID_OFFSET	4

#define CMD_RC(id)	(((id) << CMD_RC_ID_OFFSET) | CMD_RC_BASE_VERSION)
#define CMD_RC_V2(id)	(((id) << CMD_RC_ID_OFFSET) | CMD_RC_2ND_VERSION)
#define CMD_RC_V3(id)	(((id) << CMD_RC_ID_OFFSET) | CMD_RC_3RD_VERSION)

#define CMDID_RC_OPEN				CMD_RC(0x805)
#define CMDID_RC_CLOSE				CMD_RC(0x800)
#define CMDID_RC_GET_API_VERSION		CMD_RC(0xA05)
#define CMDID_RC_GET_ATTR			CMD_RC(0x004)
#define CMDID_RC_RESET_CONT			CMD_RC(0x005)
#define CMDID_RC_RESET_CONT_V2			CMD_RC_V2(0x005)
#define CMDID_RC_SET_IRQ			CMD_RC(0x010)
#define CMDID_RC_SET_IRQ_ENABLE			CMD_RC(0x012)
#define CMDID_RC_SET_IRQ_MASK			CMD_RC(0x014)
#define CMDID_RC_GET_IRQ_STATUS			CMD_RC(0x016)
#define CMDID_RC_CLEAR_IRQ_STATUS		CMD_RC(0x017)
#define CMDID_RC_GET_CONT_ID			CMD_RC(0x830)
#define CMDID_RC_GET_OBJ_COUNT			CMD_RC(0x159)
#define CMDID_RC_GET_OBJ			CMD_RC(0x15A)
#define CMDID_RC_GET_OBJ_DESC			CMD_RC(0x162)
#define CMDID_RC_GET_OBJ_REG			CMD_RC(0x15E)
#define CMDID_RC_GET_OBJ_REG_V2			CMD_RC_V2(0x15E)
#define CMDID_RC_GET_OBJ_REG_V3			CMD_RC_V3(0x15E)
#define CMDID_RC_SET_OBJ_IRQ			CMD_RC(0x15F)
#define CMDID_RC_GET_CONN			CMD_RC(0x16C)

/* ------------------------- DPIO command IDs ------------------------------- */
#define CMD_IO_BASE_VERSION	1
#define CMD_IO_ID_OFFSET	4

#define CMD_IO(id)	(((id) << CMD_IO_ID_OFFSET) | CMD_IO_BASE_VERSION)

#define CMDID_IO_OPEN				CMD_IO(0x803)
#define CMDID_IO_CLOSE				CMD_IO(0x800)
#define CMDID_IO_ENABLE				CMD_IO(0x002)
#define CMDID_IO_DISABLE			CMD_IO(0x003)
#define CMDID_IO_GET_ATTR			CMD_IO(0x004)
#define CMDID_IO_RESET				CMD_IO(0x005)
#define CMDID_IO_SET_IRQ_ENABLE			CMD_IO(0x012)
#define CMDID_IO_SET_IRQ_MASK			CMD_IO(0x014)
#define CMDID_IO_GET_IRQ_STATUS			CMD_IO(0x016)
#define CMDID_IO_ADD_STATIC_DQ_CHAN		CMD_IO(0x122)

/* ------------------------- DPNI command IDs ------------------------------- */
#define CMD_NI_BASE_VERSION	1
#define CMD_NI_2ND_VERSION	2
#define CMD_NI_4TH_VERSION	4
#define CMD_NI_ID_OFFSET	4

#define CMD_NI(id)	(((id) << CMD_NI_ID_OFFSET) | CMD_NI_BASE_VERSION)
#define CMD_NI_V2(id)	(((id) << CMD_NI_ID_OFFSET) | CMD_NI_2ND_VERSION)
#define CMD_NI_V4(id)	(((id) << CMD_NI_ID_OFFSET) | CMD_NI_4TH_VERSION)

#define CMDID_NI_OPEN				CMD_NI(0x801)
#define CMDID_NI_CLOSE				CMD_NI(0x800)
#define CMDID_NI_ENABLE				CMD_NI(0x002)
#define CMDID_NI_DISABLE			CMD_NI(0x003)
#define CMDID_NI_GET_API_VER			CMD_NI(0xA01)
#define CMDID_NI_RESET				CMD_NI(0x005)
#define CMDID_NI_GET_ATTR			CMD_NI(0x004)
#define CMDID_NI_SET_BUF_LAYOUT			CMD_NI(0x265)
#define CMDID_NI_GET_TX_DATA_OFF		CMD_NI(0x212)
#define CMDID_NI_GET_PORT_MAC_ADDR		CMD_NI(0x263)
#define CMDID_NI_SET_PRIM_MAC_ADDR		CMD_NI(0x224)
#define CMDID_NI_GET_PRIM_MAC_ADDR		CMD_NI(0x225)
#define CMDID_NI_SET_LINK_CFG			CMD_NI(0x21A)
#define CMDID_NI_GET_LINK_CFG			CMD_NI(0x278)
#define CMDID_NI_GET_LINK_STATE			CMD_NI(0x215)
#define CMDID_NI_SET_QOS_TABLE			CMD_NI(0x240)
#define CMDID_NI_CLEAR_QOS_TABLE		CMD_NI(0x243)
#define CMDID_NI_SET_POOLS			CMD_NI(0x200)
#define CMDID_NI_SET_ERR_BEHAVIOR		CMD_NI(0x20B)
#define CMDID_NI_GET_QUEUE			CMD_NI(0x25F)
#define CMDID_NI_SET_QUEUE			CMD_NI(0x260)
#define CMDID_NI_GET_QDID			CMD_NI(0x210)
#define CMDID_NI_ADD_MAC_ADDR			CMD_NI(0x226)
#define CMDID_NI_CLEAR_MAC_FILTERS		CMD_NI(0x228)
#define CMDID_NI_SET_MFL			CMD_NI(0x216)
#define CMDID_NI_SET_OFFLOAD			CMD_NI(0x26C)
#define CMDID_NI_SET_IRQ_MASK			CMD_NI(0x014)
#define CMDID_NI_SET_IRQ_ENABLE			CMD_NI(0x012)
#define CMDID_NI_GET_IRQ_STATUS			CMD_NI(0x016)
#define CMDID_NI_SET_UNI_PROMISC		CMD_NI(0x222)
#define CMDID_NI_SET_MULTI_PROMISC		CMD_NI(0x220)
#define CMDID_NI_GET_STATISTICS			CMD_NI(0x25D)
#define CMDID_NI_SET_RX_TC_DIST			CMD_NI(0x235)

/* ------------------------- DPBP command IDs ------------------------------- */
#define CMD_BP_BASE_VERSION	1
#define CMD_BP_ID_OFFSET	4

#define CMD_BP(id)	(((id) << CMD_BP_ID_OFFSET) | CMD_BP_BASE_VERSION)

#define CMDID_BP_OPEN				CMD_BP(0x804)
#define CMDID_BP_CLOSE				CMD_BP(0x800)
#define CMDID_BP_ENABLE				CMD_BP(0x002)
#define CMDID_BP_DISABLE			CMD_BP(0x003)
#define CMDID_BP_GET_ATTR			CMD_BP(0x004)
#define CMDID_BP_RESET				CMD_BP(0x005)

/* ------------------------- DPMAC command IDs ------------------------------ */
#define CMD_MAC_BASE_VERSION	1
#define CMD_MAC_2ND_VERSION	2
#define CMD_MAC_ID_OFFSET	4

#define CMD_MAC(id)	(((id) << CMD_MAC_ID_OFFSET) | CMD_MAC_BASE_VERSION)
#define CMD_MAC_V2(id)	(((id) << CMD_MAC_ID_OFFSET) | CMD_MAC_2ND_VERSION)

#define CMDID_MAC_OPEN				CMD_MAC(0x80C)
#define CMDID_MAC_CLOSE				CMD_MAC(0x800)
#define CMDID_MAC_RESET				CMD_MAC(0x005)
#define CMDID_MAC_MDIO_READ			CMD_MAC(0x0C0)
#define CMDID_MAC_MDIO_WRITE			CMD_MAC(0x0C1)
#define CMDID_MAC_GET_ADDR			CMD_MAC(0x0C5)
#define CMDID_MAC_GET_ATTR			CMD_MAC(0x004)
#define CMDID_MAC_SET_LINK_STATE		CMD_MAC_V2(0x0C3)
#define CMDID_MAC_SET_IRQ_MASK			CMD_MAC(0x014)
#define CMDID_MAC_SET_IRQ_ENABLE		CMD_MAC(0x012)
#define CMDID_MAC_GET_IRQ_STATUS		CMD_MAC(0x016)

/* ------------------------- DPCON command IDs ------------------------------ */
#define CMD_CON_BASE_VERSION	1
#define CMD_CON_ID_OFFSET	4

#define CMD_CON(id)	(((id) << CMD_BP_ID_OFFSET) | CMD_BP_BASE_VERSION)

#define CMDID_CON_OPEN				CMD_CON(0x808)
#define CMDID_CON_CLOSE				CMD_CON(0x800)
#define CMDID_CON_ENABLE			CMD_CON(0x002)
#define CMDID_CON_DISABLE			CMD_CON(0x003)
#define CMDID_CON_GET_ATTR			CMD_CON(0x004)
#define CMDID_CON_RESET				CMD_CON(0x005)
#define CMDID_CON_SET_NOTIF			CMD_CON(0x100)

/* ------------------------- End of command IDs ----------------------------- */

MALLOC_DEFINE(M_DPAA2_RC, "dpaa2_rc", "DPAA2 Resource Container");

/**
 * @brief Helper object to access fields of the DPAA2 object information
 * response.
 */
struct __packed dpaa2_obj {
	uint32_t	_reserved1;
	uint32_t	id;
	uint16_t	vendor;
	uint8_t		irq_count;
	uint8_t		reg_count;
	uint32_t	state;
	uint16_t	ver_major;
	uint16_t	ver_minor;
	uint16_t	flags;
	uint16_t	_reserved2;
	uint8_t		type[16];
	uint8_t		label[16];
};

/* Forward declarations. */

static int	discover_objects(struct dpaa2_rc_softc *sc);
static int	add_child(struct dpaa2_rc_softc *sc, dpaa2_cmd_t cmd,
		    const dpaa2_obj_t *obj);
static int	add_managed_child(struct dpaa2_rc_softc *sc, dpaa2_cmd_t cmd,
		    const dpaa2_obj_t *obj);

static int	set_irq_enable(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
		    const uint8_t irq_idx, const uint8_t enable,
		    const uint16_t cmdid);
static int	configure_irq(device_t rcdev, device_t child, int rid,
		    uint64_t addr, uint32_t data);
static int	add_dpaa2_res(device_t rcdev, device_t child,
		    enum dpaa2_dev_type devtype, int *rid, int flags);
static int	print_dpaa2_type(struct resource_list *rl,
		    enum dpaa2_dev_type type);

/* Utility routines to send commands to MC. */

static int	exec_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd,
		    const uint16_t cmdid);
static void	send_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);
static int	wait_for_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd);
static void	reset_cmd_params(dpaa2_cmd_t cmd);

/*
 * Device interface.
 */

static int
dpaa2_rc_probe(device_t dev)
{
	/* DPRC device will be added by a parent DPRC or by MC bus itself. */
	device_set_desc(dev, "DPAA2 Resource Container");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_rc_detach(device_t dev)
{
	struct dpaa2_rc_softc *sc;
	struct dpaa2_devinfo *dinfo;
	int error;

	error = bus_generic_detach(dev);
	if (error)
		return (error);

	sc = device_get_softc(dev);
	dinfo = device_get_ivars(dev);

	if (sc->portal)
		dpaa2_mcp_free_portal(sc->portal);
	if (dinfo)
		free(dinfo, M_DPAA2_RC);

	return (device_delete_children(dev));
}

static int
dpaa2_rc_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_mc_softc *mcsc;
	struct dpaa2_rc_softc *sc;
	struct dpaa2_devinfo *dinfo = NULL;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->portal = NULL;
	sc->unit = device_get_unit(dev);

	if (sc->unit == 0) {
		/* Root DPRC should be attached directly to the MC bus. */
		pdev = device_get_parent(dev);
		mcsc = device_get_softc(pdev);

		KASSERT(strcmp(device_get_name(pdev), "dpaa2_mc") == 0,
		    ("root DPRC should be attached to the MC bus"));

		/*
		 * Allocate devinfo to let the parent MC bus access ICID of the
		 * DPRC object.
		 */
		dinfo = malloc(sizeof(struct dpaa2_devinfo), M_DPAA2_RC,
		    M_WAITOK | M_ZERO);
		if (!dinfo) {
			device_printf(dev, "Failed to allocate dpaa2_devinfo\n");
			dpaa2_rc_detach(dev);
			return (ENXIO);
		}
		device_set_ivars(dev, dinfo);

		dinfo->pdev = pdev;
		dinfo->dev = dev;
		dinfo->dtype = DPAA2_DEV_RC;

		/* Prepare helper portal object to send commands to MC. */
		error = dpaa2_mcp_init_atomic(&sc->portal, mcsc->res[0],
		    &mcsc->map[0], DPAA2_PORTAL_DEF);
		if (error) {
			device_printf(dev, "Failed to initialize dpaa2_mcp: "
			    "error=%d\n", error);
			dpaa2_rc_detach(dev);
			return (ENXIO);
		}
	} else {
		/* TODO: Child DPRCs aren't supported yet. */
		return (ENXIO);
	}

	/* Create DPAA2 devices for objects in this container. */
	error = discover_objects(sc);
	if (error) {
		device_printf(dev, "Failed to discover objects in container: "
		    "error=%d\n", error);
		dpaa2_rc_detach(dev);
		return (error);
	}

	return (0);
}

/*
 * Bus interface.
 */

static struct resource_list *
dpaa2_rc_get_resource_list(device_t rcdev, device_t child)
{
	struct dpaa2_devinfo *dinfo = device_get_ivars(child);

	return (&dinfo->resources);
}

static void
dpaa2_rc_delete_resource(device_t rcdev, device_t child, int type, int rid)
{
	struct resource_list *rl;
	struct resource_list_entry *rle;
	struct dpaa2_devinfo *dinfo;

	if (device_get_parent(child) != rcdev)
		return;

	dinfo = device_get_ivars(child);
	rl = &dinfo->resources;
	rle = resource_list_find(rl, type, rid);
	if (rle == NULL)
		return;

	if (rle->res) {
		if (rman_get_flags(rle->res) & RF_ACTIVE ||
		    resource_list_busy(rl, type, rid)) {
			device_printf(rcdev, "delete_resource: "
			    "Resource still owned by child, oops. "
			    "(type=%d, rid=%d, addr=%jx)\n",
			    type, rid, rman_get_start(rle->res));
			return;
		}
		resource_list_unreserve(rl, rcdev, child, type, rid);
	}
	resource_list_delete(rl, type, rid);
}

static struct resource *
dpaa2_rc_alloc_multi_resource(device_t rcdev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct resource_list *rl;
	struct dpaa2_devinfo *dinfo;

	dinfo = device_get_ivars(child);
	rl = &dinfo->resources;

	/*
	 * By default, software portal interrupts are message-based, that is,
	 * they are issued from QMan using a 4 byte write.
	 *
	 * TODO: However this default behavior can be changed by programming one
	 *	 or more software portals to issue their interrupts via a
	 *	 dedicated software portal interrupt wire.
	 *	 See registers SWP_INTW0_CFG to SWP_INTW3_CFG for details.
	 */
	if (type == SYS_RES_IRQ && *rid == 0)
		return (NULL);

	return (resource_list_alloc(rl, rcdev, child, type, rid,
	    start, end, count, flags));
}

static struct resource *
dpaa2_rc_alloc_resource(device_t rcdev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	if (device_get_parent(child) != rcdev)
		return (BUS_ALLOC_RESOURCE(device_get_parent(rcdev), child,
		    type, rid, start, end, count, flags));

	return (dpaa2_rc_alloc_multi_resource(rcdev, child, type, rid, start,
	    end, count, flags));
}

static int
dpaa2_rc_release_resource(device_t rcdev, device_t child, int type, int rid,
    struct resource *r)
{
	struct resource_list *rl;
	struct dpaa2_devinfo *dinfo;

	if (device_get_parent(child) != rcdev)
		return (BUS_RELEASE_RESOURCE(device_get_parent(rcdev), child,
		    type, rid, r));

	dinfo = device_get_ivars(child);
	rl = &dinfo->resources;
	return (resource_list_release(rl, rcdev, child, type, rid, r));
}

static void
dpaa2_rc_child_deleted(device_t rcdev, device_t child)
{
	struct dpaa2_devinfo *dinfo;
	struct resource_list *rl;
	struct resource_list_entry *rle;

	dinfo = device_get_ivars(child);
	rl = &dinfo->resources;

	/* Free all allocated resources */
	STAILQ_FOREACH(rle, rl, link) {
		if (rle->res) {
			if (rman_get_flags(rle->res) & RF_ACTIVE ||
			    resource_list_busy(rl, rle->type, rle->rid)) {
				device_printf(child,
				    "Resource still owned, oops. "
				    "(type=%d, rid=%d, addr=%lx)\n",
				    rle->type, rle->rid,
				    rman_get_start(rle->res));
				bus_release_resource(child, rle->type, rle->rid,
				    rle->res);
			}
			resource_list_unreserve(rl, rcdev, child, rle->type,
			    rle->rid);
		}
	}
	resource_list_free(rl);

	if (dinfo)
		free(dinfo, M_DPAA2_RC);
}

static void
dpaa2_rc_child_detached(device_t rcdev, device_t child)
{
	struct dpaa2_devinfo *dinfo;
	struct resource_list *rl;

	dinfo = device_get_ivars(child);
	rl = &dinfo->resources;

	if (resource_list_release_active(rl, rcdev, child, SYS_RES_IRQ) != 0)
		device_printf(child, "Leaked IRQ resources!\n");
	if (dinfo->msi.msi_alloc != 0) {
		device_printf(child, "Leaked %d MSI vectors!\n",
		    dinfo->msi.msi_alloc);
		PCI_RELEASE_MSI(rcdev, child);
	}
	if (resource_list_release_active(rl, rcdev, child, SYS_RES_MEMORY) != 0)
		device_printf(child, "Leaked memory resources!\n");
}

static int
dpaa2_rc_setup_intr(device_t rcdev, device_t child, struct resource *irq,
    int flags, driver_filter_t *filter, driver_intr_t *intr, void *arg,
    void **cookiep)
{
	struct dpaa2_devinfo *dinfo;
	uint64_t addr;
	uint32_t data;
	void *cookie;
	int error, rid;

	error = bus_generic_setup_intr(rcdev, child, irq, flags, filter, intr,
	    arg, &cookie);
	if (error) {
		device_printf(rcdev, "bus_generic_setup_intr() failed: "
		    "error=%d\n", error);
		return (error);
	}

	/* If this is not a direct child, just bail out. */
	if (device_get_parent(child) != rcdev) {
		*cookiep = cookie;
		return (0);
	}

	rid = rman_get_rid(irq);
	if (rid == 0) {
		if (bootverbose)
			device_printf(rcdev, "Cannot setup interrupt with rid=0:"
			    " INTx are not supported by DPAA2 objects yet\n");
		return (EINVAL);
	} else {
		dinfo = device_get_ivars(child);
		KASSERT(dinfo->msi.msi_alloc > 0,
		    ("No MSI interrupts allocated"));

		/*
		 * Ask our parent to map the MSI and give us the address and
		 * data register values. If we fail for some reason, teardown
		 * the interrupt handler.
		 */
		error = PCIB_MAP_MSI(device_get_parent(rcdev), child,
		    rman_get_start(irq), &addr, &data);
		if (error) {
			device_printf(rcdev, "PCIB_MAP_MSI failed: error=%d\n",
			    error);
			(void)bus_generic_teardown_intr(rcdev, child, irq,
			    cookie);
			return (error);
		}

		/* Configure MSI for this DPAA2 object. */
		error = configure_irq(rcdev, child, rid, addr, data);
		if (error) {
			device_printf(rcdev, "Failed to configure IRQ for "
			    "DPAA2 object: rid=%d, type=%s, unit=%d\n", rid,
			    dpaa2_ttos(dinfo->dtype), device_get_unit(child));
			return (error);
		}
		dinfo->msi.msi_handlers++;
	}
	*cookiep = cookie;
	return (0);
}

static int
dpaa2_rc_teardown_intr(device_t rcdev, device_t child, struct resource *irq,
    void *cookie)
{
	struct resource_list_entry *rle;
	struct dpaa2_devinfo *dinfo;
	int error, rid;

	if (irq == NULL || !(rman_get_flags(irq) & RF_ACTIVE))
		return (EINVAL);

	/* If this isn't a direct child, just bail out */
	if (device_get_parent(child) != rcdev)
		return(bus_generic_teardown_intr(rcdev, child, irq, cookie));

	rid = rman_get_rid(irq);
	if (rid == 0) {
		if (bootverbose)
			device_printf(rcdev, "Cannot teardown interrupt with "
			    "rid=0: INTx are not supported by DPAA2 objects\n");
		return (EINVAL);
	} else {
		dinfo = device_get_ivars(child);
		rle = resource_list_find(&dinfo->resources, SYS_RES_IRQ, rid);
		if (rle->res != irq)
			return (EINVAL);
		dinfo->msi.msi_handlers--;
	}

	error = bus_generic_teardown_intr(rcdev, child, irq, cookie);
	if (rid > 0)
		KASSERT(error == 0,
		    ("%s: generic teardown failed for MSI", __func__));
	return (error);
}

static int
dpaa2_rc_print_child(device_t rcdev, device_t child)
{
	struct dpaa2_devinfo *dinfo = device_get_ivars(child);
	struct resource_list *rl = &dinfo->resources;
	int retval = 0;

	retval += bus_print_child_header(rcdev, child);

	retval += resource_list_print_type(rl, "port", SYS_RES_IOPORT, "%#jx");
	retval += resource_list_print_type(rl, "iomem", SYS_RES_MEMORY, "%#jx");
	retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ, "%jd");

	/* Print DPAA2-specific resources. */
	retval += print_dpaa2_type(rl, DPAA2_DEV_IO);
	retval += print_dpaa2_type(rl, DPAA2_DEV_BP);
	retval += print_dpaa2_type(rl, DPAA2_DEV_CON);

	retval += printf(" at %s (id=%u, irqs=%u)", dpaa2_ttos(dinfo->dtype),
	    dinfo->id, dinfo->msi.msi_msgnum);

	retval += bus_print_child_domain(rcdev, child);
	retval += bus_print_child_footer(rcdev, child);

	return (retval);
}

/*
 * Pseudo-PCI interface.
 */

/*
 * Attempt to allocate *count MSI messages. The actual number allocated is
 * returned in *count. After this function returns, each message will be
 * available to the driver as SYS_RES_IRQ resources starting at a rid 1.
 *
 * NOTE: Implementation is similar to sys/dev/pci/pci.c.
 */
static int
dpaa2_rc_alloc_msi(device_t rcdev, device_t child, int *count)
{
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(child);
	int error, actual, i, run, irqs[32];

	/* Don't let count == 0 get us into trouble. */
	if (*count == 0)
		return (EINVAL);

	/* MSI should be allocated by the resource container. */
	if (rcinfo->dtype != DPAA2_DEV_RC)
		return (ENODEV);

	/* Already have allocated messages? */
	if (dinfo->msi.msi_alloc != 0)
		return (ENXIO);

	/* Don't ask for more than the device supports. */
	actual = min(*count, dinfo->msi.msi_msgnum);

	/* Don't ask for more than 32 messages. */
	actual = min(actual, 32);

	/* MSI requires power of 2 number of messages. */
	if (!powerof2(actual))
		return (EINVAL);

	for (;;) {
		/* Try to allocate N messages. */
		error = PCIB_ALLOC_MSI(device_get_parent(rcdev), child, actual,
		    actual, irqs);
		if (error == 0)
			break;
		if (actual == 1)
			return (error);

		/* Try N / 2. */
		actual >>= 1;
	}

	/*
	 * We now have N actual messages mapped onto SYS_RES_IRQ resources in
	 * the irqs[] array, so add new resources starting at rid 1.
	 */
	for (i = 0; i < actual; i++)
		resource_list_add(&dinfo->resources, SYS_RES_IRQ, i + 1,
		    irqs[i], irqs[i], 1);

	if (bootverbose) {
		if (actual == 1) {
			device_printf(child, "Using IRQ %d for MSI\n", irqs[0]);
		} else {
			/*
			 * Be fancy and try to print contiguous runs
			 * of IRQ values as ranges.  'run' is true if
			 * we are in a range.
			 */
			device_printf(child, "Using IRQs %d", irqs[0]);
			run = 0;
			for (i = 1; i < actual; i++) {
				/* Still in a run? */
				if (irqs[i] == irqs[i - 1] + 1) {
					run = 1;
					continue;
				}

				/* Finish previous range. */
				if (run) {
					printf("-%d", irqs[i - 1]);
					run = 0;
				}

				/* Start new range. */
				printf(",%d", irqs[i]);
			}

			/* Unfinished range? */
			if (run)
				printf("-%d", irqs[actual - 1]);
			printf(" for MSI\n");
		}
	}

	/* Update counts of alloc'd messages. */
	dinfo->msi.msi_alloc = actual;
	dinfo->msi.msi_handlers = 0;
	*count = actual;
	return (0);
}

/*
 * Release the MSI messages associated with this DPAA2 device.
 *
 * NOTE: Implementation is similar to sys/dev/pci/pci.c.
 */
static int
dpaa2_rc_release_msi(device_t rcdev, device_t child)
{
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(child);
	struct resource_list_entry *rle;
	int i, irqs[32];

	/* MSI should be released by the resource container. */
	if (rcinfo->dtype != DPAA2_DEV_RC)
		return (ENODEV);

	/* Do we have any messages to release? */
	if (dinfo->msi.msi_alloc == 0)
		return (ENODEV);
	KASSERT(dinfo->msi.msi_alloc <= 32,
	    ("more than 32 alloc'd MSI messages"));

	/* Make sure none of the resources are allocated. */
	if (dinfo->msi.msi_handlers > 0)
		return (EBUSY);
	for (i = 0; i < dinfo->msi.msi_alloc; i++) {
		rle = resource_list_find(&dinfo->resources, SYS_RES_IRQ, i + 1);
		KASSERT(rle != NULL, ("missing MSI resource"));
		if (rle->res != NULL)
			return (EBUSY);
		irqs[i] = rle->start;
	}

	/* Release the messages. */
	PCIB_RELEASE_MSI(device_get_parent(rcdev), child, dinfo->msi.msi_alloc,
	    irqs);
	for (i = 0; i < dinfo->msi.msi_alloc; i++)
		resource_list_delete(&dinfo->resources, SYS_RES_IRQ, i + 1);

	/* Update alloc count. */
	dinfo->msi.msi_alloc = 0;
	return (0);
}

/**
 * @brief Return the maximum number of the MSI supported by this DPAA2 device.
 */
static int
dpaa2_rc_msi_count(device_t rcdev, device_t child)
{
	struct dpaa2_devinfo *dinfo = device_get_ivars(child);

	return (dinfo->msi.msi_msgnum);
}

static int
dpaa2_rc_get_id(device_t rcdev, device_t child, enum pci_id_type type,
    uintptr_t *id)
{
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (rcinfo->dtype != DPAA2_DEV_RC)
		return (ENODEV);

	return (PCIB_GET_ID(device_get_parent(rcdev), child, type, id));
}

/*
 * DPAA2 MC command interface.
 */

static int
dpaa2_rc_mng_get_version(device_t rcdev, dpaa2_cmd_t cmd, uint32_t *major,
    uint32_t *minor, uint32_t *rev)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !major || !minor || !rev)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_MNG_GET_VER);
	if (!error) {
		*major = cmd->params[0] >> 32;
		*minor = cmd->params[1] & 0xFFFFFFFF;
		*rev = cmd->params[0] & 0xFFFFFFFF;
	}

	return (error);
}

static int
dpaa2_rc_mng_get_soc_version(device_t rcdev, dpaa2_cmd_t cmd, uint32_t *pvr,
    uint32_t *svr)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !pvr || !svr)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_MNG_GET_SOC_VER);
	if (!error) {
		*pvr = cmd->params[0] >> 32;
		*svr = cmd->params[0] & 0xFFFFFFFF;
	}

	return (error);
}

static int
dpaa2_rc_mng_get_container_id(device_t rcdev, dpaa2_cmd_t cmd,
    uint32_t *cont_id)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !cont_id)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_MNG_GET_CONT_ID);
	if (!error)
		*cont_id = cmd->params[0] & 0xFFFFFFFF;

	return (error);
}

static int
dpaa2_rc_open(device_t rcdev, dpaa2_cmd_t cmd, uint32_t cont_id, uint16_t *token)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = cont_id;

	error = exec_command(sc->portal, cmd, CMDID_RC_OPEN);
	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

static int
dpaa2_rc_close(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_RC_CLOSE));
}

static int
dpaa2_rc_get_obj_count(device_t rcdev, dpaa2_cmd_t cmd, uint32_t *obj_count)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !obj_count)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_RC_GET_OBJ_COUNT);
	if (!error)
		*obj_count = (uint32_t)(cmd->params[0] >> 32);

	return (error);
}

static int
dpaa2_rc_get_obj(device_t rcdev, dpaa2_cmd_t cmd, uint32_t obj_idx,
    dpaa2_obj_t *obj)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_obj *pobj;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !obj)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = obj_idx;

	error = exec_command(sc->portal, cmd, CMDID_RC_GET_OBJ);
	if (!error) {
		pobj = (struct dpaa2_obj *) &cmd->params[0];
		obj->id = pobj->id;
		obj->vendor = pobj->vendor;
		obj->irq_count = pobj->irq_count;
		obj->reg_count = pobj->reg_count;
		obj->state = pobj->state;
		obj->ver_major = pobj->ver_major;
		obj->ver_minor = pobj->ver_minor;
		obj->flags = pobj->flags;
		obj->type = dpaa2_stot((const char *) pobj->type);
		memcpy(obj->label, pobj->label, sizeof(pobj->label));
	}

	/* Some DPAA2 objects might not be supported by the driver yet. */
	if (obj->type == DPAA2_DEV_NOTYPE)
		error = DPAA2_CMD_STAT_UNKNOWN_OBJ;

	return (error);
}

static int
dpaa2_rc_get_obj_descriptor(device_t rcdev, dpaa2_cmd_t cmd, uint32_t obj_id,
    enum dpaa2_dev_type dtype, dpaa2_obj_t *obj)
{
	struct __packed get_obj_desc_args {
		uint32_t	obj_id;
		uint32_t	_reserved1;
		uint8_t		type[16];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_obj *pobj;
	const char *type = dpaa2_ttos(dtype);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !obj)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct get_obj_desc_args *) &cmd->params[0];
	args->obj_id = obj_id;
	memcpy(args->type, type, min(strlen(type) + 1, TYPE_LEN_MAX));

	error = exec_command(sc->portal, cmd, CMDID_RC_GET_OBJ_DESC);
	if (!error) {
		pobj = (struct dpaa2_obj *) &cmd->params[0];
		obj->id = pobj->id;
		obj->vendor = pobj->vendor;
		obj->irq_count = pobj->irq_count;
		obj->reg_count = pobj->reg_count;
		obj->state = pobj->state;
		obj->ver_major = pobj->ver_major;
		obj->ver_minor = pobj->ver_minor;
		obj->flags = pobj->flags;
		obj->type = dpaa2_stot((const char *) pobj->type);
		memcpy(obj->label, pobj->label, sizeof(pobj->label));
	}

	/* Some DPAA2 objects might not be supported by the driver yet. */
	if (obj->type == DPAA2_DEV_NOTYPE)
		error = DPAA2_CMD_STAT_UNKNOWN_OBJ;

	return (error);
}

static int
dpaa2_rc_get_attributes(device_t rcdev, dpaa2_cmd_t cmd, dpaa2_rc_attr_t *attr)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct __packed dpaa2_rc_attr {
		uint32_t	cont_id;
		uint16_t	icid;
		uint16_t	_reserved1;
		uint32_t	options;
		uint32_t	portal_id;
	} *pattr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_RC_GET_ATTR);
	if (!error) {
		pattr = (struct dpaa2_rc_attr *) &cmd->params[0];
		attr->cont_id = pattr->cont_id;
		attr->portal_id = pattr->portal_id;
		attr->options = pattr->options;
		attr->icid = pattr->icid;
	}

	return (error);
}

static int
dpaa2_rc_get_obj_region(device_t rcdev, dpaa2_cmd_t cmd, uint32_t obj_id,
    uint8_t reg_idx, enum dpaa2_dev_type dtype, dpaa2_rc_obj_region_t *reg)
{
	struct __packed obj_region_args {
		uint32_t	obj_id;
		uint16_t	_reserved1;
		uint8_t		reg_idx;
		uint8_t		_reserved2;
		uint64_t	_reserved3;
		uint64_t	_reserved4;
		uint8_t		type[16];
	} *args;
	struct __packed obj_region {
		uint64_t	_reserved1;
		uint64_t	base_offset;
		uint32_t	size;
		uint32_t	type;
		uint32_t	flags;
		uint32_t	_reserved2;
		uint64_t	base_paddr;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	uint16_t cmdid, api_major, api_minor;
	const char *type = dpaa2_ttos(dtype);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !reg)
		return (DPAA2_CMD_STAT_ERR);

	/*
	 * If the DPRC object version was not yet cached, cache it now.
	 * Otherwise use the already cached value.
	 */
	if (!sc->portal->rc_api_major && !sc->portal->rc_api_minor) {
		error = DPAA2_CMD_RC_GET_API_VERSION(rcdev, cmd, &api_major,
		    &api_minor);
		if (error)
			return (error);
		sc->portal->rc_api_major = api_major;
		sc->portal->rc_api_minor = api_minor;
	} else {
		api_major = sc->portal->rc_api_major;
		api_minor = sc->portal->rc_api_minor;
	}

	if (api_major > 6u || (api_major == 6u && api_minor >= 6u))
		/*
		 * MC API version 6.6 changed the size of the MC portals and
		 * software portals to 64K (as implemented by hardware).
		 */
		cmdid = CMDID_RC_GET_OBJ_REG_V3;
	else if (api_major == 6u && api_minor >= 3u)
		/*
		 * MC API version 6.3 introduced a new field to the region
		 * descriptor: base_address.
		 */
		cmdid = CMDID_RC_GET_OBJ_REG_V2;
	else
		cmdid = CMDID_RC_GET_OBJ_REG;

	args = (struct obj_region_args *) &cmd->params[0];
	args->obj_id = obj_id;
	args->reg_idx = reg_idx;
	memcpy(args->type, type, min(strlen(type) + 1, TYPE_LEN_MAX));

	error = exec_command(sc->portal, cmd, cmdid);
	if (!error) {
		resp = (struct obj_region *) &cmd->params[0];
		reg->base_paddr = resp->base_paddr;
		reg->base_offset = resp->base_offset;
		reg->size = resp->size;
		reg->flags = resp->flags;
		reg->type = resp->type & 0xFu;
	}

	return (error);
}

static int
dpaa2_rc_get_api_version(device_t rcdev, dpaa2_cmd_t cmd, uint16_t *major,
    uint16_t *minor)
{
	struct __packed rc_api_version {
		uint16_t	major;
		uint16_t	minor;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !major || !minor)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_RC_GET_API_VERSION);
	if (!error) {
		resp = (struct rc_api_version *) &cmd->params[0];
		*major = resp->major;
		*minor = resp->minor;
	}

	return (error);
}

static int
dpaa2_rc_set_irq_enable(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint8_t enable)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);

	return (set_irq_enable(sc->portal, cmd, irq_idx, enable,
	    CMDID_RC_SET_IRQ_ENABLE));
}

static int
dpaa2_rc_set_obj_irq(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint64_t addr, uint32_t data, uint32_t irq_usr, uint32_t obj_id,
    enum dpaa2_dev_type dtype)
{
	struct __packed set_obj_irq_args {
		uint32_t	data;
		uint8_t		irq_idx;
		uint8_t		_reserved1[3];
		uint64_t	addr;
		uint32_t	irq_usr;
		uint32_t	obj_id;
		uint8_t		type[16];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	const char *type = dpaa2_ttos(dtype);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct set_obj_irq_args *) &cmd->params[0];
	args->irq_idx = irq_idx;
	args->addr = addr;
	args->data = data;
	args->irq_usr = irq_usr;
	args->obj_id = obj_id;
	memcpy(args->type, type, min(strlen(type) + 1, TYPE_LEN_MAX));

	return (exec_command(sc->portal, cmd, CMDID_RC_SET_OBJ_IRQ));
}

static int
dpaa2_rc_get_conn(device_t rcdev, dpaa2_cmd_t cmd, dpaa2_ep_desc_t *ep1_desc,
    dpaa2_ep_desc_t *ep2_desc, uint32_t *link_stat)
{
	struct __packed get_conn_args {
		uint32_t ep1_id;
		uint32_t ep1_ifid;
		uint8_t  ep1_type[16];
		uint64_t _reserved[4];
	} *args;
	struct __packed get_conn_resp {
		uint64_t _reserved1[3];
		uint32_t ep2_id;
		uint32_t ep2_ifid;
		uint8_t  ep2_type[16];
		uint32_t link_stat;
		uint32_t _reserved2;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !ep1_desc || !ep2_desc)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct get_conn_args *) &cmd->params[0];
	args->ep1_id = ep1_desc->obj_id;
	args->ep1_ifid = ep1_desc->if_id;;
	strncpy(args->ep1_type, dpaa2_ttos(ep1_desc->type), 16);

	error = exec_command(sc->portal, cmd, CMDID_RC_GET_CONN);
	if (!error) {
		resp = (struct get_conn_resp *) &cmd->params[0];
		ep2_desc->obj_id = resp->ep2_id;
		ep2_desc->if_id = resp->ep2_ifid;
		ep2_desc->type = dpaa2_stot((const char *) resp->ep2_type);
		if (link_stat)
			*link_stat = resp->link_stat;
	}

	return (error);
}

static int
dpaa2_rc_ni_open(device_t rcdev, dpaa2_cmd_t cmd, const uint32_t dpni_id,
    uint16_t *token)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpni_id;
	error = exec_command(sc->portal, cmd, CMDID_NI_OPEN);
 	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

static int
dpaa2_rc_ni_close(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_NI_CLOSE));
}

static int
dpaa2_rc_ni_enable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_NI_ENABLE));
}

static int
dpaa2_rc_ni_disable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_NI_DISABLE));
}

static int
dpaa2_rc_ni_get_api_version(device_t rcdev, dpaa2_cmd_t cmd, uint16_t *major,
    uint16_t *minor)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !major || !minor)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_API_VER);
	if (!error) {
		*major = cmd->params[0] & 0xFFFFU;
		*minor = (cmd->params[0] >> 16) & 0xFFFFU;
	}

	return (error);
}

static int
dpaa2_rc_ni_reset(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_NI_RESET));
}

static int
dpaa2_rc_ni_get_attributes(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_ni_attr_t *attr)
{
	struct __packed ni_attr {
		uint32_t	options;
		uint8_t		num_queues;
		uint8_t		num_rx_tcs;
		uint8_t		mac_entries;
		uint8_t		num_tx_tcs;
		uint8_t		vlan_entries;
		uint8_t		num_channels;
		uint8_t		qos_entries;
		uint8_t		_reserved1;
		uint16_t	fs_entries;
		uint16_t	_reserved2;
		uint8_t		qos_key_size;
		uint8_t		fs_key_size;
		uint16_t	wriop_ver;
		uint8_t		num_cgs;
		uint8_t		_reserved3;
		uint16_t	_reserved4;
		uint64_t	_reserved5[4];
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_ATTR);
	if (!error) {
		resp = (struct ni_attr *) &cmd->params[0];

		attr->options =	     resp->options;
		attr->wriop_ver =    resp->wriop_ver;

		attr->entries.fs =   resp->fs_entries;
		attr->entries.mac =  resp->mac_entries;
		attr->entries.vlan = resp->vlan_entries;
		attr->entries.qos =  resp->qos_entries;

		attr->num.queues =   resp->num_queues;
		attr->num.rx_tcs =   resp->num_rx_tcs;
		attr->num.tx_tcs =   resp->num_tx_tcs;
		attr->num.channels = resp->num_channels;
		attr->num.cgs =      resp->num_cgs;

		attr->key_size.fs =  resp->fs_key_size;
		attr->key_size.qos = resp->qos_key_size;
	}

	return (error);
}

static int
dpaa2_rc_ni_set_buf_layout(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_ni_buf_layout_t *bl)
{
	struct __packed set_buf_layout_args {
		uint8_t		queue_type;
		uint8_t		_reserved1;
		uint16_t	_reserved2;
		uint16_t	options;
		uint8_t		params;
		uint8_t		_reserved3;
		uint16_t	priv_data_size;
		uint16_t	data_align;
		uint16_t	head_room;
		uint16_t	tail_room;
		uint64_t	_reserved4[5];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !bl)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct set_buf_layout_args *) &cmd->params[0];
	args->queue_type = (uint8_t) bl->queue_type;
	args->options = bl->options;
	args->params = 0;
	args->priv_data_size = bl->pd_size;
	args->data_align = bl->fd_align;
	args->head_room = bl->head_size;
	args->tail_room = bl->tail_size;

	args->params |= bl->pass_timestamp	? 1U : 0U;
	args->params |= bl->pass_parser_result	? 2U : 0U;
	args->params |= bl->pass_frame_status	? 4U : 0U;
	args->params |= bl->pass_sw_opaque	? 8U : 0U;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_BUF_LAYOUT));
}

static int
dpaa2_rc_ni_get_tx_data_offset(device_t rcdev, dpaa2_cmd_t cmd, uint16_t *offset)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !offset)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_TX_DATA_OFF);
	if (!error)
		*offset = cmd->params[0] & 0xFFFFU;

	return (error);
}

static int
dpaa2_rc_ni_get_port_mac_addr(device_t rcdev, dpaa2_cmd_t cmd, uint8_t *mac)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !mac)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_PORT_MAC_ADDR);
	if (!error) {
		mac[0] = (cmd->params[0] >> 56) & 0xFFU;
		mac[1] = (cmd->params[0] >> 48) & 0xFFU;
		mac[2] = (cmd->params[0] >> 40) & 0xFFU;
		mac[3] = (cmd->params[0] >> 32) & 0xFFU;
		mac[4] = (cmd->params[0] >> 24) & 0xFFU;
		mac[5] = (cmd->params[0] >> 16) & 0xFFU;
	}

	return (error);
}

static int
dpaa2_rc_ni_set_prim_mac_addr(device_t rcdev, dpaa2_cmd_t cmd, uint8_t *mac)
{
	struct __packed set_prim_mac_args {
		uint8_t		_reserved[2];
		uint8_t		mac[ETHER_ADDR_LEN];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !mac)
		return (DPAA2_CMD_STAT_EINVAL);

	args = (struct set_prim_mac_args *) &cmd->params[0];
	for (int i = 1; i <= ETHER_ADDR_LEN; i++)
		args->mac[i - 1] = mac[ETHER_ADDR_LEN - i];

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_PRIM_MAC_ADDR));
}

static int
dpaa2_rc_ni_get_prim_mac_addr(device_t rcdev, dpaa2_cmd_t cmd, uint8_t *mac)
{
	struct __packed get_prim_mac_resp {
		uint8_t		_reserved[2];
		uint8_t		mac[ETHER_ADDR_LEN];
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !mac)
		return (DPAA2_CMD_STAT_EINVAL);

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_PRIM_MAC_ADDR);
	if (!error) {
		resp = (struct get_prim_mac_resp *) &cmd->params[0];
		for (int i = 1; i <= ETHER_ADDR_LEN; i++)
			mac[ETHER_ADDR_LEN - i] = resp->mac[i - 1];
	}

	return (error);
}

static int
dpaa2_rc_ni_set_link_cfg(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_ni_link_cfg_t *cfg)
{
	struct __packed link_cfg_args {
		uint64_t	_reserved1;
		uint32_t	rate;
		uint32_t	_reserved2;
		uint64_t	options;
		uint64_t	adv_speeds;
		uint64_t	_reserved3[3];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !cfg)
		return (DPAA2_CMD_STAT_EINVAL);

	args = (struct link_cfg_args *) &cmd->params[0];
	args->rate = cfg->rate;
	args->options = cfg->options;
	args->adv_speeds = cfg->adv_speeds;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_LINK_CFG));
}

static int
dpaa2_rc_ni_get_link_cfg(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_ni_link_cfg_t *cfg)
{
	struct __packed link_cfg_resp {
		uint64_t	_reserved1;
		uint32_t	rate;
		uint32_t	_reserved2;
		uint64_t	options;
		uint64_t	adv_speeds;
		uint64_t	_reserved3[3];
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !cfg)
		return (DPAA2_CMD_STAT_EINVAL);

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_LINK_CFG);
	if (!error) {
		resp = (struct link_cfg_resp *) &cmd->params[0];
		cfg->rate = resp->rate;
		cfg->options = resp->options;
		cfg->adv_speeds = resp->adv_speeds;
	}

	return (error);
}

static int
dpaa2_rc_ni_get_link_state(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_ni_link_state_t *state)
{
	struct __packed link_state_resp {
		uint32_t	_reserved1;
		uint32_t	flags;
		uint32_t	rate;
		uint32_t	_reserved2;
		uint64_t	options;
		uint64_t	supported;
		uint64_t	advert;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !state)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_LINK_STATE);
	if (!error) {
		resp = (struct link_state_resp *) &cmd->params[0];
		state->options = resp->options;
		state->adv_speeds = resp->advert;
		state->sup_speeds = resp->supported;
		state->rate = resp->rate;

		state->link_up = resp->flags & 0x1u ? true : false;
		state->state_valid = resp->flags & 0x2u ? true : false;
	}

	return (error);
}

static int
dpaa2_rc_ni_set_qos_table(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_ni_qos_table_t *tbl)
{
	struct __packed qos_table_args {
		uint32_t	_reserved1;
		uint8_t		default_tc;
		uint8_t		options;
		uint16_t	_reserved2;
		uint64_t	_reserved[5];
		uint64_t	kcfg_busaddr;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !tbl)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct qos_table_args *) &cmd->params[0];
	args->default_tc = tbl->default_tc;
	args->kcfg_busaddr = tbl->kcfg_busaddr;

	args->options |= tbl->discard_on_miss	? 1U : 0U;
	args->options |= tbl->keep_entries	? 2U : 0U;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_QOS_TABLE));
}

static int
dpaa2_rc_ni_clear_qos_table(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	return (exec_command(sc->portal, cmd, CMDID_NI_CLEAR_QOS_TABLE));
}

static int
dpaa2_rc_ni_set_pools(device_t rcdev, dpaa2_cmd_t cmd, dpaa2_ni_pools_cfg_t *cfg)
{
	struct __packed set_pools_args {
		uint8_t		pools_num;
		uint8_t		backup_pool_mask;
		uint8_t		_reserved1;
		uint8_t		pool_as; /* assigning: 0 - QPRI, 1 - QDBIN */
		uint32_t	bp_obj_id[DPAA2_NI_MAX_POOLS];
		uint16_t	buf_sz[DPAA2_NI_MAX_POOLS];
		uint32_t	_reserved2;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_pools_args *) &cmd->params[0];
	args->pools_num = cfg->pools_num < DPAA2_NI_MAX_POOLS
	    ? cfg->pools_num : DPAA2_NI_MAX_POOLS;
	for (uint32_t i = 0; i < args->pools_num; i++) {
		args->bp_obj_id[i] = cfg->pools[i].bp_obj_id;
		args->buf_sz[i] = cfg->pools[i].buf_sz;
		args->backup_pool_mask |= (cfg->pools[i].backup_flag & 1) << i;
	}

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_POOLS));
}

static int
dpaa2_rc_ni_set_err_behavior(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_ni_err_cfg_t *cfg)
{
	struct __packed err_behavior_args {
		uint32_t	err_mask;
		uint8_t		flags;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !cfg)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct err_behavior_args *) &cmd->params[0];
	args->err_mask = cfg->err_mask;

	args->flags |= cfg->set_err_fas ? 0x10u : 0u;
	args->flags |= ((uint8_t) cfg->action) & 0x0Fu;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_ERR_BEHAVIOR));
}

static int
dpaa2_rc_ni_get_queue(device_t rcdev, dpaa2_cmd_t cmd, dpaa2_ni_queue_cfg_t *cfg)
{
	struct __packed get_queue_args {
		uint8_t		queue_type;
		uint8_t		tc;
		uint8_t		idx;
		uint8_t		chan_id;
	} *args;
	struct __packed get_queue_resp {
		uint64_t	_reserved1;
		uint32_t	dest_id;
		uint16_t	_reserved2;
		uint8_t		priority;
		uint8_t		flags;
		uint64_t	flc;
		uint64_t	user_ctx;
		uint32_t	fqid;
		uint16_t	qdbin;
		uint16_t	_reserved3;
		uint8_t		cgid;
		uint8_t		_reserved[15];
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !cfg)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct get_queue_args *) &cmd->params[0];
	args->queue_type = (uint8_t) cfg->type;
	args->tc = cfg->tc;
	args->idx = cfg->idx;
	args->chan_id = cfg->chan_id;

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_QUEUE);
	if (!error) {
		resp = (struct get_queue_resp *) &cmd->params[0];

		cfg->dest_id = resp->dest_id;
		cfg->priority = resp->priority;
		cfg->flow_ctx = resp->flc;
		cfg->user_ctx = resp->user_ctx;
		cfg->fqid = resp->fqid;
		cfg->qdbin = resp->qdbin;
		cfg->cgid = resp->cgid;

		cfg->dest_type = (enum dpaa2_ni_dest_type) resp->flags & 0x0Fu;
		cfg->cgid_valid = (resp->flags & 0x20u) > 0u ? true : false;
		cfg->stash_control = (resp->flags & 0x40u) > 0u ? true : false;
		cfg->hold_active = (resp->flags & 0x80u) > 0u ? true : false;

#if 0
		if (bootverbose)
			device_printf(rcdev, "Reading queue config "
			    "(queue_type=%d, tc=%d, idx=%d, chan_id=%d): \n"
			    "\tfqid=%d, dest_id=%d, priority=%d\n"
			    "\tflc=%#jx, user_ctx=%#jx\n"
			    "\tcgid=%d, flags=%#x, qdbin=%d\n",
			    cfg->type, cfg->tc, cfg->idx, cfg->chan_id,
			    cfg->fqid, cfg->dest_id, cfg->priority,
			    cfg->flc, cfg->user_ctx,
			    cfg->cgid, resp->flags, cfg->qdbin
			);
#endif
	}

	return (error);
}

static int
dpaa2_rc_ni_set_queue(device_t rcdev, dpaa2_cmd_t cmd, dpaa2_ni_queue_cfg_t *cfg)
{
	struct __packed set_queue_args {
		uint8_t		queue_type;
		uint8_t		tc;
		uint8_t		idx;
		uint8_t		options;
		uint32_t	_reserved1;
		uint32_t	dest_id;
		uint16_t	_reserved2;
		uint8_t		priority;
		uint8_t		flags;
		uint64_t	flc;
		uint64_t	user_ctx;
		uint8_t		cgid;
		uint8_t		chan_id;
		uint8_t		_reserved[23];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !cfg)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_queue_args *) &cmd->params[0];
	args->queue_type = (uint8_t) cfg->type;
	args->tc = cfg->tc;
	args->idx = cfg->idx;
	args->options = cfg->options;
	args->dest_id = cfg->dest_id;
	args->priority = cfg->priority;
	args->flc = cfg->flow_ctx;
	args->user_ctx = cfg->user_ctx;
	args->cgid = cfg->cgid;
	args->chan_id = cfg->chan_id;

	args->flags |= (uint8_t)(cfg->dest_type & 0x0Fu);
	args->flags |= cfg->stash_control ? 0x40u : 0u;
	args->flags |= cfg->hold_active ? 0x80u : 0u;

#if 0
	if (bootverbose)
		device_printf(rcdev, "Writing queue config: \n"
		    "\tqueue_type=%d tc=%d, idx=%d, options=%#x\n"
		    "\tdest_id=%d, priority=%d, flc=%#jx, user_ctx=%#jx\n"
		    "\tcgid=%d, chan_id=%d, flags=%#x\n",
		    args->queue_type, args->tc, args->idx, args->options,
		    args->dest_id, args->priority, args->flc, args->user_ctx,
		    args->cgid, args->chan_id, args->flags
		);
#endif
	return (exec_command(sc->portal, cmd, CMDID_NI_SET_QUEUE));
}

static int
dpaa2_rc_ni_get_qdid(device_t rcdev, dpaa2_cmd_t cmd,
    enum dpaa2_ni_queue_type type, uint16_t *qdid)
{
	struct __packed get_qdid_args {
		uint8_t		queue_type;
	} *args;
	struct __packed get_qdid_resp {
		uint16_t	qdid;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct get_qdid_args *) &cmd->params[0];
	args->queue_type = (uint8_t) type;

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_QDID);
	if (!error) {
		resp = (struct get_qdid_resp *) &cmd->params[0];
		*qdid = resp->qdid;
	}

	return (error);
}

static int
dpaa2_rc_ni_add_mac_addr(device_t rcdev, dpaa2_cmd_t cmd, uint8_t *mac)
{
	struct __packed add_mac_args {
		uint8_t		flags;
		uint8_t		_reserved;
		uint8_t		mac[ETHER_ADDR_LEN];
		uint8_t		tc_id;
		uint8_t		fq_id;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !mac)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct add_mac_args *) &cmd->params[0];
	for (int i = 1; i <= ETHER_ADDR_LEN; i++)
		args->mac[i - 1] = mac[ETHER_ADDR_LEN - i];

	return (exec_command(sc->portal, cmd, CMDID_NI_ADD_MAC_ADDR));
}

static int
dpaa2_rc_ni_clear_mac_filters(device_t rcdev, dpaa2_cmd_t cmd, bool rm_uni,
    bool rm_multi)
{
	struct __packed clear_mac_filters_args {
		uint8_t		flags;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct clear_mac_filters_args *) &cmd->params[0];
	args->flags |= rm_uni ? 0x1 : 0x0;
	args->flags |= rm_multi ? 0x2 : 0x0;

	return (exec_command(sc->portal, cmd, CMDID_NI_CLEAR_MAC_FILTERS));
}

static int
dpaa2_rc_ni_set_mfl(device_t rcdev, dpaa2_cmd_t cmd, uint16_t length)
{
	struct __packed set_mfl_args {
		uint16_t length;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_mfl_args *) &cmd->params[0];
	args->length = length;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_MFL));
}

static int
dpaa2_rc_ni_set_offload(device_t rcdev, dpaa2_cmd_t cmd,
    enum dpaa2_ni_ofl_type ofl_type, bool en)
{
	struct __packed set_ofl_args {
		uint8_t		_reserved[3];
		uint8_t		ofl_type;
		uint32_t	config;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_ofl_args *) &cmd->params[0];
	args->ofl_type = (uint8_t) ofl_type;
	args->config = en ? 1u : 0u;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_OFFLOAD));
}

static int
dpaa2_rc_ni_set_irq_mask(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint32_t mask)
{
	struct __packed set_irq_mask_args {
		uint32_t	mask;
		uint8_t		irq_idx;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_irq_mask_args *) &cmd->params[0];
	args->mask = mask;
	args->irq_idx = irq_idx;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_IRQ_MASK));
}

static int
dpaa2_rc_ni_set_irq_enable(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    bool en)
{
	struct __packed set_irq_enable_args {
		uint32_t	en;
		uint8_t		irq_idx;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_irq_enable_args *) &cmd->params[0];
	args->en = en ? 1u : 0u;
	args->irq_idx = irq_idx;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_IRQ_ENABLE));
}

static int
dpaa2_rc_ni_get_irq_status(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint32_t *status)
{
	struct __packed get_irq_stat_args {
		uint32_t	status;
		uint8_t		irq_idx;
	} *args;
	struct __packed get_irq_stat_resp {
		uint32_t	status;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !status)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct get_irq_stat_args *) &cmd->params[0];
	args->status = *status;
	args->irq_idx = irq_idx;

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_IRQ_STATUS);
	if (!error) {
		resp = (struct get_irq_stat_resp *) &cmd->params[0];
		*status = resp->status;
	}

	return (error);
}

static int
dpaa2_rc_ni_set_uni_promisc(device_t rcdev, dpaa2_cmd_t cmd, bool en)
{
	struct __packed set_uni_promisc_args {
		uint8_t	en;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_uni_promisc_args *) &cmd->params[0];
	args->en = en ? 1u : 0u;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_UNI_PROMISC));
}

static int
dpaa2_rc_ni_set_multi_promisc(device_t rcdev, dpaa2_cmd_t cmd, bool en)
{
	/*
	 * TODO: Implementation is the same as for ni_set_uni_promisc().
	 */
	struct __packed set_multi_promisc_args {
		uint8_t	en;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_multi_promisc_args *) &cmd->params[0];
	args->en = en ? 1u : 0u;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_MULTI_PROMISC));
}

static int
dpaa2_rc_ni_get_statistics(device_t rcdev, dpaa2_cmd_t cmd, uint8_t page,
    uint16_t param, uint64_t *cnt)
{
	struct __packed get_statistics_args {
		uint8_t		page;
		uint16_t	param;
	} *args;
	struct __packed get_statistics_resp {
		uint64_t	cnt[7];
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !cnt)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct get_statistics_args *) &cmd->params[0];
	args->page = page;
	args->param = param;

	error = exec_command(sc->portal, cmd, CMDID_NI_GET_STATISTICS);
	if (!error) {
		resp = (struct get_statistics_resp *) &cmd->params[0];
		for (int i = 0; i < DPAA2_NI_STAT_COUNTERS; i++)
			cnt[i] = resp->cnt[i];
	}

	return (error);
}

static int
dpaa2_rc_ni_set_rx_tc_dist(device_t rcdev, dpaa2_cmd_t cmd, uint16_t dist_size,
    uint8_t tc, enum dpaa2_ni_dist_mode dist_mode, bus_addr_t key_cfg_buf)
{
	struct __packed set_rx_tc_dist_args {
		uint16_t	dist_size;
		uint8_t		tc;
		uint8_t		ma_dm; /* miss action + dist. mode */
		uint32_t	_reserved1;
		uint64_t	_reserved2[5];
		uint64_t	key_cfg_iova;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_rx_tc_dist_args *) &cmd->params[0];
	args->dist_size = dist_size;
	args->tc = tc;
	args->ma_dm = ((uint8_t) dist_mode) & 0x0Fu;
	args->key_cfg_iova = key_cfg_buf;

	return (exec_command(sc->portal, cmd, CMDID_NI_SET_RX_TC_DIST));
}

static int
dpaa2_rc_io_open(device_t rcdev, dpaa2_cmd_t cmd, const uint32_t dpio_id,
    uint16_t *token)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpio_id;
	error = exec_command(sc->portal, cmd, CMDID_IO_OPEN);
	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

static int
dpaa2_rc_io_close(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_IO_CLOSE));
}

static int
dpaa2_rc_io_enable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_IO_ENABLE));
}

static int
dpaa2_rc_io_disable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_IO_DISABLE));
}

static int
dpaa2_rc_io_reset(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_IO_RESET));
}

static int
dpaa2_rc_io_get_attributes(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_io_attr_t *attr)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct __packed dpaa2_io_attr {
		uint32_t	id;
		uint16_t	swp_id;
		uint8_t		priors_num;
		uint8_t		chan_mode;
		uint64_t	swp_ce_paddr;
		uint64_t	swp_ci_paddr;
		uint32_t	swp_version;
		uint32_t	_reserved1;
		uint32_t	swp_clk;
		uint32_t	_reserved2[5];
	} *pattr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_IO_GET_ATTR);
	if (!error) {
		pattr = (struct dpaa2_io_attr *) &cmd->params[0];

		attr->swp_ce_paddr = pattr->swp_ce_paddr;
		attr->swp_ci_paddr = pattr->swp_ci_paddr;
		attr->swp_version = pattr->swp_version;
		attr->swp_clk = pattr->swp_clk;
		attr->id = pattr->id;
		attr->swp_id = pattr->swp_id;
		attr->priors_num = pattr->priors_num;
		attr->chan_mode = (enum dpaa2_io_chan_mode)
		    pattr->chan_mode;
	}

	return (error);
}

static int
dpaa2_rc_io_set_irq_mask(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint32_t mask)
{
	/*
	 * TODO: Extract similar *_set_irq_mask() into one function.
	 */
	struct __packed set_irq_mask_args {
		uint32_t	mask;
		uint8_t		irq_idx;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_irq_mask_args *) &cmd->params[0];
	args->mask = mask;
	args->irq_idx = irq_idx;

	return (exec_command(sc->portal, cmd, CMDID_IO_SET_IRQ_MASK));
}

static int
dpaa2_rc_io_get_irq_status(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint32_t *status)
{
	/*
	 * TODO: Extract similar *_get_irq_status() into one function.
	 */
	struct __packed get_irq_stat_args {
		uint32_t	status;
		uint8_t		irq_idx;
	} *args;
	struct __packed get_irq_stat_resp {
		uint32_t	status;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !status)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct get_irq_stat_args *) &cmd->params[0];
	args->status = *status;
	args->irq_idx = irq_idx;

	error = exec_command(sc->portal, cmd, CMDID_IO_GET_IRQ_STATUS);
	if (!error) {
		resp = (struct get_irq_stat_resp *) &cmd->params[0];
		*status = resp->status;
	}

	return (error);
}

static int
dpaa2_rc_io_set_irq_enable(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    bool en)
{
	/*
	 * TODO: Extract similar *_set_irq_enable() into one function.
	 */
	struct __packed set_irq_enable_args {
		uint32_t	en;
		uint8_t		irq_idx;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_irq_enable_args *) &cmd->params[0];
	args->en = en ? 1u : 0u;
	args->irq_idx = irq_idx;

	return (exec_command(sc->portal, cmd, CMDID_IO_SET_IRQ_ENABLE));
}

static int
dpaa2_rc_io_add_static_dq_chan(device_t rcdev, dpaa2_cmd_t cmd,
    uint32_t dpcon_id, uint8_t *chan_idx)
{
	struct __packed add_static_dq_chan_args {
		uint32_t	dpcon_id;
	} *args;
	struct __packed add_static_dq_chan_resp {
		uint8_t		chan_idx;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !chan_idx)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct add_static_dq_chan_args *) &cmd->params[0];
	args->dpcon_id = dpcon_id;

	error = exec_command(sc->portal, cmd, CMDID_IO_ADD_STATIC_DQ_CHAN);
	if (!error) {
		resp = (struct add_static_dq_chan_resp *) &cmd->params[0];
		*chan_idx = resp->chan_idx;
	}

	return (error);
}

static int
dpaa2_rc_bp_open(device_t rcdev, dpaa2_cmd_t cmd, const uint32_t dpbp_id,
    uint16_t *token)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpbp_id;
	error = exec_command(sc->portal, cmd, CMDID_BP_OPEN);
	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

static int
dpaa2_rc_bp_close(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_BP_CLOSE));
}

static int
dpaa2_rc_bp_enable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_BP_ENABLE));
}

static int
dpaa2_rc_bp_disable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_BP_DISABLE));
}

static int
dpaa2_rc_bp_reset(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_BP_RESET));
}

static int
dpaa2_rc_bp_get_attributes(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_bp_attr_t *attr)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct __packed dpaa2_bp_attr {
		uint16_t	_reserved1;
		uint16_t	bpid;
		uint32_t	id;
	} *pattr;

	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_BP_GET_ATTR);
	if (!error) {
		pattr = (struct dpaa2_bp_attr *) &cmd->params[0];
		attr->id = pattr->id;
		attr->bpid = pattr->bpid;
	}

	return (error);
}

static int
dpaa2_rc_mac_open(device_t rcdev, dpaa2_cmd_t cmd, const uint32_t dpmac_id,
    uint16_t *token)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpmac_id;
	error = exec_command(sc->portal, cmd, CMDID_MAC_OPEN);
	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}

static int
dpaa2_rc_mac_close(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_MAC_CLOSE));
}

static int
dpaa2_rc_mac_reset(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_MAC_RESET));
}

static int
dpaa2_rc_mac_mdio_read(device_t rcdev, dpaa2_cmd_t cmd, uint8_t phy,
    uint16_t reg, uint16_t *val)
{
	struct __packed mdio_read_args {
		uint8_t		clause; /* set to 0 by default */
		uint8_t		phy;
		uint16_t	reg;
		uint32_t	_reserved1;
		uint64_t	_reserved2[6];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !val)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct mdio_read_args *) &cmd->params[0];
	args->phy = phy;
	args->reg = reg;
	args->clause = 0;

	error = exec_command(sc->portal, cmd, CMDID_MAC_MDIO_READ);
	if (!error)
		*val = cmd->params[0] & 0xFFFF;

	return (error);
}

static int
dpaa2_rc_mac_mdio_write(device_t rcdev, dpaa2_cmd_t cmd, uint8_t phy,
    uint16_t reg, uint16_t val)
{
	struct __packed mdio_write_args {
		uint8_t		clause; /* set to 0 by default */
		uint8_t		phy;
		uint16_t	reg;
		uint16_t	val;
		uint16_t	_reserved1;
		uint64_t	_reserved2[6];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct mdio_write_args *) &cmd->params[0];
	args->phy = phy;
	args->reg = reg;
	args->val = val;
	args->clause = 0;

	return (exec_command(sc->portal, cmd, CMDID_MAC_MDIO_WRITE));
}

static int
dpaa2_rc_mac_get_addr(device_t rcdev, dpaa2_cmd_t cmd, uint8_t *mac)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !mac)
		return (DPAA2_CMD_STAT_ERR);

	error = exec_command(sc->portal, cmd, CMDID_MAC_GET_ADDR);
	if (!error) {
		mac[0] = (cmd->params[0] >> 56) & 0xFFU;
		mac[1] = (cmd->params[0] >> 48) & 0xFFU;
		mac[2] = (cmd->params[0] >> 40) & 0xFFU;
		mac[3] = (cmd->params[0] >> 32) & 0xFFU;
		mac[4] = (cmd->params[0] >> 24) & 0xFFU;
		mac[5] = (cmd->params[0] >> 16) & 0xFFU;
	}

	return (error);
}

static int
dpaa2_rc_mac_get_attributes(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_mac_attr_t *attr)
{
	struct __packed mac_attr_resp {
		uint8_t		eth_if;
		uint8_t		link_type;
		uint16_t	id;
		uint32_t	max_rate;

		uint8_t		fec_mode;
		uint8_t		ifg_mode;
		uint8_t		ifg_len;
		uint8_t		_reserved1;
		uint32_t	_reserved2;

		uint8_t		sgn_post_pre;
		uint8_t		serdes_cfg_mode;
		uint8_t		eq_amp_red;
		uint8_t		eq_post1q;
		uint8_t		eq_preq;
		uint8_t		eq_type;
		uint16_t	_reserved3;

		uint64_t	_reserved[4];
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_EINVAL);

	error = exec_command(sc->portal, cmd, CMDID_MAC_GET_ATTR);
	if (!error) {
		resp = (struct mac_attr_resp *) &cmd->params[0];
		attr->id = resp->id;
		attr->max_rate = resp->max_rate;
		attr->eth_if = resp->eth_if;
		attr->link_type = resp->link_type;
	}

	return (error);
}

static int
dpaa2_rc_mac_set_link_state(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_mac_link_state_t *state)
{
	struct __packed mac_set_link_args {
		uint64_t	options;
		uint32_t	rate;
		uint32_t	_reserved1;
		uint32_t	flags;
		uint32_t	_reserved2;
		uint64_t	supported;
		uint64_t	advert;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !state)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct mac_set_link_args *) &cmd->params[0];
	args->options = state->options;
	args->rate = state->rate;
	args->supported = state->supported;
	args->advert = state->advert;

	args->flags |= state->up ? 0x1u : 0u;
	args->flags |= state->state_valid ? 0x2u : 0u;

	return (exec_command(sc->portal, cmd, CMDID_MAC_SET_LINK_STATE));
}

static int
dpaa2_rc_mac_set_irq_mask(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint32_t mask)
{
	/*
	 * TODO: Implementation is the same as for ni_set_irq_mask().
	 */
	struct __packed set_irq_mask_args {
		uint32_t	mask;
		uint8_t		irq_idx;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_irq_mask_args *) &cmd->params[0];
	args->mask = mask;
	args->irq_idx = irq_idx;

	return (exec_command(sc->portal, cmd, CMDID_MAC_SET_IRQ_MASK));
}

static int
dpaa2_rc_mac_set_irq_enable(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    bool en)
{
	/*
	 * TODO: Implementation is the same as for ni_set_irq_enable().
	 */
	struct __packed set_irq_enable_args {
		uint32_t	en;
		uint8_t		irq_idx;
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct set_irq_enable_args *) &cmd->params[0];
	args->en = en ? 1u : 0u;
	args->irq_idx = irq_idx;

	return (exec_command(sc->portal, cmd, CMDID_MAC_SET_IRQ_ENABLE));
}

static int
dpaa2_rc_mac_get_irq_status(device_t rcdev, dpaa2_cmd_t cmd, uint8_t irq_idx,
    uint32_t *status)
{
	/*
	 * TODO: Implementation is the same as ni_get_irq_status().
	 */
	struct __packed get_irq_stat_args {
		uint32_t	status;
		uint8_t		irq_idx;
	} *args;
	struct __packed get_irq_stat_resp {
		uint32_t	status;
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !status)
		return (DPAA2_CMD_STAT_EINVAL);

	reset_cmd_params(cmd);

	args = (struct get_irq_stat_args *) &cmd->params[0];
	args->status = *status;
	args->irq_idx = irq_idx;

	error = exec_command(sc->portal, cmd, CMDID_MAC_GET_IRQ_STATUS);
	if (!error) {
		resp = (struct get_irq_stat_resp *) &cmd->params[0];
		*status = resp->status;
	}

	return (error);
}

static int
dpaa2_rc_con_open(device_t rcdev, dpaa2_cmd_t cmd, const uint32_t dpcon_id,
    uint16_t *token)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_cmd_header *hdr;
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !token)
		return (DPAA2_CMD_STAT_ERR);

	cmd->params[0] = dpcon_id;
	error = exec_command(sc->portal, cmd, CMDID_CON_OPEN);
	if (!error) {
		hdr = (struct dpaa2_cmd_header *) &cmd->header;
		*token = hdr->token;
	}

	return (error);
}


static int
dpaa2_rc_con_close(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_CON_CLOSE));
}

static int
dpaa2_rc_con_reset(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_CON_RESET));
}

static int
dpaa2_rc_con_enable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_CON_ENABLE));
}

static int
dpaa2_rc_con_disable(device_t rcdev, dpaa2_cmd_t cmd)
{
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	return (exec_command(sc->portal, cmd, CMDID_CON_DISABLE));
}

static int
dpaa2_rc_con_get_attributes(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_con_attr_t *attr)
{
	struct __packed con_attr_resp {
		uint32_t	id;
		uint16_t	chan_id;
		uint8_t		prior_num;
		uint8_t		_reserved1;
		uint64_t	_reserved2[6];
	} *resp;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	int error;

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd || !attr)
		return (DPAA2_CMD_STAT_EINVAL);

	error = exec_command(sc->portal, cmd, CMDID_CON_GET_ATTR);
	if (!error) {
		resp = (struct con_attr_resp *) &cmd->params[0];
		attr->id = resp->id;
		attr->chan_id = resp->chan_id;
		attr->prior_num = resp->prior_num;
	}

	return (error);
}

static int
dpaa2_rc_con_set_notif(device_t rcdev, dpaa2_cmd_t cmd,
    dpaa2_con_notif_cfg_t *cfg)
{
	struct __packed set_notif_args {
		uint32_t	dpio_id;
		uint8_t		prior;
		uint8_t		_reserved1;
		uint16_t	_reserved2;
		uint64_t	ctx;
		uint64_t	_reserved3[5];
	} *args;
	struct dpaa2_rc_softc *sc = device_get_softc(rcdev);
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);

	if (!rcinfo || rcinfo->dtype != DPAA2_DEV_RC)
		return (DPAA2_CMD_STAT_ERR);
	if (!sc->portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct set_notif_args *) &cmd->params[0];
	args->dpio_id = cfg->dpio_id;
	args->prior = cfg->prior;
	args->ctx = cfg->qman_ctx;

	return (exec_command(sc->portal, cmd, CMDID_CON_SET_NOTIF));
}

/*
 * Internal functions.
 */

/**
 * @internal
 * @brief Create and add devices for DPAA2 objects in this resource container.
 */
static int
discover_objects(struct dpaa2_rc_softc *sc)
{
	device_t rcdev = sc->dev;
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	dpaa2_cmd_t cmd = NULL;
	dpaa2_rc_attr_t dprc_attr;
	dpaa2_obj_t obj;
	uint32_t major, minor, rev, obj_count;
	uint16_t rc_token;
	int rc;

	/* Allocate a command to send to MC hardware. */
	rc = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (rc) {
		device_printf(rcdev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    rc);
		return (ENXIO);
	}

	/* Print MC firmware version. */
	rc = DPAA2_CMD_MNG_GET_VERSION(rcdev, cmd, &major, &minor, &rev);
	if (rc) {
		device_printf(rcdev, "Failed to get MC firmware version: "
		    "error=%d\n", rc);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}
	device_printf(rcdev, "MC firmware version: %u.%u.%u\n", major, minor,
	    rev);

	/* Obtain container ID associated with a given MC portal. */
	rc = DPAA2_CMD_MNG_GET_CONTAINER_ID(rcdev, cmd, &sc->cont_id);
	if (rc) {
		device_printf(rcdev, "Failed to get container ID: error=%d\n",
		    rc);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(rcdev, "Resource container ID: %u\n", sc->cont_id);

	/* Open the resource container. */
	rc = DPAA2_CMD_RC_OPEN(rcdev, cmd, sc->cont_id, &rc_token);
	if (rc) {
		device_printf(rcdev, "Failed to open container ID=%u: "
		    "error=%d\n", sc->cont_id, rc);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}

	/* Obtain a number of objects in this container. */
	rc = DPAA2_CMD_RC_GET_OBJ_COUNT(rcdev, cmd, &obj_count);
	if (rc) {
		device_printf(rcdev, "Failed to count objects in container "
		    "ID=%u: error=%d\n", sc->cont_id, rc);
		DPAA2_CMD_RC_CLOSE(rcdev, cmd);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(rcdev, "Objects in container: %u\n", obj_count);

	/* Obtain container attributes (including ICID). */
	rc = DPAA2_CMD_RC_GET_ATTRIBUTES(rcdev, cmd, &dprc_attr);
	if (rc) {
		device_printf(rcdev, "Failed to get attributes of the "
		    "container ID=%u: error=%d\n", sc->cont_id, rc);
		DPAA2_CMD_RC_CLOSE(rcdev, cmd);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(rcdev, "ICID: %u\n", dprc_attr.icid);
	if (rcinfo) {
		rcinfo->id = dprc_attr.cont_id;
		rcinfo->portal_id = dprc_attr.portal_id;
		rcinfo->icid = dprc_attr.icid;
	}

	/* Add managed devices to the resource container. */
	for (uint32_t i = 0; i < obj_count; i++) {
		rc = DPAA2_CMD_RC_GET_OBJ(rcdev, cmd, i, &obj);
		if (rc && bootverbose) {
			if (rc == DPAA2_CMD_STAT_UNKNOWN_OBJ) {
				device_printf(rcdev, "Skip unsupported DPAA2 "
				    "object: index=%u, objects=%u\n", i,
				    obj_count);
				continue;
			} else {
				device_printf(rcdev, "Failed to get information "
				    "about DPAA2 object: index=%u, error=%d\n",
				    i, rc);
				continue;
			}
		}
		add_managed_child(sc, cmd, &obj);
	}

	/* Probe and attach managed devices properly. */
	bus_generic_probe(rcdev);
	rc = bus_generic_attach(rcdev);
	if (rc) {
		DPAA2_CMD_RC_CLOSE(rcdev, cmd);
		dpaa2_mcp_free_command(cmd);
		return (rc);
	}

	/* Add other devices to the resource container. */
	for (uint32_t i = 0; i < obj_count; i++) {
		rc = DPAA2_CMD_RC_GET_OBJ(rcdev, cmd, i, &obj);
		if (rc == DPAA2_CMD_STAT_UNKNOWN_OBJ && bootverbose) {
			device_printf(rcdev, "Skip unsupported DPAA2 object: "
			    "index=%u, objects=%u\n", i, obj_count);
			continue;
		} else if (rc) {
			device_printf(rcdev, "Failed to get object: index=%u, "
			    "error=%d\n", i, rc);
			continue;
		}
		add_child(sc, cmd, &obj);
	}

	DPAA2_CMD_RC_CLOSE(rcdev, cmd);
	dpaa2_mcp_free_command(cmd);

	/* Probe and attach the rest of devices. */
	bus_generic_probe(rcdev);
	return (bus_generic_attach(rcdev));
}

/**
 * @internal
 * @brief Add a new DPAA2 device to the resource container bus.
 */
static int
add_child(struct dpaa2_rc_softc *sc, dpaa2_cmd_t cmd,
    const dpaa2_obj_t *obj)
{
	device_t rcdev, dev;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	struct resource_spec *res_spec;
	const char *devclass;
	int rid, error;

	rcdev = sc->dev;
	rcinfo = device_get_ivars(rcdev);

	switch (obj->type) {
	case DPAA2_DEV_NI:
		devclass = "dpaa2_ni";
		res_spec = dpaa2_ni_spec;
		break;
	default:
		return (ENXIO);
	}

	/* Add a device for the DPAA2 object. */
	dev = device_add_child(rcdev, devclass, -1);
	if (dev == NULL) {
		device_printf(rcdev, "Failed to add a child device for DPAA2 "
		    "object: type=%s, id=%u\n", dpaa2_ttos(obj->type), obj->id);
		return (ENXIO);
	}

	/* Allocate devinfo for a child. */
	dinfo = malloc(sizeof(struct dpaa2_devinfo), M_DPAA2_RC,
	    M_WAITOK | M_ZERO);
	if (!dinfo) {
		device_printf(rcdev, "Failed to allocate dpaa2_devinfo "
		    "for: type=%s, id=%u\n", dpaa2_ttos(obj->type), obj->id);
		return (ENXIO);
	}
	device_set_ivars(dev, dinfo);

	dinfo->pdev = rcdev;
	dinfo->dev = dev;
	dinfo->id = obj->id;
	dinfo->dtype = obj->type;
	/* Children share their parent container's ICID and portal ID. */
	dinfo->icid = rcinfo->icid;
	dinfo->portal_id = rcinfo->portal_id;
	/* MSI configuration */
	dinfo->msi.msi_msgnum = obj->irq_count;
	dinfo->msi.msi_alloc = 0;
	dinfo->msi.msi_handlers = 0;

	/* Initialize a resource list for the child. */
	resource_list_init(&dinfo->resources);

	/* Add DPAA2-specific resources to the resource list. */
	for (; res_spec && res_spec->type != -1; res_spec++) {
		rid = res_spec->rid;
		error = add_dpaa2_res(rcdev, dev, res_spec->type, &rid,
		    res_spec->flags);
		if (error)
			device_printf(rcdev, "add_dpaa2_res() failed: "
			    "error=%d\n", error);
	}

	return (0);
}

/**
 * @internal
 * @brief Add a new managed DPAA2 device to the resource container bus.
 *
 * There are DPAA2 objects (DPIO, DPBP) which have their own drivers and can be
 * allocated as resources or associated with the other DPAA2 objects. This
 * function is supposed to discover such managed objects in the resource
 * container and add them as children to perform a proper initialization.
 *
 * NOTE: It must be called together with bus_generic_probe() and
 *       bus_generic_attach() before add_child().
 */
static int
add_managed_child(struct dpaa2_rc_softc *sc, dpaa2_cmd_t cmd,
    const dpaa2_obj_t *obj)
{
	device_t rcdev, dev;
	struct dpaa2_devinfo *rcinfo, *dinfo;
	dpaa2_rc_obj_region_t reg;
	const char *devclass;
	uint64_t start, end, count;
	uint32_t flags = 0;
	int error;

	rcdev = sc->dev;
	rcinfo = device_get_ivars(rcdev);

	switch (obj->type) {
	case DPAA2_DEV_IO:
		devclass = "dpaa2_io";
		flags = DPAA2_MC_DEV_ALLOCATABLE | DPAA2_MC_DEV_SHAREABLE;
		break;
	case DPAA2_DEV_BP:
		devclass = "dpaa2_bp";
		flags = DPAA2_MC_DEV_ALLOCATABLE;
		break;
	case DPAA2_DEV_CON:
		devclass = "dpaa2_con";
		flags = DPAA2_MC_DEV_ALLOCATABLE;
		break;
	case DPAA2_DEV_MAC:
		devclass = "dpaa2_mac";
		flags = DPAA2_MC_DEV_ASSOCIATED;
		break;
	default:
		/* Only managed devices above are supported. */
		return (EINVAL);
	}

	/* Add a device for the DPAA2 object. */
	dev = device_add_child(rcdev, devclass, -1);
	if (dev == NULL) {
		device_printf(rcdev, "Failed to add a child device for "
		    "managed DPAA2 object: type=%s, id=%u\n",
		    dpaa2_ttos(obj->type), obj->id);
		return (ENXIO);
	}

	/* Allocate devinfo for a child. */
	dinfo = malloc(sizeof(struct dpaa2_devinfo), M_DPAA2_RC,
	    M_WAITOK | M_ZERO);
	if (!dinfo) {
		device_printf(rcdev, "Failed to allocate dpaa2_devinfo "
		    "for: type=%s, id=%u\n", dpaa2_ttos(obj->type), obj->id);
		return (ENXIO);
	}
	device_set_ivars(dev, dinfo);

	dinfo->pdev = rcdev;
	dinfo->dev = dev;
	dinfo->id = obj->id;
	dinfo->dtype = obj->type;
	/* Children share their parent container's ICID and portal ID. */
	dinfo->icid = rcinfo->icid;
	dinfo->portal_id = rcinfo->portal_id;
	/* MSI configuration */
	dinfo->msi.msi_msgnum = obj->irq_count;
	dinfo->msi.msi_alloc = 0;
	dinfo->msi.msi_handlers = 0;

	/* Initialize a resource list for the child. */
	resource_list_init(&dinfo->resources);

	/* Add memory regions to the resource list. */
	for (uint8_t i = 0; i < obj->reg_count; i++) {
		error = DPAA2_CMD_RC_GET_OBJ_REGION(rcdev, cmd, obj->id, i,
		    obj->type, &reg);
		if (error) {
			device_printf(rcdev, "Failed to obtain memory region "
			    "for type=%s, id=%u, reg_idx=%u: error=%d\n",
			    dpaa2_ttos(obj->type), obj->id, i, error);
			continue;
		}
		count = reg.size;
		start = reg.base_paddr + reg.base_offset;
		end = reg.base_paddr + reg.base_offset + reg.size - 1;

		resource_list_add(&dinfo->resources, SYS_RES_MEMORY,
		    i, start, end, count);
	}

	/* Inform MC about a new managed device. */
	error = DPAA2_MC_MANAGE_DEV(rcdev, dev, flags);
	if (error) {
		device_printf(rcdev, "Failed to add a managed DPAA2 device: "
		    "type=%s, id=%u, error=%d\n", dpaa2_ttos(obj->type),
		    obj->id, error);
		return (ENXIO);
	}

	return (0);
}

/**
 * @internal
 * @brief Configure given IRQ using MC command interface.
 */
static int
configure_irq(device_t rcdev, device_t child, int rid, uint64_t addr,
    uint32_t data)
{
	struct dpaa2_rc_softc *rcsc;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	dpaa2_cmd_t cmd;
	uint16_t rc_token;
	int rc = EINVAL;

	if (device_get_parent(child) == rcdev && rid >= 1) {
		rcsc = device_get_softc(rcdev);
		rcinfo = device_get_ivars(rcdev);
		dinfo = device_get_ivars(child);

		/* Allocate a command to send to MC hardware. */
		rc = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
		if (rc) {
			device_printf(rcdev, "Failed to allocate dpaa2_cmd: "
			    "error=%d\n", rc);
			return (ENODEV);
		}

		/* Open resource container. */
		rc = DPAA2_CMD_RC_OPEN(rcdev, cmd, rcinfo->id, &rc_token);
		if (rc) {
			dpaa2_mcp_free_command(cmd);
			device_printf(rcdev, "Failed to open DPRC: error=%d\n",
			    rc);
			return (ENODEV);
		}
		/* Set MSI address and value. */
		rc = DPAA2_CMD_RC_SET_OBJ_IRQ(rcdev, cmd, rid - 1, addr, data,
		    rid, dinfo->id, dinfo->dtype);
		if (rc) {
			dpaa2_mcp_free_command(cmd);
			device_printf(rcdev, "Failed to setup IRQ: "
			    "rid=%d, addr=%jx, data=%x, error=%d\n",
			    rid, addr, data, rc);
			return (ENODEV);
		}
		/* Close resource container. */
		rc = DPAA2_CMD_RC_CLOSE(rcdev, cmd);
		if (rc) {
			dpaa2_mcp_free_command(cmd);
			device_printf(rcdev, "Failed to close DPRC: "
			    "error=%d\n", rc);
			return (ENODEV);
		}

		dpaa2_mcp_free_command(cmd);
		rc = 0;
	}

	return (rc);
}

/**
 * @internal
 * @brief General implementation of the MC command to enable IRQ.
 */
static int
set_irq_enable(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, const uint8_t irq_idx,
    const uint8_t enable, const uint16_t cmdid)
{
	struct __packed set_irq_enable_args {
		uint8_t		enable;
		uint8_t		_reserved1;
		uint16_t	_reserved2;
		uint8_t		irq_idx;
		uint8_t		_reserved3;
		uint16_t	_reserved4;
		uint64_t	_reserved5[6];
	} *args;

	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	args = (struct set_irq_enable_args *) &cmd->params[0];
	args->irq_idx = irq_idx;
	args->enable = enable == 0u ? 0u : 1u;

	return (exec_command(portal, cmd, cmdid));
}

/**
 * @internal
 * @brief Sends a command to MC and waits for response.
 */
static int
exec_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd, uint16_t cmdid)
{
	struct dpaa2_cmd_header *hdr;
	uint16_t flags;
	int error;

	if (!portal || !cmd)
		return (DPAA2_CMD_STAT_ERR);

	/* Prepare a command for the MC hardware. */
	hdr = (struct dpaa2_cmd_header *) &cmd->header;
	hdr->cmdid = cmdid;
	hdr->status = DPAA2_CMD_STAT_READY;

	dpaa2_mcp_lock(portal, &flags);
	if (flags & DPAA2_PORTAL_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_mcp_unlock(portal);
		return (DPAA2_CMD_STAT_INVALID_STATE);
	}

	/* Send a command to MC and wait for the result. */
	send_command(portal, cmd);
	error = wait_for_command(portal, cmd);
	if (error) {
		dpaa2_mcp_unlock(portal);
		return (DPAA2_CMD_STAT_ERR);
	}
	if (hdr->status != DPAA2_CMD_STAT_OK) {
		dpaa2_mcp_unlock(portal);
		return (int)(hdr->status);
	}
	dpaa2_mcp_unlock(portal);

	return (DPAA2_CMD_STAT_OK);
}

/**
 * @internal
 * @brief Writes a command to the MC command portal.
 */
static void
send_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	/* Write command parameters. */
	for (uint32_t i = 1; i <= DPAA2_CMD_PARAMS_N; i++)
		bus_write_8(portal->map, sizeof(uint64_t) * i, cmd->params[i-1]);

	bus_barrier(portal->map, 0, sizeof(struct dpaa2_cmd),
	    BUS_SPACE_BARRIER_WRITE);

	/* Write command header to trigger execution. */
	bus_write_8(portal->map, 0, cmd->header);
}

/**
 * @internal
 * @brief Polls the MC command portal in order to receive a result of the
 *        command execution.
 */
static int
wait_for_command(dpaa2_mcp_t portal, dpaa2_cmd_t cmd)
{
	const uint8_t atomic_portal = portal->atomic;
	const uint32_t attempts = atomic_portal ? CMD_SPIN_ATTEMPTS
	    : CMD_SLEEP_ATTEMPTS;
	struct dpaa2_cmd_header *hdr;
	uint64_t val;
	uint32_t i;
	int rc;

	/* Wait for a command execution result from the MC hardware. */
	for (i = 1; i <= attempts; i++) {
		val = bus_read_8(portal->map, 0);
		hdr = (struct dpaa2_cmd_header *) &val;
		if (hdr->status != DPAA2_CMD_STAT_READY)
			break;

		if (atomic_portal)
			DELAY(CMD_SPIN_TIMEOUT);
		else
			pause("dpaa2", CMD_SLEEP_TIMEOUT);
	}

	/* Return an error on expired timeout, OK - otherwise. */
	rc = i > attempts ? DPAA2_CMD_STAT_TIMEOUT : DPAA2_CMD_STAT_OK;

	/* Read command response. */
	cmd->header = val;
	for (i = 1; i <= DPAA2_CMD_PARAMS_N; i++)
		cmd->params[i-1] = bus_read_8(portal->map, i * sizeof(uint64_t));

	return (rc);
}

/**
 * @internal
 * @brief Reserve a DPAA2-specific device of the given devtype for the child.
 */
static int
add_dpaa2_res(device_t rcdev, device_t child, enum dpaa2_dev_type devtype,
    int *rid, int flags)
{
	device_t dpaa2_dev;
	struct dpaa2_devinfo *dinfo = device_get_ivars(child);
	struct resource *res;
	bool shared = false;
	int error;

	/* Request a free DPAA2 device of the given type from MC. */
	error = DPAA2_MC_GET_FREE_DEV(rcdev, &dpaa2_dev, devtype);
	if (error && !(flags & RF_SHAREABLE)) {
		device_printf(rcdev, "Failed to obtain a free %s (rid=%d) for: "
		    "%s (id=%u)\n", dpaa2_ttos(devtype), *rid,
		    dpaa2_ttos(dinfo->dtype), dinfo->id);
		return (error);
	}

	/* Request a shared DPAA2 device of the given type from MC. */
	if (error) {
		error = DPAA2_MC_GET_SHARED_DEV(rcdev, &dpaa2_dev, devtype);
		if (error) {
			device_printf(rcdev, "Failed to obtain a shared "
			    "%s (rid=%d) for: %s (id=%u)\n", dpaa2_ttos(devtype),
			    *rid, dpaa2_ttos(dinfo->dtype), dinfo->id);
			return (error);
		}
		shared = true;
	}

	/* Add DPAA2 device to the resource list of the child device. */
	resource_list_add(&dinfo->resources, devtype, *rid,
	    (rman_res_t) dpaa2_dev, (rman_res_t) dpaa2_dev, 1);

	/* Reserve a newly added DPAA2 resource. */
	res = resource_list_reserve(&dinfo->resources, rcdev, child, devtype,
	    rid, (rman_res_t) dpaa2_dev, (rman_res_t) dpaa2_dev, 1,
	    flags & ~RF_ACTIVE);
	if (!res) {
		device_printf(rcdev, "Failed to reserve %s (rid=%d) for: %s "
		    "(id=%u)\n", dpaa2_ttos(devtype), *rid,
		    dpaa2_ttos(dinfo->dtype), dinfo->id);
		return (EBUSY);
	}

	/* Reserve a shared DPAA2 device of the given type. */
	if (shared) {
		error = DPAA2_MC_RESERVE_DEV(rcdev, dpaa2_dev, devtype);
		if (error) {
			device_printf(rcdev, "Failed to reserve a shared "
			    "%s (rid=%d) for: %s (id=%u)\n", dpaa2_ttos(devtype),
			    *rid, dpaa2_ttos(dinfo->dtype), dinfo->id);
			return (error);
		}
	}

	return (0);
}

/**
 * @internal
 */
static int
print_dpaa2_type(struct resource_list *rl, enum dpaa2_dev_type type)
{
	struct dpaa2_devinfo *dinfo;
	struct resource_list_entry *rle;
	int printed = 0;
	int retval = 0;

	STAILQ_FOREACH(rle, rl, link) {
		if (rle->type == type) {
			dinfo = device_get_ivars((device_t) rle->start);

			if (printed == 0)
				retval += printf(" %s (id=",
				    dpaa2_ttos(dinfo->dtype));
			else
				retval += printf(",");
			printed++;

			retval += printf("%u", dinfo->id);
		}
	}
	if (printed)
		retval += printf(")");

	return (retval);
}

/**
 * @internal
 */
static void
reset_cmd_params(dpaa2_cmd_t cmd)
{
	if (!cmd)
		return;
	memset(cmd->params, 0, sizeof(cmd->params[0]) * DPAA2_CMD_PARAMS_N);
}

static device_method_t dpaa2_rc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			dpaa2_rc_probe),
	DEVMETHOD(device_attach,		dpaa2_rc_attach),
	DEVMETHOD(device_detach,		dpaa2_rc_detach),

	/* Bus interface */
	DEVMETHOD(bus_get_resource_list,	dpaa2_rc_get_resource_list),
	DEVMETHOD(bus_delete_resource,		dpaa2_rc_delete_resource),
	DEVMETHOD(bus_alloc_resource,		dpaa2_rc_alloc_resource),
	DEVMETHOD(bus_release_resource,		dpaa2_rc_release_resource),
	DEVMETHOD(bus_child_deleted,		dpaa2_rc_child_deleted),
	DEVMETHOD(bus_child_detached,		dpaa2_rc_child_detached),
	DEVMETHOD(bus_setup_intr,		dpaa2_rc_setup_intr),
	DEVMETHOD(bus_teardown_intr,		dpaa2_rc_teardown_intr),
	DEVMETHOD(bus_print_child,		dpaa2_rc_print_child),
	DEVMETHOD(bus_add_child,		device_add_child_ordered),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_activate_resource, 	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, 	bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,		bus_generic_adjust_resource),

	/* Pseudo-PCI interface */
	DEVMETHOD(pci_alloc_msi,		dpaa2_rc_alloc_msi),
	DEVMETHOD(pci_release_msi,		dpaa2_rc_release_msi),
	DEVMETHOD(pci_msi_count,		dpaa2_rc_msi_count),
	DEVMETHOD(pci_get_id,			dpaa2_rc_get_id),

	/* DPAA2 MC command interface */
	DEVMETHOD(dpaa2_cmd_mng_get_version,	dpaa2_rc_mng_get_version),
	DEVMETHOD(dpaa2_cmd_mng_get_soc_version, dpaa2_rc_mng_get_soc_version),
	DEVMETHOD(dpaa2_cmd_mng_get_container_id, dpaa2_rc_mng_get_container_id),
	/*	DPRC commands */
	DEVMETHOD(dpaa2_cmd_rc_open,		dpaa2_rc_open),
	DEVMETHOD(dpaa2_cmd_rc_close,		dpaa2_rc_close),
	DEVMETHOD(dpaa2_cmd_rc_get_obj_count,	dpaa2_rc_get_obj_count),
	DEVMETHOD(dpaa2_cmd_rc_get_obj,		dpaa2_rc_get_obj),
	DEVMETHOD(dpaa2_cmd_rc_get_obj_descriptor, dpaa2_rc_get_obj_descriptor),
	DEVMETHOD(dpaa2_cmd_rc_get_attributes,	dpaa2_rc_get_attributes),
	DEVMETHOD(dpaa2_cmd_rc_get_obj_region,	dpaa2_rc_get_obj_region),
	DEVMETHOD(dpaa2_cmd_rc_get_api_version, dpaa2_rc_get_api_version),
	DEVMETHOD(dpaa2_cmd_rc_set_irq_enable,	dpaa2_rc_set_irq_enable),
	DEVMETHOD(dpaa2_cmd_rc_set_obj_irq,	dpaa2_rc_set_obj_irq),
	DEVMETHOD(dpaa2_cmd_rc_get_conn,	dpaa2_rc_get_conn),
	/*	DPNI commands */
	DEVMETHOD(dpaa2_cmd_ni_open,		dpaa2_rc_ni_open),
	DEVMETHOD(dpaa2_cmd_ni_close,		dpaa2_rc_ni_close),
	DEVMETHOD(dpaa2_cmd_ni_enable,		dpaa2_rc_ni_enable),
	DEVMETHOD(dpaa2_cmd_ni_disable,		dpaa2_rc_ni_disable),
	DEVMETHOD(dpaa2_cmd_ni_get_api_version,	dpaa2_rc_ni_get_api_version),
	DEVMETHOD(dpaa2_cmd_ni_reset,		dpaa2_rc_ni_reset),
	DEVMETHOD(dpaa2_cmd_ni_get_attributes,	dpaa2_rc_ni_get_attributes),
	DEVMETHOD(dpaa2_cmd_ni_set_buf_layout,	dpaa2_rc_ni_set_buf_layout),
	DEVMETHOD(dpaa2_cmd_ni_get_tx_data_off, dpaa2_rc_ni_get_tx_data_offset),
	DEVMETHOD(dpaa2_cmd_ni_get_port_mac_addr, dpaa2_rc_ni_get_port_mac_addr),
	DEVMETHOD(dpaa2_cmd_ni_set_prim_mac_addr, dpaa2_rc_ni_set_prim_mac_addr),
	DEVMETHOD(dpaa2_cmd_ni_get_prim_mac_addr, dpaa2_rc_ni_get_prim_mac_addr),
	DEVMETHOD(dpaa2_cmd_ni_set_link_cfg,	dpaa2_rc_ni_set_link_cfg),
	DEVMETHOD(dpaa2_cmd_ni_get_link_cfg,	dpaa2_rc_ni_get_link_cfg),
	DEVMETHOD(dpaa2_cmd_ni_get_link_state,	dpaa2_rc_ni_get_link_state),
	DEVMETHOD(dpaa2_cmd_ni_set_qos_table,	dpaa2_rc_ni_set_qos_table),
	DEVMETHOD(dpaa2_cmd_ni_clear_qos_table, dpaa2_rc_ni_clear_qos_table),
	DEVMETHOD(dpaa2_cmd_ni_set_pools,	dpaa2_rc_ni_set_pools),
	DEVMETHOD(dpaa2_cmd_ni_set_err_behavior,dpaa2_rc_ni_set_err_behavior),
	DEVMETHOD(dpaa2_cmd_ni_get_queue,	dpaa2_rc_ni_get_queue),
	DEVMETHOD(dpaa2_cmd_ni_set_queue,	dpaa2_rc_ni_set_queue),
	DEVMETHOD(dpaa2_cmd_ni_get_qdid,	dpaa2_rc_ni_get_qdid),
	DEVMETHOD(dpaa2_cmd_ni_add_mac_addr,	dpaa2_rc_ni_add_mac_addr),
	DEVMETHOD(dpaa2_cmd_ni_clear_mac_filters, dpaa2_rc_ni_clear_mac_filters),
	DEVMETHOD(dpaa2_cmd_ni_set_mfl,		dpaa2_rc_ni_set_mfl),
	DEVMETHOD(dpaa2_cmd_ni_set_offload,	dpaa2_rc_ni_set_offload),
	DEVMETHOD(dpaa2_cmd_ni_set_irq_mask,	dpaa2_rc_ni_set_irq_mask),
	DEVMETHOD(dpaa2_cmd_ni_set_irq_enable,	dpaa2_rc_ni_set_irq_enable),
	DEVMETHOD(dpaa2_cmd_ni_get_irq_status,	dpaa2_rc_ni_get_irq_status),
	DEVMETHOD(dpaa2_cmd_ni_set_uni_promisc,	dpaa2_rc_ni_set_uni_promisc),
	DEVMETHOD(dpaa2_cmd_ni_set_multi_promisc, dpaa2_rc_ni_set_multi_promisc),
	DEVMETHOD(dpaa2_cmd_ni_get_statistics,	dpaa2_rc_ni_get_statistics),
	DEVMETHOD(dpaa2_cmd_ni_set_rx_tc_dist,	dpaa2_rc_ni_set_rx_tc_dist),
	/*	DPIO commands */
	DEVMETHOD(dpaa2_cmd_io_open,		dpaa2_rc_io_open),
	DEVMETHOD(dpaa2_cmd_io_close,		dpaa2_rc_io_close),
	DEVMETHOD(dpaa2_cmd_io_enable,		dpaa2_rc_io_enable),
	DEVMETHOD(dpaa2_cmd_io_disable,		dpaa2_rc_io_disable),
	DEVMETHOD(dpaa2_cmd_io_reset,		dpaa2_rc_io_reset),
	DEVMETHOD(dpaa2_cmd_io_get_attributes,	dpaa2_rc_io_get_attributes),
	DEVMETHOD(dpaa2_cmd_io_set_irq_mask,	dpaa2_rc_io_set_irq_mask),
	DEVMETHOD(dpaa2_cmd_io_get_irq_status,	dpaa2_rc_io_get_irq_status),
	DEVMETHOD(dpaa2_cmd_io_set_irq_enable,	dpaa2_rc_io_set_irq_enable),
	DEVMETHOD(dpaa2_cmd_io_add_static_dq_chan, dpaa2_rc_io_add_static_dq_chan),
	/*	DPBP commands */
	DEVMETHOD(dpaa2_cmd_bp_open,		dpaa2_rc_bp_open),
	DEVMETHOD(dpaa2_cmd_bp_close,		dpaa2_rc_bp_close),
	DEVMETHOD(dpaa2_cmd_bp_enable,		dpaa2_rc_bp_enable),
	DEVMETHOD(dpaa2_cmd_bp_disable,		dpaa2_rc_bp_disable),
	DEVMETHOD(dpaa2_cmd_bp_reset,		dpaa2_rc_bp_reset),
	DEVMETHOD(dpaa2_cmd_bp_get_attributes,	dpaa2_rc_bp_get_attributes),
	/*	DPMAC commands */
	DEVMETHOD(dpaa2_cmd_mac_open,		dpaa2_rc_mac_open),
	DEVMETHOD(dpaa2_cmd_mac_close,		dpaa2_rc_mac_close),
	DEVMETHOD(dpaa2_cmd_mac_reset,		dpaa2_rc_mac_reset),
	DEVMETHOD(dpaa2_cmd_mac_mdio_read,	dpaa2_rc_mac_mdio_read),
	DEVMETHOD(dpaa2_cmd_mac_mdio_write,	dpaa2_rc_mac_mdio_write),
	DEVMETHOD(dpaa2_cmd_mac_get_addr,	dpaa2_rc_mac_get_addr),
	DEVMETHOD(dpaa2_cmd_mac_get_attributes, dpaa2_rc_mac_get_attributes),
	DEVMETHOD(dpaa2_cmd_mac_set_link_state,	dpaa2_rc_mac_set_link_state),
	DEVMETHOD(dpaa2_cmd_mac_set_irq_mask,	dpaa2_rc_mac_set_irq_mask),
	DEVMETHOD(dpaa2_cmd_mac_set_irq_enable,	dpaa2_rc_mac_set_irq_enable),
	DEVMETHOD(dpaa2_cmd_mac_get_irq_status,	dpaa2_rc_mac_get_irq_status),
	/*	DPCON commands */
	DEVMETHOD(dpaa2_cmd_con_open,		dpaa2_rc_con_open),
	DEVMETHOD(dpaa2_cmd_con_close,		dpaa2_rc_con_close),
	DEVMETHOD(dpaa2_cmd_con_reset,		dpaa2_rc_con_reset),
	DEVMETHOD(dpaa2_cmd_con_enable,		dpaa2_rc_con_enable),
	DEVMETHOD(dpaa2_cmd_con_disable,	dpaa2_rc_con_disable),
	DEVMETHOD(dpaa2_cmd_con_get_attributes,	dpaa2_rc_con_get_attributes),
	DEVMETHOD(dpaa2_cmd_con_set_notif,	dpaa2_rc_con_set_notif),

	DEVMETHOD_END
};

static driver_t dpaa2_rc_driver = {
	"dpaa2_rc",
	dpaa2_rc_methods,
	sizeof(struct dpaa2_rc_softc),
};

static devclass_t dpaa2_rc_devclass;

/* For root container */
DRIVER_MODULE(dpaa2_rc, dpaa2_mc, dpaa2_rc_driver, dpaa2_rc_devclass, 0, 0);
/* For child containers */
DRIVER_MODULE(dpaa2_rc, dpaa2_rc, dpaa2_rc_driver, dpaa2_rc_devclass, 0, 0);
