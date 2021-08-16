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

#include <machine/bus.h>
#include <machine/resource.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

/* Macros to enable/disable IRQ using MC command interface. */
#define dpaa2_rc_enable_irq(rc, dev, rid, addr, data) \
    dpaa2_rc_configure_irq((rc), (dev), (rid), 1u, (addr), (data))
#define dpaa2_rc_disable_irq(rc, dev, rid) \
    dpaa2_rc_configure_irq((rc), (dev), (rid), 0u, 0u, 0u)

MALLOC_DEFINE(M_DPAA2_RC, "dpaa2_rc_memory", "DPAA2 Resource Container memory");

/* Forward declarations. */
static int dpaa2_rc_detach(device_t dev);
static int dpaa2_rc_discover_objects(struct dpaa2_rc_softc *sc);
static int dpaa2_rc_add_child(struct dpaa2_rc_softc *sc, dpaa2_cmd_t cmd,
    const dpaa2_obj_t *obj);
static int dpaa2_rc_configure_irq(device_t rcdev, device_t child, int rid,
    uint8_t enable, uint64_t addr, uint32_t data);

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
		dinfo = device_get_ivars(pdev);
		if (dinfo && dinfo->dtype != DPAA2_DEV_MC) {
			device_printf(dev, "Root DPRC attached not to the MC "
			    "bus: dtype=%d\n", dinfo->dtype);
			return (ENXIO);
		}
		dinfo = NULL;

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
		error = dpaa2_mcp_init_portal(&sc->portal, mcsc->res[0],
		    &mcsc->map[0], DPAA2_PORTAL_DEF);
		if (error) {
			device_printf(dev, "Failed to allocate dpaa2_mcp: "
			    "error=%d\n", error);
			dpaa2_rc_detach(dev);
			return (ENXIO);
		}
	} else {
		/* Child DPRCs aren't supported yet. */
		return (ENXIO);
	}

	/* Create DPAA2 devices for objects in this container. */
	error = dpaa2_rc_discover_objects(sc);
	if (error) {
		device_printf(dev, "Failed to discover objects in container: "
		    "error=%d\n", error);
		dpaa2_rc_detach(dev);
		return (error);
	}

	return (0);
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
    rman_res_t start, rman_res_t end, rman_res_t count, u_long num,
    u_int flags)
{
	struct resource_list *rl;
	struct dpaa2_devinfo *dinfo;

	dinfo = device_get_ivars(child);
	rl = &dinfo->resources;
	if (type == SYS_RES_IRQ) {
		/*
		 * Can't alloc legacy interrupt once MSI messages have
		 * been allocated.
		 */
		if (*rid == 0 && dinfo->msi.msi_alloc > 0)
			return (NULL);
	}

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

	if (bootverbose)
		device_printf(rcdev, "%s: Allocating resource for a child: "
		    "type=%d, rid=%d, start=%jx, end=%jx, count=%d, flags=%d\n",
		    __func__, type, *rid, start, end, count, flags);

	return (dpaa2_rc_alloc_multi_resource(rcdev, child, type, rid, start,
	    end, count, 1, flags));
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

	/*
	 * Have to deallocate IRQs before releasing any MSI messages and
	 * have to release MSI messages before deallocating any memory
	 * BARs.
	 */
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
	if (error)
		return (error);

	/* If this is not a direct child, just bail out. */
	if (device_get_parent(child) != rcdev) {
		*cookiep = cookie;
		return (0);
	}

	rid = rman_get_rid(irq);
	if (rid == 0) {
		if (bootverbose)
			device_printf(rcdev, "Cannot setup interrupt with "
			    "rid=0: INTx are not supported by DPAA2 objects\n");
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
		error = PCIB_MAP_MSI(rcdev, child, rman_get_start(irq), &addr,
		    &data);
		if (error) {
			(void)bus_generic_teardown_intr(rcdev, child, irq,
			    cookie);
			return (error);
		}

		/* Enable MSI for this DPAA2 object. */
		error = dpaa2_rc_enable_irq(rcdev, child, rid, addr, data);
		if (error) {
			device_printf(rcdev, "Failed to enable IRQ for "
			    "DPAA2 object: rid=%d, type=%s, unit=%d\n", rid,
			    dpaa2_get_type(dinfo->dtype),
			    device_get_unit(child));
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

		/* Disable MSI for this DPAA2 object. */
		error = dpaa2_rc_disable_irq(rcdev, child, rid);
		if (error) {
			device_printf(rcdev, "Failed to disable IRQ for "
			    "DPAA2 object: rid=%d, type=%s, unit=%d\n", rid,
			    dpaa2_get_type(dinfo->dtype),
			    device_get_unit(child));
			return (error);
		}
		dinfo->msi.msi_handlers--;
	}

	error = bus_generic_teardown_intr(rcdev, child, irq, cookie);
	if (rid > 0)
		KASSERT(error == 0,
		    ("%s: generic teardown failed for MSI", __func__));
	return (error);
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

	if (bootverbose)
		device_printf(child,
		    "attempting to allocate %d MSI vectors (%d supported)\n",
		    *count, dinfo->msi.msi_msgnum);

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
		if (actual == 1)
			device_printf(child, "using IRQ %d for MSI\n", irqs[0]);
		else {
			/*
			 * Be fancy and try to print contiguous runs
			 * of IRQ values as ranges.  'run' is true if
			 * we are in a range.
			 */
			device_printf(child, "using IRQs %d", irqs[0]);
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

/**
 * @internal
 * @brief Create and add devices for DPAA2 objects in this resource container.
 */
static int
dpaa2_rc_discover_objects(struct dpaa2_rc_softc *sc)
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
	rc = dpaa2_cmd_mng_get_version(sc->portal, cmd, &major, &minor, &rev);
	if (rc) {
		device_printf(rcdev, "Failed to get MC firmware version: "
		    "error=%d\n", rc);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}
	device_printf(rcdev, "MC firmware version: %u.%u.%u\n", major, minor,
	    rev);

	/* Obtain container ID associated with a given MC portal. */
	rc = dpaa2_cmd_mng_get_container_id(sc->portal, cmd, &sc->cont_id);
	if (rc) {
		device_printf(rcdev, "Failed to get container ID: error=%d\n",
		    rc);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(rcdev, "Resource container ID: %u\n", sc->cont_id);

	/* Open the resource container. */
	rc = dpaa2_cmd_rc_open(sc->portal, cmd, sc->cont_id, &rc_token);
	if (rc) {
		device_printf(rcdev, "Failed to open container ID=%u: "
		    "error=%d\n", sc->cont_id, rc);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}

	/* Obtain a number of objects in this container. */
	rc = dpaa2_cmd_rc_get_obj_count(sc->portal, cmd, &obj_count);
	if (rc) {
		device_printf(rcdev, "Failed to count objects in container "
		    "ID=%u: error=%d\n", sc->cont_id, rc);
		dpaa2_cmd_rc_close(sc->portal, cmd);
		dpaa2_mcp_free_command(cmd);
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(rcdev, "Objects in container: %u\n", obj_count);

	/* Obtain container attributes (including ICID). */
	rc = dpaa2_cmd_rc_get_attributes(sc->portal, cmd, &dprc_attr);
	if (rc) {
		device_printf(rcdev, "Failed to get attributes of the "
		    "container ID=%u: error=%d\n", sc->cont_id, rc);
		dpaa2_cmd_rc_close(sc->portal, cmd);
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

	/* Add devices to the resource container. */
	for (uint32_t i = 0; i < obj_count; i++) {
		rc = dpaa2_cmd_rc_get_obj(sc->portal, cmd, i, &obj);
		if (rc) {
			device_printf(rcdev, "Failed to get object: index=%u, "
			    "error=%d\n", i, rc);
			continue;
		}
		dpaa2_rc_add_child(sc, cmd, &obj);
	}

	dpaa2_cmd_rc_close(sc->portal, cmd);
	dpaa2_mcp_free_command(cmd);

	bus_generic_probe(rcdev);
	return (bus_generic_attach(rcdev));
}

/**
 * @internal
 * @brief Add a new DPAA2 device to the resource container bus.
 */
static int
dpaa2_rc_add_child(struct dpaa2_rc_softc *sc, dpaa2_cmd_t cmd,
    const dpaa2_obj_t *obj)
{
	device_t rcdev = sc->dev;
	device_t dev;
	struct dpaa2_devinfo *rcinfo = device_get_ivars(rcdev);
	struct dpaa2_devinfo *dinfo;
	dpaa2_rc_obj_region_t reg;
	uint64_t start, end, count;
	int rc;

	/* Add a device if it is DPIO. */
	if (strncmp("dpio", obj->type, strlen("dpio")) == 0) {
		dev = device_add_child(rcdev, "dpaa2_io", obj->id);
		if (dev == NULL) {
			device_printf(rcdev, "Failed to add a child device: "
			    "type=%s, id=%u\n", (const char *)obj->type,
			    obj->id);
			return (ENXIO);
		}

		/* Allocate devinfo for a child device. */
		dinfo = malloc(sizeof(struct dpaa2_devinfo), M_DPAA2_RC,
		    M_WAITOK | M_ZERO);
		if (!dinfo) {
			device_printf(rcdev, "Failed to allocate dpaa2_devinfo "
			    "for: type=%s, id=%u\n", (const char *)obj->type,
			    obj->id);
			return (ENXIO);
		}
		device_set_ivars(dev, dinfo);

		dinfo->pdev = rcdev;
		dinfo->dev = dev;
		dinfo->id = obj->id;
		dinfo->dtype = DPAA2_DEV_IO;
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
			rc = dpaa2_cmd_rc_get_obj_region(sc->portal, cmd,
			    obj->id, i, "dpio", &reg);
			if (rc) {
				device_printf(rcdev, "Failed to obtain memory "
				    "region for obj=dpio, id=%u, reg_idx=%u: "
				    "error=%d\n", obj->id, i, rc);
				continue;
			}
			count = reg.size;
			start = reg.base_paddr + reg.base_offset;
			end = reg.base_paddr + reg.base_offset + reg.size - 1;
			resource_list_add(&dinfo->resources, SYS_RES_MEMORY,
			    i, start, end, count);
		}
	}

	return (0);
}

/**
 * @internal
 * @brief Configure given IRQ using MC command interface.
 */
static int
dpaa2_rc_configure_irq(device_t rcdev, device_t child, int rid, uint8_t enable,
    uint64_t addr, uint32_t data)
{
	struct dpaa2_rc_softc *rcsc;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	dpaa2_cmd_t cmd;
	uint16_t rc_token, io_token;
	int rc, error = EINVAL;

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
		rc = dpaa2_cmd_rc_open(rcsc->portal, cmd, rcinfo->id, &rc_token);
		if (rc) {
			dpaa2_mcp_free_command(cmd);
			device_printf(rcdev, "Failed to open DPRC: error=%d\n",
			    rc);
			return (ENODEV);
		}
		if (enable) {
			/* Set MSI address and value. */
			rc = dpaa2_cmd_rc_set_obj_irq(rcsc->portal, cmd, rid - 1,
			    addr, data, rid, dinfo->id,
			    dpaa2_get_type(dinfo->dtype));
			if (rc) {
				dpaa2_mcp_free_command(cmd);
				device_printf(rcdev, "Failed to setup IRQ: "
				    "rid=%d, addr=%jx, data=%x, error=%d\n",
				    rid, addr, data, rc);
				return (ENODEV);
			}
		}

		/* Enable or disable IRQ. */
		switch (dinfo->dtype) {
		case DPAA2_DEV_IO:
			/* Open a control session for the DPAA2 object. */
			rc = dpaa2_cmd_io_open(rcsc->portal, cmd, dinfo->id,
			    &io_token);
			if (rc) {
				dpaa2_mcp_free_command(cmd);
				device_printf(rcdev, "Failed to open DPIO: "
				    "id=%d, error=%d\n", dinfo->id, rc);
				return (ENODEV);
			}
			/* Enable or disable IRQ. */
			error = dpaa2_cmd_io_set_irq_enable(rcsc->portal, cmd,
			    rid - 1, enable);
			if (error) {
				device_printf(rcdev, "Failed to %s IRQ: "
				    "rid=%d, error=%d\n",
				    enable ? "enable" : "disable", rid, error);
			}
			/* Close the control session of the object. */
			rc = dpaa2_cmd_io_close(rcsc->portal, cmd);
			if (rc) {
				dpaa2_mcp_free_command(cmd);
				device_printf(rcdev, "Failed to close DPIO: "
				    "id=%d, error=%d\n", dinfo->id, rc);
				return (ENODEV);
			}
			break;
		default:
			if (bootverbose)
				device_printf(rcdev, "Cannot %s IRQ for "
				    "unsupported DPAA2 object: rid=%d, "
				    "type=%s\n", enable ? "enable" : "disable",
				    rid, dpaa2_get_type(dinfo->dtype));
			break;
		}

		/* Close resource container. */
		dpaa2_mcp_set_token(cmd, rc_token);
		rc = dpaa2_cmd_rc_close(rcsc->portal, cmd);
		if (rc) {
			dpaa2_mcp_free_command(cmd);
			device_printf(rcdev, "Failed to close DPRC: "
			    "error=%d\n", rc);
			return (ENODEV);
		}

		dpaa2_mcp_free_command(cmd);
	}

	return (error);
}

static device_method_t dpaa2_rc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_rc_probe),
	DEVMETHOD(device_attach,	dpaa2_rc_attach),
	DEVMETHOD(device_detach,	dpaa2_rc_detach),

	/* Bus interface */
	DEVMETHOD(bus_get_resource_list,dpaa2_rc_get_resource_list),
	DEVMETHOD(bus_set_resource,	bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,	bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,	dpaa2_rc_delete_resource),
	DEVMETHOD(bus_alloc_resource,	dpaa2_rc_alloc_resource),
	DEVMETHOD(bus_adjust_resource,	bus_generic_adjust_resource),
	DEVMETHOD(bus_release_resource,	dpaa2_rc_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_child_deleted,	dpaa2_rc_child_deleted),
	DEVMETHOD(bus_child_detached,	dpaa2_rc_child_detached),
	DEVMETHOD(bus_setup_intr,	dpaa2_rc_setup_intr),
	DEVMETHOD(bus_teardown_intr,	dpaa2_rc_teardown_intr),

	/* Pseudo-PCI interface */
	DEVMETHOD(pci_alloc_msi,	dpaa2_rc_alloc_msi),
	DEVMETHOD(pci_release_msi,	dpaa2_rc_release_msi),
	DEVMETHOD(pci_msi_count,	dpaa2_rc_msi_count),
	DEVMETHOD(pci_get_id,		dpaa2_rc_get_id),

	DEVMETHOD_END
};

static driver_t dpaa2_rc_driver = {
	"dpaa2_rc",
	dpaa2_rc_methods,
	sizeof(struct dpaa2_rc_softc),
};

static devclass_t dpaa2_rc_devclass;

DRIVER_MODULE(dpaa2_rc, dpaa2_mc, dpaa2_rc_driver, dpaa2_rc_devclass, 0, 0);
DRIVER_MODULE(dpaa2_rc, dpaa2_rc, dpaa2_rc_driver, dpaa2_rc_devclass, 0, 0);
