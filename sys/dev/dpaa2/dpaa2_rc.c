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

MALLOC_DEFINE(M_DPAA2_RC, "dpaa2_rc_memory", "DPAA2 Resource Container memory");

/* Forward declarations. */
static int dpaa2_rc_detach(device_t dev);
static int dpaa2_rc_discover_objects(struct dpaa2_rc_softc *sc);
static int dpaa2_rc_add_child(struct dpaa2_rc_softc *sc, dpaa2_cmd_t cmd,
    const dpaa2_obj_t *obj);

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

void
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
		(void)pci_release_msi(child);
	}
	if (resource_list_release_active(rl, rcdev, child, SYS_RES_MEMORY) != 0)
		device_printf(child, "Leaked memory resources!\n");
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
	if (rcinfo)
		rcinfo->icid = dprc_attr.icid;

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
		dinfo->dtype = DPAA2_DEV_IO;

		/* Children share their parent container's ICID. */
		dinfo->icid = rcinfo->icid;

		/* MSI configuration */
		dinfo->msi.msi_msgnum = obj->irq_count;
		dinfo->msi.msi_alloc = 0;
		dinfo->msi.msi_handlers = 0;

		/* Initialize a resource list for this child device. */
		resource_list_init(&dinfo->resources);

		/* Add memory regions to the resource list. */
		for (uint8_t i = 0; i < obj->reg_count; i++) {
			rc = dpaa2_cmd_rc_get_obj_region(sc->portal, cmd,
			    obj->id, i, "dpio", &reg);
			if (rc) {
				device_printf(rcdev, "Failed to obtain info "
				    "about memory region: type=%s, id=%u, "
				    "reg_idx=%u\n", "dpio", obj->id, i);
				continue;
			}

			if (bootverbose)
				device_printf(rcdev, "Adding DPIO memory region "
				    "#%u: paddr=%jx, base_offset=%jx, "
				    "size=%u, flags=%u, type=%s\n", i,
				    (uintmax_t)reg.base_paddr,
				    (uintmax_t)reg.base_offset,
				    reg.size, reg.flags,
				    reg.type == DPAA2_RC_REG_MC_PORTAL ?
				    "mc_portal" : "qbman_portal");

			count = reg.size;
			start = reg.base_paddr;
			end = reg.base_paddr + reg.size - 1;
			resource_list_add(&dinfo->resources, SYS_RES_MEMORY,
			    i, start, end, count);
		}
	}

	return (0);
}

static device_method_t dpaa2_rc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_rc_probe),
	DEVMETHOD(device_attach,	dpaa2_rc_attach),
	DEVMETHOD(device_detach,	dpaa2_rc_detach),

	/* Bus interface */
	DEVMETHOD(bus_get_resource_list,dpaa2_rc_get_resource_list),
	DEVMETHOD(bus_child_deleted,	dpaa2_rc_child_deleted),
	DEVMETHOD(bus_child_detached,	dpaa2_rc_child_detached),
	DEVMETHOD(bus_set_resource,	bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,	bus_generic_rl_get_resource),

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
