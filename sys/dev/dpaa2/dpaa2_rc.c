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
static void dpaa2_rc_add_child(device_t rcdev, struct dpaa2_devinfo *dinfo);

/*
 * Device interface.
 */

static int
dpaa2_rc_probe(device_t dev)
{
	/*
	 * Do not perform any checks. DPRC device will be added by a parent DPRC
	 * or by MC itself.
	 */
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
	dpaa2_cmd_t cmd;
	dpaa2_rc_attr_t dprc_attr;
	uint32_t major, minor, rev;
	uint32_t cont_id, obj_count;
	uint16_t rc_token;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->portal = NULL;
	sc->unit = device_get_unit(dev);

	if (sc->unit == 0) {
		/* Root DPRC attached directly to the MC bus. */
		pdev = device_get_parent(dev);
		mcsc = device_get_softc(pdev);

		/*
		 * Allocate device info to let the MC bus access ICID of the
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
			device_printf(dev, "Failed to allocate dpaa2_mcp\n");
			dpaa2_rc_detach(dev);
			return (ENXIO);
		}
	} else {
		/* Child DPRCs aren't supported yet. */
		return (ENXIO);
	}

	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd\n");
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}

	/*
	 * Print info about MC and DPAA2 objects.
	 */

	error = dpaa2_cmd_mng_get_version(sc->portal, cmd, &major, &minor, &rev);
	if (error) {
		device_printf(dev, "Failed to get MC firmware version\n");
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "MC firmware version: %u.%u.%u\n", major, minor, rev);

	error = dpaa2_cmd_mng_get_container_id(sc->portal, cmd, &cont_id);
	if (error) {	
		device_printf(dev, "Failed to get container ID\n");
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "Resource container ID: %u\n", cont_id);

	error = dpaa2_cmd_rc_open(sc->portal, cmd, cont_id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open container: ID=%u\n", cont_id);
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}

	error = dpaa2_cmd_rc_get_obj_count(sc->portal, cmd, &obj_count);
	if (error) {
		device_printf(dev, "Failed to count objects in container: "
		    "ID=%u\n", cont_id);
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "Objects in container: %u\n", obj_count);

	error = dpaa2_cmd_rc_get_attributes(sc->portal, cmd, &dprc_attr);
	if (error) {
		device_printf(dev, "Failed to get attributes of the container: "
		    "ID=%u\n", cont_id);
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "ICID: %u\n", dprc_attr.icid);
	if (dinfo)
		dinfo->icid = dprc_attr.icid;

	/*
	 * Ask MC bus to allocate MSI for this DPRC using its pseudo-pcib
	 * interface.
	 */

	/* error = PCIB_ALLOC_MSI(device_get_parent(dev), dev, 1, 1, irqs); */
	/* if (error) { */
	/* 	device_printf(dev, "Failed to allocate MSI for DPRC: " */
	/* 	    "error=%u\n", error); */
	/* 	dpaa2_mcp_free_command(cmd); */
	/* 	dpaa2_rc_detach(dev); */
	/* 	return (ENXIO); */
	/* } */
	/* device_printf(dev, "MSI allocated: irq=%d\n", irqs[0]); */

	dpaa2_cmd_rc_close(sc->portal, cmd);
	dpaa2_mcp_free_command(cmd);

	return (0);
}

static int
dpaa2_rc_detach(device_t dev)
{
	struct dpaa2_rc_softc *sc;
	struct dpaa2_devinfo *dinfo;

	sc = device_get_softc(dev);
	dinfo = device_get_ivars(dev);

	if (sc->portal)
		dpaa2_mcp_free_portal(sc->portal);
	if (dinfo)
		free(dinfo, M_DPAA2_RC);

	return (0);
}

/*
 * Bus interface.
 */

static int
dpaa2_rc_setup_intr(device_t rcdev, device_t child, struct resource *irq,
    int flags, driver_filter_t *filter, driver_intr_t *intr, void *arg,
    void **cookiep)
{
	return (ENODEV);
}

static int
dpaa2_rc_teardown_intr(device_t rcdev, device_t child, struct resource *irq,
    void *cookie)
{
	return (ENODEV);
}

static struct resource *
dpaa2_rc_alloc_resource(device_t rcdev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	return (NULL);
}

static int
dpaa2_rc_release_resource(device_t rcdev, device_t child, int type, int rid,
    struct resource *r)
{
	return (ENODEV);
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
	int actual, i, run, irqs[32];

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
	int error, i, irqs[32];

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

/*
 * Return the max supported MSI messages this DPAA2 device supports.
 * Basically, assuming the MD code can alloc messages, this function
 * should return the maximum value that pci_alloc_msi() can return.
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
		return (ENXIO);

	return (PCIB_GET_ID(device_get_parent(rcdev), child, type, id));
}

static void
dpaa2_rc_add_child(device_t rcdev, struct dpaa2_devinfo *dinfo)
{
	return;
}

static device_method_t dpaa2_rc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_rc_probe),
	DEVMETHOD(device_attach,	dpaa2_rc_attach),
	DEVMETHOD(device_detach,	dpaa2_rc_detach),

	/* Bus interface */
	/* DEVMETHOD(bus_setup_intr,	dpaa2_rc_setup_intr), */
	/* DEVMETHOD(bus_teardown_intr,	dpaa2_rc_teardown_intr), */
	/* DEVMETHOD(bus_alloc_resource,	dpaa2_rc_alloc_resource), */
	/* DEVMETHOD(bus_release_resource,	dpaa2_rc_release_resource), */

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
