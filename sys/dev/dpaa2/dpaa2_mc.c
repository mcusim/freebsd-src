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
 * The DPAA2 Management Complex (MC) bus driver.
 *
 * MC is a hardware resource manager which can be found in several NXP
 * SoCs (LX2160A, for example) and provides an access to the specialized
 * hardware objects used in network-oriented packet processing applications.
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

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

/* Macros to read/write MC registers */
#define	mcreg_read_4(_sc, _r)		bus_read_4(&(_sc)->map[1], (_r))
#define	mcreg_write_4(_sc, _r, _v)	bus_write_4(&(_sc)->map[1], (_r), (_v))

#define IORT_DEVICE_NAME		"MCE"

MALLOC_DEFINE(M_DPAA2_MC, "dpaa2_mc_memory", "DPAA2 Management Complex memory");

static struct resource_spec dpaa2_mc_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_UNMAPPED },
	{ SYS_RES_MEMORY, 1, RF_ACTIVE | RF_UNMAPPED | RF_OPTIONAL },
	RESOURCE_SPEC_END
};

/* Forward declarations. */
static u_int dpaa2_mc_get_xref(device_t mcdev, device_t child);
static u_int dpaa2_mc_map_id(device_t mcdev, device_t child, uintptr_t *id);

/*
 * For device interface.
 */

int
dpaa2_mc_attach(device_t dev)
{
	struct dpaa2_mc_softc *sc;
	struct resource_map_request req;
	struct dpaa2_devinfo *dinfo = NULL;
	uint32_t val;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	error = bus_alloc_resources(sc->dev, dpaa2_mc_spec, sc->res);
	if (error) {
		device_printf(dev, "Failed to allocate resources\n");
		return (ENXIO);
	}

	if (sc->res[1]) {
		resource_init_map_request(&req);
		req.memattr = VM_MEMATTR_DEVICE;
		error = bus_map_resource(sc->dev, SYS_RES_MEMORY, sc->res[1],
		    &req, &sc->map[1]);
		if (error) {
			device_printf(dev, "Failed to map control registers\n");
			dpaa2_mc_detach(dev);
			return (ENXIO);
		}

		/* Reset P1_STOP bit to resume MC processor. */
		val = mcreg_read_4(sc, MC_REG_GCR1) & (~GCR1_P1_STOP);
		mcreg_write_4(sc, MC_REG_GCR1, val);
	}

	/* At least 64 bytes of the command portal should be available. */
	if (rman_get_size(sc->res[0]) < DPAA2_MCP_MEM_WIDTH) {
		device_printf(dev, "MC portal memory region too small: %jd\n",
		    rman_get_size(sc->res[0]));
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}

	/* Map MC portal memory resource. */
	resource_init_map_request(&req);
	req.memattr = VM_MEMATTR_DEVICE;
	error = bus_map_resource(sc->dev, SYS_RES_MEMORY, sc->res[0],
	    &req, &sc->map[0]);
	if (error) {
		device_printf(dev, "Failed to map MC portal memory\n");
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}

	/* Initialize resource manager of the I/O memory. */
	sc->io_rman.rm_type = RMAN_ARRAY;
	sc->io_rman.rm_descr = "DPAA2 I/O memory";
	error = rman_init(&sc->io_rman);
	if (error) {
		device_printf(dev, "rman_init() failed. error = %d\n", error);
		return (ENXIO);
	}

	/* Allocate devinfo to keep information about the MC bus itself. */
	dinfo = malloc(sizeof(struct dpaa2_devinfo), M_DPAA2_MC,
	    M_WAITOK | M_ZERO);
	if (!dinfo) {
		device_printf(dev, "Failed to allocate dpaa2_devinfo\n");
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}
	device_set_ivars(dev, dinfo);
	dinfo->pdev = device_get_parent(dev);
	dinfo->dev = dev;
	dinfo->dtype = DPAA2_DEV_MC;

	/*
	 * Add a root resource container as the only child of the bus. All of
	 * the direct descendant containers will be attached to the root one
	 * instead of the MC device.
	 */
	sc->rcdev = device_add_child(dev, "dpaa2_rc", 0);
	if (sc->rcdev == NULL) {
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}
	bus_generic_probe(dev);
	bus_generic_attach(dev);

	return (0);
}

int
dpaa2_mc_detach(device_t dev)
{
	struct dpaa2_mc_softc *sc;
	struct dpaa2_devinfo *dinfo = NULL;
	int error;

	bus_generic_detach(dev);

	sc = device_get_softc(dev);
	if (sc->rcdev)
		device_delete_child(dev, sc->rcdev);
	bus_release_resources(dev, dpaa2_mc_spec, sc->res);

	dinfo = device_get_ivars(dev);
	if (dinfo)
		free(dinfo, M_DPAA2_MC);

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	return (device_delete_children(dev));
}

/*
 * For bus interface.
 */

struct resource *
dpaa2_mc_alloc_resource(device_t mcdev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct dpaa2_mc_softc *sc;
	struct resource *res;
	struct rman *rm;
	int error;

	sc = device_get_softc(mcdev);
	rm = &sc->io_rman;

	/*
	 * I/O region which should be managed by the MC is not previously known.
	 */
	error = rman_manage_region(rm, start, end);
	if (error) {
		device_printf(mcdev, "rman_manage_region() failed. error = %d\n",
		    error);
		goto fail;
	}

	if (bootverbose)
		device_printf(mcdev, "rman_reserve_resource: start=%#jx, "
		    "end=%#jx, count=%#jx\n", start, end, count);
	res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (res == NULL)
		goto fail;

	rman_set_rid(res, *rid);

	if (flags & RF_ACTIVE) {
		if (bootverbose)
			device_printf(mcdev, "bus_activate_resource: rid=%d, "
			    "res=%#jx\n", *rid, (uintmax_t) res);
		if (bus_activate_resource(child, type, *rid, res)) {
			rman_release_resource(res);
			goto fail;
		}
	}

	return (res);

 fail:
	device_printf(mcdev, "%s FAIL: type=%d, rid=%d, "
	    "start=%016jx, end=%016jx, count=%016jx, flags=%x\n",
	    __func__, type, *rid, start, end, count, flags);

	return (NULL);
}

static int dpaa2_mc_activate_resource(device_t mcdev, device_t child, int type,
    int rid, struct resource *r)
{
	struct dpaa2_mc_softc *sc;
	int res;

	sc = device_get_softc(mcdev);

	if ((res = rman_activate_resource(r)) != 0)
		return (res);

	return (BUS_ACTIVATE_RESOURCE(device_get_parent(mcdev), child, type,
	    rid, r));
}

/*
 * For pseudo-pcib interface.
 */

int
dpaa2_mc_alloc_msi(device_t mcdev, device_t child, int count, int maxcount,
    int *irqs)
{
#if defined(INTRNG)
	return (intr_alloc_msi(mcdev, child, dpaa2_mc_get_xref(mcdev, child),
	    count, maxcount, irqs));
#else
	return (ENXIO);
#endif
}

int
dpaa2_mc_release_msi(device_t mcdev, device_t child, int count, int *irqs)
{
#if defined(INTRNG)
	return (intr_release_msi(mcdev, child, dpaa2_mc_get_xref(mcdev, child),
	    count, irqs));
#else
	return (ENXIO);
#endif
}

int
dpaa2_mc_map_msi(device_t mcdev, device_t child, int irq, uint64_t *addr,
    uint32_t *data)
{
#if defined(INTRNG)
	return (intr_map_msi(mcdev, child, dpaa2_mc_get_xref(mcdev, child), irq,
	    addr, data));
#else
	return (ENXIO);
#endif
}

int
dpaa2_mc_get_id(device_t mcdev, device_t child, enum pci_id_type type,
    uintptr_t *id)
{
	struct dpaa2_devinfo *mcinfo;
	struct dpaa2_devinfo *dinfo;

	mcinfo = device_get_ivars(mcdev);
	dinfo = device_get_ivars(child);

	if (mcinfo->dtype != DPAA2_DEV_MC)
		return (ENXIO);

	if (type == PCI_ID_MSI)
		return (dpaa2_mc_map_id(mcdev, child, id));

	*id = dinfo->icid;
	return (0);
}

const char *
dpaa2_get_type(enum dpaa2_dev_type dtype)
{
	switch (dtype) {
	case DPAA2_DEV_MC:
		return ("mc");
	case DPAA2_DEV_RC:
		return ("dprc");
	case DPAA2_DEV_IO:
		return ("dpio");
	case DPAA2_DEV_NI:
		return ("dpni");
	case DPAA2_DEV_MCP:
		return ("dpmcp");
	default:
		return ("unknown");
	}
	return ("unknown");
}

static u_int
dpaa2_mc_get_xref(device_t mcdev, device_t child)
{
	struct dpaa2_devinfo *dinfo;
	u_int xref, devid;
	int error;

	dinfo = device_get_ivars(child);
	if (dinfo) {
		error = acpi_iort_map_named_msi("MCE0", dinfo->icid, &xref,
		    &devid);
		if (error)
			return (0);
		return (xref);
	}
	return (0);
}

static u_int
dpaa2_mc_map_id(device_t mcdev, device_t child, uintptr_t *id)
{
	struct dpaa2_devinfo *dinfo;
	u_int xref, devid;
	int error;

	dinfo = device_get_ivars(child);
	if (dinfo) {
		/*
		 * The first named components from IORT table with the given
		 * name (as a substring) will be used.
		 *
		 * TODO: Find a way to form a device name based on "mcdev", i.e.
		 *       dpaa2_mcX -> MCEx?
		 */
		error = acpi_iort_map_named_msi(IORT_DEVICE_NAME, dinfo->icid,
		    &xref, &devid);
		if (error == 0)
			*id = devid;
		else
			*id = dinfo->icid; /* RID not in IORT, likely FW bug */

		return (0);
	}
	return (ENXIO);
}

static device_method_t dpaa2_mc_methods[] = {
	DEVMETHOD_END
};

DEFINE_CLASS_0(dpaa2_mc, dpaa2_mc_driver, dpaa2_mc_methods,
    sizeof(struct dpaa2_mc_softc));
