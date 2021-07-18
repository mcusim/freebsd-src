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

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

/* Macros to read/write MC registers */
#define	mcreg_read_4(_sc, _r)		bus_read_4(&(_sc)->map[1], (_r))
#define	mcreg_write_4(_sc, _r, _v)	bus_write_4(&(_sc)->map[1], (_r), (_v))

static struct resource_spec dpaa2_mc_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_UNMAPPED },
	{ SYS_RES_MEMORY, 1, RF_ACTIVE | RF_UNMAPPED | RF_OPTIONAL },
	RESOURCE_SPEC_END
};

/* Forward declarations. */
static u_int dpaa2_mc_get_xref(device_t mcdev, device_t child);

int
dpaa2_mc_attach(device_t dev)
{
	struct dpaa2_mc_softc *sc;
	struct resource_map_request req;
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

		/* Might be necessary to configure StreamID for SMMU. */
		/* ... */
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
	int error;

	bus_generic_detach(dev);

	sc = device_get_softc(dev);
	if (sc->rcdev)
		device_delete_child(dev, sc->rcdev);
	bus_release_resources(dev, dpaa2_mc_spec, sc->res);

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	return (device_delete_children(dev));
}

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

static device_method_t dpaa2_mc_methods[] = {
	DEVMETHOD_END
};

DEFINE_CLASS_0(dpaa2_mc, dpaa2_mc_driver, dpaa2_mc_methods,
    sizeof(struct dpaa2_mc_softc));
