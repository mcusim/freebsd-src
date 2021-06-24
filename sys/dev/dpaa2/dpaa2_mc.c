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
 * The DPAA2 Management Complex (MC) Bus Driver.
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

#include <machine/bus.h>
#include <machine/resource.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include "dpaa2_mcvar.h"

MALLOC_DEFINE(M_DPMC, "dpmc_memory", "DPAA2 Management Complex driver memory");

/* Device interface */
static int dpaa2_mc_probe(device_t dev);
static int dpaa2_mc_attach(device_t dev);
static int dpaa2_mc_detach(device_t dev);

/* Macros to read/write MC registers */
#define	mc_reg_r4(_sc, _r)		bus_read_4(&(_sc)->map[1], (_r))
#define	mc_reg_w4(_sc, _r, _v)		bus_write_4(&(_sc)->map[1], (_r), (_v))

static struct resource_spec dpaa2_mc_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_UNMAPPED },
	{ SYS_RES_MEMORY, 1, RF_ACTIVE | RF_UNMAPPED | RF_OPTIONAL },
	RESOURCE_SPEC_END
};

/*
 * Device interface.
 */

#ifdef DEV_ACPI
static int
dpaa2_mc_acpi_probe(device_t dev)
{
	static char *dpaa2_mc_ids[] = { "NXP0008", NULL };
	int rv;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, dpaa2_mc_ids, NULL);
	if (rv <= 0)
		device_set_desc(dev, "DPAA2 Management Complex");

	return (rv);
}
#endif

#ifdef FDT
static int
dpaa2_mc_fdt_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "fsl,qoriq-mc"))
		return (ENXIO);

	device_set_desc(dev, "DPAA2 Management Complex");
	return (BUS_PROBE_DEFAULT);
}
#endif

static int
dpaa2_mc_attach(device_t dev)
{
	struct dpaa2_mc_softc *sc;
	struct resource_map_request req;
	dpaa2_mcp_t *mcp;
	device_t rcdev;
	uint32_t val;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);
	sc->mcp = NULL;

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
		val = mc_reg_r4(sc, MC_REG_GCR1) & (~GCR1_P1_STOP);
		mc_reg_w4(sc, MC_REG_GCR1, val);

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

	/* Populate MCP helper object. */
	mcp = malloc(sizeof(dpaa2_mcp_t), M_DPMC, M_WAITOK | M_ZERO);
	if (!mcp) {
		device_printf(dev, "Failed to allocate memory for dpaa2_mcp\n");
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}
	mcp->dev = NULL; /* No DPMCP device created yet. */
	mcp->portal = sc->res[0];
	mcp->mportal = &sc->map[0];
	mtx_init(&mcp->mutex, device_get_nameunit(dev),
	    "root MC portal lock", MTX_DEF);
	sc->mcp = mcp;

	/*
	 * Add a root resource container as the only child of the bus. All of
	 * the direct descendant containers will be attached to the root one
	 * instead of the MC bus device.
	 */
	rcdev = device_add_child(dev, "dpaa2_rc", 0);
	if (rcdev == NULL) {
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}
	bus_generic_probe(dev);
	bus_generic_attach(dev);

	return (0);
}

static int
dpaa2_mc_detach(device_t dev)
{
	struct dpaa2_mc_softc *sc;

	sc = device_get_softc(dev);

	if (sc->mcp) {
		mtx_destroy(&sc->mcp->mutex);
		free(sc->mcp, M_DPMC);
	}
	bus_release_resources(dev, dpaa2_mc_spec, sc->res);

	return (0);
}

#ifdef DEV_ACPI
static devclass_t dpaa2_mc_acpi_devclass;
static device_method_t dpaa2_mc_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mc_acpi_probe),
	DEVMETHOD(device_attach,	dpaa2_mc_attach),
	DEVMETHOD(device_detach,	dpaa2_mc_detach),
	DEVMETHOD_END
};
static driver_t dpaa2_mc_acpi_driver = {
	"dpaa2_mc",
	dpaa2_mc_acpi_methods,
	sizeof(struct dpaa2_mc_softc),
};
DRIVER_MODULE(dpaa2_mc, acpi, dpaa2_mc_acpi_driver, dpaa2_mc_acpi_devclass,
    NULL, NULL);
#endif

#ifdef FDT
static devclass_t dpaa2_mc_fdt_devclass;
static device_method_t dpaa2_mc_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mc_fdt_probe),
	DEVMETHOD(device_attach,	dpaa2_mc_attach),
	DEVMETHOD(device_detach,	dpaa2_mc_detach),
	DEVMETHOD_END
};
static driver_t dpaa2_mc_fdt_driver = {
	"dpaa2_mc",
	dpaa2_mc_fdt_methods,
	sizeof(struct dpaa2_mc_softc),
};
DRIVER_MODULE(dpaa2_mc, simplebus, dpaa2_mc_fdt_driver, dpaa2_mc_fdt_devclass,
    NULL, NULL);
#endif
