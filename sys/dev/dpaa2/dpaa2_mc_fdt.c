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
 * The DPAA2 Management Complex (MC) Bus Driver (FDT-based).
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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"
#include "dpaa2_mc_if.h"

/*
 * Device interface.
 */

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

static int
dpaa2_mc_fdt_attach(device_t dev)
{
	struct dpaa2_mc_softc *sc;

	sc = device_get_softc(dev);
	sc->acpi_based = false;
	sc->ofw_node = ofw_bus_get_node(dev);

	return (dpaa2_mc_attach(dev));
}

/*
 * OFW bus interface.
 */

const char *
dpaa2_mc_fdt_get_compat(device_t bus, device_t dev)
{
	struct dpaa2_mc_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(dev);

	/* TODO: Hardcoded string for now. */
	if (sc && dinfo && dinfo->dtype == DPAA2_DEV_MC)
		return ("fsl,qoriq-mc");
	return (NULL);
}

const char *
dpaa2_mc_fdt_get_name(device_t bus, device_t dev)
{
	struct dpaa2_mc_softc *sc = device_get_softc(dev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(dev);

	/* TODO: Hardcoded string for now. */
	if (sc && dinfo && dinfo->dtype == DPAA2_DEV_MC)
		return ("fsl-mc");
	return (NULL);
}

phandle_t
dpaa2_mc_fdt_get_node(device_t bus, device_t dev)
{
	struct dpaa2_mc_softc *sc;

	sc = device_get_softc(dev);

	return (sc->ofw_node);
}

static device_method_t dpaa2_mc_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mc_fdt_probe),
	DEVMETHOD(device_attach,	dpaa2_mc_fdt_attach),
	DEVMETHOD(device_detach,	dpaa2_mc_detach),

	/* Bus interface */
	DEVMETHOD(bus_alloc_resource,	dpaa2_mc_alloc_resource),
	DEVMETHOD(bus_adjust_resource,	dpaa2_mc_adjust_resource),
	DEVMETHOD(bus_release_resource,	dpaa2_mc_release_resource),
	DEVMETHOD(bus_activate_resource, dpaa2_mc_activate_resource),
	DEVMETHOD(bus_deactivate_resource, dpaa2_mc_deactivate_resource),
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),

	/* Pseudo-PCIB interface */
	DEVMETHOD(pcib_alloc_msi,	dpaa2_mc_alloc_msi),
	DEVMETHOD(pcib_release_msi,	dpaa2_mc_release_msi),
	DEVMETHOD(pcib_map_msi,		dpaa2_mc_map_msi),
	DEVMETHOD(pcib_get_id,		dpaa2_mc_get_id),

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_compat,	dpaa2_mc_fdt_get_compat),
	DEVMETHOD(ofw_bus_get_name,	dpaa2_mc_fdt_get_name),
	DEVMETHOD(ofw_bus_get_node,	dpaa2_mc_fdt_get_node),

	/* DPAA2 MC bus driver interface */
	DEVMETHOD(dpaa2_mc_manage_dev,	dpaa2_mc_manage_dev),
	DEVMETHOD(dpaa2_mc_get_free_dev,dpaa2_mc_get_free_dev),
	DEVMETHOD(dpaa2_mc_get_dev,	dpaa2_mc_get_dev),
	DEVMETHOD(dpaa2_mc_get_shared_dev, dpaa2_mc_get_shared_dev),

	DEVMETHOD_END
};

DEFINE_CLASS_1(dpaa2_mc, dpaa2_mc_fdt_driver, dpaa2_mc_fdt_methods,
    sizeof(struct dpaa2_mc_softc), dpaa2_mc_driver);

static devclass_t dpaa2_mc_fdt_devclass;

DRIVER_MODULE(dpaa2_mc, simplebus, dpaa2_mc_fdt_driver, dpaa2_mc_fdt_devclass,
    0, 0);
