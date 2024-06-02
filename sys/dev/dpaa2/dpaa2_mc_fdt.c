/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright © 2021-2024 Dmitry Salychev
 * Copyright © 2022 Bjoern A. Zeeb
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
#include <dev/fdt/simplebus.h>

#include "pcib_if.h"
#include "pci_if.h"
#include "ofw_bus_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"
#include "dpaa2_mc_if.h"

/*
 * Device interface
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
 * DPAA2 MC bus interface
 */

static int
dpaa2_mc_fdt_manage_dev(device_t mcdev, device_t dev, uint32_t flags)
{
	struct dpaa2_mc_softc *sc;
	struct dpaa2_devinfo *dinfo;
	struct dpaa2_macinfo *macinfo;
	phandle_t node, child;
	uint32_t reg;

	sc = device_get_softc(mcdev);
	dinfo = device_get_ivars(dev);

	/* Perform a type-specific management routine. */
	switch (dinfo->dtype) {
	case DPAA2_DEV_MAC:
		break;
	default:
		goto skip_fdt_manage_dev;
	}

	macinfo = (struct dpaa2_macinfo *)dinfo;
	macinfo->node = 0;
	macinfo->valid = false;

	/* Find corresponding dpmac node: fsl-mc -> dpmacs -> dpmac */
	node = OF_child(sc->ofw_node);
	child = ofw_bus_find_compatible(node, "fsl,qoriq-mc-dpmac");
	for (; child > 0; child = OF_peer(child)) {
		if (!ofw_bus_node_is_compatible(child, "fsl,qoriq-mc-dpmac"))
			continue;
		if (!OF_hasprop(child, "reg"))
			continue;
		/* XXX-DSL: How to use simplebus_get_property here? */
		if (OF_getencprop(child, "reg", &reg, sizeof(reg)) == -1)
			continue;

		if (reg == dinfo->id) {
			macinfo->node = child;
			macinfo->valid = true;
			break;
		}
	}

skip_fdt_manage_dev:
	return (dpaa2_mc_manage_dev(mcdev, dev, flags));
}

static device_method_t dpaa2_mc_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mc_fdt_probe),
	DEVMETHOD(device_attach,	dpaa2_mc_fdt_attach),
	DEVMETHOD(device_detach,	dpaa2_mc_detach),

	/* Bus interface */
	DEVMETHOD(bus_get_rman,		dpaa2_mc_rman),
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

	/* DPAA2 MC bus interface */
	DEVMETHOD(dpaa2_mc_manage_dev,	dpaa2_mc_fdt_manage_dev),
	DEVMETHOD(dpaa2_mc_get_free_dev,dpaa2_mc_get_free_dev),
	DEVMETHOD(dpaa2_mc_get_dev,	dpaa2_mc_get_dev),
	DEVMETHOD(dpaa2_mc_get_shared_dev, dpaa2_mc_get_shared_dev),
	DEVMETHOD(dpaa2_mc_reserve_dev,	dpaa2_mc_reserve_dev),
	DEVMETHOD(dpaa2_mc_release_dev, dpaa2_mc_release_dev),

	DEVMETHOD_END
};

DEFINE_CLASS_1(dpaa2_mc, dpaa2_mc_fdt_driver, dpaa2_mc_fdt_methods,
    sizeof(struct dpaa2_mc_softc), dpaa2_mc_driver);

DRIVER_MODULE(dpaa2_mc, simplebus, dpaa2_mc_fdt_driver, 0, 0);
