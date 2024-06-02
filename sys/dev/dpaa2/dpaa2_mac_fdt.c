/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright © 2022 Bjoern A. Zeeb
 * Copyright © 2024 Dmitry Salychev
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
 * The DPAA2 MAC driver (with the FDT-based configuration).
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/kobj.h>
#include <sys/systm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/simplebus.h>

#include "dpaa2_mac.h"

/**
 * @struct dpaa2_mac_fdt_softc
 * @brief Software context for DPMAC device with the FDT-based configuration.
 */
struct dpaa2_mac_fdt_softc {
	struct dpaa2_mac_softc	scc; /* Must stay first. */
	uint32_t		reg;
	phandle_t		sfp;
	phandle_t		pcs_handle;
	phandle_t		phy_handle;
	char			managed[64];
	char			phy_conn_type[64];
};
CTASSERT(sizeof(struct dpaa2_mac_fdt_softc) >= sizeof(struct dpaa2_mac_softc));

/*
 * Device interface
 */

static int
dpaa2_mac_fdt_probe(device_t dev)
{
	struct dpaa2_macinfo *macinfo;
	phandle_t node;
	uint32_t reg;

	macinfo = device_get_ivars(dev);
	if (!macinfo->valid)
		goto skip_fdt_probe;

	node = macinfo->node;
	if (!ofw_bus_node_is_compatible(node, "fsl,qoriq-mc-dpmac")) {
		device_printf(dev, "%s: '%s' not fsl,qoriq-mc-dpmac "
		    "compatible\n", __func__, ofw_bus_get_name(dev));
		return (ENXIO);
	}

	/* DPMAC number supplied via "reg" property is considered mandatory. */
	if (!OF_hasprop(node, "reg") ||
	    (OF_getencprop(node, "reg", &reg, sizeof(reg)) == -1)) {
		device_printf(dev, "%s: '%s' has no 'reg' property\n",
		    __func__, ofw_bus_get_name(dev));
		return (ENXIO);
	}

skip_fdt_probe:
	return (dpaa2_mac_probe(dev));
}

static int
dpaa2_mac_fdt_attach(device_t dev)
{
	struct dpaa2_mac_fdt_softc *sc;
	struct dpaa2_devinfo *dinfo;
	struct dpaa2_macinfo *macinfo;
	phandle_t node, xref;
	ssize_t s;

	macinfo = device_get_ivars(dev);
	if (!macinfo->valid) {
		if (bootverbose)
			device_printf(dev, "%s: no FDT node\n", __func__);
		goto skip_fdt_attach;
	}

	sc = device_get_softc(dev);
	dinfo = (struct dpaa2_devinfo *)macinfo;
	node = macinfo->node;

	/* XXX-DSL: How to use simplebus_get_property here? */

	s = OF_getencprop(node, "reg", &sc->reg, sizeof(sc->reg));
	if (s == -1) {
		device_printf(dev, "%s: cannot find 'reg' property\n", __func__);
		return (ENXIO);
	} else if (dinfo->id != sc->reg) {
		device_printf(dev, "%s: '%s' 'reg' (%u) property does not match "
		    "DPMAC ID (%u)\n", __func__, ofw_bus_get_name(dev), sc->reg,
		    dinfo->id);
		return (ENXIO);
	}

	(void)OF_getprop(node, "managed", sc->managed, sizeof(sc->managed));
	(void)OF_getprop(node, "phy-connection-type", sc->phy_conn_type,
	    sizeof(sc->phy_conn_type));

	s = OF_getencprop(node, "pcs-handle", &xref, sizeof(xref));
	sc->pcs_handle = s > 0 ? OF_node_from_xref(xref) : 0u;

	/* 'sfp' and 'phy-handle' are optional but we need one or the other. */
	s = OF_getencprop(node, "sfp", &xref, sizeof(xref));
	sc->sfp = s > 0 ? OF_node_from_xref(xref) : 0u;
	s = OF_getencprop(node, "phy-handle", &xref, sizeof(xref));
	sc->phy_handle = s > 0 ? OF_node_from_xref(xref) : 0u;

	if (bootverbose)
		device_printf(dev, "node %#x: reg %#x sfp %#x pcs-handle "
		    "%#x phy-handle %#x managed '%s' phy-conn-type '%s'\n",
		    node, sc->reg, sc->sfp, sc->pcs_handle, sc->phy_handle,
		    sc->managed, sc->phy_conn_type);

skip_fdt_attach:
	return (dpaa2_mac_attach(dev));
}

static device_method_t dpaa2_mac_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mac_fdt_probe),
	DEVMETHOD(device_attach,	dpaa2_mac_fdt_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(dpaa2_mac, dpaa2_mac_fdt_driver, dpaa2_mac_fdt_methods,
    sizeof(struct dpaa2_mac_fdt_softc), dpaa2_mac_driver);

DRIVER_MODULE(dpaa2_mac, dpaa2_rc, dpaa2_mac_fdt_driver, 0, 0);
MODULE_DEPEND(dpaa2_mac, memac_mdio_fdt, 1, 1, 1);
