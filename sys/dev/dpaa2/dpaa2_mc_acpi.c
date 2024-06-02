/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright © 2021-2024 Dmitry Salychev
 * Copyright © 2021 Bjoern A. Zeeb
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
 * The DPAA2 Management Complex (MC) Bus Driver (ACPI-based).
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

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>

#include "acpi_bus_if.h"
#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"
#include "dpaa2_mc_if.h"

#define	_COMPONENT	ACPI_BUS
ACPI_MODULE_NAME("DPAA2_MC")

#define	ACPI_SCAN_DEPTH		2

/* Context for walking PRxx child devices. */
struct dpaa2_mc_acpi_prxx_walk_ctx {
	device_t	dev;
	int		count;
	int		countok;
};

static ACPI_STATUS dpaa2_mc_acpi_probe_child(ACPI_HANDLE h, device_t *dev,
    int level, void *arg);

/*
 * Device interface
 */

static int
dpaa2_mc_acpi_probe(device_t dev)
{
	static char *dpaa2_mc_ids[] = { "NXP0008", NULL };
	int rc;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	rc = ACPI_ID_PROBE(device_get_parent(dev), dev, dpaa2_mc_ids, NULL);
	if (rc <= 0)
		device_set_desc(dev, "DPAA2 Management Complex");

	return (rc);
}

static int
dpaa2_mc_acpi_attach(device_t dev)
{
	struct dpaa2_mc_softc *sc;

	sc = device_get_softc(dev);
	sc->acpi_based = true;

	return (dpaa2_mc_attach(dev));
}

/*
 * DPAA2 MC bus interface
 */

static int
dpaa2_mc_acpi_manage_dev(device_t mcdev, device_t dev, uint32_t flags)
{
	struct dpaa2_devinfo *dinfo;
	struct dpaa2_mc_acpi_prxx_walk_ctx ctx;

	dinfo = device_get_ivars(dev);

	switch (dinfo->dtype) {
	case DPAA2_DEV_MAC:
		break;
	default:
		goto skip_acpi_manage_dev;
	}

	ctx.dev = dev;
	ctx.count = 0;
	ctx.countok = 0;
	ACPI_SCAN_CHILDREN(device_get_parent(mcdev), mcdev, ACPI_SCAN_DEPTH,
	    dpaa2_mc_acpi_probe_child, &ctx);

skip_acpi_manage_dev:
	return (dpaa2_mc_manage_dev(mcdev, dev, flags));
}

static ACPI_STATUS
dpaa2_mc_acpi_probe_child(ACPI_HANDLE h, device_t *mcdev, int level, void *arg)
{
	ACPI_STATUS status;
	const ACPI_OBJECT *obj;
	struct dpaa2_devinfo *dinfo;
	struct dpaa2_macinfo *macinfo;
	struct dpaa2_mc_acpi_prxx_walk_ctx *ctx;
	uint32_t uid;

	ctx = (struct dpaa2_mc_acpi_prxx_walk_ctx *)arg;
	ctx->count++;

	dinfo = device_get_ivars(ctx->dev);
	macinfo = (struct dpaa2_macinfo *)dinfo;
	macinfo->handle = 0;
	macinfo->valid = false;

	if (ACPI_FAILURE(acpi_GetInteger(h, "_UID", &uid)))
		return (AE_OK);

	status = acpi_GetProperty(*mcdev, __DECONST(char *, "reg"), &obj);
	if (ACPI_FAILURE(status))
		return (AE_OK);

	switch (obj->Type) {
	case ACPI_TYPE_INTEGER:
		if (obj->Integer.Value == dinfo->id) {
			macinfo->handle = h;
			macinfo->valid = true;
		}
		break;
	default:
		break;
	}

	ctx->countok++;
	return (AE_OK);
}

static device_method_t dpaa2_mc_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mc_acpi_probe),
	DEVMETHOD(device_attach,	dpaa2_mc_acpi_attach),
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
	DEVMETHOD(dpaa2_mc_manage_dev,	dpaa2_mc_acpi_manage_dev),
	DEVMETHOD(dpaa2_mc_get_free_dev,dpaa2_mc_get_free_dev),
	DEVMETHOD(dpaa2_mc_get_dev,	dpaa2_mc_get_dev),
	DEVMETHOD(dpaa2_mc_get_shared_dev, dpaa2_mc_get_shared_dev),
	DEVMETHOD(dpaa2_mc_reserve_dev,	dpaa2_mc_reserve_dev),
	DEVMETHOD(dpaa2_mc_release_dev, dpaa2_mc_release_dev),

	DEVMETHOD_END
};

DEFINE_CLASS_1(dpaa2_mc, dpaa2_mc_acpi_driver, dpaa2_mc_acpi_methods,
    sizeof(struct dpaa2_mc_softc), dpaa2_mc_driver);

/* Make sure miibus gets procesed first. */
DRIVER_MODULE_ORDERED(dpaa2_mc, acpi, dpaa2_mc_acpi_driver, NULL, NULL,
    SI_ORDER_ANY);
MODULE_DEPEND(dpaa2_mc, memac_mdio_acpi, 1, 1, 1);
