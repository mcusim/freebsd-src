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
 * QBMan command interface and the DPAA2 I/O (DPIO) driver.
 *
 * The DPIO object allows configuration of the QBMan software portal with
 * optional notification capabilities.
 *
 * Software portals are used by the driver to communicate with the QBMan. The
 * DPIO object’s main purpose is to enable the driver to perform I/O – enqueue
 * and dequeue operations, as well as buffer release and acquire operations –
 * using QBMan.
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

/*
 * Interrupts:
 *	0: MSI, should be allocated separately.
 * Memory:
 *	0: cache-enabled part of the QBMan software portal.
 *	1: cache-inhibited part of the QBMan software portal.
 *	2: control registers of the QBMan software portal?
 */
static struct resource_spec dpaa2_io_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_MEMORY, 1, RF_ACTIVE | RF_UNMAPPED },
	{ SYS_RES_MEMORY, 2, RF_ACTIVE | RF_UNMAPPED | RF_OPTIONAL },
	RESOURCE_SPEC_END
};

/* Forward declarations. */
static int dpaa2_io_setup_msi(struct dpaa2_io_softc *sc);
static void dpaa2_io_msi_intr(void *arg);

/*
 * Device interface.
 */

static int
dpaa2_io_probe(device_t dev)
{
	/* DPIO device will be added by a parent resource container itself. */
	device_set_desc(dev, "DPAA2 I/O");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_io_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_rc_softc *rcsc;
	struct dpaa2_io_softc *sc;
	struct resource_map_request req;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	pdev = device_get_parent(dev);
	rcsc = device_get_softc(pdev);
	rcinfo = device_get_ivars(pdev);

	error = bus_alloc_resources(sc->dev, dpaa2_io_spec, sc->res);
	if (error) {
		device_printf(dev, "Failed to allocate resources: error=%d\n",
		    error);
		return (ENXIO);
	}

	/* An attempt to map a region with control registers. */
	if (sc->res[2]) {
		resource_init_map_request(&req);
		req.memattr = VM_MEMATTR_DEVICE;
		error = bus_map_resource(sc->dev, SYS_RES_MEMORY, sc->res[2],
		    &req, &sc->map[2]);
		if (error) {
			device_printf(dev, "Failed to map control registers: "
			    "error=%d\n", error);
			dpaa2_mc_detach(dev);
			return (ENXIO);
		}
	}

	/* Map cache-inhibited part of the software portal memory. */
	resource_init_map_request(&req);
	req.memattr = VM_MEMATTR_DEVICE;
	error = bus_map_resource(sc->dev, SYS_RES_MEMORY, sc->res[0],
	    &req, &sc->map[0]);
	if (error) {
		device_printf(dev, "Failed to map cache-inhibited part of the "
		    "software portal memory: error=%d\n", error);
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}

	error = dpaa2_io_setup_msi(sc);
	if (error) {
		device_printf(dev, "Failed to allocate MSI: error=%d\n",
		    error);
		dpaa2_mc_detach(dev);
		return (ENXIO);
	}
	if ((sc->irq_resource = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ,
	    &sc->irq_rid[0], RF_ACTIVE | RF_SHAREABLE)) == NULL) {
		device_printf(sc->dev, "Failed to allocate interrupt\n");
		return (ENXIO);
	}
	if (bus_setup_intr(sc->dev, sc->irq_resource, INTR_TYPE_CAM|INTR_MPSAFE,
	    NULL, dpaa2_io_msi_intr, sc, &sc->intr)) {
		device_printf(sc->dev, "Failed to setup interrupt\n");
		return (ENXIO);
	}

	return (0);
}

static int
dpaa2_io_detach(device_t dev)
{
	return (0);
}

/**
 * @internal
 * @brief Allocate MSI interrupts for this DPAA2 I/O object.
 */
static int
dpaa2_io_setup_msi(struct dpaa2_io_softc *sc)
{
    int val;

    val = pci_msi_count(sc->dev);
    if (val < DPAA2_IO_MSI_COUNT)
	    device_printf(sc->dev, "got %d MSI messages\n", val);
    val = MIN(val, DPAA2_IO_MSI_COUNT);
    if (pci_alloc_msi(sc->dev, &val) != 0)
	    return (EINVAL);

    if (bootverbose)
	    device_printf(sc->dev, "Using %d MSI interrupt%s\n", val,
		(val != 1) ? "s" : "");

    for (int i = 0; i < val; i++)
	sc->irq_rid[i] = i + 1;

    return (0);
}

/**
 * @internal
 * DPAA2 I/O interrupt handler.
 */
static void
dpaa2_io_msi_intr(void *arg)
{
	/* NOTE: Useless interrupt handler. */
	volatile uint32 val = 0;
	for (uint32_t i = 0; i < 100; i++)
		val++;
}

static device_method_t dpaa2_io_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_io_probe),
	DEVMETHOD(device_attach,	dpaa2_io_attach),
	DEVMETHOD(device_detach,	dpaa2_io_detach),

	DEVMETHOD_END
};

static driver_t dpaa2_io_driver = {
	"dpaa2_io",
	dpaa2_io_methods,
	sizeof(struct dpaa2_io_softc),
};

static devclass_t dpaa2_io_devclass;

DRIVER_MODULE(dpaa2_io, dpaa2_rc, dpaa2_io_driver, dpaa2_io_devclass, 0, 0);
