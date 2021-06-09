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
 * The NXP DPAA2 Management Complex (MC) Driver.
 *
 * MC is a hardware resource manager which can be found in several NXP
 * SoCs (LX2160A, for example) and provides an access to the specialized
 * hardware objects used in network-oriented packet processing applications.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "mcbus.h"

/* Device interface */
static int mcbus_probe(device_t dev);
static int mcbus_attach(device_t dev);
static int mcbus_detach(device_t dev);

/* Macros to read/write registers */
#define	mcbus_reg_r4(_sc, _r)		bus_read_4((_sc)->regs_res, (_r))
#define	mcbus_reg_w4(_sc, _r, _v)	bus_write_4((_sc)->regs_res, (_r), (_v))

/* Device interface */

static int
mcbus_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "fsl,qoriq-mc"))
		return (ENXIO);

	device_set_desc(dev, "NXP DPAA2 Management Complex");
	return (BUS_PROBE_DEFAULT);
}

static int
mcbus_attach(device_t dev)
{
	struct mcbus_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);

	/* Allocate memory resource for control registers */
	sc->regs_rid = MC_REGS_MEM_RID;
	sc->regs_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->regs_rid, RF_ACTIVE);
	if (sc->regs_res) {
		/* ...clear P1_STOP bit to run MC processor... */
	}

	/* Allocate memory resource for command portal */
	sc->portal_rid = MC_PORTAL_MEM_RID;
	sc->portal_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->portal_rid, RF_ACTIVE);
	if (sc->portal_res == NULL) {
		device_printf(sc->dev, "failed to allocate memory resource "
		    "for command portal\n");
		mcbus_detach(dev);
		return (ENXIO);
	}

	/*
	 * Don't forget to add root DPRC as a child device:
	 *
	 *	device_add_child(dev, "dprc", 0);
	 *
	 */

	/* bus_generic_probe(dev); */
	/* bus_generic_attach(dev); */

	return (0);
}

static int
mcbus_detach(device_t dev)
{
	return (0);
}

static device_method_t mcbus_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mcbus_probe),
	DEVMETHOD(device_attach,	mcbus_attach),
	DEVMETHOD(device_detach,	mcbus_detach),
	/* DEVMETHOD(device_shutdown,	mcbus_shutdown), */
	/* DEVMETHOD(device_suspend,	mcbus_suspend), */
	/* DEVMETHOD(device_resume,	mcbus_resume), */
	DEVMETHOD_END
};

static driver_t mcbus_driver = {
	"mcbus",
	mcbus_methods,
	sizeof(struct mcbus_softc),
};

static devclass_t mcbus_devclass;

DRIVER_MODULE(mcbus, simplebus, mcbus_driver, mcbus_devclass, 0, 0);
