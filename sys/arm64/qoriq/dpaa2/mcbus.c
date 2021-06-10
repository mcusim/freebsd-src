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
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/module.h>

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
#define	mcbus_reg_r4(_sc, _r)		bus_read_4((_sc)->res[1], (_r))
#define	mcbus_reg_w4(_sc, _r, _v)	bus_write_4((_sc)->res[1], (_r), (_v))

static struct resource_spec mcbus_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },		/* MC portal */
	{ SYS_RES_MEMORY, 1, RF_ACTIVE | RF_OPTIONAL },	/* MC control regs */
	RESOURCE_SPEC_END
};

/*
 * Device interface.
 */
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
	uint32_t val;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);

	error = bus_alloc_resources(dev, mcbus_spec, sc->res);
	if (error) {
		device_printf(dev, "failed to allocate resources\n");
		return (ENXIO);
	}

	/* Reset P1_STOP bit to resume MC processor. */
	if (sc->res[1]) {
		val = mcbus_reg_r4(sc, MC_REG_GCR1) & (~GCR1_P1_STOP);
		mcbus_reg_w4(sc, MC_REG_GCR1, val);
	}

	/*
	 * Add a root resource container as the only child of the bus. All of
	 * the direct descendant containers will be attached to the root one
	 * instead of the MC bus device.
	 */
	sc->rcdev = device_add_child(dev, "dprc", 0);
	if (sc->rcdev == NULL) {
		mcbus_detach(dev);
		return (ENXIO);
	}
	bus_generic_probe(dev);
	bus_generic_attach(dev);

	return (0);
}

static int
mcbus_detach(device_t dev)
{
	struct mcbus_softc *sc;

	sc = device_get_softc(dev);
	bus_release_resources(dev, mcbus_spec, sc->res);

	return (0);
}

static device_method_t mcbus_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mcbus_probe),
	DEVMETHOD(device_attach,	mcbus_attach),
	DEVMETHOD(device_detach,	mcbus_detach),
	DEVMETHOD_END
};

static driver_t mcbus_driver = {
	"mcbus",
	mcbus_methods,
	sizeof(struct mcbus_softc),
};

static devclass_t mcbus_devclass;

DRIVER_MODULE(mcbus, simplebus, mcbus_driver, mcbus_devclass, 0, 0);
