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
 * The DPAA2 MAC driver.
 *
 * For every DPAA2 MAC, there is an MC object named DPMAC, for MDIO and link
 * state updates. The DPMAC virtualizes the MDIO interface, so each PHY driver
 * may see a private interface (removing the need for synchronization in GPP on
 * the multiplexed MDIO hardware).
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

#include <dev/pci/pcivar.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"
#include "dpaa2_mc.h"

/*
 * Device interface.
 */

static int
dpaa2_mac_probe(device_t dev)
{
	/* DPIO device will be added by a parent resource container itself. */
	device_set_desc(dev, "DPAA2 MAC");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_mac_attach(device_t dev)
{
	struct dpaa2_mac_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	return (0);
}

static int
dpaa2_mac_detach(device_t dev)
{
	return (0);
}

static device_method_t dpaa2_mac_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mac_probe),
	DEVMETHOD(device_attach,	dpaa2_mac_attach),
	DEVMETHOD(device_detach,	dpaa2_mac_detach),

	DEVMETHOD_END
};

static driver_t dpaa2_mac_driver = {
	"dpaa2_mac",
	dpaa2_mac_methods,
	sizeof(struct dpaa2_mac_softc),
};

static devclass_t dpaa2_mac_devclass;

DRIVER_MODULE(dpaa2_mac, dpaa2_rc, dpaa2_mac_driver, dpaa2_mac_devclass, 0, 0);
