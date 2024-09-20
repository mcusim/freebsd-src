/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
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
 * The maclink layer.
 *
 * It implements a loose coupling between a network interface and a complex set
 * of devices (PCS, SFP/SFF, PHY) which constitute a connection between MAC
 * and a physical media.
 *
 * Topology example:
 *
 *	netif0                     -> network interface
 *	  maclink_bus0             -> maclink bus
 *	    maclink_phy0     --|   -> maclink adapter
 *	...                    |
 *	pcs0                 --|   \
 *	...                    |   |
 *	maclink_bus1         --|   -> discoverable via FDT/ACPI
 *	  phy0                 |   |
 *	...                    |   |
 *	sfp0                 --|   /
 *
 * "maclink_phy0" adapter is expected to discover "pcs0", "maclink_bus1" and
 * "sfp0" and register itself for each of the interesting devices.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/sbuf.h>

#include "maclink.h"

#include "maclink_bus_if.h"
#include "maclink_dev_if.h"

MODULE_VERSION(maclink, 1);

/**
 * @brief Helper function to attach a maclink bus.
 *
 * @param dev[in] Parent device (e.g. network interface) for the maclink bus.
 * @param mlbus[in,out] The attached maclink bus.
 */
int
maclink_attach(device_t dev, device_t *mlbus)
{
	device_t bus, adapter, *adapters;
	struct maclink_devinfo *businfo, *adpinfo;
	int rc, nadapters;
	bool allocated;

	bus = NULL;
	businfo = NULL;
	adapter = NULL;
	adapters = NULL;
	rc = ENXIO;
	nadapters = 0;
	allocated = false;

	if (mlbus == NULL)
		goto err;

	/* Attach a new maclink bus if it wasn't done already. */
	if (*mlbus == NULL) {
		businfo = malloc(sizeof(*businfo), M_DEVBUF, M_NOWAIT);
		if (businfo == NULL) {
			rc = ENOMEM;
			goto err;
		}
		allocated = true;
		bus = device_add_child(dev, "maclink_bus", DEVICE_UNIT_ANY);
		if (bus == NULL) {
			rc = ENXIO;
			goto err_businfo;
		}
		device_set_ivars(bus, businfo);
	} else {
		bus = *mlbus;
		businfo = device_get_ivars(bus);
		if (businfo == NULL) {
			rc = ENXIO;
			goto err;
		}
	}

	/*
	 * Make sure there is no adapter connected to the bus already and
	 * attach one if so.
	 */
	rc = device_get_children(bus, &adapters, &nadapters);
	if (rc == 0) {
		free(adapters, M_TEMP);
		if (nadapters == 0) {
			adpinfo = malloc(sizeof(*adpinfo), M_DEVBUF, M_NOWAIT);
			if (adpinfo == NULL)
				goto err_allocated;
			adapter = device_add_child(bus, NULL, DEVICE_UNIT_ANY);
			if (adapter == NULL)
				goto err_adpinfo;
			device_set_ivars(adapter, adpinfo);
		} else {
			/* There's an adapter already. */
			rc = ENXIO;
			goto err_allocated;
		}
	} else
		goto err_allocated;

	/* XXX-DSL: see mii_attach() for reference. */

	return (0);

err_adpinfo:
	free(adpinfo, M_DEVBUF);
err_allocated:
	if (allocated)
		device_delete_child(dev, bus);
err_businfo:
	if (allocated)
		free(businfo, M_DEVBUF);
err:
	return (rc);
}

static int
maclink_bus_probe(device_t dev)
{
	device_set_desc(dev, "MACLINK bus");

	return (BUS_PROBE_SPECIFIC);
}

static int
maclink_bus_attach(device_t dev)
{
	/* XXX-DSL: to be done. */

	return (bus_generic_attach(dev));
}

int
maclink_bus_validate(device_t bus, struct maclink_conf *conf)
{
	if (device_get_parent(bus) != NULL)
		return (MACLINK_BUS_VALIDATE(device_get_parent(bus), conf));
	return (ENXIO);
}

void
maclink_bus_statchg(device_t bus, struct maclink_state *state)
{
	if (device_get_parent(bus) != NULL)
		MACLINK_BUS_STATCHG(device_get_parent(bus), state);
}

void
maclink_bus_linkchg(device_t bus, struct maclink_link_state *state)
{
	if (device_get_parent(bus) != NULL)
		MACLINK_BUS_LINKCHG(device_get_parent(bus), state);
}

static device_method_t maclink_bus_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		maclink_bus_probe),
	DEVMETHOD(device_attach,	maclink_bus_attach),

	/* maclink bus interface */
	DEVMETHOD(maclink_bus_validate,	maclink_bus_validate),
	DEVMETHOD(maclink_bus_statchg,	maclink_bus_statchg),
	DEVMETHOD(maclink_bus_linkchg,	maclink_bus_linkchg),

	DEVMETHOD_END
};

DEFINE_CLASS_0(maclink_bus, maclink_bus_driver, maclink_bus_methods,
    sizeof(struct maclink_data));
