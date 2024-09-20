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
 * It implements a loose coupling between a network interface and a maclink
 * adapter hiding a complex set of devices (PCSs, SFP/SFF) which constitute a
 * connection between MAC and a physical media.
 *
 * Topology example:
 *
 *                                            /-- maclink_device0 (PCS)
 * IF/MAC -- maclink_bus -- maclink_adapter ----- maclink_device1 (bus) -- (PHY)
 *                                            \-- maclink_deviceN (SFP)
 *
 * The maclink adapter is expected to discover devices (PCS, PHY, SFP, etc.)
 * which constitute a connection between a network interface (or MAC) the
 * adapter is connected to and a physical media. Adapter should register
 * each of the interesting devices.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/sbuf.h>

#include "maclink_if.h"
#include "maclink.h"

/**
 * @brief Attaches the maclink bus to the network interface or MAC.
 *
 * @param dev[in] Parent device (e.g. network interface) for the maclink bus.
 * @param mlbus[out] The attached maclink bus.
 */
int
maclink_attach(device_t dev, device_t *mlbus)
{
	struct maclink_bus_ivars *ivars;
	struct maclink_ivars *adp_ivars;
	device_t *adapter;
	int first, rc, nadapters;

	first = 0;
	if (*bus == NULL) {
		first = 1;
		ivars = malloc(sizeof(*ivars), M_DEVBUF, M_NOWAIT);
		if (ivars == NULL)
			return (ENOMEM);

		*bus = device_add_child(dev, "maclink_bus", DEVICE_UNIT_ANY);
		if (*bus == NULL) {
			rc = ENXIO;
			goto fail;
		}
		device_set_ivars(*bus, ivars);
	} else {
		ivars = device_get_ivars(*maclink_bus);
	}

	if (device_get_children(*maclink_bus, &adapter, &nadapters) != 0) {
		adapter = NULL;
		nadapters = 0;
	}

	/* Make sure we don't have an adapter connected to the bus already. */
	if (nadapters == 0) {
	} else {
		/* There's an adapter, leaving. */
		rc = ENXIO;
		goto fail;
	}

	/*
	 * Make sure we haven't already configured a PHY at this
	 * address.  This allows mii_attach() to be called
	 * multiple times.
	 */
	for (i = 0; i < nchildren; i++) {
		args = device_get_ivars(children[i]);
		if (args->mii_phyno == ma.mii_phyno) {
			/*
			 * Yes, there is already something
			 * configured at this address.
			 */
			goto skip;
		}
	}

	/*
	 * Check to see if there is a PHY at this address.  Note,
	 * many braindead PHYs report 0/0 in their ID registers,
	 * so we test for media in the BMSR.
	 */
	bmsr = MIIBUS_READREG(dev, ma.mii_phyno, MII_BMSR);
	if (bmsr == 0 || bmsr == 0xffff ||
	    (bmsr & (BMSR_EXTSTAT | BMSR_MEDIAMASK)) == 0) {
		/* Assume no PHY at this address. */
		continue;
	}

	/*
	 * There is a PHY at this address.  If we were given an
	 * `offset' locator, skip this PHY if it doesn't match.
	 */
	if (offloc != MII_OFFSET_ANY && offloc != ivars->mii_offset)
		goto skip;

	/*
	 * Skip this PHY if it's not included in the phymask hint.
	 */
	if ((phymask & (1 << ma.mii_phyno)) == 0)
		goto skip;

	/*
	 * Extract the IDs.  Braindead PHYs will be handled by
	 * the `ukphy' driver, as we have no ID information to
	 * match on.
	 */
	ma.mii_id1 = MIIBUS_READREG(dev, ma.mii_phyno, MII_PHYIDR1);
	ma.mii_id2 = MIIBUS_READREG(dev, ma.mii_phyno, MII_PHYIDR2);

	ma.mii_offset = ivars->mii_offset;
	args = malloc(sizeof(struct mii_attach_args), M_DEVBUF,
	    M_NOWAIT);
	if (args == NULL)
		goto skip;
	bcopy((char *)&ma, (char *)args, sizeof(ma));
	phy = device_add_child(*miibus, NULL, -1);
	if (phy == NULL) {
		free(args, M_DEVBUF);
		goto skip;
	}
	device_set_ivars(phy, args);
 skip:
	ivars->mii_offset++;

	free(children, M_TEMP);

	if (first != 0) {
		rv = device_set_driver(*miibus, &miibus_driver);
		if (rv != 0)
			goto fail;
		bus_enumerate_hinted_children(*miibus);
		rv = device_get_children(*miibus, &children, &nchildren);
		if (rv != 0)
			goto fail;
		free(children, M_TEMP);
		if (nchildren == 0) {
			rv = ENXIO;
			goto fail;
		}
		rv = bus_generic_attach(dev);
		if (rv != 0)
			goto fail;

		/* Attaching of the PHY drivers is done in miibus_attach(). */
		return (0);
	}
	rv = bus_generic_attach(*miibus);
	if (rv != 0)
		goto fail;

	return (0);

fail:
	if (*maclink_bus != NULL)
		device_delete_child(dev, *maclink_bus);
	free(ivars, M_DEVBUF);
	if (first != 0)
		*maclink_bus = NULL;
	return (rv);
}

int
maclink_register(device_t mlbus, device_t mldev)
{
	return (0);
}

static device_method_t maclink_bus_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		maclink_bus_probe),
	DEVMETHOD(device_attach,	maclink_bus_attach),
	DEVMETHOD(device_detach,	maclink_bus_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	/* bus interface */
	DEVMETHOD(bus_print_child,	maclink_bus_print_child),
	DEVMETHOD(bus_read_ivar,	maclink_bus_read_ivar),
	DEVMETHOD(bus_child_detached,	maclink_bus_child_detached),
	DEVMETHOD(bus_child_pnpinfo,	maclink_bus_child_pnpinfo),
	DEVMETHOD(bus_child_location,	maclink_bus_child_location),
	DEVMETHOD(bus_hinted_child,	maclink_bus_hinted_child),

	/* maclink interface */
	DEVMETHOD(maclink_register,	maclink_bus_register),
	DEVMETHOD(maclink_get_data,	maclink_bus_get_data),
	DEVMETHOD(maclink_validate,	maclink_bus_validate),
	DEVMETHOD(maclink_statchg,	maclink_bus_statchg),
	DEVMETHOD(maclink_linkchg,	maclink_bus_linkchg),

	DEVMETHOD_END
};

DEFINE_CLASS_0(maclink_bus, maclink_bus_driver, maclink_bus_methods,
    sizeof(struct maclink_data));
