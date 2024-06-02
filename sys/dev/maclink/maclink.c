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
 * It implements a loose coupling between the network interfaces and maclink
 * adapters hiding a complex set of devices (PCSs, SFP/SFF) which constitute a
 * connection between MAC and a physical media.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/sbuf.h>

#include "maclink_bus_if.h"

#include "maclink.h"

/* for device interface */

static int
maclink_bus_probe(device_t dev)
{
	device_set_desc(dev, "MACLINK bus");

	return (BUS_PROBE_SPECIFIC);
}

static int
maclink_bus_attach(device_t dev)
{
	return (0);
}

static int
maclink_bus_detach(device_t dev)
{
	return (0);
}

/* for maclink bus interface */

static int
maclink_bus_validate(device_t dev, struct maclink_conf *conf)
{
	device_t parent;

	parent = device_get_parent(dev);
	return (MACLINK_BUS_VALIDATE(parent, conf));
}

static void
maclink_bus_statchg(device_t dev, struct maclink_state *state)
{
	device_t parent;

	parent = device_get_parent(dev);
	return (MACLINK_BUS_STATCHG(parent, state));
}

static void
maclink_bus_linkchg(device_t dev, struct maclink_link_state *state)
{
	device_t parent;

	parent = device_get_parent(dev);
	return (MACLINK_BUS_LINKCHG(parent, state));
}

/* helper routines */

/**
 * @brief Helper function used to attach maclink bus to the network interface
 *        driver.
 *
 * @param dev Parent device (e.g. network interface) for the maclink bus.
 * @param bus The maclink bus device to attach.
 * @param args Used to probe and attach an adapter to the maclink bus.
 */
int
maclink_attach(device_t dev, device_t *bus, const struct maclink_ivars *args)
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

		*bus = device_add_child(dev, "maclink_bus", -1);
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

static device_method_t maclink_bus_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		maclink_bus_probe),
	DEVMETHOD(device_attach,	maclink_bus_attach),
	DEVMETHOD(device_detach,	maclink_bus_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	/* maclink bus interface */
	DEVMETHOD(maclink_bus_validate,	maclink_bus_validate),
	DEVMETHOD(maclink_bus_statchg,	maclink_bus_statchg),
	DEVMETHOD(maclink_bus_linkchg,	maclink_bus_linkchg),

	DEVMETHOD_END
};

DEFINE_CLASS_0(maclink_bus, maclink_bus_driver, maclink_bus_methods,
    sizeof(struct maclink_bus_softc));

MODULE_VERSION(maclink_bus, 1);
