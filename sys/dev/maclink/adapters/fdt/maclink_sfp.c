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
 * Maclink SFP adapter (FDT-based).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/sbuf.h>
#include <sys/kernel.h>

#include <dev/maclink/maclink.h>

#include "maclink_bus_if.h"
#include "maclink_dev_if.h"

struct maclink_sfp_softc {
	int placeholder;
};

/* for device interface */

static int
maclink_sfp_probe(device_t dev)
{
	device_set_desc(dev, "MACLINK SFP adapter");

	return (BUS_PROBE_DEFAULT);
}

static int
maclink_sfp_attach(device_t dev)
{
	/* XXX-DSL: to be done. */

	return (0);
}

static device_method_t maclink_sfp_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		maclink_sfp_probe),
	DEVMETHOD(device_attach,	maclink_sfp_attach),

	/* maclink bus interface */
	DEVMETHOD(maclink_bus_validate,	maclink_bus_validate),
	DEVMETHOD(maclink_bus_statchg,	maclink_bus_statchg),
	DEVMETHOD(maclink_bus_linkchg,	maclink_bus_linkchg),

	DEVMETHOD_END
};

static driver_t maclink_sfp_driver = {
	"maclink_sfp",
	maclink_sfp_methods,
	sizeof(struct maclink_sfp_softc),
};

DRIVER_MODULE(maclink_sfp, maclink_bus, maclink_sfp_driver, 0, 0);
