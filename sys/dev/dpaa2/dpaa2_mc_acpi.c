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
#include <dev/acpica/acpivar.h>

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

#include "pcib_if.h"
#include "acpi_bus_if.h"

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
	return (dpaa2_mc_attach(dev));
}

static int
dpaa2_mc_acpi_detach(device_t dev)
{
	return (dpaa2_mc_detach(dev));
}

static int
dpaa2_mc_acpi_alloc_msi(device_t mcdev, device_t child, int count,
    int maxcount, int *irqs)
{
	return (dpaa2_mc_alloc_msi(mcdev, child, count, maxcount, irqs));
}

static int
dpaa2_mc_acpi_release_msi(device_t mcdev, device_t child, int count, int *irqs)
{
	return (dpaa2_mc_release_msi(mcdev, child, count, irqs));
}

static int
dpaa2_mc_acpi_map_msi(device_t mcdev, device_t child, int irq, uint64_t *addr,
    uint32_t *data)
{
	return (dpaa2_mc_map_msi(mcdev, child, irq, addr, data));
}

static int
dpaa2_mc_acpi_get_id(device_t mcdev, device_t child, enum pci_id_type type,
    uintptr_t *id)
{
	return (dpaa2_mc_get_id(mcdev, child, type, id));
}

static device_method_t dpaa2_mc_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mc_acpi_probe),
	DEVMETHOD(device_attach,	dpaa2_mc_acpi_attach),
	DEVMETHOD(device_detach,	dpaa2_mc_acpi_detach),

	/* Pseudo-PCIB interface */
	DEVMETHOD(pcib_alloc_msi,	dpaa2_mc_acpi_alloc_msi),
	DEVMETHOD(pcib_release_msi,	dpaa2_mc_acpi_release_msi),
	DEVMETHOD(pcib_map_msi,		dpaa2_mc_acpi_map_msi),
	DEVMETHOD(pcib_get_id,		dpaa2_mc_acpi_get_id),

	DEVMETHOD_END
};

DEFINE_CLASS_1(dpaa2_mc, dpaa2_mc_acpi_driver, dpaa2_mc_acpi_methods,
    sizeof(struct dpaa2_mc_softc), dpaa2_mc_driver);

static devclass_t dpaa2_mc_acpi_devclass;

DRIVER_MODULE(dpaa2_mc, acpi, dpaa2_mc_acpi_driver, dpaa2_mc_acpi_devclass,
    0, 0);
