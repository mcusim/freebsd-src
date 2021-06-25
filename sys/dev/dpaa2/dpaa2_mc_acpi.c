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

#include "dpaa2_mcvar.h"

static int
dpaa2_mc_acpi_probe(device_t dev)
{
	static char *dpaa2_mc_ids[] = { "NXP0008", NULL };
	int rc;

	/* --- FOR DEBUG ONLY --- */
	ACPI_DEVICE_INFO *pdev_info = NULL;
	ACPI_DEVICE_INFO *dev_info = NULL;
	ACPI_HANDLE pdev_h, dev_h;
	char cbuf[128];

	device_printf(dev, "Probed from ACPI probe\n");

	pdev_h = acpi_get_handle(device_get_parent(dev));
	dev_h = acpi_get_handle(dev);

	if (pdev_h)
		AcpiGetObjectInfo(pdev_h, &pdev_info);
	if (dev_h)
		AcpiGetObjectInfo(dev_h, &dev_info);

	if (pdev_h && pdev_info) {
		snprintf(cbuf, sizeof(cbuf), "%s:%02lX",
		    (pdev_info->Valid & ACPI_VALID_HID) ?
		    pdev_info->HardwareId.String : "Unknown",
		    (pdev_info->Valid & ACPI_VALID_UID) ?
		    strtoul(pdev_info->UniqueId.String, NULL, 10) : 0UL);
		device_printf(dev, "Parent: %s\n", cbuf);
		AcpiOsFree(pdev_info);
	}

	if (dev_h && dev_info) {
		snprintf(cbuf, sizeof(cbuf), "%s:%02lX",
		    (dev_info->Valid & ACPI_VALID_HID) ?
		    dev_info->HardwareId.String : "Unknown",
		    (dev_info->Valid & ACPI_VALID_UID) ?
		    strtoul(dev_info->UniqueId.String, NULL, 10) : 0UL);
		device_printf(dev, "Device: %s\n", cbuf);
		AcpiOsFree(dev_info);
	}
	/* --- FOR DEBUG ONLY --- */

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

static device_method_t dpaa2_mc_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mc_acpi_probe),
	DEVMETHOD(device_attach,	dpaa2_mc_acpi_attach),
	DEVMETHOD(device_detach,	dpaa2_mc_acpi_detach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(dpaa2_mc, dpaa2_mc_acpi_driver, dpaa2_mc_acpi_methods,
    sizeof(struct dpaa2_mc_softc), dpaa2_mc_driver);

static devclass_t dpaa2_mc_acpi_devclass;

DRIVER_MODULE(dpaa2_mc, acpi, dpaa2_mc_acpi_driver, dpaa2_mc_acpi_devclass,
    0, 0);
