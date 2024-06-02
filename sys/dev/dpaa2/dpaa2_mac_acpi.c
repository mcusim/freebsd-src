/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright © 2022 Bjoern A. Zeeb
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
 * The DPAA2 MAC driver (with the ACPI-based configuration).
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/kobj.h>
#include <sys/systm.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include "acpi_bus_if.h"

#include "dpaa2_mac.h"

/**
 * @struct dpaa2_mac_acpi_softc
 * @brief Software context for DPMAC device with the ACPI-based configuration.
 */
struct dpaa2_mac_acpi_softc {
	struct dpaa2_mac_softc	scc; /* Must stay first. */
	int			uid;
	uint64_t		reg;
	char			managed[64];
	char			phy_conn_type[64];
	char			phy_mode[64];
	ACPI_HANDLE		phy_channel;
};
CTASSERT(sizeof(struct dpaa2_mac_acpi_softc) >= sizeof(struct dpaa2_mac_softc));

/*
 * Device interface
 */

static int
dpaa2_mac_acpi_probe(device_t dev)
{
	struct dpaa2_macinfo *macinfo;
	struct acpi_device *ad; /* to call device_get_property only */
	uint64_t reg;
	ssize_t s;

	macinfo = device_get_ivars(dev);
	if (!macinfo->valid)
		goto skip_acpi_probe;

	if ((ad = malloc(sizeof(*ad), M_DEVBUF, M_NOWAIT | M_ZERO)) == NULL)
		return (ENOMEM);

	ad->ad_handle = macinfo->handle;
	ad->ad_cls_class = 0xffffff;
	/* resource_list_init(&ad->ad_rl); */
	device_set_ivars(dev, ad); /* NOTE: temporarily! */

	/* DPMAC number supplied via "reg" property is considered mandatory. */
	s = device_get_property(dev, "reg", &reg, sizeof(reg),
	    DEVICE_PROP_UINT64);
	device_set_ivars(dev, macinfo);
	free(ad, M_DEVBUF);
	if (s == -1) {
		return (ENXIO);
	}

skip_acpi_probe:
	return (dpaa2_mac_probe(dev));
}

static int
dpaa2_mac_acpi_attach(device_t dev)
{
	struct dpaa2_mac_acpi_softc *sc;
	struct dpaa2_macinfo *macinfo;
	struct acpi_device *ad; /* to call device_get_property only */
	ACPI_HANDLE h;
	ssize_t s;

	macinfo = device_get_ivars(dev);
	if (!macinfo->valid) {
		if (bootverbose)
			device_printf(dev, "%s: no ACPI handle\n", __func__);
		goto skip_acpi_attach;
	}

	if ((ad = malloc(sizeof(*ad), M_DEVBUF, M_NOWAIT | M_ZERO)) == NULL)
		return (ENOMEM);

	ad->ad_handle = macinfo->handle;
	ad->ad_cls_class = 0xffffff;
	/* resource_list_init(&ad->ad_rl); */
	device_set_ivars(dev, ad); /* NOTE: temporarily! */

	sc = device_get_softc(dev);
	h = macinfo->handle;
	if (h == NULL) {
		device_set_ivars(dev, macinfo);
		free(ad, M_DEVBUF);
		return (ENXIO);
	}

	s = acpi_GetInteger(h, "_UID", &sc->uid);
	if (ACPI_FAILURE(s)) {
		device_printf(dev, "Cannot find '_UID' property: %zd\n", s);
		device_set_ivars(dev, macinfo);
		free(ad, M_DEVBUF);
		return (ENXIO);
	}

	/* DPMAC number supplied via "reg" property is considered mandatory. */
	s = device_get_property(dev, "reg", &sc->reg, sizeof(sc->reg),
	    DEVICE_PROP_UINT64);
	if (s == -1) {
		device_printf(dev, "Cannot find 'reg' property: %zd\n", s);
		device_set_ivars(dev, macinfo);
		free(ad, M_DEVBUF);
		return (ENXIO);
	}

	s = device_get_property(dev, "managed", sc->managed,
	    sizeof(sc->managed), DEVICE_PROP_ANY);
	s = device_get_property(dev, "phy-connection-type", sc->phy_conn_type,
	    sizeof(sc->phy_conn_type), DEVICE_PROP_ANY);
	s = device_get_property(dev, "phy-mode", sc->phy_mode,
	    sizeof(sc->phy_mode), DEVICE_PROP_ANY);
	s = device_get_property(dev, "phy-handle", &sc->phy_channel,
	    sizeof(sc->phy_channel), DEVICE_PROP_HANDLE);

	if (bootverbose)
		device_printf(dev, "UID %#04x reg %#04jx managed '%s' "
		    "phy-connection-type '%s' phy-mode '%s' phy-handle '%s'\n",
		    sc->uid, sc->reg, sc->managed[0] != '\0' ? sc->managed : "",
		    sc->phy_conn_type[0] != '\0' ? sc->phy_conn_type : "",
		    sc->phy_mode[0] != '\0' ? sc->phy_mode : "",
		    sc->phy_channel != NULL ? acpi_name(sc->phy_channel) : "");

	device_set_ivars(dev, macinfo);
	free(ad, M_DEVBUF);

skip_acpi_attach:
	return (dpaa2_mac_attach(dev));
}

static device_method_t dpaa2_mac_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_mac_acpi_probe),
	DEVMETHOD(device_attach,	dpaa2_mac_acpi_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(dpaa2_mac, dpaa2_mac_acpi_driver, dpaa2_mac_acpi_methods,
    sizeof(struct dpaa2_mac_acpi_softc), dpaa2_mac_driver);

DRIVER_MODULE(dpaa2_mac, dpaa2_rc, dpaa2_mac_acpi_driver, 0, 0);
MODULE_DEPEND(dpaa2_mac, memac_mdio_fdt, 1, 1, 1);
