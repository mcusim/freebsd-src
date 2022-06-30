/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021-2022 Bjoern A. Zeeb
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

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/endian.h>
#include <sys/socket.h>

#include <machine/bus.h>
#include <machine/resource.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/simplebus.h>
#endif

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "memac_mdio_if.h"
#ifdef DEV_ACPI
#include "acpi_bus_if.h"
#endif
#ifdef FDT
#include "ofw_bus_if.h"
#endif
#include "miibus_if.h"

/* -------------------------------------------------------------------------- */

struct memacphy_softc {
	int			phy;
#ifdef FDT
	uint32_t		reg;
	phandle_t		xref;
#endif
#ifdef DEV_ACPI
	int			uid;
	uint64_t		phy_channel;
	char			compatible[64];
#endif
	device_t		dpnidev;
};

static int
memacphy_miibus_readreg(device_t dev, int phy, int reg)
{

	return (MIIBUS_READREG(device_get_parent(dev), phy, reg));
}

static int
memacphy_miibus_writereg(device_t dev, int phy, int reg, int data)
{

	return (MIIBUS_WRITEREG(device_get_parent(dev), phy, reg, data));
}

static void
memacphy_miibus_statchg(device_t dev)
{
	struct memacphy_softc *sc;

	sc = device_get_softc(dev);

	if (sc->dpnidev != NULL)
		MIIBUS_STATCHG(sc->dpnidev);
}

static int
memacphy_set_ni_dev(device_t dev, device_t nidev)
{
	struct memacphy_softc *sc;

	if (nidev == NULL)
		return (EINVAL);

#if 0
	if (bootverbose)
		device_printf(dev, "setting nidev %p (%s)\n",
		    nidev, device_get_nameunit(nidev));
#endif

	sc = device_get_softc(dev);
	if (sc->dpnidev != NULL)
		return (EBUSY);

	sc->dpnidev = nidev;
	return (0);
}

static int
memacphy_get_phy_loc(device_t dev, int *phy_loc)
{
	struct memacphy_softc *sc;
	int error;

	if (phy_loc == NULL)
		return (EINVAL);

	sc = device_get_softc(dev);
	if (sc->phy == -1) {
		*phy_loc = MII_PHY_ANY;
		error = ENODEV;
	} else {
		*phy_loc = sc->phy;
		error = 0;
	}

#if 0
	if (bootverbose)
		device_printf(dev, "returning phy_loc %d, error %d\n",
		    *phy_loc, error);
#endif

	return (error);
}

#ifdef FDT
static int
memacphy_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	device_set_desc(dev, "MEMAC PHY (fdt)");
	return (BUS_PROBE_DEFAULT);
} 

static int
memacphy_fdt_attach(device_t dev)
{
	struct memacphy_softc *sc;
	phandle_t node;
	ssize_t s;
	int error;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	s = device_get_property(dev, "reg", &sc->reg, sizeof(sc->reg),
	    DEVICE_PROP_UINT32);
	if (s != -1)
		sc->phy = sc->reg;
	else
		sc->phy = -1;
	sc->xref = OF_xref_from_node(node);

	error = OF_device_register_xref(sc->xref, dev);
	if (error != 0)
		device_printf(dev, "Failed to register xref %#x\n", sc->xref);

	if (bootverbose)
		device_printf(dev, "node %#x '%s': reg %#x xref %#x phy %u\n",
		    node, ofw_bus_get_name(dev), sc->reg, sc->xref, sc->phy);

	if (sc->phy == -1)
		error = ENXIO;
	return (error);
}

static device_method_t memacphy_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		memacphy_fdt_probe),
	DEVMETHOD(device_attach,	memacphy_fdt_attach),
	DEVMETHOD(device_detach,	bus_generic_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	memacphy_miibus_readreg),
	DEVMETHOD(miibus_writereg,	memacphy_miibus_writereg),
	DEVMETHOD(miibus_statchg,	memacphy_miibus_statchg),

	/* memac */
	DEVMETHOD(memac_mdio_set_ni_dev, memacphy_set_ni_dev),
	DEVMETHOD(memac_mdio_get_phy_loc, memacphy_get_phy_loc),

	DEVMETHOD_END
};

DEFINE_CLASS_0(memacphy_fdt, memacphy_fdt_driver, memacphy_fdt_methods,
    sizeof(struct memacphy_softc));

EARLY_DRIVER_MODULE(memacphy_fdt, memac_mdio, memacphy_fdt_driver, 0, 0,
    BUS_PASS_SUPPORTDEV);
DRIVER_MODULE(miibus, memacphy_fdt, miibus_driver, 0, 0);
MODULE_DEPEND(memacphy_fdt, miibus, 1, 1, 1);
#endif

#ifdef DEV_ACPI
static int
memacphy_acpi_probe(device_t dev)
{

	device_set_desc(dev, "MEMAC PHY (acpi)");
	return (BUS_PROBE_DEFAULT);
}

static int
memacphy_acpi_attach(device_t dev)
{
	struct memacphy_softc *sc;
	ACPI_HANDLE h;
	ssize_t s;

	sc = device_get_softc(dev);
	h = acpi_get_handle(dev);

	s = acpi_GetInteger(h, "_UID", &sc->uid);
	if (ACPI_FAILURE(s)) {
		device_printf(dev, "Cannot get '_UID' property: %zd\n", s);
		return (ENXIO);
	}

	s = device_get_property(dev, "phy-channel",
	    &sc->phy_channel, sizeof(sc->phy_channel), DEVICE_PROP_UINT64);
	if (s != -1)
		sc->phy = sc->phy_channel;
	else
		sc->phy = -1;
	s = device_get_property(dev, "compatible",
	    sc->compatible, sizeof(sc->compatible), DEVICE_PROP_ANY);

	if (bootverbose)
		device_printf(dev, "UID %#04x phy-channel %ju compatible '%s' phy %u\n",
		    sc->uid, sc->phy_channel,
		    sc->compatible[0] != '\0' ? sc->compatible : "", sc->phy);

	if (sc->phy == -1)
		return (ENXIO);
	return (0);
}

static device_method_t memacphy_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		memacphy_acpi_probe),
	DEVMETHOD(device_attach,	memacphy_acpi_attach),
	DEVMETHOD(device_detach,	bus_generic_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	memacphy_miibus_readreg),
	DEVMETHOD(miibus_writereg,	memacphy_miibus_writereg),
	DEVMETHOD(miibus_statchg,	memacphy_miibus_statchg),

	/* memac */
	DEVMETHOD(memac_mdio_set_ni_dev, memacphy_set_ni_dev),
	DEVMETHOD(memac_mdio_get_phy_loc, memacphy_get_phy_loc),

	DEVMETHOD_END
};

DEFINE_CLASS_0(memacphy_acpi, memacphy_acpi_driver, memacphy_acpi_methods,
    sizeof(struct memacphy_softc));

EARLY_DRIVER_MODULE(memacphy_acpi, memac_mdio, memacphy_acpi_driver, 0, 0,
    BUS_PASS_SUPPORTDEV);
DRIVER_MODULE(miibus, memacphy_acpi, miibus_driver, 0, 0);
MODULE_DEPEND(memacphy_acpi, miibus, 1, 1, 1);
#endif

/* -------------------------------------------------------------------------- */

struct memac_mdio_softc {
#ifdef FDT
	struct simplebus_softc	sb_sc;
#endif
	struct resource		*mem_res;
	bool			is_little_endian;
};

/*
 * MDIO Ethernet Management Interface Registers (internal PCS MDIO PHY)
 * 0x0030	MDIO Configuration Register (MDIO_CFG)
 * 0x0034	MDIO Control Register (MDIO_CTL)
 * 0x0038	MDIO Data Register (MDIO_DATA)
 * 0x003c	MDIO Register Address Register (MDIO_ADDR)
 *
 * External MDIO interfaces
 * 0x0030	External MDIO Configuration Register (EMDIO_CFG)
 * 0x0034	External MDIO Control Register (EMDIO_CTL)
 * 0x0038	External MDIO Data Register (EMDIO_DATA)
 * 0x003c	External MDIO Register Address Register (EMDIO_ADDR)
 */
#define	MDIO_CFG			0x00030
#define	MDIO_CFG_MDIO_RD_ER		(1 << 1)
#define	MDIO_CFG_ENC45			(1 << 6)
#define	MDIO_CFG_BUSY			(1 << 31)
#define	MDIO_CTL			0x00034
#define	MDIO_CTL_READ			(1 << 15)
#define	MDIO_CTL_PORT_ADDR(_x)		(((_x) & 0x1f) << 5)
#define	MDIO_CTL_DEV_ADDR(_x)		((_x) & 0x1f)
#define	MDIO_DATA			0x00038
#define	MDIO_ADDR			0x0003c

static uint32_t
memac_read_4(struct memac_mdio_softc *sc, uint32_t reg)
{
	uint32_t v, r;

	v = bus_read_4(sc->mem_res, reg);
	if (sc->is_little_endian)
		r = le32toh(v);
	else
		r = be32toh(v);

	return (r);
}

static void
memac_write_4(struct memac_mdio_softc *sc, uint32_t reg, uint32_t val)
{
	uint32_t v;

	if (sc->is_little_endian)
		v = htole32(val);
	else
		v = htobe32(val);
	bus_write_4(sc->mem_res, reg, v);
}

static uint32_t
memac_miibus_wait_no_busy(struct memac_mdio_softc *sc)
{
	uint32_t count, val;

	for (count = 1000; count > 0; count--) {
		val = memac_read_4(sc, MDIO_CFG);
		if ((val & MDIO_CFG_BUSY) == 0)
			break;
		DELAY(1);
	}

	if (count == 0)
		return (0xffff);

	return (0);
}

static int
memac_miibus_readreg(device_t dev, int phy, int reg)
{
	struct memac_mdio_softc *sc;
	uint32_t cfg, ctl, val;

	sc = device_get_softc(dev);

	/* Set proper Clause 45 mode. */
	cfg = memac_read_4(sc, MDIO_CFG);
	/* XXX 45 support? */
	cfg &= ~MDIO_CFG_ENC45;	/* Use Clause 22 */
	memac_write_4(sc, MDIO_CFG, cfg);

	val = memac_miibus_wait_no_busy(sc);
	if (val != 0)
		return (0xffff);

	/* To whom do we want to talk to.. */
	ctl = MDIO_CTL_PORT_ADDR(phy) | MDIO_CTL_DEV_ADDR(reg);
	/* XXX do we need two writes for this to work reliably? */
	memac_write_4(sc, MDIO_CTL, ctl | MDIO_CTL_READ);

	val = memac_miibus_wait_no_busy(sc);
	if (val != 0)
		return (0xffff);

	cfg = memac_read_4(sc, MDIO_CFG);
	if (cfg & MDIO_CFG_MDIO_RD_ER)
		return (0xffff);

	val = memac_read_4(sc, MDIO_DATA);
	val &= 0xffff;

#if 0
	device_printf(dev, "phy read %d:%d = %#06x\n", phy, reg, val);
#endif

        return (val);
}

static int
memac_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct memac_mdio_softc *sc;
	uint32_t cfg, ctl, val;

	sc = device_get_softc(dev);

#if 0
	device_printf(dev, "phy write %d:%d\n", phy, reg);
#endif

	/* Set proper Clause 45 mode. */
	cfg = memac_read_4(sc, MDIO_CFG);
	/* XXX 45 support? */
	cfg &= ~MDIO_CFG_ENC45;	/* Use Clause 22 */
	memac_write_4(sc, MDIO_CFG, cfg);

	val = memac_miibus_wait_no_busy(sc);
	if (val != 0)
		return (0xffff);

	/* To whom do we want to talk to.. */
	ctl = MDIO_CTL_PORT_ADDR(phy) | MDIO_CTL_DEV_ADDR(reg);
	memac_write_4(sc, MDIO_CTL, ctl);

	memac_write_4(sc, MDIO_DATA, data & 0xffff);

	val = memac_miibus_wait_no_busy(sc);
	if (val != 0)
		return (0xffff);

	return (0);
}

static ssize_t
memac_mdio_get_property(device_t dev, device_t child, const char *propname,
    void *propvalue, size_t size, device_property_type_t type)
{

	return (bus_generic_get_property(dev, child, propname, propvalue, size, type));
}

static int
memac_mdio_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{

	return (BUS_READ_IVAR(device_get_parent(dev), dev, index, result));
}


static int
memac_mdio_generic_attach(device_t dev)
{
	struct memac_mdio_softc *sc;
	int rid;

	sc = device_get_softc(dev);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "%s: cannot allocate mem resource\n",
		    __func__);
		return (ENXIO);
	}

	sc->is_little_endian = device_has_property(dev, "little-endian");

	return (0);
}

static int
memac_mdio_detach(device_t dev)
{
	struct memac_mdio_softc *sc;

	sc = device_get_softc(dev);

	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);

	return (0);
}

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{ "fsl,fman-memac-mdio",		1 },
	{ NULL,					0 }
};

static int
memac_mdio_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Freescale XGMAC MDIO Bus (FDT)");
	return (BUS_PROBE_DEFAULT);
}

static int
memac_mdio_fdt_probe_child(device_t bus, phandle_t child)
{
	device_t childdev;

	/* make sure we do not aliready have a device. */
	childdev = ofw_bus_find_child_device_by_phandle(bus, child);
	if (childdev != NULL)
		return (0);

	childdev = simplebus_add_device(bus, child, 0, NULL, -1, NULL);
	if (childdev == NULL)
		return (ENXIO);

	return (device_probe_and_attach(childdev));
}

static int
memac_mdio_fdt_attach(device_t dev)
{
	phandle_t node, child;
	int error;

	error = memac_mdio_generic_attach(dev);
	if (error != 0)
		return (error);

	/* Attach the *phy* children represented in the device tree. */
	bus_generic_probe(dev);
	bus_enumerate_hinted_children(dev);
	node = ofw_bus_get_node(dev);
	simplebus_init(dev, node);
	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		if (!OF_hasprop(child, "reg"))
			continue;
		if (memac_mdio_fdt_probe_child(dev, child) != 0)
			continue;
	}

	return (bus_generic_attach(dev));
}

static const struct ofw_bus_devinfo *
memac_simplebus_get_devinfo(device_t bus, device_t child)
{

	return (OFW_BUS_GET_DEVINFO(device_get_parent(bus), child));
}

static device_method_t memac_mdio_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		memac_mdio_fdt_probe),
	DEVMETHOD(device_attach,	memac_mdio_fdt_attach),
	DEVMETHOD(device_detach,	memac_mdio_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	memac_miibus_readreg),
	DEVMETHOD(miibus_writereg,	memac_miibus_writereg),

	/* OFW/simplebus */
	DEVMETHOD(ofw_bus_get_devinfo,	memac_simplebus_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

	/* .. */
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(bus_read_ivar,	memac_mdio_read_ivar),
	DEVMETHOD(bus_get_property,	memac_mdio_get_property),

	DEVMETHOD_END
};

DEFINE_CLASS_0(memac_mdio, memac_mdio_fdt_driver, memac_mdio_fdt_methods,
    sizeof(struct memac_mdio_softc));

EARLY_DRIVER_MODULE(memac_mdio, simplebus, memac_mdio_fdt_driver, 0, 0,
    BUS_PASS_SUPPORTDEV);
#endif

#ifdef DEV_ACPI
/* Context for walking PHY child devices. */
struct memac_mdio_walk_ctx {
	device_t	dev;
	int		count;
	int		countok;
};

static char *memac_mdio_ids[] = {
	"NXP0006",
	NULL
};

static int
memac_mdio_acpi_probe(device_t dev)
{
	int rc;

	if (acpi_disabled("fsl_memac_mdio"))
		return (ENXIO);

	rc = ACPI_ID_PROBE(device_get_parent(dev), dev, memac_mdio_ids, NULL);
	if (rc <= 0)
		device_set_desc(dev, "Freescale XGMAC MDIO Bus");

	return (rc);
}

static ACPI_STATUS
memac_mdio_acpi_probe_child(ACPI_HANDLE h, device_t *dev, int level, void *arg)
{
	struct memac_mdio_walk_ctx *ctx;
	struct acpi_device *ad;
	device_t child;
	uint32_t adr;

	ctx = (struct memac_mdio_walk_ctx *)arg;
	ctx->count++;

	if (ACPI_FAILURE(acpi_GetInteger(h, "_ADR", &adr)))
		return (AE_OK);

	/* Technically M_ACPIDEV */
	if ((ad = malloc(sizeof(*ad), M_DEVBUF, M_NOWAIT | M_ZERO)) == NULL)
		return (AE_OK);

	child = device_add_child(ctx->dev, "memacphy_acpi", -1);
	if (child == NULL) {
		free(ad, M_DEVBUF);
		return (AE_OK);
	}
	ad->ad_handle = h;
	ad->ad_cls_class = 0xffffff;
	resource_list_init(&ad->ad_rl);
	device_set_ivars(child, ad);
	*dev = child;

	ctx->countok++;
	return (AE_OK);
}

static int
memac_mdio_acpi_attach(device_t dev)
{
	struct memac_mdio_walk_ctx ctx;
	int error;

	error = memac_mdio_generic_attach(dev);
	if (error != 0)
		return (error);

	ctx.dev = dev;
	ctx.count = 0;
	ctx.countok = 0;
	ACPI_SCAN_CHILDREN(device_get_parent(dev), dev, 1,
	    memac_mdio_acpi_probe_child, &ctx);
	if (ctx.countok > 0) {
		bus_generic_probe(dev);
		bus_generic_attach(dev);
	}

	return (0);
}

static device_method_t memac_mdio_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		memac_mdio_acpi_probe),
	DEVMETHOD(device_attach,	memac_mdio_acpi_attach),
	DEVMETHOD(device_detach,	memac_mdio_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	memac_miibus_readreg),
	DEVMETHOD(miibus_writereg,	memac_miibus_writereg),

	/* .. */
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(bus_read_ivar,	memac_mdio_read_ivar),
	DEVMETHOD(bus_get_property,	memac_mdio_get_property),

	DEVMETHOD_END
};

DEFINE_CLASS_0(memac_mdio, memac_mdio_acpi_driver, memac_mdio_acpi_methods,
    sizeof(struct memac_mdio_softc));

EARLY_DRIVER_MODULE(memac_mdio, acpi, memac_mdio_acpi_driver, 0, 0,
    BUS_PASS_SUPPORTDEV);
#endif

DRIVER_MODULE(miibus, memac_mdio, miibus_driver, 0, 0);
MODULE_DEPEND(memac_mdio, miibus, 1, 1, 1);
MODULE_VERSION(memac_mdio, 1);
