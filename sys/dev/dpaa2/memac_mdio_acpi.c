/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Bjoern A. Zeeb
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

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "acpi_bus_if.h"
#include "miibus_if.h"

/* -------------------------------------------------------------------------- */

struct memacphy_softc {
	int			uid;
	uint64_t		phy_channel;
	char			compatible[64];
};

static int
memacphy_acpi_probe(device_t dev)
{

	device_set_desc(dev, "MEMAC PHY");
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
	    &sc->phy_channel, sizeof(sc->phy_channel));
	s = device_get_property(dev, "compatible",
	    sc->compatible, sizeof(sc->compatible));
	if (bootverbose)
		device_printf(dev, "UID %#04x phy-channel %ju compatible '%s'\n",
		    sc->uid, sc->phy_channel,
		    sc->compatible[0] != '\0' ? sc->compatible : "");

	return (0);
}

static int
memacphy_miibus_readreg(device_t dev, int phy, int reg)
{
#if 0
	device_printf(dev, "phy read %d:%d\n", phy, reg);
#endif
	return (MIIBUS_READREG(device_get_parent(dev), phy, reg));
}

static int
memacphy_miibus_writereg(device_t dev, int phy, int reg, int data)
{
#if 0
	device_printf(dev, "phy write %d:%d\n", phy, reg);
#endif
	return (MIIBUS_WRITEREG(device_get_parent(dev), phy, reg, data));
}

static void
memacphy_miibus_statchg(device_t dev)
{
	device_printf(dev, "statchg\n");
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

	DEVMETHOD_END
};

DEFINE_CLASS_0(memacphy, memacphy_acpi_driver, memacphy_acpi_methods,
    sizeof(struct memacphy_softc));

static devclass_t memacphy_devclass;

DRIVER_MODULE(memacphy, memac_mdio, memacphy_acpi_driver, memacphy_devclass,
    0, 0);
DRIVER_MODULE(miibus, memacphy, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(memacphy, miibus, 1, 1, 1);

/* -------------------------------------------------------------------------- */

struct memac_mdio_softc {
	device_t	dev;
	struct resource	*mem_res;
	bool		is_little_endian;
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

	child = BUS_ADD_CHILD(ctx->dev, 0, "memacphy", -1);
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
	struct memac_mdio_softc *sc;
	struct memac_mdio_walk_ctx ctx;
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

static int
memac_mdio_acpi_detach(device_t dev)
{
	struct memac_mdio_softc *sc;

	sc = device_get_softc(dev);

	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);

	return (0);
}

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

static void
memac_miibus_statchg(device_t dev)
{
	struct memac_mdio_softc *sc;

	sc = device_get_softc(dev);
	device_printf(dev, "statchg\n");
}

static ssize_t
memac_mdio_get_property(device_t dev, device_t child, const char *propname,
    void *propvalue, size_t size)
{

	return (acpi_bus_get_prop(dev, child, propname, propvalue, size));
}

static int
memac_mdio_acpi_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{

	return (acpi_read_ivar(dev, child, index, result));
}


static device_method_t memac_mdio_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		memac_mdio_acpi_probe),
	DEVMETHOD(device_attach,	memac_mdio_acpi_attach),
	DEVMETHOD(device_detach,	memac_mdio_acpi_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	memac_miibus_readreg),
	DEVMETHOD(miibus_writereg,	memac_miibus_writereg),
	DEVMETHOD(miibus_statchg,	memac_miibus_statchg),

	/* .. */
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(bus_read_ivar,	memac_mdio_acpi_read_ivar),
	DEVMETHOD(bus_get_property,	memac_mdio_get_property),

	DEVMETHOD_END
};

DEFINE_CLASS_0(memac_mdio, memac_mdio_acpi_driver, memac_mdio_acpi_methods,
    sizeof(struct memac_mdio_softc));

static devclass_t memac_mdio_acpi_devclass;

DRIVER_MODULE(memac_mdio, acpi, memac_mdio_acpi_driver, memac_mdio_acpi_devclass,
    0, 0);
MODULE_VERSION(memac_mdio, 1);
DRIVER_MODULE(miibus, memac_mdio, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(memac_mdio, miibus, 1, 1, 1);
