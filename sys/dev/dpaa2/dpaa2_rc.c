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
 * The DPAA2 Resource Container (DPRC) bus driver.
 *
 * DPRC holds all the resources and object information that a software context
 * (kernel, virtual machine, etc.) can access or use.
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

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

MALLOC_DEFINE(M_DPAA2_RC, "dpaa2_rc_memory", "DPAA2 Resource Container memory");

/* Forward declarations. */
static int dpaa2_rc_detach(device_t dev);

/*
 * Device interface.
 */
static int
dpaa2_rc_probe(device_t dev)
{
	/*
	 * Do not perform any checks. DPRC device will be added by a parent DPRC
	 * or by MC itself.
	 */
	device_set_desc(dev, "DPAA2 Resource Container");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_rc_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_mc_softc *mcsc;
	struct dpaa2_rc_softc *sc;
	struct dpaa2_devinfo *dinfo = NULL;
	dpaa2_cmd_t cmd;
	dpaa2_rc_attr_t dprc_attr;
	uint32_t major, minor, rev;
	uint32_t cont_id, obj_count;
	uint16_t rc_token;
	int irqs[1];
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->portal = NULL;
	sc->unit = device_get_unit(dev);

	if (sc->unit == 0) {
		/* Root DPRC attached directly to the MC bus. */
		pdev = device_get_parent(dev);
		mcsc = device_get_softc(pdev);

		/*
		 * Allocate device info to let the MC bus access ICID of the
		 * DPRC object.
		 */
		dinfo = malloc(sizeof(struct dpaa2_devinfo), M_DPAA2_RC,
		    M_WAITOK | M_ZERO);
		if (!dinfo) {
			device_printf(dev, "Failed to allocate dpaa2_devinfo\n");
			dpaa2_rc_detach(dev);
			return (ENXIO);
		}
		device_set_ivars(dev, dinfo);
		dinfo->pdev = pdev;
		dinfo->dev = dev;

		/* Prepare helper portal object to send commands to MC. */
		error = dpaa2_mcp_init_portal(&sc->portal, mcsc->res[0],
		    &mcsc->map[0], DPAA2_PORTAL_DEF);
		if (error) {
			device_printf(dev, "Failed to allocate dpaa2_mcp\n");
			dpaa2_rc_detach(dev);
			return (ENXIO);
		}
	} else {
		/* Child DPRCs aren't supported yet. */
		return (ENXIO);
	}

	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd\n");
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}

	/*
	 * Print info about MC and DPAA2 objects.
	 */

	error = dpaa2_cmd_mng_get_version(sc->portal, cmd, &major, &minor, &rev);
	if (error) {
		device_printf(dev, "Failed to get MC firmware version\n");
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "MC firmware version: %u.%u.%u\n", major, minor, rev);

	error = dpaa2_cmd_mng_get_container_id(sc->portal, cmd, &cont_id);
	if (error) {	
		device_printf(dev, "Failed to get container ID\n");
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "Resource container ID: %u\n", cont_id);

	error = dpaa2_cmd_rc_open(sc->portal, cmd, cont_id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open container: ID=%u\n", cont_id);
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}

	error = dpaa2_cmd_rc_get_obj_count(sc->portal, cmd, &obj_count);
	if (error) {
		device_printf(dev, "Failed to count objects in container: "
		    "ID=%u\n", cont_id);
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "Objects in container: %u\n", obj_count);

	error = dpaa2_cmd_rc_get_attributes(sc->portal, cmd, &dprc_attr);
	if (error) {
		device_printf(dev, "Failed to get attributes of the container: "
		    "ID=%u\n", cont_id);
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "ICID: %u\n", dprc_attr.icid);
	if (dinfo)
		dinfo->icid = dprc_attr.icid;

	/*
	 * Ask MC bus to allocate MSI for this DPRC using its pseudo-pcib
	 * interface.
	 */
	error = PCIB_ALLOC_MSI(device_get_parent(dev), dev, 1, 1, irqs);
	if (error) {
		device_printf(dev, "Failed to allocate MSI for DPRC: "
		    "error=%u\n", error);
		dpaa2_mcp_free_command(cmd);
		dpaa2_rc_detach(dev);
		return (ENXIO);
	}
	device_printf(dev, "MSI allocated: irq=%d\n", irqs[0]);

	dpaa2_cmd_rc_close(sc->portal, cmd);
	dpaa2_mcp_free_command(cmd);

	return (0);
}

static int
dpaa2_rc_detach(device_t dev)
{
	struct dpaa2_rc_softc *sc;
	struct dpaa2_devinfo *dinfo;

	sc = device_get_softc(dev);
	dinfo = device_get_ivars(dev);

	if (sc->portal)
		dpaa2_mcp_free_portal(sc->portal);
	if (dinfo)
		free(dinfo, M_DPAA2_RC);

	return (0);
}

static device_method_t dpaa2_rc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_rc_probe),
	DEVMETHOD(device_attach,	dpaa2_rc_attach),
	DEVMETHOD(device_detach,	dpaa2_rc_detach),
	DEVMETHOD_END
};

static driver_t dpaa2_rc_driver = {
	"dpaa2_rc",
	dpaa2_rc_methods,
	sizeof(struct dpaa2_rc_softc),
};

static devclass_t dpaa2_rc_devclass;

DRIVER_MODULE(dpaa2_rc, dpaa2_mc, dpaa2_rc_driver, dpaa2_rc_devclass, 0, 0);
DRIVER_MODULE(dpaa2_rc, dpaa2_rc, dpaa2_rc_driver, dpaa2_rc_devclass, 0, 0);
