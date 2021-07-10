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

#include "dpaa2_mcp.h"
#include "dpaa2_mc.h"

/* Device interface */
static int dpaa2_rc_probe(device_t dev);
static int dpaa2_rc_attach(device_t dev);
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
	dpaa2_cmd_t cmd;
	dpaa2_obj_t obj;
	uint32_t major, minor, rev;
	uint32_t cont_id, obj_count;
	uint16_t rc_token;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->portal = NULL;
	sc->unit = device_get_unit(dev);

	if (sc->unit == 0) {
		/* Root DPRC attached directly to the MC bus. */
		pdev = device_get_parent(dev);
		mcsc = device_get_softc(pdev);

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

	for (uint32_t i = 0; i < obj_count; i++) {
		error = dpaa2_cmd_rc_get_obj(sc->portal, cmd, i, &obj);
		if (error) {
			device_printf(dev, "Failed to get object: ID=%u\n", i);
			continue;
		}
		device_printf(dev, "Object: id=%u vendor=%u irqs=%u "
		    "regions=%u version=%u.%u\n", obj.id, obj.vendor,
		    obj.irq_count, obj.reg_count, obj.ver_major, obj.ver_minor);
	}

	dpaa2_cmd_rc_close(sc->portal, cmd);
	dpaa2_mcp_free_command(cmd);

	return (0);
}

static int
dpaa2_rc_detach(device_t dev)
{
	struct dpaa2_rc_softc *sc;

	sc = device_get_softc(dev);
	if (sc->portal)
		dpaa2_mcp_free_portal(sc->portal);

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
