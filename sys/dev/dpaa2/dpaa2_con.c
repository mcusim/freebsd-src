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
 * The DPAA2 Concentrator (DPCON) driver.
 *
 * The DPCON object provides advanced scheduling of ingress packets, including
 * scheduling between different network interfaces.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/mutex.h>

#include <vm/vm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>

#include "pcib_if.h"
#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"
#include "dpaa2_mc.h"
#include "dpaa2_cmd_if.h"

/* Forward declarations. */

static int dpaa2_con_detach(device_t dev);

/*
 * Device interface.
 */

static int
dpaa2_con_probe(device_t dev)
{
	/* DPCON device will be added by a parent resource container itself. */
	device_set_desc(dev, "DPAA2 Concentrator");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_con_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_rc_softc *rcsc;
	struct dpaa2_con_softc *sc;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	dpaa2_cmd_t cmd;
	uint16_t rc_token, con_token;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	pdev = device_get_parent(dev);
	rcsc = device_get_softc(pdev);
	rcinfo = device_get_ivars(pdev);
	dinfo = device_get_ivars(dev);

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Open resource container and DPCON object. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: error=%d\n", error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_CON_OPEN(dev, cmd, dinfo->id, &con_token);
	if (error) {
		device_printf(dev, "Failed to open DPCON: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}

	/* Prepare DPCON object. */
	error = DPAA2_CMD_CON_RESET(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to reset DPCON: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_CON_GET_ATTRIBUTES(dev, cmd, &sc->attr);
	if (error) {
		device_printf(dev, "Failed to get DPCON attributes: id=%d, "
		    "error=%d\n", dinfo->id, error);
		goto err_free_cmd;
	}
	if (bootverbose)
		device_printf(dev, "channel_id=%d, priorities=%d\n",
		    sc->attr.chan_id, sc->attr.prior_num);

	error = DPAA2_CMD_CON_ENABLE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to enable DPCON: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}

	/* Close the DPCON object and the resource container. */
	error = DPAA2_CMD_CON_CLOSE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to close DPCON: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(cmd, rc_token));
	if (error) {
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
		goto err_free_cmd;
	}

	return (0);

 err_free_cmd:
	dpaa2_mcp_free_command(cmd);
 err_exit:
	dpaa2_con_detach(dev);
	return (ENXIO);
}

static int
dpaa2_con_detach(device_t dev)
{
	return (0);
}

static device_method_t dpaa2_con_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_con_probe),
	DEVMETHOD(device_attach,	dpaa2_con_attach),
	DEVMETHOD(device_detach,	dpaa2_con_detach),

	DEVMETHOD_END
};

static driver_t dpaa2_con_driver = {
	"dpaa2_con",
	dpaa2_con_methods,
	sizeof(struct dpaa2_con_softc),
};

static devclass_t dpaa2_con_devclass;

DRIVER_MODULE(dpaa2_con, dpaa2_rc, dpaa2_con_driver, dpaa2_con_devclass, 0, 0);
