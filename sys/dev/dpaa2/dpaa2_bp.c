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
 * The DPAA2 Buffer Pool (DPBP) driver.
 *
 * The DPBP configures a buffer pool that can be associated with DPAA2 network
 * and accelerator interfaces.
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

#include "dpaa2_mc.h"
#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"
#include "dpaa2_swp_if.h"
#include "dpaa2_cmd_if.h"

/* Forward declarations. */

static int	dpaa2_bp_detach(device_t dev);

/*
 * Device interface.
 */

static int
dpaa2_bp_probe(device_t dev)
{
	/* DPIO device will be added by a parent resource container itself. */
	device_set_desc(dev, "DPAA2 Buffer Pool");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_bp_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_rc_softc *rcsc;
	struct dpaa2_bp_softc *sc;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	dpaa2_cmd_t cmd;
	uint16_t rc_token, bp_token;
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

	/* Open resource container and DPBP object. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: error=%d\n", error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_BP_OPEN(dev, cmd, dinfo->id, &bp_token);
	if (error) {
		device_printf(dev, "Failed to open DPBP: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_rc;
	}

	/* Prepare DPBP object. */
	error = DPAA2_CMD_BP_RESET(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to reset DPBP: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_bp;
	}
	error = DPAA2_CMD_BP_ENABLE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to enable DPBP: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_bp;
	}
	error = DPAA2_CMD_BP_GET_ATTRIBUTES(dev, cmd, &sc->attr);
	if (error) {
		device_printf(dev, "Failed to get DPBP attributes: id=%d, "
		    "error=%d\n", dinfo->id, error);
		goto err_disable_bp;
	}

	/* Close the DPBP object and the resource container. */
	error = DPAA2_CMD_BP_CLOSE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to close DPBP: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_rc;
	}
	error = DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(cmd, rc_token));
	if (error) {
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
		goto err_free_cmd;
	}

	return (0);

err_disable_bp:
	DPAA2_CMD_BP_DISABLE(dev, cmd);
err_close_bp:
	DPAA2_CMD_BP_CLOSE(dev, dpaa2_mcp_tk(cmd, bp_token));
err_close_rc:
	DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(cmd, rc_token));
err_free_cmd:
	dpaa2_mcp_free_command(cmd);
err_exit:
	dpaa2_bp_detach(dev);
	return (ENXIO);
}

static int
dpaa2_bp_detach(device_t dev)
{
	return (0);
}

static device_method_t dpaa2_bp_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_bp_probe),
	DEVMETHOD(device_attach,	dpaa2_bp_attach),
	DEVMETHOD(device_detach,	dpaa2_bp_detach),

	DEVMETHOD_END
};

static driver_t dpaa2_bp_driver = {
	"dpaa2_bp",
	dpaa2_bp_methods,
	sizeof(struct dpaa2_bp_softc),
};

static devclass_t dpaa2_bp_devclass;

DRIVER_MODULE(dpaa2_bp, dpaa2_rc, dpaa2_bp_driver, dpaa2_bp_devclass, 0, 0);
