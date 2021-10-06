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
 * QBMan command interface and the DPAA2 I/O (DPIO) driver.
 *
 * The DPIO object allows configuration of the QBMan software portal with
 * optional notification capabilities.
 *
 * Software portals are used by the driver to communicate with the QBMan. The
 * DPIO object’s main purpose is to enable the driver to perform I/O – enqueue
 * and dequeue operations, as well as buffer release and acquire operations –
 * using QBMan.
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

/*
 * Memory:
 *	0: cache-enabled part of the QBMan software portal.
 *	1: cache-inhibited part of the QBMan software portal.
 *	2: control registers of the QBMan software portal?
 *
 * Note that MSI should be allocated separately using pseudo-PCI interface.
 */
static struct resource_spec dpaa2_io_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_UNMAPPED },
	{ SYS_RES_MEMORY, 1, RF_ACTIVE | RF_UNMAPPED },
	{ SYS_RES_MEMORY, 2, RF_ACTIVE | RF_UNMAPPED | RF_OPTIONAL },

	RESOURCE_SPEC_END
};

/* Forward declarations. */
static int	setup_msi(struct dpaa2_io_softc *sc);
static void	msi_intr(void *arg);

/*
 * Device interface.
 */

static int
dpaa2_io_probe(device_t dev)
{
	/* DPIO device will be added by a parent resource container itself. */
	device_set_desc(dev, "DPAA2 I/O");
	return (BUS_PROBE_DEFAULT);
}

static int
dpaa2_io_attach(device_t dev)
{
	device_t pdev;
	struct dpaa2_rc_softc *rcsc;
	struct dpaa2_io_softc *sc;
	struct dpaa2_devinfo *rcinfo;
	struct dpaa2_devinfo *dinfo;
	struct resource_map_request req;
	dpaa2_cmd_t cmd;
	dpaa2_io_attr_t attr;
	uint16_t rc_token, io_token;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	pdev = device_get_parent(dev);
	rcsc = device_get_softc(pdev);
	rcinfo = device_get_ivars(pdev);
	dinfo = device_get_ivars(dev);

	error = bus_alloc_resources(sc->dev, dpaa2_io_spec, sc->res);
	if (error) {
		device_printf(dev, "Failed to allocate resources: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Map cache-enabled part of the software portal memory. */
	resource_init_map_request(&req);
	req.memattr = VM_MEMATTR_WRITE_BACK;
	error = bus_map_resource(sc->dev, SYS_RES_MEMORY, sc->res[0],
	    &req, &sc->map[0]);
	if (error) {
		device_printf(dev, "Failed to map cache-enabled part of the "
		    "software portal memory: error=%d\n", error);
		goto err_exit;
	}

	/* Map cache-inhibited part of the software portal memory. */
	resource_init_map_request(&req);
	req.memattr = VM_MEMATTR_DEVICE;
	error = bus_map_resource(sc->dev, SYS_RES_MEMORY, sc->res[1],
	    &req, &sc->map[1]);
	if (error) {
		device_printf(dev, "Failed to map cache-inhibited part of the "
		    "software portal memory: error=%d\n", error);
		goto err_exit;
	}

	/* An attempt to map a region with control registers. */
	if (sc->res[2]) {
		resource_init_map_request(&req);
		req.memattr = VM_MEMATTR_DEVICE;
		error = bus_map_resource(sc->dev, SYS_RES_MEMORY, sc->res[2],
		    &req, &sc->map[2]);
		if (error) {
			device_printf(dev, "Failed to map control registers: "
			    "error=%d\n", error);
			goto err_exit;
		}
	}

	/* Allocate a command to send to MC hardware. */
	error = dpaa2_mcp_init_command(&cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Open resource container and DPIO object. */
	error = DPAA2_CMD_RC_OPEN(dev, cmd, rcinfo->id, &rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: error=%d\n", error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_IO_OPEN(dev, cmd, dinfo->id, &io_token);
	if (error) {
		device_printf(dev, "Failed to open DPIO: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}

	/* Prepare DPIO object. */
	error = DPAA2_CMD_IO_RESET(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to reset DPIO: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_IO_GET_ATTRIBUTES(dev, cmd, &attr);
	if (error) {
		device_printf(dev, "Failed to get DPIO attributes: id=%d, "
		    "error=%d\n", dinfo->id, error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_IO_ENABLE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to enable DPIO: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}

	/* Close the DPIO object and the resource container. */
	error = DPAA2_CMD_IO_CLOSE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to close DPIO: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_free_cmd;
	}
	dpaa2_mcp_set_token(cmd, rc_token);
	error = DPAA2_CMD_RC_CLOSE(dev, cmd);
	if (error) {
		device_printf(dev, "Failed to close DPRC: error=%d\n", error);
		goto err_free_cmd;
	}

	/* Prepare helper object to work with the QBMan software portal. */
	sc->swp_desc.dpio_dev = dev;
	sc->swp_desc.swp_version = attr.swp_version;
	sc->swp_desc.swp_id = attr.swp_id;
	sc->swp_desc.has_notif = attr.priors_num ? true : false;
	sc->swp_desc.has_8prio = attr.priors_num == 8u ? true : false;
	sc->swp_desc.cena_res = sc->res[0];
	sc->swp_desc.cena_map = &sc->map[0];
	sc->swp_desc.cinh_res = sc->res[1];
	sc->swp_desc.cinh_map = &sc->map[1];
	error = dpaa2_swp_init_portal(&sc->swp, &sc->swp_desc, DPAA2_SWP_DEF);
	if (error) {
		device_printf(dev, "Failed to initialize dpaa2_swp: error=%d\n",
		    error);
		goto err_free_cmd;
	}

	/* Enable only DQRR interrupts for now */
	DPAA2_SWP_SET_INTR_TRIGGER(dev, DPAA2_SWP_INTR_DQRI);
	DPAA2_SWP_CLEAR_INTR_STATUS(dev, 0xffffffff);
	if (sc->swp_desc.has_notif)
		DPAA2_SWP_SET_PUSH_DEQUEUE(dev, 0, true);

	/* Configure IRQs. */
	error = setup_msi(sc);
	if (error) {
		device_printf(dev, "Failed to allocate MSI: error=%d\n", error);
		goto err_free_swp;
	}
	if ((sc->irq_resource = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid[0], RF_ACTIVE | RF_SHAREABLE)) == NULL) {
		device_printf(dev, "Failed to allocate IRQ resource\n");
		goto err_free_swp;
	}
	if (bus_setup_intr(dev, sc->irq_resource, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, msi_intr, sc, &sc->intr)) {
		device_printf(dev, "Failed to setup IRQ resource\n");
		goto err_free_swp;
	}

	return (0);

 err_free_swp:
	dpaa2_swp_free_portal(sc->swp);
 err_free_cmd:
	dpaa2_mcp_free_command(cmd);
 err_exit:
	dpaa2_mc_detach(dev);
	return (ENXIO);
}

static int
dpaa2_io_detach(device_t dev)
{
	return (0);
}

/*
 * QBMan software portal interface.
 */

/**
 * @brief Enable interrupts for a software portal.
 */
static void
dpaa2_io_set_intr_trigger(device_t iodev, uint32_t mask)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(iodev);

	if (sc && dinfo && dinfo->dtype == DPAA2_DEV_IO && sc->swp)
		dpaa2_swp_write_reg(sc->swp, DPAA2_SWP_CINH_IER, mask);
	else
		device_printf(iodev, "%s failed\n", __func__);
}

/**
 * @brief Return the value in the SWP_IER register.
 */
static uint32_t
dpaa2_io_get_intr_trigger(device_t iodev)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(iodev);

	if (sc && dinfo && dinfo->dtype == DPAA2_DEV_IO && sc->swp)
		return dpaa2_swp_read_reg(sc->swp, DPAA2_SWP_CINH_IER);
	else
		device_printf(iodev, "%s failed\n", __func__);

	return (0);
}

/**
 * @brief Return the value in the SWP_ISR register.
 */
static uint32_t
dpaa2_io_read_intr_status(device_t iodev)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(iodev);

	if (sc && dinfo && dinfo->dtype == DPAA2_DEV_IO && sc->swp)
		return dpaa2_swp_read_reg(sc->swp, DPAA2_SWP_CINH_ISR);
	else
		device_printf(iodev, "%s failed\n", __func__);

	return (0);
}

/**
 * @brief Clear SWP_ISR register according to the given mask.
 */
static void
dpaa2_io_clear_intr_status(device_t iodev, uint32_t mask)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(iodev);

	if (sc && dinfo && dinfo->dtype == DPAA2_DEV_IO && sc->swp)
		dpaa2_swp_write_reg(sc->swp, DPAA2_SWP_CINH_ISR, mask);
	else
		device_printf(iodev, "%s failed\n", __func__);
}

/**
 * @brief Enable or disable push dequeue.
 *
 * p:		the software portal object
 * chan_idx:	the channel index (0 to 15)
 * en:		enable or disable push dequeue
 */
static void
dpaa2_io_set_push_dequeue(device_t iodev, uint8_t chan_idx, bool en)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);
	struct dpaa2_devinfo *dinfo = device_get_ivars(iodev);
	uint16_t dqsrc;

	if (sc && dinfo && dinfo->dtype == DPAA2_DEV_IO && sc->swp) {
		if (chan_idx > 15) {
			device_printf(iodev, "%s: channel index should "
			    "be <= 15: chan_idx=%d\n", __func__, chan_idx);
			return;
		}

		if (en)
			sc->swp->sdq |= 1 << chan_idx;
		else
			sc->swp->sdq &= ~(1 << chan_idx);

		/*
		 * Read make the complete src map. If no channels are enabled
		 * the SDQCR must be 0 or else QMan will assert errors.
		 */
		dqsrc = (sc->swp->sdq >> DPAA2_SDQCR_SRC_SHIFT) &
		    DPAA2_SDQCR_SRC_MASK;
		dpaa2_swp_write_reg(sc->swp, DPAA2_SWP_CINH_SDQCR, dqsrc != 0
		    ? sc->swp->sdq : 0);
	} else
		device_printf(iodev, "%s failed\n", __func__);
}

/**
 * @brief Enqueue multiple frames to a frame queue using one fqid.
 */
static int
dpaa2_io_enq_multiple_fq(device_t iodev, uint32_t fqid,
    const dpaa2_fd_t *fd, int frames_n)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);
	dpaa2_eq_desc_t ed;
	uint32_t flags = 0;

	/* Setup enqueue descriptor. */
	dpaa2_swp_clear_ed(&ed);
	dpaa2_swp_set_ed_norp(&ed, 0);
	dpaa2_swp_set_ed_fq(&ed, fqid);

	return (dpaa2_swp_enq_mult(sc->swp, &ed, fd, &flags, frames_n));
}

/*
 * Internal functions.
 */

/**
 * @internal
 * @brief Allocate MSI interrupts for this DPAA2 I/O object.
 */
static int
setup_msi(struct dpaa2_io_softc *sc)
{
	int val;

	val = pci_msi_count(sc->dev);
	if (val < DPAA2_IO_MSI_COUNT)
		device_printf(sc->dev, "Have %d MSI messages\n", val);
	val = MIN(val, DPAA2_IO_MSI_COUNT);

	if (pci_alloc_msi(sc->dev, &val) != 0)
		return (EINVAL);

	for (int i = 0; i < val; i++)
		sc->irq_rid[i] = i + 1;

	return (0);
}

/**
 * @internal
 * @brief DPAA2 I/O interrupt handler.
 */
static void
msi_intr(void *arg)
{
	/* NOTE: Useless interrupt handler. */
	volatile uint32_t val = 0;
	for (uint32_t i = 0; i < 100; i++)
		val++;
}

static device_method_t dpaa2_io_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_io_probe),
	DEVMETHOD(device_attach,	dpaa2_io_attach),
	DEVMETHOD(device_detach,	dpaa2_io_detach),

	/* QBMan software portal interface */
	DEVMETHOD(dpaa2_swp_set_intr_trigger,	dpaa2_io_set_intr_trigger),
	DEVMETHOD(dpaa2_swp_get_intr_trigger,	dpaa2_io_get_intr_trigger),
	DEVMETHOD(dpaa2_swp_read_intr_status,	dpaa2_io_read_intr_status),
	DEVMETHOD(dpaa2_swp_clear_intr_status,	dpaa2_io_clear_intr_status),
	DEVMETHOD(dpaa2_swp_set_push_dequeue,	dpaa2_io_set_push_dequeue),
	DEVMETHOD(dpaa2_swp_enq_multiple_fq,	dpaa2_io_enq_multiple_fq),

	DEVMETHOD_END
};

static driver_t dpaa2_io_driver = {
	"dpaa2_io",
	dpaa2_io_methods,
	sizeof(struct dpaa2_io_softc),
};

static devclass_t dpaa2_io_devclass;

DRIVER_MODULE(dpaa2_io, dpaa2_rc, dpaa2_io_driver, dpaa2_io_devclass, 0, 0);
