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
#include "dpaa2_io.h"

#define DPIO_IRQ_INDEX		0 /* index of the only DPIO IRQ */
#define DPIO_POLL_MAX		32

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

static int setup_dpio_irqs(device_t dev);
static int setup_msi(struct dpaa2_io_softc *sc);

/* ISRs */

static void dpio_msi_intr(void *arg);

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
dpaa2_io_detach(device_t dev)
{
	struct dpaa2_io_softc *sc = device_get_softc(dev);

	DPAA2_CMD_IO_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->io_token));
	DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->rc_token));
	dpaa2_mcp_free_command(sc->cmd);

	sc->cmd = NULL;
	sc->io_token = 0;
	sc->rc_token = 0;

	return (0);
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
	error = dpaa2_mcp_init_command(&sc->cmd, DPAA2_CMD_DEF);
	if (error) {
		device_printf(dev, "Failed to allocate dpaa2_cmd: error=%d\n",
		    error);
		goto err_exit;
	}

	/* Open resource container and DPIO object. */
	error = DPAA2_CMD_RC_OPEN(dev, sc->cmd, rcinfo->id, &sc->rc_token);
	if (error) {
		device_printf(dev, "Failed to open DPRC: error=%d\n", error);
		goto err_free_cmd;
	}
	error = DPAA2_CMD_IO_OPEN(dev, sc->cmd, dinfo->id, &sc->io_token);
	if (error) {
		device_printf(dev, "Failed to open DPIO: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_rc;
	}

	/* Prepare DPIO object. */
	error = DPAA2_CMD_IO_RESET(dev, sc->cmd);
	if (error) {
		device_printf(dev, "Failed to reset DPIO: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_io;
	}
	error = DPAA2_CMD_IO_GET_ATTRIBUTES(dev, sc->cmd, &sc->attr);
	if (error) {
		device_printf(dev, "Failed to get DPIO attributes: id=%d, "
		    "error=%d\n", dinfo->id, error);
		goto err_close_io;
	}
	error = DPAA2_CMD_IO_ENABLE(dev, sc->cmd);
	if (error) {
		device_printf(dev, "Failed to enable DPIO: id=%d, error=%d\n",
		    dinfo->id, error);
		goto err_close_io;
	}

	/* Prepare helper object to work with the QBMan software portal. */
	sc->swp_desc.dpio_dev = dev;
	sc->swp_desc.swp_version = sc->attr.swp_version;
	sc->swp_desc.swp_clk = sc->attr.swp_clk;
	sc->swp_desc.swp_id = sc->attr.swp_id;
	sc->swp_desc.has_notif = sc->attr.priors_num ? true : false;
	sc->swp_desc.has_8prio = sc->attr.priors_num == 8u ? true : false;
	sc->swp_desc.cena_res = sc->res[0];
	sc->swp_desc.cena_map = &sc->map[0];
	sc->swp_desc.cinh_res = sc->res[1];
	sc->swp_desc.cinh_map = &sc->map[1];
	/*
	 * Compute how many 256 QBMAN cycles fit into one ns. This is because
	 * the interrupt timeout period register needs to be specified in QBMAN
	 * clock cycles in increments of 256.
	 */
	sc->swp_desc.swp_cycles_ratio = 256000 /
	    (sc->swp_desc.swp_clk / 1000000);

	/* Initialize helper object to work with the QBMan software portal. */
	error = dpaa2_swp_init_portal(&sc->swp, &sc->swp_desc, DPAA2_SWP_DEF,
	    true);
	if (error) {
		device_printf(dev, "Failed to initialize dpaa2_swp: error=%d\n",
		    error);
		goto err_close_io;
	}

	error = setup_dpio_irqs(dev);
	if (error) {
		device_printf(dev, "Failed to setup IRQs: error=%d\n", error);
		goto err_free_swp;
	}

	/* TODO: Enable debug output via sysctl (to reduce output). */
	if (bootverbose)
		device_printf(dev, "dpio_id=%d, swp_id=%d, chan_mode=%s, "
		    "notif_priors=%d, swp_version=0x%x\n",
		    sc->attr.id, sc->attr.swp_id,
		    sc->attr.chan_mode == DPAA2_IO_LOCAL_CHANNEL
		    ? "local_channel" : "no_channel", sc->attr.priors_num,
		    sc->attr.swp_version);

	return (0);

err_free_swp:
	dpaa2_swp_free_portal(sc->swp);
err_close_io:
	DPAA2_CMD_IO_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->io_token));
err_close_rc:
	DPAA2_CMD_RC_CLOSE(dev, dpaa2_mcp_tk(sc->cmd, sc->rc_token));
err_free_cmd:
	dpaa2_mcp_free_command(sc->cmd);
err_exit:
	return (ENXIO);
}

/*
 * QBMan software portal interface.
 */

/**
 * @brief Enqueue multiple frames to a frame queue using one FQID.
 */
static int
dpaa2_io_enq_multiple_fq(device_t iodev, uint32_t fqid,
    struct dpaa2_fd *fd, int frames_n)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);
	struct dpaa2_swp *swp = sc->swp;
	struct dpaa2_eq_desc ed;
	uint32_t flags = 0;

	memset(&ed, 0, sizeof(ed));

	/* Setup enqueue descriptor. */
	dpaa2_swp_set_ed_norp(&ed, false);
	dpaa2_swp_set_ed_fq(&ed, fqid);

	return (swp->enq_mult(swp, &ed, fd, &flags, frames_n));
}

/**
 * @brief Configure the channel data availability notification (CDAN)
 * in a particular WQ channel paired with DPIO.
 */
static int
dpaa2_io_conf_wq_channel(device_t iodev, struct dpaa2_io_notif_ctx *ctx)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);

	/* Enable generation of the CDAN notifications. */
	if (ctx->cdan_en)
		return (dpaa2_swp_conf_wq_channel(sc->swp, ctx->fq_chan_id,
		    DPAA2_WQCHAN_WE_EN | DPAA2_WQCHAN_WE_CTX, ctx->cdan_en,
		    ctx->qman_ctx));

	return (0);
}

/**
 * @brief Release one or more buffer pointers to a QBMan buffer pool.
 */
static int
dpaa2_io_release_bufs(device_t iodev, uint16_t bpid, bus_addr_t *buf,
    uint32_t buf_num)
{
	struct dpaa2_io_softc *sc = device_get_softc(iodev);

	return (dpaa2_swp_release_bufs(sc->swp, bpid, buf, buf_num));
}

/*
 * Internal functions.
 */

/**
 * @internal
 * @brief Configure DPNI object to generate interrupts.
 */
static int
setup_dpio_irqs(device_t dev)
{
	struct dpaa2_io_softc *sc = device_get_softc(dev);
	int error;

	/*
	 * Setup interrupts generated by the software portal.
	 */
	dpaa2_swp_set_intr_trigger(sc->swp, DPAA2_SWP_INTR_DQRI);
	dpaa2_swp_clear_intr_status(sc->swp, 0xFFFFFFFFu);

	/* Configure IRQs. */
	error = setup_msi(sc);
	if (error) {
		device_printf(dev, "Failed to allocate MSI: error=%d\n", error);
		return (error);
	}
	if ((sc->irq_resource = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid[0], RF_ACTIVE | RF_SHAREABLE)) == NULL) {
		device_printf(dev, "Failed to allocate IRQ resource\n");
		return (ENXIO);
	}
	if (bus_setup_intr(dev, sc->irq_resource, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, dpio_msi_intr, sc, &sc->intr)) {
		device_printf(dev, "Failed to setup IRQ resource\n");
		return (ENXIO);
	}

	/*
	 * Setup and enable Static Dequeue Command to receive CDANs from
	 * channel 0.
	 */
	if (sc->swp_desc.has_notif)
		dpaa2_swp_set_push_dequeue(sc->swp, 0, true);

	return (0);
}

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
		device_printf(sc->dev, "MSI: actual=%d, expected=%d\n", val,
		    DPAA2_IO_MSI_COUNT);
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
dpio_msi_intr(void *arg)
{
	struct dpaa2_io_softc *sc = (struct dpaa2_io_softc *) arg;
	struct dpaa2_io_notif_ctx *ctx[DPIO_POLL_MAX];
	struct dpaa2_dq dq;
	uint32_t idx;
	uint16_t flags;
	int error, cdan_n = 0;

	dpaa2_swp_lock(sc->swp, &flags);
	if (flags & DPAA2_SWP_DESTROYED) {
		/* Terminate operation if portal is destroyed. */
		dpaa2_swp_unlock(sc->swp);
		return;
	}
	for (int i = 0; i < DPIO_POLL_MAX; i++) {
		error = dpaa2_swp_dqrr_next_locked(sc->swp, &dq, &idx);
		if (error)
			break;

		if ((dq.common.verb & DPAA2_DQRR_RESULT_MASK) ==
		    DPAA2_DQRR_RESULT_CDAN)
			ctx[cdan_n++] = (struct dpaa2_io_notif_ctx *) dq.scn.ctx;
		else
			device_printf(sc->dev, "unknown DQRR entry\n");

		dpaa2_swp_write_reg(sc->swp, DPAA2_SWP_CINH_DCAP, idx & 0x7u);
	}
	dpaa2_swp_unlock(sc->swp);

	/* Enqueue notification tasks. */
	for (int i = 0; i < cdan_n; i++)
		taskqueue_enqueue(ctx[i]->tq, ctx[i]->notif_task);

	/* Enable software portal interrupts back */
	dpaa2_swp_clear_intr_status(sc->swp, 0xFFFFFFFFu);
	dpaa2_swp_write_reg(sc->swp, DPAA2_SWP_CINH_IIR, 0);
}

static device_method_t dpaa2_io_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dpaa2_io_probe),
	DEVMETHOD(device_attach,	dpaa2_io_attach),
	DEVMETHOD(device_detach,	dpaa2_io_detach),

	/* QBMan software portal interface */
	DEVMETHOD(dpaa2_swp_enq_multiple_fq,	dpaa2_io_enq_multiple_fq),
	DEVMETHOD(dpaa2_swp_conf_wq_channel,	dpaa2_io_conf_wq_channel),
	DEVMETHOD(dpaa2_swp_release_bufs,	dpaa2_io_release_bufs),

	DEVMETHOD_END
};

static driver_t dpaa2_io_driver = {
	"dpaa2_io",
	dpaa2_io_methods,
	sizeof(struct dpaa2_io_softc),
};

static devclass_t dpaa2_io_devclass;

DRIVER_MODULE(dpaa2_io, dpaa2_rc, dpaa2_io_driver, dpaa2_io_devclass, 0, 0);
