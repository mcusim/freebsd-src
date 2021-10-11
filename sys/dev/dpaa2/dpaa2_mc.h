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

#ifndef	_DPAA2_MC_H
#define	_DPAA2_MC_H

#include <sys/rman.h>

#include "pci_if.h"

#include "dpaa2_mcp.h"
#include "dpaa2_swp.h"

#define DPAA2_MCP_MEM_WIDTH	0x40 /* Minimal size of the MC portal. */
#define DPAA2_IO_MSI_COUNT	1

/* MC Registers */
#define MC_REG_GCR1		0x00u
#define GCR1_P1_STOP		0x80000000
#define MC_REG_GSR		0x08u
#define MC_REG_FAPR		0x28u

enum dpaa2_dev_type {
	DPAA2_DEV_MC = 7500,	/* Management Complex (firmware bus) */
	DPAA2_DEV_RC,		/* Resource Container (firmware bus) */
	DPAA2_DEV_IO,		/* I/O object (to work with QBMan portal) */
	DPAA2_DEV_NI,		/* Network Interface */
	DPAA2_DEV_MCP,		/* MC portal (to configure MC portal) */
	DPAA2_DEV_BP,		/* Buffer Pool */
	DPAA2_DEV_CON,		/* Concentrator */

	DPAA2_DEV_NOTYPE	/* Shouldn't be assigned to any DPAA2 device. */
};

/**
 * @brief Software context for the DPAA2 Management Complex (MC) driver.
 *
 * dev:		Device associated with this software context.
 * rcdev:	Child device associated with the root resource container.
 * res:		Unmapped MC command portal and control registers resources.
 * map:		Mapped MC command portal and control registers resources.
 * io_rman:	I/O memory resource manager.
 * msi_rman:	Message-signalled interrupts resource manager.
 * dpni_rman:	Data Path I/O objects resource manager.
 * dpbp_rman:	Data Path Buffer Pools resource manager.
 * dpcon_rman:	Data Path Concentrators resource manager.
 */
struct dpaa2_mc_softc {
	device_t		 dev;
	device_t		 rcdev;

	struct resource 	*res[2];
	struct resource_map	 map[2];

	struct rman		 io_rman;
	struct rman		 msi_rman;
	struct rman		 dpio_rman;
	struct rman		 dpbp_rman;
	struct rman		 dpcon_rman;
};

/**
 * @brief Software context for the DPAA2 Resource Container (RC) driver.
 *
 * dev:		Device associated with this software context.
 * portal:	Helper object to send commands to the MC portal.
 * unit:	Helps to distinguish between root (0) and child DRPCs.
 * cont_id:	Container ID.
 */
struct dpaa2_rc_softc {
	device_t		 dev;
	dpaa2_mcp_t		 portal;
	int			 unit;
	uint32_t		 cont_id;
};

/**
 * @brief Software context for the DPAA2 I/O driver.
 */
struct dpaa2_io_softc {
	device_t		 dev;
	struct resource 	*res[3];
	struct resource_map	 map[3];
	int			 irq_rid[DPAA2_IO_MSI_COUNT];
	struct resource		*irq_resource;
	void			*intr; /* interrupt handle */
	dpaa2_swp_desc_t	 swp_desc;
	dpaa2_swp_t		 swp;
};

/**
 * @brief Software context for the DPAA2 Buffer Pool driver.
 */
struct dpaa2_bp_softc {
	device_t		 dev;
};

/**
 * @brief Software context for the DPAA2 Network Interface driver.
 *
 * dev:		Device associated with this software context.
 * api_major:	Major version of the DPNI API.
 * api_minor:	Minor version of the DPNI API.
 * rx_bufsz:	Size of a buffer to receive frames.
 * tx_data_off: ...
 * attr:	Attributes of the DPNI object.
 * mac:		Details about DPMAC connected to this DPNI object (if exists).
 */
struct dpaa2_ni_softc {
	device_t		 dev;
	struct resource 	*res[3];
	uint16_t		 api_major;
	uint16_t		 api_minor;
	uint16_t		 rx_bufsz;
	uint16_t		 tx_data_off;
	dpaa2_ni_attr_t		 attr;

	struct {
		uint32_t	 dpmac_id;
		uint8_t		 addr[6];
	} mac;
};

/**
 * @brief Information about MSI messages supported by the DPAA2 object.
 *
 * msi_msgnum:	 Number of MSI messages supported by the DPAA2 object.
 * msi_alloc:	 Number of MSI messages allocated for the DPAA2 object.
 * msi_handlers: Number of MSI message handlers configured.
 */
struct dpaa2_msinfo {
	uint8_t			 msi_msgnum;
	uint8_t			 msi_alloc;
	uint32_t		 msi_handlers;
};

/**
 * @brief Information about DPAA2 device.
 *
 * pdev:	Parent device.
 * dev:		Device this devinfo is associated with.
 * id:		ID of a logical DPAA2 object resource.
 * portal_id:	ID of the MC portal which belongs to the object's container.
 * icid:	Isolation context ID of the DPAA2 object. It is shared
 *		between a resource container and all of its children.
 * dtype:	Type of the DPAA2 object.
 * resources:	Resources available for this DPAA2 device.
 * msi:		Information about MSI messages supported by the DPAA2 object.
 */
struct dpaa2_devinfo {
	device_t		 pdev;
	device_t		 dev;
	uint32_t		 id;
	uint32_t		 portal_id;
	uint16_t		 icid;
	enum dpaa2_dev_type	 dtype;
	struct resource_list	 resources;
	struct dpaa2_msinfo	 msi;
};

DECLARE_CLASS(dpaa2_mc_driver);

/* For device interface */
int dpaa2_mc_attach(device_t dev);
int dpaa2_mc_detach(device_t dev);

/* For bus interface */
struct resource * dpaa2_mc_alloc_resource(device_t mcdev, device_t child,
    int type, int *rid, rman_res_t start, rman_res_t end, rman_res_t count,
    u_int flags);
int dpaa2_mc_adjust_resource(device_t mcdev, device_t child, int type,
    struct resource *r, rman_res_t start, rman_res_t end);
int dpaa2_mc_release_resource(device_t mcdev, device_t child, int type,
    int rid, struct resource *r);
int dpaa2_mc_activate_resource(device_t mcdev, device_t child, int type,
    int rid, struct resource *r);
int dpaa2_mc_deactivate_resource(device_t mcdev, device_t child, int type,
    int rid, struct resource *r);

/* For pseudo-pcib interface */
int dpaa2_mc_alloc_msi(device_t mcdev, device_t child, int count, int maxcount,
    int *irqs);
int dpaa2_mc_release_msi(device_t mcdev, device_t child, int count, int *irqs);
int dpaa2_mc_map_msi(device_t mcdev, device_t child, int irq, uint64_t *addr,
    uint32_t *data);
int dpaa2_mc_get_id(device_t mcdev, device_t child, enum pci_id_type type,
    uintptr_t *id);

/* For DPAA2 Management Complex bus driver interface */
int dpaa2_mc_manage_device(device_t mcdev, device_t dpaa2_dev);
int dpaa2_mc_first_free_device(device_t mcdev, device_t *dpaa2_dev,
    enum dpaa2_dev_type devtype);
int dpaa2_mc_last_free_device(device_t mcdev, device_t *dpaa2_dev,
    enum dpaa2_dev_type devtype);

/* Convert DPAA2 type to/from string. */
const char *dpaa2_ttos(enum dpaa2_dev_type type);
enum dpaa2_dev_type dpaa2_stot(const char *str);

#endif /* _DPAA2_MC_H */
