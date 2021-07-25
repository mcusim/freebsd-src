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

#define DPAA2_MCP_MEM_WIDTH	0x40 /* Expected minimal size of the portal. */

/* MC Registers */
#define MC_REG_GCR1		0x00u
#define GCR1_P1_STOP		0x80000000
#define MC_REG_GSR		0x08u
#define MC_REG_FAPR		0x28u

enum dpaa2_dev_type {
	DPAA2_DEV_MC = 75,	/* Management Complex (firmware bus) */
	DPAA2_DEV_RC,		/* Resource Container (firmware bus) */
	DPAA2_DEV_IO,		/* I/O object (to work with QBMAN portal) */
	DPAA2_DEV_NI,		/* Network Interface */
	DPAA2_DEV_MCP		/* Management Complex Portal */
};

/*
 * Software context for the DPAA2 Management Complex (MC) driver.
 *
 * dev: Device associated with this software context.
 * rcdev: Child device associated with the root resource container.
 * res: Unmapped MC command portal and control registers resources.
 * map: Mapped MC command portal and control registers resources.
 */
struct dpaa2_mc_softc {
	device_t		 dev;
	device_t		 rcdev;
	struct resource 	*res[2];
	struct resource_map	 map[2];
};

/*
 * Software context for the DPAA2 Resource Container (RC) driver.
 *
 * dev: Device associated with this software context.
 * portal: Helper object to send commands to the MC portal.
 * unit: Helps to distinguish between root (0) and child DRPCs.
 */
struct dpaa2_rc_softc {
	device_t		 dev;
	dpaa2_mcp_t		 portal;
	int			 unit;
};

/*
 * Information about MSI messages supported by the DPAA2 object.
 *
 * msi_msgnum: Number of MSI messages supported by the DPAA2 object.
 * msi_alloc: Number of MSI messages allocated for the DPAA2 object.
 * msi_handlers: Number of MSI message handlers configured.
 */
struct dpaa2_msinfo {
	uint8_t			 msi_msgnum;
	uint8_t			 msi_alloc;
	uint32_t		 msi_handlers;
};

/*
 * Information about DPAA2 device.
 *
 * pdev: Parent device.
 * dev: Devinfo is associated with this device.
 * icid: Isolation context ID of the DPAA2 object. It is shared between a
 *       resource container and all of its children.
 * dtype: Type of the DPAA2 object.
 * resources: Resources allocated for this DPAA2 device.
 * msi: Information about MSI messages supported by the DPAA2 object.
 */
struct dpaa2_devinfo {
	device_t		 pdev;
	device_t		 dev;
	uint16_t		 icid;
	enum dpaa2_dev_type	 dtype;
	struct resource_list	 resources;
	struct dpaa2_msinfo	 msi;
};

DECLARE_CLASS(dpaa2_mc_driver);

/* For device interface */
int dpaa2_mc_attach(device_t dev);
int dpaa2_mc_detach(device_t dev);

/* For pseudo-pcib interface */
int dpaa2_mc_alloc_msi(device_t mcdev, device_t child, int count, int maxcount,
    int *irqs);
int dpaa2_mc_release_msi(device_t mcdev, device_t child, int count, int *irqs);
int dpaa2_mc_map_msi(device_t mcdev, device_t child, int irq, uint64_t *addr,
    uint32_t *data);
int dpaa2_mc_get_id(device_t mcdev, device_t child, enum pci_id_type type,
    uintptr_t *id);


#endif /* _DPAA2_MC_H */
