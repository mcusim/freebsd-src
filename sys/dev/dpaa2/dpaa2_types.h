/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright © 2021-2024 Dmitry Salychev
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

#ifndef	_DPAA2_TYPES_H
#define	_DPAA2_TYPES_H

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/types.h>
#include <sys/rman.h>
#include <sys/bus.h>

#include <machine/atomic.h>
#include <machine/bus.h>

#define DPAA2_MAGIC	((uint32_t) 0xD4AA2C0Du)

#define DPAA2_MAX_CHANNELS	 16 /* CPU cores */
#define DPAA2_MAX_TCS		 8  /* Traffic classes */

/**
 * @brief Types of the DPAA2 devices.
 */
enum dpaa2_dev_type {
	DPAA2_DEV_MC = 7500,	/* Management Complex (firmware bus) */
	DPAA2_DEV_RC,		/* Resource Container (firmware bus) */
	DPAA2_DEV_IO,		/* I/O object (to work with QBMan portal) */
	DPAA2_DEV_NI,		/* Network Interface */
	DPAA2_DEV_MCP,		/* MC portal */
	DPAA2_DEV_BP,		/* Buffer Pool */
	DPAA2_DEV_CON,		/* Concentrator */
	DPAA2_DEV_MAC,		/* MAC object */
	DPAA2_DEV_MUX,		/* MUX (Datacenter bridge) object */
	DPAA2_DEV_SW,		/* Ethernet Switch */

	DPAA2_DEV_NOTYPE	/* Shouldn't be assigned to any DPAA2 device. */
};

/**
 * @brief Types of the DPNI queues.
 */
enum dpaa2_ni_queue_type {
	DPAA2_NI_QUEUE_RX = 0,
	DPAA2_NI_QUEUE_TX,
	DPAA2_NI_QUEUE_TX_CONF,
	DPAA2_NI_QUEUE_RX_ERR
};

struct dpaa2_atomic {
	volatile int counter;
};

/**
 * @brief Tx ring.
 *
 * fq:		Parent (TxConf) frame queue.
 * fqid:	ID of the logical Tx queue.
 * br:		Ring buffer for mbufs to transmit.
 * lock:	Lock for the ring buffer.
 */
struct dpaa2_ni_tx_ring {
	struct dpaa2_ni_fq	*fq;
	uint32_t		 fqid;
	uint32_t		 txid; /* Tx ring index */

	struct buf_ring		*br;
	struct mtx		 lock;
} __aligned(CACHE_LINE_SIZE);

/**
 * @brief Frame Queue is the basic queuing structure used by the QMan.
 *
 * It comprises a list of frame descriptors (FDs), so it can be thought of
 * as a queue of frames.
 *
 * NOTE: When frames on a FQ are ready to be processed, the FQ is enqueued
 *	 onto a work queue (WQ).
 *
 * fqid:	Frame queue ID, can be used to enqueue/dequeue or execute other
 *		commands on the queue through DPIO.
 * txq_n:	Number of configured Tx queues.
 * tx_fqid:	Frame queue IDs of the Tx queues which belong to the same flowid.
 *		Note that Tx queues are logical queues and not all management
 *		commands are available on these queue types.
 * qdbin:	Queue destination bin. Can be used with the DPIO enqueue
 *		operation based on QDID, QDBIN and QPRI. Note that all Tx queues
 *		with the same flowid have the same destination bin.
 */
struct dpaa2_ni_fq {
	struct dpaa2_channel	*chan;
	uint32_t		 fqid;
	uint16_t		 flowid;
	uint8_t			 tc;
	enum dpaa2_ni_queue_type type;

	/* Optional fields (for TxConf queue). */
	struct dpaa2_ni_tx_ring	 tx_rings[DPAA2_MAX_TCS];
	uint32_t		 tx_qdbin;
} __aligned(CACHE_LINE_SIZE);

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
 *
 * id:		ID of a logical DPAA2 object resource.
 * portal_id:	ID of the MC portal which belongs to the object's container.
 * icid:	Isolation context ID of the DPAA2 object. It is shared
 *		between a resource container and all of its children.
 *
 * dtype:	Type of the DPAA2 object.
 * resources:	Resources available for this DPAA2 device.
 * msi:		Information about MSI messages supported by the DPAA2 object.
 */
struct dpaa2_devinfo {
	device_t		 pdev;
	device_t		 dev;

	uint32_t		 id;
	uint32_t		 portal_id;
	uint32_t		 icid;

	enum dpaa2_dev_type	 dtype;
	struct resource_list	 resources;
	struct dpaa2_msinfo	 msi;

	/*
	 * DPAA2 object might or might not have its own portal allocated to
	 * execute MC commands. If the portal has been allocated, it takes
	 * precedence over the portal owned by the resource container.
	 */
	struct dpaa2_mcp	*portal;
};

/* Handy wrappers over atomic operations. */
#define DPAA2_ATOMIC_XCHG(a, val) \
	(atomic_swap_int(&(a)->counter, (val)))
#define DPAA2_ATOMIC_READ(a) \
	(atomic_load_acq_int(&(a)->counter))
#define DPAA2_ATOMIC_ADD(a, val) \
	(atomic_add_acq_int(&(a)->counter, (val)))

const char *dpaa2_ttos(enum dpaa2_dev_type);
enum dpaa2_dev_type dpaa2_stot(const char *);
void dpaa2_dmamap_oneseg_cb(void *, bus_dma_segment_t *, int, int);

#endif /* _DPAA2_TYPES_H */
