/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright © 2024 Dmitry Salychev
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

#ifndef	_MACLINK_H
#define	_MACLINK_H

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/queue.h>

DECLARE_CLASS(maclink_bus_driver);

/**
 * @struct maclink_conf
 * @brief Configuration of the maclink devices.
 */
struct maclink_conf {
	int placeholder;
};

/**
 * @struct maclink_state
 * @brief State of a maclink device.
 */
struct maclink_state {
	int placeholder;
};

/**
 * @struct maclink_link_state
 * @brief Link state of the maclink devices.
 */
struct maclink_link_state {
	int placeholder;
};

/**
 * @struct maclink_devinfo
 * @brief Structure to describe a maclink device.
 */
struct maclink_devinfo {
	SLIST_ENTRY(maclink_devinfo) link;
};

/**
 * @struct maclink_data
 * @brief Interface between the maclink devices.
 */
struct maclink_data {
	device_t pdev;	/**< Parent maclink device */
	device_t dev;	/**< Maclink device this interface is associated with */
	SLIST_HEAD(, maclink_devinfo) mldevs; /**< Registered maclink devices */
	struct mtx lock; /**< Used to protect the whole structure */
};

int maclink_attach(device_t dev, device_t *mlbus);

int maclink_bus_validate(device_t, struct maclink_conf *);
void maclink_bus_statchg(device_t, struct maclink_state *);
void maclink_bus_linkchg(device_t, struct maclink_link_state *);

#endif /* _MACLINK_H */
