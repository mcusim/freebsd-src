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
#include <sys/kobj.h>

struct maclink_conf {
};

struct maclink_state {
};

struct maclink_link_state {
};


/**
 * @brief Information about a maclink bus device.
 */
struct maclink_bus_ivars {
};

/**
 * @brief Software context for the maclink bus.
 */
struct maclink_bus_softc {
};


/**
 * @brief Used to attach a maclink adapter to the bus.
 */
struct maclink_ivars {
};

/**
 * @brief Each maclink adapter driver's softc has one of these as the first
 *        member.
 */
struct maclink_softc {
};

DECLARE_CLASS(maclink_bus_driver);
DECLARE_CLASS(maclink_phy_driver);
DECLARE_CLASS(maclink_sfp_driver);

int maclink_attach(device_t, device_t *);

#endif /* _MACLINK_H */
