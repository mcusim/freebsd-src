#-
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright © 2024 Dmitry Salychev
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#

#include <machine/bus.h>
#include <dev/maclink/maclink.h>

/**
 * @brief Interface of a maclink bus. It is supposed to attach a maclink
 *        adapter which acts as an observer of the indirectly related maclink
 *        devices (SFP, PCS, PHYs and so on).
 */
INTERFACE maclink_bus;

/**
 * @brief Requests the maclink bus to validate a configuration suggested by
 *        the maclink device.
 *
 * The bus itself can provide a hint about the supported options via the
 * suggested configuration back to the device in case of a validation error.
 * It is recommended to save the correctly validated options in the bus driver
 * softc till the next validation request.
 *
 * @param bus[in]      The maclink bus device.
 * @param conf[in,out] Configuration options to validate.
 *
 * @return 0 on success or a standard errno value.
 */
METHOD int validate {
	device_t		 bus;
	struct maclink_conf	*conf;
};

/**
 * @brief Notifies the maclink bus about the state change.
 *
 * The new state (e.g. PCS sync, SFP module present, PHY auto-neg, etc.) doesn't
 * always mean a link up/down event and is merely needed to let the maclink bus
 * know that something is going on.
 *
 * @param bus[in]   The maclink bus device.
 * @param state[in] New state provided by the maclink device.
 *
 * @return void.
 */
METHOD void statchg {
	device_t		 bus;
	struct maclink_state	*state;
};

/**
 * @brief Notifies the maclink bus about the link state change.
 *
 * There are several devices which participate in the link up sequence usually.
 * The maclink bus will be notified by the last device in the sequence which
 * caused a link up event, or by the first device which caused a link down
 * event. The new link state will be provided by the state argument.
 *
 * @param bus[in]   The maclink bus device.
 * @param state[in] New link state.
 *
 * @return void.
 */
METHOD void linkchg {
	device_t		   bus;
	struct maclink_link_state *state;
};
