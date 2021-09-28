#-
# SPDX-License-Identifier: BSD-2-Clause-FreeBSD
#
# Copyright (c) 2021 Dmitry Salychev <dsl@mcusim.org>
# All rights reserved.
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
#include <dev/dpaa2/dpaa2_mc.h>
#include <dev/dpaa2/dpaa2_swp.h>

/**
 * @brief QBMan software portal interface.
 *
 * Software portals are used by data path software executing on a processor core
 * to communicate with the Queue Manager (QMan) which acts as a central resource
 * in DPAA2, managing the queueing of data between multiple processor cores,
 * network interfaces, and hardware accelerators in a multicore SoC. These
 * portals are memory mapped in the system.
 */
INTERFACE dpaa2_swp;

METHOD void set_intr_trigger {
	device_t dev;
	uint32_t mask;
};

METHOD uint32_t get_intr_trigger {
	device_t dev;
};

METHOD uint32_t read_intr_status {
	device_t dev;
};

METHOD void clear_intr_status {
	device_t dev;
	uint32_t mask;
};

METHOD void set_push_dequeue {
	device_t dev;
	uint8_t chan_idx;
	bool en;
};
