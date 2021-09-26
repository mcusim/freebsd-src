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
#include <dev/dpaa2/dpaa2_mcp.h>

/**
 * @brief DPAA2 MC command interface.
 *
 * The primary purpose of the MC provided DPAA2 objects is to simplify DPAA2
 * hardware block usage through abstraction and encapsulation.
 */
INTERFACE dpaa2_cmd;

#
# Default implementation of the commands.
#
CODE {
	static void
	panic_on_mc(device_t dev)
	{
		struct dpaa2_devinfo *dinfo;

		dinfo = device_get_ivars(dev);
		if (dinfo != NULL && dinfo->dtype == DPAA2_DEV_MC)
			panic("No one can handle a command above DPAA2 MC");
	}

	static int
	bypass_mng_get_version(device_t dev, dpaa2_cmd_t cmd, uint32_t *major,
		uint32_t *minor, uint32_t *rev)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MNG_GET_VERSION(device_get_parent(dev),
				cmd, major, minor, rev));
		return (ENXIO);
	}

	static int
	bypass_mng_get_soc_version(device_t dev, dpaa2_cmd_t cmd, uint32_t *pvr,
		uint32_t *svr)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MNG_GET_SOC_VERSION(
				device_get_parent(dev), cmd, pvr, svr));
		return (ENXIO);
	}

	static int
	bypass_mng_get_container_id(device_t dev, dpaa2_cmd_t cmd,
		uint32_t *cont_id)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MNG_GET_CONTAINER_ID(
				device_get_parent(dev), cmd, cont_id));
		return (ENXIO);
	}

	static int
	bypass_rc_open(device_t dev, dpaa2_cmd_t cmd, uint32_t cont_id,
		uint16_t *token)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_OPEN(
				device_get_parent(dev), cmd, cont_id, token));
		return (ENXIO);
	}

	static int
	bypass_rc_close(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_CLOSE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}

	static int
	bypass_rc_get_obj_count(device_t dev, dpaa2_cmd_t cmd,
		uint32_t *obj_count)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_GET_OBJ_COUNT(
				device_get_parent(dev), cmd, obj_count));
		return (ENXIO);
	}

	static int
	bypass_rc_get_obj(device_t dev, dpaa2_cmd_t cmd, uint32_t obj_idx,
		dpaa2_obj_t *obj)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_GET_OBJ(
				device_get_parent(dev), cmd, obj_idx, obj));
		return (ENXIO);
	}

	static int
	bypass_rc_get_obj_descriptor(device_t dev, dpaa2_cmd_t cmd,
		uint32_t obj_id, const char *type, dpaa2_obj_t *obj)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_GET_OBJ_DESCRIPTOR(
				device_get_parent(dev), cmd, obj_id, type, obj));
		return (ENXIO);
	}

	static int
	bypass_rc_get_attributes(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_rc_attr_t *attr)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_GET_ATTRIBUTES(
				device_get_parent(dev), cmd, attr));
		return (ENXIO);
	}

	static int
	bypass_rc_get_obj_region(device_t dev, dpaa2_cmd_t cmd, uint32_t obj_id,
		uint8_t reg_idx, const char *type, dpaa2_rc_obj_region_t *reg)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_GET_OBJ_REGION(
				device_get_parent(dev), cmd, obj_id, reg_idx,
				type, reg));
		return (ENXIO);
	}

	static int
	bypass_rc_get_api_version(device_t dev, dpaa2_cmd_t cmd,
		uint16_t *major, uint16_t *minor)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_GET_API_VERSION(
				device_get_parent(dev), cmd, major, minor));
		return (ENXIO);
	}

	static int
	bypass_rc_set_irq_enable(device_t dev, dpaa2_cmd_t cmd, uint8_t irq_idx,
		uint8_t enable)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_SET_IRQ_ENABLE(
				device_get_parent(dev), cmd, irq_idx, enable));
		return (ENXIO);
	}

	static int
	bypass_rc_set_obj_irq(device_t dev, dpaa2_cmd_t cmd, uint8_t irq_idx,
		uint64_t addr, uint32_t data, uint32_t irq_usr, uint32_t obj_id,
		const char *type)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_SET_OBJ_IRQ(
				device_get_parent(dev), cmd, irq_idx, addr, data,
				irq_usr, obj_id, type));
		return (ENXIO);
	}
};

/**
 * @brief Data Path Management (DPMNG) commands.
 */

METHOD int mng_get_version {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	*major;
	uint32_t	*minor;
	uint32_t	*rev;
} DEFAULT bypass_mng_get_version;

METHOD int mng_get_soc_version {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	*pvr;
	uint32_t	*svr;
} DEFAULT bypass_mng_get_soc_version;

METHOD int mng_get_container_id {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	*cont_id;
} DEFAULT bypass_mng_get_container_id;

/**
 * @brief Data Path Resource Containter (DPRC) commands.
 */

METHOD int rc_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	 cont_id;
	uint16_t	*token;
} DEFAULT bypass_rc_open;

METHOD int rc_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_rc_close;

METHOD int rc_get_obj_count {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	*obj_count;
} DEFAULT bypass_rc_get_obj_count;

METHOD int rc_get_obj {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	 obj_idx;
	dpaa2_obj_t	*obj;
} DEFAULT bypass_rc_get_obj;

METHOD int rc_get_obj_descriptor {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	 obj_id;
	const char	*type;
	dpaa2_obj_t	*obj;
} DEFAULT bypass_rc_get_obj_descriptor;

METHOD int rc_get_attributes {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_rc_attr_t	*attr;
} DEFAULT bypass_rc_get_attributes;

METHOD int rc_get_obj_region {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint32_t	 obj_id;
	uint8_t		 reg_idx;
	const char	*type;
	dpaa2_rc_obj_region_t *reg;
} DEFAULT bypass_rc_get_obj_region;

METHOD int rc_get_api_version {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint16_t	*major;
	uint16_t	*minor;
} DEFAULT bypass_rc_get_api_version;

METHOD int rc_set_irq_enable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint8_t		 irq_idx;
	uint8_t		 enable;
} DEFAULT bypass_rc_set_irq_enable;

METHOD int rc_set_obj_irq {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint8_t		 irq_idx;
	uint64_t	 addr;
	uint32_t	 data;
	uint32_t	 irq_usr;
	uint32_t	 obj_id;
	const char	*type;
} DEFAULT bypass_rc_set_obj_irq;

/**
 * @brief Data Path Network Interface (DPNI) commands.
 */

METHOD int ni_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpni_id;
	uint16_t	*token;
};

METHOD int ni_close {
	device_t dev;
	dpaa2_cmd_t cmd;
};

/**
 * @brief Data Path I/O (DPIO) commands.
 */

METHOD int io_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpio_id;
	uint16_t	*token;
};

METHOD int io_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int io_enable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int io_disable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int io_reset {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int io_get_attributes {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_io_attr_t	*attr;
};

/**
 * @brief Data Path Buffer Pool (DPBP) commands.
 */

METHOD int bp_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpbp_id;
	uint16_t	*token;
};

METHOD int bp_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int bp_enable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int bp_disable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int bp_reset {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
};

METHOD int bp_get_attributes {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_bp_attr_t	*attr;
};
