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
#include <dev/dpaa2/dpaa2_types.h>
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
		if (strcmp(device_get_name(dev), "dpaa2_mc") == 0)
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
		uint32_t obj_id, enum dpaa2_dev_type type, dpaa2_obj_t *obj)
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
		uint8_t reg_idx, enum dpaa2_dev_type type,
		dpaa2_rc_obj_region_t *reg)
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
		enum dpaa2_dev_type type)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_SET_OBJ_IRQ(
				device_get_parent(dev), cmd, irq_idx, addr, data,
				irq_usr, obj_id, type));
		return (ENXIO);
	}
	static int
	bypass_rc_get_conn(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ep_desc_t *ep1_desc, dpaa2_ep_desc_t *ep2_desc,
		uint32_t *link_stat)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_RC_GET_CONN(
				device_get_parent(dev), cmd, ep1_desc, ep2_desc,
				link_stat));
		return (ENXIO);
	}

	static int
	bypass_ni_open(device_t dev, dpaa2_cmd_t cmd, const uint32_t dpni_id,
		uint16_t *token)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_OPEN(
				device_get_parent(dev), cmd, dpni_id, token));
		return (ENXIO);
	}
	static int
	bypass_ni_close(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_CLOSE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_ni_get_api_version(device_t dev, dpaa2_cmd_t cmd,
		uint16_t *major, uint16_t *minor)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_GET_API_VERSION(
				device_get_parent(dev), cmd, major, minor));
		return (ENXIO);
	}
	static int
	bypass_ni_reset(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_RESET(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_ni_get_attributes(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ni_attr_t *attr)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_GET_ATTRIBUTES(
				device_get_parent(dev), cmd, attr));
		return (ENXIO);
	}
	static int
	bypass_ni_set_buf_layout(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ni_buf_layout_t *bl)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_SET_BUF_LAYOUT(
				device_get_parent(dev), cmd, bl));
		return (ENXIO);
	}
	static int
	bypass_ni_get_tx_data_off(device_t dev, dpaa2_cmd_t cmd,
		uint16_t *offset)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_GET_TX_DATA_OFF(
				device_get_parent(dev), cmd, offset));
		return (ENXIO);
	}
	static int
	bypass_ni_set_link_cfg(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ni_link_cfg_t *cfg)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_SET_LINK_CFG(
				device_get_parent(dev), cmd, cfg));
		return (ENXIO);
	}
	static int
	bypass_ni_get_link_cfg(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ni_link_cfg_t *cfg)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_GET_LINK_CFG(
				device_get_parent(dev), cmd, cfg));
		return (ENXIO);
	}
	static int
	bypass_ni_get_port_mac_addr(device_t dev, dpaa2_cmd_t cmd, uint8_t *mac)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_GET_PORT_MAC_ADDR(
				device_get_parent(dev), cmd, mac));
		return (ENXIO);
	}
	static int
	bypass_ni_set_qos_table(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ni_qos_table_t *tbl)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_SET_QOS_TABLE(
				device_get_parent(dev), cmd, tbl));
		return (ENXIO);
	}
	static int
	bypass_ni_clear_qos_table(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_CLEAR_QOS_TABLE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_ni_set_pools(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ni_pools_cfg_t *cfg)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_SET_POOLS(
				device_get_parent(dev), cmd, cfg));
		return (ENXIO);
	}
	static int
	bypass_ni_set_err_behavior(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_ni_err_cfg_t *cfg)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_NI_SET_ERR_BEHAVIOR(
				device_get_parent(dev), cmd, cfg));
		return (ENXIO);
	}

	static int
	bypass_io_open(device_t dev, dpaa2_cmd_t cmd, const uint32_t dpio_id,
		uint16_t *token)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_IO_OPEN(
				device_get_parent(dev), cmd, dpio_id, token));
		return (ENXIO);
	}
	static int
	bypass_io_close(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_IO_CLOSE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_io_enable(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_IO_ENABLE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_io_disable(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_IO_DISABLE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_io_reset(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_IO_RESET(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_io_get_attributes(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_io_attr_t *attr)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_IO_GET_ATTRIBUTES(
				device_get_parent(dev), cmd, attr));
		return (ENXIO);
	}

	static int
	bypass_bp_open(device_t dev, dpaa2_cmd_t cmd, const uint32_t dpbp_id,
		uint16_t *token)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_BP_OPEN(
				device_get_parent(dev), cmd, dpbp_id, token));
		return (ENXIO);
	}
	static int
	bypass_bp_close(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_BP_CLOSE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_bp_enable(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_BP_ENABLE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_bp_disable(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_BP_DISABLE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_bp_reset(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_BP_RESET(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_bp_get_attributes(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_bp_attr_t *attr)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_BP_GET_ATTRIBUTES(
				device_get_parent(dev), cmd, attr));
		return (ENXIO);
	}

	static int
	bypass_mac_open(device_t dev, dpaa2_cmd_t cmd,
		const uint32_t dpmac_id, uint16_t *token)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MAC_OPEN(
				device_get_parent(dev), cmd, dpmac_id, token));
		return (ENXIO);
	}
	static int
	bypass_mac_close(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MAC_CLOSE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_mac_reset(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MAC_RESET(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_mac_mdio_read(device_t dev, dpaa2_cmd_t cmd, uint8_t phy,
		uint16_t reg, uint16_t *val)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MAC_MDIO_READ(
				device_get_parent(dev), cmd, phy, reg, val));
		return (ENXIO);
	}
	static int
	bypass_mac_mdio_write(device_t dev, dpaa2_cmd_t cmd, uint8_t phy,
		uint16_t reg, uint16_t val)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MAC_MDIO_WRITE(
				device_get_parent(dev), cmd, phy, reg, val));
		return (ENXIO);
	}
	static int
	bypass_mac_get_addr(device_t dev, dpaa2_cmd_t cmd, uint8_t *mac)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MAC_GET_ADDR(
				device_get_parent(dev), cmd, mac));
		return (ENXIO);
	}
	static int
	bypass_mac_get_attributes(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_mac_attr_t *attr)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_MAC_GET_ATTRIBUTES(
				device_get_parent(dev), cmd, attr));
		return (ENXIO);
	}

	static int
	bypass_con_open(device_t dev, dpaa2_cmd_t cmd, const uint32_t dpcon_id,
		uint16_t *token)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_CON_OPEN(
				device_get_parent(dev), cmd, dpcon_id, token));
		return (ENXIO);
	}
	static int
	bypass_con_close(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_CON_CLOSE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_con_reset(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_CON_RESET(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_con_enable(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_CON_ENABLE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_con_disable(device_t dev, dpaa2_cmd_t cmd)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_CON_DISABLE(
				device_get_parent(dev), cmd));
		return (ENXIO);
	}
	static int
	bypass_con_get_attributes(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_con_attr_t *attr)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_CON_GET_ATTRIBUTES(
				device_get_parent(dev), cmd, attr));
		return (ENXIO);
	}
	static int
	bypass_con_set_notif(device_t dev, dpaa2_cmd_t cmd,
		dpaa2_con_notif_cfg_t *cfg)
	{
		panic_on_mc(dev);
		if (device_get_parent(dev) != NULL)
			return (DPAA2_CMD_CON_SET_NOTIF(
				device_get_parent(dev), cmd, cfg));
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
	enum dpaa2_dev_type type;
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
	enum dpaa2_dev_type type;
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
	enum dpaa2_dev_type type;
} DEFAULT bypass_rc_set_obj_irq;

METHOD int rc_get_conn {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ep_desc_t *ep1_desc;
	dpaa2_ep_desc_t *ep2_desc;
	uint32_t	*link_stat;
} DEFAULT bypass_rc_get_conn;

/**
 * @brief Data Path Network Interface (DPNI) commands.
 */

METHOD int ni_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpni_id;
	uint16_t	*token;
} DEFAULT bypass_ni_open;

METHOD int ni_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_ni_close;

METHOD int ni_get_api_version {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint16_t	*major;
	uint16_t	*minor;
} DEFAULT bypass_ni_get_api_version;

METHOD int ni_reset {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_ni_reset;

METHOD int ni_get_attributes {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ni_attr_t	*attr;
} DEFAULT bypass_ni_get_attributes;

METHOD int ni_set_buf_layout {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ni_buf_layout_t *bl;
} DEFAULT bypass_ni_set_buf_layout;

METHOD int ni_get_tx_data_off {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint16_t	*offset;
} DEFAULT bypass_ni_get_tx_data_off;

METHOD int ni_set_link_cfg {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ni_link_cfg_t *cfg;
} DEFAULT bypass_ni_set_link_cfg;

METHOD int ni_get_link_cfg {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ni_link_cfg_t *cfg;
} DEFAULT bypass_ni_get_link_cfg;

METHOD int ni_get_port_mac_addr {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint8_t		*mac;
} DEFAULT bypass_ni_get_port_mac_addr;

METHOD int ni_set_qos_table {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ni_qos_table_t *tbl;
} DEFAULT bypass_ni_set_qos_table;

METHOD int ni_clear_qos_table {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_ni_clear_qos_table;

METHOD int ni_set_pools {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ni_pools_cfg_t *cfg;
} DEFAULT bypass_ni_set_pools;

METHOD int ni_set_err_behavior {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_ni_err_cfg_t *cfg;
} DEFAULT bypass_ni_set_err_behavior;

/**
 * @brief Data Path I/O (DPIO) commands.
 */

METHOD int io_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpio_id;
	uint16_t	*token;
} DEFAULT bypass_io_open;

METHOD int io_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_io_close;

METHOD int io_enable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_io_enable;

METHOD int io_disable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_io_disable;

METHOD int io_reset {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_io_reset;

METHOD int io_get_attributes {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_io_attr_t	*attr;
} DEFAULT bypass_io_get_attributes;

/**
 * @brief Data Path Buffer Pool (DPBP) commands.
 */

METHOD int bp_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpbp_id;
	uint16_t	*token;
} DEFAULT bypass_bp_open;

METHOD int bp_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_bp_close;

METHOD int bp_enable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_bp_enable;

METHOD int bp_disable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_bp_disable;

METHOD int bp_reset {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_bp_reset;

METHOD int bp_get_attributes {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	dpaa2_bp_attr_t	*attr;
} DEFAULT bypass_bp_get_attributes;

/**
 * @brief Data Path MAC (DPMAC) commands.
 */

METHOD int mac_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpmac_id;
	uint16_t	*token;
} DEFAULT bypass_mac_open;

METHOD int mac_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_mac_close;

METHOD int mac_reset {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_mac_reset;

METHOD int mac_mdio_read {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint8_t		 phy;
	uint16_t	 reg;
	uint16_t	*val;
} DEFAULT bypass_mac_mdio_read;

METHOD int mac_mdio_write {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint8_t		 phy;
	uint16_t	 reg;
	uint16_t	 val;
} DEFAULT bypass_mac_mdio_write;

METHOD int mac_get_addr {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	uint8_t		*mac;
} DEFAULT bypass_mac_get_addr;

METHOD int mac_get_attributes {
	device_t	  dev;
	dpaa2_cmd_t	  cmd;
	dpaa2_mac_attr_t *attr;
} DEFAULT bypass_mac_get_attributes;

/**
 * @brief Data Path Concentrator (DPCON) commands.
 */

METHOD int con_open {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
	const uint32_t	 dpcon_id;
	uint16_t	*token;
} DEFAULT bypass_con_open;

METHOD int con_close {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_con_close;

METHOD int con_reset {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_con_reset;

METHOD int con_enable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_con_enable;

METHOD int con_disable {
	device_t	 dev;
	dpaa2_cmd_t	 cmd;
} DEFAULT bypass_con_disable;

METHOD int con_get_attributes {
	device_t	  dev;
	dpaa2_cmd_t	  cmd;
	dpaa2_con_attr_t *attr;
} DEFAULT bypass_con_get_attributes;

METHOD int con_set_notif {
	device_t	  dev;
	dpaa2_cmd_t	  cmd;
	dpaa2_con_notif_cfg_t *cfg;
} DEFAULT bypass_con_set_notif;
