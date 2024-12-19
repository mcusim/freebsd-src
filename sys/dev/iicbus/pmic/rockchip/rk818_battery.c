/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Toby Kurien <toby@tobykurien.com>
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

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/kthread.h>
#include <sys/rwlock.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/iicbus/pmic/rockchip/rk8xx.h>

#include "rk818_battery.h"

static struct rwlock my_rwlock;
static volatile int stop_thread = 0; // Flag to stop the thread

static struct battery_info 
{
	device_t pmic_dev;
	int count;					// Number of batteries
	int charge_level;	// Battery charge level (0-100)
	int millivolts;		// Battery voltage in millivolts
	int rate;						// Discharge rate in milliwatts
	int status;					// status: 1 discharging, 2 charging, 4 critically low
} battery_info;

// kernel thread to poll battery status
static void 
rk818_battery_thread(void *arg) 
{
		int err, current, millivolts, vcalib0, vcalib1, volt_k, volt_b;
		device_t dev;
		uint8_t data0, data1, data2, data3;

    while (!stop_thread) {
				dev = battery_info.pmic_dev;

				// get latest battery millivolts
				err = rk8xx_read(dev, BAT_VOL_REGL, &data0, 1);
				if (err) goto out;
				err = rk8xx_read(dev, BAT_VOL_REGH, &data1, 1);
				if (err) goto out;
				millivolts = (int)data0 | ((int)data1 << 8);

				// calibrate voltage
				err = rk8xx_read(dev, VCALIB0_REGL, &data0, 1);
				if (err) goto out;
				err = rk8xx_read(dev, VCALIB0_REGH, &data1, 1);
				if (err) goto out;
				err = rk8xx_read(dev, VCALIB1_REGL, &data2, 1);
				if (err) goto out;
				err = rk8xx_read(dev, VCALIB1_REGH, &data3, 1);
				if (err) goto out;
				vcalib0 = (int)data0 | ((int)data1 << 8);
				vcalib1 = (int)data2 | ((int)data3 << 8);
				volt_k = (4200 - 3000)/(vcalib1 - vcalib0);
				volt_b = 4200 - (volt_k * vcalib1);
				millivolts = volt_k * millivolts + volt_b;

				rw_wlock(&my_rwlock);
				battery_info.millivolts = millivolts;
				rw_wunlock(&my_rwlock);

				// get battery current
				err = rk8xx_read(dev, BAT_CUR_AVG_REGH, &data1, 1);
				if (err) goto out;
				err = rk8xx_read(dev, BAT_CUR_AVG_REGL, &data2, 1);
				if (err) goto out;
        current = (int)data2 | ((int)data1 << 8);
        if (current & 0x800) current -= 4096;
        current = current * 1056 / 1000;

				// get the battery status
				err = rk8xx_read(dev, SUP_STS_REG, &data0, 1);
				if (err) goto out;
        data3 = (data0 & 0x70) >> 4; // charge status

				rw_wlock(&my_rwlock);
				battery_info.count = (data0 & 0x80) >> 7;
        switch (data3) {
          case 1: // Dead charge
				    battery_info.status = 4;
            break;
          case 2: // Trickle charge
          case 3: // CC-CV charging
          case 4: // Charge termination
				    battery_info.status = 1;
            break;
          default: 
            // discharging
				    battery_info.status = 0;
        }
				battery_info.rate = current;
				rw_wunlock(&my_rwlock);

        // get latest battery capacity
				//err = rk8xx_read(dev, GASCNT_REG0, &data0, 1);
				//if (err) goto out;
				//err = rk8xx_read(dev, GASCNT_REG1, &data1, 1);
				//if (err) goto out;
				//err = rk8xx_read(dev, GASCNT_REG2, &data2, 1);
				//if (err) goto out;
				//err = rk8xx_read(dev, GASCNT_REG3, &data3, 1);
				//if (err) goto out;

				// estimate life based on voltage because RK818 does not document how to do this,
				// and linux driver looks really complicated. So we will use a CRUDE approximation.
				if (battery_info.status == 1) {
					// charger on
					vcalib0 = (millivolts - 3400) * 100 / (4300 - 3400);
				} else {
					// discharging, so voltage will sag
					vcalib0 = (millivolts - 3100) * 100 / (4000 - 3100);
				}

				if (vcalib0 > 100) vcalib0 = 100;
        if (vcalib0 < 0) vcalib0 = 0;

				rw_wlock(&my_rwlock);
        //battery_info.charge_level = (int)data0 | ((int)data1 << 8) | ((int)data2 << 16) | ((int)data3 << 24);
				battery_info.charge_level = vcalib0;
				rw_wunlock(&my_rwlock);


				goto next;

out:
				device_printf(dev, "error reading from RK818");

next:
        // Sleep for 5 seconds
        tsleep(&my_rwlock, PWAIT, "rk818_battery_thread", 5 * hz); // 5 seconds
    }
		rw_destroy(&my_rwlock);
}

// Sysctl handler for battery charge level
static int
sysctl_battery_charge_level(SYSCTL_HANDLER_ARGS) 
{
		int data;

		rw_rlock(&my_rwlock);
		data = battery_info.charge_level;
		rw_runlock(&my_rwlock);

		// Handle the sysctl request
    return sysctl_handle_int(oidp, &data, 0, req);
}

// Sysctl handler for battery volts
static int
sysctl_battery_millivolts(SYSCTL_HANDLER_ARGS) 
{
		int data;

		rw_rlock(&my_rwlock);
		data = battery_info.millivolts;
		rw_runlock(&my_rwlock);

    return sysctl_handle_int(oidp, &data, 0, req);
}

// Sysctl handler for battery count
static int
sysctl_battery_count(SYSCTL_HANDLER_ARGS) 
{
		int data;

		rw_rlock(&my_rwlock);
		data = battery_info.count;
		rw_runlock(&my_rwlock);

    return sysctl_handle_int(oidp, &data, 0, req);
}

// Sysctl handler for battery status
static int
sysctl_battery_status(SYSCTL_HANDLER_ARGS) 
{
		int data;

		rw_rlock(&my_rwlock);
		data = battery_info.status;
		rw_runlock(&my_rwlock);

    return sysctl_handle_int(oidp, &data, 0, req);
}

// Sysctl handler for battery discharge rate
static int
sysctl_battery_rate(SYSCTL_HANDLER_ARGS) 
{
		int data;

		rw_rlock(&my_rwlock);
		data = battery_info.rate;
		rw_runlock(&my_rwlock);

    return sysctl_handle_int(oidp, &data, 0, req);
}

static int
rk818_charger_init(device_t dev, phandle_t bnode)
{
  int err, max_charge_v, max_charge_i, charge_terminate_i;
	uint8_t data;
  uint32_t data32;

  if (bnode == 0) {
    // No device tree info, apply conservative defaults
    data = 0xB2; // defaulting to 4.2V 1.5A max charge
    err = rk8xx_write(dev, CHRG_CTRL_REG1, &data, 1);
    data = 0x4A; // 150mA termination current, default timeouts for trickle and CC/CV
    err = rk8xx_write(dev, CHRG_CTRL_REG2, &data, 1);
  } else {
    // Extract charging values from battery node
    OF_getencprop(bnode, "voltage-max-design-microvolt", &data32, sizeof(data32));
    if (bootverbose) device_printf(dev, "Max voltage %d microvolts\n", data32);
    OF_getencprop(bnode, "charge-full-design-microamp-hours", &data32, sizeof(data32));
    if (bootverbose) device_printf(dev, "Battery capacity %d microamp-hours\n", data32);
    OF_getencprop(bnode, "constant-charge-current-max-microamp", &max_charge_i, sizeof(max_charge_i));
    if (bootverbose) device_printf(dev, "Max charge current %d microamp\n", max_charge_i);
    OF_getencprop(bnode, "constant-charge-voltage-max-microvolt", &max_charge_v, sizeof(max_charge_v));
    if (bootverbose) device_printf(dev, "Max charge voltage %d microvolt\n", max_charge_v);
    OF_getencprop(bnode, "charge-term-current-microamp", &charge_terminate_i, sizeof(charge_terminate_i));
    if (bootverbose) device_printf(dev, "Charge termination current %d microamp\n", charge_terminate_i);

    data = 0x80; // enable charging
    
    // set max charging voltage
    if (max_charge_v <= 4050000) {
      data = data | (0x0 << 4);
    } else if (max_charge_v <= 4100000) {
      data = data | (0x1 << 4);
    } else if (max_charge_v <= 4150000) {
      data = data | (0x2 << 4);
    } else if (max_charge_v <= 4200000) {
      data = data | (0x3 << 4);
    } else if (max_charge_v <= 4250000) {
      data = data | (0x4 << 4);
    } else if (max_charge_v <= 4300000) {
      data = data | (0x5 << 4);
    } else {
      data = data | (0x6 << 4);
    }

    // set max charging current
    if (max_charge_i <= 1000000) {
      data = data | 0x0;
    } else if (max_charge_i <= 1200000) {
      data = data | 0x1;
    } else if (max_charge_i <= 1400000) {
      data = data | 0x2;
    } else if (max_charge_i <= 1600000) {
      data = data | 0x3;
    } else if (max_charge_i <= 1800000) {
      data = data | 0x4;
    } else if (max_charge_i <= 2000000) {
      data = data | 0x5;
    } else if (max_charge_i <= 2200000) {
      data = data | 0x6;
    } else if (max_charge_i <= 2400000) {
      data = data | 0x7;
    } else if (max_charge_i <= 2600000) {
      data = data | 0x8;
    } else if (max_charge_i <= 2800000) {
      data = data | 0x9;
    } else {
      data = data | 0xA;
    }
    err = rk8xx_write(dev, CHRG_CTRL_REG1, &data, 1);

    data = 0xA; 
    if (charge_terminate_i <= 100000) {
      data = data | (0x0 << 6);
    } else if (charge_terminate_i <= 150000) {
      data = data | (0x1 << 6);
    } else if (charge_terminate_i <= 200000) {
      data = data | (0x2 << 6);
    } else {
      data = data | (0x3 << 6);
    }
    err = rk8xx_write(dev, CHRG_CTRL_REG2, &data, 1);

    // write design capacity of battery
    if (data32 > 0) {
    	data = (data32 >> 24) & 0xff;
    	err = rk8xx_write(dev, GASCNT_CAL_REG3, &data, 1);
    	data = (data32 >> 16) & 0xff;
    	err = rk8xx_write(dev, GASCNT_CAL_REG2, &data, 1);
    	data = (data32 >> 8) & 0xff;
    	err = rk8xx_write(dev, GASCNT_CAL_REG1, &data, 1);
    	data = (data32 & 0xff);
    	err = rk8xx_write(dev, GASCNT_CAL_REG0, &data, 1);
    	if (err) goto err;

      // TODO - set up fuel gauge registers (not well documented)
    }
  }

  // set up USB current limit
  data = 0x4B; // Defaulting to 3A USB current
  err = rk8xx_write(dev, USB_CTRL_REG, &data, 1);
  if (err) goto err;

  // enable charging
  data = 0x0E; // Enable CC/CV charge timeout, enable trickle charge timeout
  err = rk8xx_write(dev, CHRG_CTRL_REG3, &data, 1);
  if (err) goto err;

  goto done;

err:
  device_printf(dev, "Error initializing charger (I2C returned error)\n");
	return 1;

done:
	return 0;
}

void
rk818_battery_attach(device_t pmic_dev)
{
  phandle_t rnode, tmpnode, bnode;
	int len;
	int data32;
  struct sysctl_oid *sysctl_node;

  rnode = ofw_bus_find_child(ofw_bus_get_node(pmic_dev), "charger");
  if (rnode == 0) {
		device_printf(pmic_dev, "Charger node not found, skipping battery driver\n");
		return;
	}

	// initialize
	battery_info.pmic_dev = pmic_dev;
	len = OF_getencprop(rnode, "monitored-battery", &tmpnode, sizeof(tmpnode));
	if (len != sizeof(tmpnode)) {
		device_printf(pmic_dev, "Charger node with monitored-battery property not found, skipping battery driver\n");
		return;
	}

  bnode = OF_node_from_xref(tmpnode);
  if (rk818_charger_init(pmic_dev, bnode) != 0) return;

	// Create a sysctl node for battery information
	sysctl_node = SYSCTL_ADD_NODE(NULL, SYSCTL_STATIC_CHILDREN(_hw), OID_AUTO,
																 "acpi", CTLFLAG_RD, 0, "ACPI Information");
	sysctl_node = SYSCTL_ADD_NODE(NULL, SYSCTL_CHILDREN(sysctl_node), OID_AUTO,
																 "battery", CTLFLAG_RD, 0, "Battery Information");
	SYSCTL_ADD_PROC(NULL, SYSCTL_CHILDREN(sysctl_node), OID_AUTO, "units",
									CTLTYPE_INT | CTLFLAG_RD, NULL, 0, sysctl_battery_count, "I", "Number of batteries");
	SYSCTL_ADD_PROC(NULL, SYSCTL_CHILDREN(sysctl_node), OID_AUTO, "life",
									CTLTYPE_INT | CTLFLAG_RD, NULL, 0, sysctl_battery_charge_level, "I", "Battery charge level");
	SYSCTL_ADD_PROC(NULL, SYSCTL_CHILDREN(sysctl_node), OID_AUTO, "status",
									CTLTYPE_INT | CTLFLAG_RD, NULL, 0, sysctl_battery_status, "I", "Battery status: 1 discharging, 2 charging, 4 low");
	SYSCTL_ADD_PROC(NULL, SYSCTL_CHILDREN(sysctl_node), OID_AUTO, "rate",
									CTLTYPE_INT | CTLFLAG_RD, NULL, 0, sysctl_battery_rate, "I", "Current battery discharging rate in millwatts");
	SYSCTL_ADD_PROC(NULL, SYSCTL_CHILDREN(sysctl_node), OID_AUTO, "millivolts",
									CTLTYPE_INT | CTLFLAG_RD, NULL, 0, sysctl_battery_millivolts, "I", "Current battery voltage in millivolts");

	// Start the worker thread
	stop_thread = 0;
	rw_init(&my_rwlock, "rk818_battery_rwlock");
	kthread_add(rk818_battery_thread, NULL, NULL, NULL, 0, 0, "rk818_battery_thread");

	device_printf(pmic_dev, "RK818 battery driver loaded\n");
}

void
rk818_battery_detach(device_t pmic_dev)
{
	stop_thread = 1;
}
