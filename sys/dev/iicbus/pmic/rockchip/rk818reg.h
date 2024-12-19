/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2018-2021 Emmanuel Vadot <manu@FreeBSD.org>
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

#ifndef _RK818REG_H_
#define	 _RK818REG_H_

/* RTC registers */
#define	RK818_RTC_SECS		0x00
#define	 RK818_RTC_SECS_MASK	0x7f
#define	RK818_RTC_MINUTES	0x01
#define	 RK818_RTC_MINUTES_MASK	0x7f
#define	RK818_RTC_HOURS		0x02
#define	 RK818_RTC_HOURS_MASK	0x3f
#define	RK818_RTC_DAYS		0x03
#define	 RK818_RTC_DAYS_MASK	0x3f
#define	RK818_RTC_MONTHS	0x04
#define	 RK818_RTC_MONTHS_MASK	0x1f
#define	RK818_RTC_YEARS		0x05
#define	RK818_RTC_WEEKS		0x06 /* day of week */
#define	 RK818_RTC_WEEKS_MASK	0x07
#define	RK818_ALARM_SECONDS	0x8
#define	RK818_ALARM_MINUTES	0x9
#define	RK818_ALARM_HOURS	0xA
#define	RK818_ALARM_DAYS	0xB
#define	RK818_ALARM_MONTHS	0xC
#define	RK818_ALARM_YEARS	0xD
#define	RK818_RTC_CTRL		0x10
#define	 RK818_RTC_CTRL_STOP	(1 << 0)
#define	 RK818_RTC_AMPM_MODE	(1 << 3)
#define	 RK818_RTC_GET_TIME	(1 << 6)
#define	 RK818_RTC_READSEL	(1 << 7)
#define	RK818_RTC_STATUS	0x11
#define	RK818_RTC_INT		0x12
#define	RK818_RTC_COMP_LSB	0x13
#define	RK818_RTC_COMP_MSB	0x14

/* Misc registers*/
#define	RK818_CLK32KOUT		0x20
#define	RK818_VB_MON		0x21
#define	RK818_THERMAL		0x22

/* Power channel control and monitoring registers */
#define	RK818_DCDC_EN		0x23
#define	RK818_LDO_EN		0x24
#define	RK818_SLEEP_SET_OFF_1	0x25
#define	RK818_SLEEP_SET_OFF_2	0x26
#define	RK818_DCDC_UV_STS	0x27
#define	RK818_DCDC_UV_ACT	0x28
#define	RK818_LDO_UV_STS	0x29
#define	RK818_LDO_UV_ACT	0x2A
#define	RK818_DCDC_PG		0x2B
#define	RK818_LDO_PG		0x2C
#define	RK818_VOUT_MON_TDB	0x2D

/* Power channel configuration registers */
#define	RK818_BUCK1_CONFIG	0x2E
#define	RK818_BUCK1_ON_VSEL	0x2F
#define	RK818_BUCK1_SLP_VSEL	0x30
#define	RK818_BUCK2_CONFIG	0x32
#define	RK818_BUCK2_ON_VSEL	0x33
#define	RK818_BUCK2_SLEEP_VSEL	0x34
#define	RK818_BUCK3_CONFIG	0x36
#define	RK818_BUCK4_CONFIG	0x37
#define	RK818_BUCK4_ON_VSEL	0x38
#define	RK818_BUCK4_SLEEP_VSEL	0x39
#define RK818_BOOST_CONFIG  0x3A
#define RK818_H5V_EN_REG  0x52
#define RK818_SLEEP_SET_OFF_REG3  0x53
#define RK818_BOOST_LDO9_ON_VSEL_REG  0x54
#define RK818_BOOST_LDO9_SLP_VSEL_REG  0x55
#define RK818_BOOST_CTRL_REG  0x56 
#define	RK818_DCDC_ILMAX_REG	0x90
#define	RK818_LDO1_ON_VSEL	0x3B
#define	RK818_LDO1_SLEEP_VSEL	0x3C
#define	RK818_LDO2_ON_VSEL	0x3D
#define	RK818_LDO2_SLEEP_VSEL	0x3E
#define	RK818_LDO3_ON_VSEL	0x3F
#define	RK818_LDO3_SLEEP_VSEL	0x40
#define	RK818_LDO4_ON_VSEL	0x41
#define	RK818_LDO4_SLEEP_VSEL	0x42
#define	RK818_LDO5_ON_VSEL	0x43
#define	RK818_LDO5_SLEEP_VSEL	0x44
#define	RK818_LDO6_ON_VSEL	0x45
#define	RK818_LDO6_SLEEP_VSEL	0x46
#define	RK818_LDO7_ON_VSEL	0x47
#define	RK818_LDO7_SLEEP_VSEL	0x48
#define	RK818_LDO8_ON_VSEL	0x49
#define	RK818_LDO8_SLEEP_VSEL	0x4A

#define	RK818_DEV_CTRL		0x4B
#define	 RK818_DEV_CTRL_OFF	(1 << 0)
#define	 RK818_DEV_CTRL_SLP	(1 << 1)

enum rk818_regulator {
	RK818_BUCK1 = 0,
	RK818_BUCK2,
	RK818_BUCK3,
	RK818_BUCK4,
	RK818_BOOST,
	RK818_LDO1,
	RK818_LDO2,
	RK818_LDO3,
	RK818_LDO4,
	RK818_LDO5,
	RK818_LDO6,
	RK818_LDO7,
	RK818_LDO8,
	RK818_LDO9,
	RK818_SWITCH1,
  RK818_OTG_SWITCH,
};

#endif /* _RK818REG_H_ */
