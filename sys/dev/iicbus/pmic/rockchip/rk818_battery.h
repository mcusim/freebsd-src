
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

#ifndef _RK818_BATT_H_
#define	_RK818_BATT_H_

#define SUP_STS_REG       0xA0
#define USB_CTRL_REG			0xA1
#define CHRG_CTRL_REG1    0xA3
#define CHRG_CTRL_REG2    0xA4
#define CHRG_CTRL_REG3    0xA5

#define GGSTS_REG         0xB1
#define FLAG_BAT_CON      (1 << 4)

#define	VCALIB0_REGH			0xD5
#define	VCALIB0_REGL			0xD6
#define	VCALIB1_REGH			0xD7
#define	VCALIB1_REGL			0xD8

#define GASCNT_CAL_REG3		0xB4
#define GASCNT_CAL_REG2		0xB5
#define GASCNT_CAL_REG1		0xB6
#define GASCNT_CAL_REG0		0xB7
#define GASCNT_REG3       0xB8
#define GASCNT_REG2       0xB9
#define GASCNT_REG1       0xBA
#define GASCNT_REG0       0xBB
#define BAT_CUR_AVG_REGH  0xBC
#define BAT_CUR_AVG_REGL  0xBD
#define BAT_VOL_REGH      0xC4
#define BAT_VOL_REGL      0xC5

void rk818_battery_attach(device_t pmic_dev);
void rk818_battery_detach(device_t pmic_dev);

#endif /* _RK818_BATT_H_ */
