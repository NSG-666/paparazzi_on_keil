/*
 * Copyright (C) 2011-2013 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file boards/baro_board_BMP280_i2c.c
 *
 * Driver for onboard BMP280 baro via I2C.
 *
 */
#include "generated/airframe.h"
#include "mcu_periph/i2c.h"
#include "subsystems/sensors/baro.h"
#include "peripherals/bmp280_i2c.h"

#include "mcu_periph/sys_time.h"
#include "led.h"
#include "std.h"
#include "subsystems/abi.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifdef BARO_PERIODIC_FREQUENCY
#if BARO_PERIODIC_FREQUENCY > 100
#error "For BMP280 BARO_PERIODIC_FREQUENCY has to be < 100"
#endif
#endif


/** default i2c address
 * when CSB is set to GND addr is 0xEE
 * when CSB is set to VCC addr is 0xEC
 *
 * Note: Aspirin 2.1 has CSB bound to GND.
 */
#ifndef BB_BMP280_SLAVE_ADDR
#define BB_BMP280_SLAVE_ADDR 0xEC
#endif

struct BMP280_I2c bb_BMP280;


void baro_init(void)
{
  BMP280_i2c_init(&bb_BMP280, &BB_BMP280_I2C_DEV, BB_BMP280_SLAVE_ADDR);

#ifdef BARO_LED
  LED_OFF(BARO_LED);
#endif
}

void baro_periodic(void)
{
  if (sys_time.nb_sec > 1) {

    /* call the convenience periodic that initializes the sensor and starts reading*/
    BMP280_i2c_periodic(&bb_BMP280);

#if DEBUG
    if (bb_BMP280.initialized)
      RunOnceEvery((50 * 30), DOWNLINK_SEND_BMP280_COEFF(DefaultChannel, DefaultDevice,
                   &bb_BMP280.data.c[0],
                   &bb_BMP280.data.c[1],
                   &bb_BMP280.data.c[2],
                   &bb_BMP280.data.c[3],
                   &bb_BMP280.data.c[4],
                   &bb_BMP280.data.c[5],
                   &bb_BMP280.data.c[6],
                   &bb_BMP280.data.c[7]));
#endif
  }
}

void baro_event(void)
{
  if (sys_time.nb_sec > 1) {
    BMP280_i2c_event(&bb_BMP280);

    if (bb_BMP280.data_available) {
      float pressure = (float)bb_BMP280.data.pressure;
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
      float temp = bb_BMP280.data.temperature / 100.0f;
      AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp);
      bb_BMP280.data_available = FALSE;

#ifdef BARO_LED
      RunOnceEvery(10, LED_TOGGLE(BARO_LED));
#endif

#if DEBUG
      float fbaroms = bb_BMP280.data.pressure / 100.;
      DOWNLINK_SEND_BARO_BMP280(DefaultChannel, DefaultDevice,
                                &bb_BMP280.data.d1, &bb_BMP280.data.d2,
                                &fbaroms, &temp);
#endif
    }
  }
}
