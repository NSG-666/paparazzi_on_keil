/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 */

/**
 * @file peripherals/BMP280_i2c.c
 * Measurement Specialties (Intersema) BMP280-01BA and MS5607-02BA03 pressure/temperature sensor interface for I2C.
 *
 */

#include <string.h>
#include "peripherals/bmp280_i2c.h"

struct bmp280_calib_param_t {
    uint16_t dig_T1; /* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
    int32_t t_fine; /* calibration t_fine data */
}bmp280_cal;


void BMP280_i2c_init(struct BMP280_I2c *ms, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  ms->i2c_p = i2c_p;

  /* slave address */
  ms->i2c_trans.slave_addr = addr;
  /* set initial status: Success or Done */
  ms->i2c_trans.status = I2CTransDone;

  ms->data_available = FALSE;
  ms->initialized = FALSE;
  ms->status = BMP280_STATUS_UNINIT;
  ms->prom_cnt = 0;
}

void BMP280_i2c_start_configure(struct BMP280_I2c *ms)
{
  if (ms->status == BMP280_STATUS_UNINIT) {
    ms->initialized = FALSE;
    ms->prom_cnt = 0;
    ms->i2c_trans.buf[0] = BMP280_CHIP_ID_REG;
    i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, 1);
    ms->status = BMP280_STATUS_GET_ID;
  }
}

void BMP280_i2c_start_conversion(struct BMP280_I2c *ms)
{
  if (ms->status == BMP280_STATUS_IDLE &&
      ms->i2c_trans.status == I2CTransDone) {
    /* start D1 conversion */
    ms->i2c_trans.buf[0] = BMP280_CTRL_MEAS_REG;
	ms->i2c_trans.buf[1] = BMP280_MODE;  
    i2c_transmit(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 2);
    ms->status = BMP280_STATUS_CONV_D;
  }
}

/**
 * Periodic function to ensure proper delay after triggering reset or conversion.
 * Should run at 100Hz max.
 * Typical conversion time is 8.22ms at max resolution.
 */
#pragma O0
void BMP280_i2c_periodic_check(struct BMP280_I2c *ms)
{
  switch (ms->status) {
    case BMP280_STATUS_GET_ID:
      ms->status = BMP280_STATUS_GET_ID_OK;
      break;
    case BMP280_STATUS_GET_ID_OK:
      if (ms->i2c_trans.status == I2CTransDone) {
		if(ms->i2c_trans.buf[0] == BMP280_DEFAULT_CHIP_ID)
		{			  
			/* start getting prom data */
			ms->i2c_trans.buf[0] = BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG | (ms->prom_cnt << 1);
			i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, 2);
			ms->status = BMP280_STATUS_PROM;
		}
		else
		{
			ms->status = BMP280_STATUS_UNINIT;
		}
      }
      break;
    case BMP280_STATUS_CONV_D:
      ms->status = BMP280_STATUS_CONV_D_OK;
      break;
    case BMP280_STATUS_CONV_D_OK:
      if (ms->i2c_trans.status == I2CTransDone) {
        /* read D1 adc */
        ms->i2c_trans.buf[0] = BMP280_PRESSURE_MSB_REG;
        i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, BMP280_DATA_FRAME_SIZE);
        ms->status = BMP280_STATUS_ADC_D;
      }
      break;
    default:
      break;
  }
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
    bmp280_cal.t_fine = var1 + var2;
    T = (bmp280_cal.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);
    return (uint32_t)p;
}

uint32_t bmp280_up = 0;
uint32_t bmp280_ut = 0;
void bmp280_calculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    int32_t t;
    uint32_t p;
    t = bmp280_compensate_T(bmp280_ut);
    p = bmp280_compensate_P(bmp280_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;
}

bool_t BMP280_calc(struct BMP280Data *ms)
{
	bmp280_calculate(&ms->pressure, &ms->temperature);
	return TRUE;
}

#pragma O0
void BMP280_i2c_event(struct BMP280_I2c *ms)
{
  if (ms->initialized) {
    if (ms->i2c_trans.status == I2CTransFailed) {
      ms->status = BMP280_STATUS_IDLE;
      ms->i2c_trans.status = I2CTransDone;
    } else if (ms->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      switch (ms->status) {

        case BMP280_STATUS_ADC_D:
          /* read d*/
		  memcpy(ms->data.d, (uint8_t *)ms->i2c_trans.buf, BMP280_DATA_FRAME_SIZE);
		  bmp280_up = (int32_t)((((uint32_t)(ms->i2c_trans.buf[0])) << 12) | (((uint32_t)(ms->i2c_trans.buf[1])) << 4) | ((uint32_t)ms->i2c_trans.buf[2] >> 4));
		  bmp280_ut = (int32_t)((((uint32_t)(ms->i2c_trans.buf[3])) << 12) | (((uint32_t)(ms->i2c_trans.buf[4])) << 4) | ((uint32_t)ms->i2c_trans.buf[5] >> 4));
          ms->i2c_trans.status = I2CTransDone;
		  ms->data_available = BMP280_calc(&(ms->data));
          ms->status = BMP280_STATUS_IDLE;
          break;

        default:
          ms->i2c_trans.status = I2CTransDone;
          break;
      }
    }
  } else if (ms->status != BMP280_STATUS_UNINIT) { // Configuring but not yet initialized
    switch (ms->i2c_trans.status) {

      case I2CTransFailed:
        /* try again */
        ms->status = BMP280_STATUS_UNINIT;
        ms->i2c_trans.status = I2CTransDone;
        break;

      case I2CTransSuccess:
        if (ms->status == BMP280_STATUS_PROM) {
          /* read prom data */
          ((uint8_t *)&(bmp280_cal))[ms->prom_cnt++] = (ms->i2c_trans.buf[0] << 8) |
                                       ms->i2c_trans.buf[1];
          ms->i2c_trans.status = I2CTransDone;
          if (ms->prom_cnt < PROM_NB) {
            /* get next prom data */
            ms->i2c_trans.buf[0] = BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG | (ms->prom_cnt << 1);
            i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, 2);
          } else {
			ms->status = BMP280_STATUS_CONFIG;
            /* start config */
            ms->i2c_trans.buf[0] = BMP280_CTRL_MEAS_REG;
			ms->i2c_trans.buf[1] = BMP280_MODE;  
            i2c_transmit(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 2);
          }
        } 
		else if(ms->status == BMP280_STATUS_CONFIG)
		{
           ms->i2c_trans.status = I2CTransDone;
		   ms->initialized = TRUE;
           ms->status = BMP280_STATUS_IDLE;
		}
		else {
          ms->i2c_trans.status = I2CTransDone;
        }
        break;

      default:
        ms->i2c_trans.status = I2CTransDone;
        break;
    }
  }
}
