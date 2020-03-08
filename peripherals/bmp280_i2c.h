/*
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
 * @file peripherals/BMP280_i2c.h
 *
 * Measurement Specialties (Intersema) BMP280-01BA and MS5607-02BA03 pressure/temperature sensor interface for I2C.
 */

#ifndef BMP280_I2C_H
#define BMP280_I2C_H

#include "mcu_periph/i2c.h"

#define BMP280_I2C_ADDR                      (0x76)
#define BMP280_DEFAULT_CHIP_ID               (0x58)

#define BMP280_CHIP_ID_REG                   (0xD0)  /* Chip ID Register */
#define BMP280_RST_REG                       (0xE0)  /* Softreset Register */
#define BMP280_STAT_REG                      (0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG                    (0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
#define BMP280_FORCED_MODE                   (0x01)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE               (6)

#define BMP280_OVERSAMP_SKIPPED          (0x00)
#define BMP280_OVERSAMP_1X               (0x01)
#define BMP280_OVERSAMP_2X               (0x02)
#define BMP280_OVERSAMP_4X               (0x03)
#define BMP280_OVERSAMP_8X               (0x04)
#define BMP280_OVERSAMP_16X              (0x05)

// configure pressure and temperature oversampling, forced sampling mode
#define BMP280_PRESSURE_OSR              (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR           (BMP280_OVERSAMP_1X)
#define BMP280_MODE                      (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_FORCED_MODE)

#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms

#define PROM_NB							 BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH	

enum BMP280Status {
  BMP280_STATUS_UNINIT,
  BMP280_STATUS_GET_ID,
  BMP280_STATUS_GET_ID_OK,
  BMP280_STATUS_PROM,
  BMP280_STATUS_CONFIG,
  BMP280_STATUS_IDLE,
  BMP280_STATUS_CONV_D,
  BMP280_STATUS_CONV_D_OK,
  BMP280_STATUS_ADC_D,
};

struct BMP280Data {
  uint32_t pressure;    ///< pressure in Pascal (0.01mbar)
  int32_t temperature;  ///< temperature with 0.01 degrees Celsius resolution
  uint8_t d[BMP280_DATA_FRAME_SIZE];
};

struct BMP280_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum BMP280Status status;
  bool_t initialized;                 ///< config done flag
  volatile bool_t data_available;     ///< data ready flag
  struct BMP280Data data;
  int32_t prom_cnt;                   ///< number of bytes read from PROM
};

// Functions
extern void BMP280_i2c_init(struct BMP280_I2c *ms, struct i2c_periph *i2c_p, uint8_t addr);
extern void BMP280_i2c_start_configure(struct BMP280_I2c *ms);
extern void BMP280_i2c_start_conversion(struct BMP280_I2c *ms);
extern void BMP280_i2c_periodic_check(struct BMP280_I2c *ms);
extern void BMP280_i2c_event(struct BMP280_I2c *ms);

/** convenience function to trigger new measurement.
 * (or start configuration if not already initialized)
 * Still need to regularly run BMP280_i2c_periodic_check to complete the measurement.
 */
static void BMP280_i2c_read(struct BMP280_I2c *ms)
{
  if (ms->initialized) {
    BMP280_i2c_start_conversion(ms);
  } else {
    BMP280_i2c_start_configure(ms);
  }
}

/// convenience function
static void BMP280_i2c_periodic(struct BMP280_I2c *ms)
{
  BMP280_i2c_read(ms);
  BMP280_i2c_periodic_check(ms);
}


#endif /* BMP280_I2C_H */
