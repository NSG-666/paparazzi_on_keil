/* This file has been generated from /home/qq/paparazzi/conf/airframes/untested/hex_naze32.xml */
/* Version v5.8.1_stable-0-ga7c97e1-dirty */
/* Please DO NOT EDIT */

#ifndef MODULES_H
#define MODULES_H

#define MODULES_IDLE  0
#define MODULES_RUN   1
#define MODULES_START 2
#define MODULES_STOP  3

#define MODULES_FREQUENCY 512

#ifdef MODULES_C
#define EXTERN_MODULES
#else
#define EXTERN_MODULES extern
#endif
#include "std.h"
#include "sensors/mag_hmc58xx.h"
#include "gps/gps_ubx_ucenter.h"
#include "calibration/send_imu_mag_current.h"

#define MAG_HMC58XX_MODULE_PERIODIC_PERIOD 0.016667
#define MAG_HMC58XX_MODULE_PERIODIC_FREQ 60.000000
#define MAG_HMC58XX_REPORT_PERIOD 0.100000
#define MAG_HMC58XX_REPORT_FREQ 10.000000
#define GPS_UBX_UCENTER_PERIODIC_PERIOD 0.250000
#define GPS_UBX_UCENTER_PERIODIC_FREQ 4.000000
#define SEND_IMU_MAG_CURRENT_PERIOD 0.050000
#define SEND_IMU_MAG_CURRENT_FREQ 20.000000

EXTERN_MODULES uint8_t sensors_mag_hmc58xx_report_status;
EXTERN_MODULES uint8_t gps_ubx_gps_ubx_ucenter_periodic_status;
EXTERN_MODULES uint8_t calibration_send_imu_mag_current_status;

#ifdef MODULES_C

static inline void modules_init(void) {
  mag_hmc58xx_module_init();
  sensors_mag_hmc58xx_report_status = MODULES_IDLE;
  gps_ubx_ucenter_init();
  gps_ubx_gps_ubx_ucenter_periodic_status = MODULES_START;
  calibration_send_imu_mag_current_status = MODULES_IDLE;
}

static inline void modules_periodic_task(void) {
  static uint8_t i8; i8++; if (i8>=8) i8=0;
  static uint8_t i25; i25++; if (i25>=25) i25=0;
  static uint8_t i51; i51++; if (i51>=51) i51=0;
  static uint8_t i128; i128++; if (i128>=128) i128=0;

  if (sensors_mag_hmc58xx_report_status == MODULES_START) {
    sensors_mag_hmc58xx_report_status = MODULES_RUN;
  }
  if (sensors_mag_hmc58xx_report_status == MODULES_STOP) {
    sensors_mag_hmc58xx_report_status = MODULES_IDLE;
  }

  if (gps_ubx_gps_ubx_ucenter_periodic_status == MODULES_START) {
    gps_ubx_ucenter_init();
    gps_ubx_gps_ubx_ucenter_periodic_status = MODULES_RUN;
  }
  if (gps_ubx_gps_ubx_ucenter_periodic_status == MODULES_STOP) {
    gps_ubx_gps_ubx_ucenter_periodic_status = MODULES_IDLE;
  }

  if (calibration_send_imu_mag_current_status == MODULES_START) {
    calibration_send_imu_mag_current_status = MODULES_RUN;
  }
  if (calibration_send_imu_mag_current_status == MODULES_STOP) {
    calibration_send_imu_mag_current_status = MODULES_IDLE;
  }

  if (i8 == 0) {
    mag_hmc58xx_module_periodic();
  }
  if (i25 == 4 && calibration_send_imu_mag_current_status == MODULES_RUN) {
    send_imu_mag_current();
  }
  if (i51 == 16 && sensors_mag_hmc58xx_report_status == MODULES_RUN) {
    mag_hmc58xx_report();
  }
  if (i128 == 41 && gps_ubx_gps_ubx_ucenter_periodic_status == MODULES_RUN) {
    gps_ubx_ucenter_periodic();
  }
}

static inline void modules_event_task(void) {
  mag_hmc58xx_module_event();
}

#endif // MODULES_C

#ifdef MODULES_DATALINK_C

#include "messages.h"
#include "generated/airframe.h"
static inline void modules_parse_datalink(uint8_t msg_id __attribute__ ((unused))) {
}

#endif // MODULES_DATALINK_C

#endif // MODULES_H