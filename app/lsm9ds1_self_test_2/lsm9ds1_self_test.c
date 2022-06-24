/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements self test procedure.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI159V1
 * - NUCLEO_F411RE + STEVAL-MKI159V11
 * - DISCOVERY_SPC584B + STEVAL-MKI159V11
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F411RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#include <ubinos.h>

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "lsm9ds1_reg.h"
#include "lsm9ds1_self_test.h"

/* Private macro -------------------------------------------------------------*/

#define    BOOT_TIME            20 //ms

#define    WAIT_TIME_MAG        60 //ms
#define    WAIT_TIME_XL        200 //ms
#define    WAIT_TIME_GY        800 //ms

#define    SAMPLES               5 //number of samples

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Self test limits in mgauss @ 12G*/
static const float min_st_mag_limit[] = {1000.0f, 1000.0f,  100.0f};
static const float max_st_mag_limit[] = {3000.0f, 3000.0f, 1000.0f};

/* Self test limits in mg @ 2g*/
static const float min_st_xl_limit[] = {70.0f, 70.0f,  70.0f};
static const float max_st_xl_limit[] = {1500.0f, 1500.0f, 1500.0f};

/* Self test limits in mdps @ 2000 dps*/
static const float min_st_gy_limit[] = {200000.0f, 200000.0f, 200000.0f};
static const float max_st_gy_limit[] = {800000.0f, 800000.0f, 800000.0f};

/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Main Example --------------------------------------------------------------*/
void lsm9ds1_self_test(void)
{
  stmdev_ctx_t dev_ctx_imu;
  stmdev_ctx_t dev_ctx_mag;
  uint8_t tx_buffer[1000];
  int16_t data_raw[3];
  lsm9ds1_status_t reg;
  lsm9ds1_id_t whoamI;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t rst;
  uint8_t i;
  uint8_t j;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
    while (1) {
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                PROPERTY_ENABLE);
  /* Initialize test variable */
  st_result = ST_PASS;
  /*
   * START MAGNETOMETER SELF TEST PROCEDURE
   */
  /* Set Full Scale */
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_12Ga);
  /* Set Output Data Rate */
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_LP_80Hz);
  /* Wait stable output */
  platform_delay(WAIT_TIME_MAG);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
  } while (!reg.status_mag.zyxda);

  /* Read dummy data and discard it */
  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES; i++) {
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
    } while (!reg.status_mag.zyxda);

    /* Read data and accumulate */
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm9ds1_from_fs12gauss_to_mG(data_raw[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= SAMPLES;
  }

  /* Enable Self Test */
  lsm9ds1_mag_self_test_set(&dev_ctx_mag, PROPERTY_ENABLE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_MAG);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
  } while (!reg.status_mag.zyxda);

  /* Read dummy data and discard it */
  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES; i++) {
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
    } while (!reg.status_mag.zyxda);

    /* Read data and accumulate */
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm9ds1_from_fs12gauss_to_mG(data_raw[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= SAMPLES;
  }

  /* Calculate the values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( min_st_mag_limit[i] > test_val[i] ) ||
        ( test_val[i] > max_st_mag_limit[i])) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm9ds1_mag_self_test_set(&dev_ctx_mag, PROPERTY_DISABLE);
  /* Disable sensor. */
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_POWER_DOWN);
  /*
   * END MAGNETOMETER SELF TEST PROCEDURE
   */
  /*
   * START ACCELEROMETER SELF TEST PROCEDURE
   */
  /* Set Full Scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_2g);
  /* Set Output Data Rate */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_GY_OFF_XL_50Hz);
  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while (!reg.status_imu.xlda);

  /* Read dummy data and discard it */
  lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES; i++) {
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while (!reg.status_imu.xlda);

    /* Read data and accumulate */
    lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm9ds1_from_fs2g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= SAMPLES;
  }

  /* Enable Self Test */
  lsm9ds1_xl_self_test_set(&dev_ctx_imu, PROPERTY_ENABLE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while (!reg.status_imu.xlda);

  /* Read dummy data and discard it */
  lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES; i++) {
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while (!reg.status_imu.xlda);

    /* Read data and accumulate */
    lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm9ds1_from_fs2g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= SAMPLES;
  }

  /* Calculate the values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( min_st_xl_limit[i] > test_val[i] ) ||
        ( test_val[i] > max_st_xl_limit[i])) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm9ds1_xl_self_test_set(&dev_ctx_imu, PROPERTY_DISABLE);
  /* Disable sensor. */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_OFF);
  /*
   * END ACCELEROMETER SELF TEST PROCEDURE
   */
  /*
   * START GYROSCOPE SELF TEST PROCEDURE
   */
  /* Set Full Scale */
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  /* Set Output Data Rate */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_XL_OFF_GY_238Hz);
  /* Wait stable output */
  platform_delay(WAIT_TIME_GY);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while (!reg.status_imu.gda);

  /* Read dummy data and discard it */
  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES; i++) {
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while (!reg.status_imu.gda);

    /* Read data and accumulate */
    lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm9ds1_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= SAMPLES;
  }

  /* Enable Self Test */
  lsm9ds1_gy_self_test_set(&dev_ctx_imu, PROPERTY_ENABLE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_GY);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while (!reg.status_imu.gda);

  /* Read dummy data and discard it */
  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES; i++) {
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while (!reg.status_imu.gda);

    /* Read data and accumulate */
    lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm9ds1_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= SAMPLES;
  }

  /* Calculate the values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( min_st_gy_limit[i] > test_val[i] ) ||
        ( test_val[i] > max_st_gy_limit[i])) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm9ds1_gy_self_test_set(&dev_ctx_imu, PROPERTY_DISABLE);
  /* Disable sensor. */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_OFF);

  /*
   * END GYROSCOPE SELF TEST PROCEDURE
   */

  if (st_result == ST_PASS) {
    sprintf((char *)tx_buffer, "Self Test - PASS\r\n" );
  }

  else {
    sprintf((char *)tx_buffer, "Self Test - FAIL\r\n" );
  }

  tx_com(tx_buffer, strlen((char const *)tx_buffer));
}

