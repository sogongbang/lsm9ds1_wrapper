/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to get data from sensor.
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
#include <time.h>
#include <sys/time.h>

#include "main.h"
#include "lsm9ds1_read_data_event.h"
#include "lsm9ds1_reg.h"

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            20 //ms

/* Private variables ---------------------------------------------------------*/

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];
static stmdev_ctx_t dev_ctx_imu;
static stmdev_ctx_t dev_ctx_mag;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Main Example --------------------------------------------------------------*/
void lsm9ds1_read_data_event_setup(void)
{
    /* Initialize inertial sensors (IMU) driver interface */
    dev_ctx_imu.write_reg = platform_write_imu;
    dev_ctx_imu.read_reg = platform_read_imu;
    dev_ctx_imu.handle = (void *)&imu_bus;
    /* Initialize magnetic sensors driver interface */
    dev_ctx_mag.write_reg = platform_write_mag;
    dev_ctx_mag.read_reg = platform_read_mag;
    dev_ctx_mag.handle = (void *)&mag_bus;
    /* Initialize platform specific hardware */
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
    lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
    /* Set full scale */
    lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
    lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
    lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
    /* Configure filtering chain - See datasheet for filtering chain details */
    /* Accelerometer filtering chain */
    lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
    lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
    lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
    /* Gyroscope filtering chain */
    lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
    lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);
    /* Set Output Data Rate / Power mode */
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_952Hz);
    lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_POWER_DOWN);
}

void lsm9ds1_read_data_event_loop(void)
{
    struct timeval tv;
    uint64_t tmp_time;
    int data_size = 0;
    int buf_offset = 0;

    memset(acceleration_mg, 0, sizeof(acceleration_mg));
    memset(angular_rate_mdps, 0, sizeof(angular_rate_mdps));
    memset(magnetic_field_mgauss, 0, sizeof(magnetic_field_mgauss));

    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

    if ( reg.status_imu.xlda && reg.status_imu.gda ) {
        gettimeofday(&tv, NULL);

        /* Read imu data */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
        lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration);
        lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate);

        acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[0]);
        acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[2]);
        angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
        angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
        angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
        magnetic_field_mgauss[0] = 0.0;
        magnetic_field_mgauss[1] = 0.0;
        magnetic_field_mgauss[2] = 0.0;

        tx_buffer[0] = 0x76; // start code 1
        tx_buffer[1] = 0x07; // start code 2
        tx_buffer[2] = 0x03; // start code 3
        tx_buffer[3] = 0;    // data size
        buf_offset = 4;

        tmp_time = tv.tv_sec;
        data_size = sizeof(tmp_time);
        memcpy(&tx_buffer[buf_offset], &tmp_time, data_size);
        buf_offset += data_size;
    
        tmp_time = tv.tv_usec;
        data_size = sizeof(tmp_time);
        memcpy(&tx_buffer[buf_offset], &tmp_time, data_size);
        buf_offset += data_size;

        data_size = sizeof(acceleration_mg);
        memcpy(&tx_buffer[buf_offset], acceleration_mg, data_size);
        buf_offset += data_size;

        data_size = sizeof(angular_rate_mdps);
        memcpy(&tx_buffer[buf_offset], angular_rate_mdps, data_size);
        buf_offset += data_size;

        data_size = sizeof(magnetic_field_mgauss);
        memcpy(&tx_buffer[buf_offset], magnetic_field_mgauss, data_size);
        buf_offset += data_size;

        tx_buffer[buf_offset + 0] = 0x03; // end code 1
        tx_buffer[buf_offset + 1] = 0x07; // end code 2
        tx_buffer[buf_offset + 2] = 0x76; // end code 3
        tx_buffer[buf_offset + 3] = '\n'; // new line
        buf_offset += 4;

        tx_buffer[3] = buf_offset - 4;

        tx_com(tx_buffer, buf_offset);
    }
}
