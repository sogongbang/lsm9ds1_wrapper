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

#include "main.h"
#include "lsm9ds1_reg.h"
#include "lsm9ds1_read_data_polling.h"

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            20 //ms

/* Private variables ---------------------------------------------------------*/

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Main Example --------------------------------------------------------------*/
void lsm9ds1_read_data_polling(void)
{
    stmdev_ctx_t dev_ctx_imu;
    stmdev_ctx_t dev_ctx_mag;
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
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
    lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

    /* Read samples in polling mode (no int) */
    while (1) {
        memset(acceleration_mg, 0, sizeof(acceleration_mg));
        memset(angular_rate_mdps, 0, sizeof(angular_rate_mdps));
        memset(magnetic_field_mgauss, 0, sizeof(magnetic_field_mgauss));

        /* Read device status register */
        lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

        if ( reg.status_imu.xlda && reg.status_imu.gda ) {
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
            // sprintf((char *)tx_buffer,
            //         "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
            //         acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
            //         angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            // tx_com(tx_buffer, strlen((char const *)tx_buffer));

            sprintf((char *)tx_buffer,
                    "acc, ang: %12.2f, %12.2f, %12.2f,    %12.2f, %12.2f, %12.2f\n",
                    acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
                    angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }

        if ( reg.status_mag.zyxda ) {
            /* Read magnetometer data */
            memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
            lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
            magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[0]);
            magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[1]);
            magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[2]);
            // sprintf((char *)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
            //         magnetic_field_mgauss[0], magnetic_field_mgauss[1],
            //         magnetic_field_mgauss[2]);
            // tx_com(tx_buffer, strlen((char const *)tx_buffer));

            sprintf((char *)tx_buffer,
                    "mag     : %12.2f, %12.2f, %12.2f\n",
                    magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }

        // sprintf((char *)tx_buffer,
        //   "3D-ACC[mg], 3D-ANG[mdps], 3D-MAG[mG]: ");
        // tx_com(tx_buffer, strlen((char const *)tx_buffer));
        // sprintf((char *)tx_buffer,
        //         "%12.2f, %12.2f, %12.2f,    %12.2f, %12.2f, %12.2f,    %12.2f, %12.2f, %12.2f\n",
        //         acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
        //         angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
        //         magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2]);
        // tx_com(tx_buffer, strlen((char const *)tx_buffer));

        // platform_delay(500);
    }
}
