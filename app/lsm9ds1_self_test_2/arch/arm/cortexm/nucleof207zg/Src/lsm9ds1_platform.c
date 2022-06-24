#include <ubinos.h>

#if (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NUCLEOF207ZG)
#if (UBINOS__BSP__BOARD_VARIATION__NUCLEOF207ZG == 1)

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "lsm9ds1_reg.h"

#define SENSOR_BUS I2CxHandle

sensbus_t mag_bus = {&SENSOR_BUS,
                      LSM9DS1_MAG_I2C_ADD_H,
                      0,
                      0
                      };
sensbus_t imu_bus = {&SENSOR_BUS,
                      LSM9DS1_IMU_I2C_ADD_H,
                      0,
                      0
                      };

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                  I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp,
                                 uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp,
                                 uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  dtty_putn((char *) tx_buffer, len);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms)
{
  if (ubik_istask())
  {
    task_sleepms(ms);
  }
  else
  {
    bsp_busywaitms(ms);
  }
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
void platform_init(void)
{
  HAL_StatusTypeDef stm_err;
  
  I2CxHandle.Instance             = I2Cx;
  I2CxHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2CxHandle.Init.ClockSpeed      = I2Cx_CLOCK;
  I2CxHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2CxHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2CxHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2CxHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2CxHandle.Init.OwnAddress1     = I2Cx_ADDRESS;
  I2CxHandle.Init.OwnAddress2     = 0;

  stm_err = HAL_I2C_Init(&I2CxHandle);
  ubi_assert(stm_err == HAL_OK);
}

#endif /* (UBINOS__BSP__BOARD_VARIATION__NUCLEOF207ZG == 1) */
#endif /* (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NUCLEOF207ZG) */
