#include <ubinos.h>

#if (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NRF52840DK)
#if (UBINOS__BSP__BOARD_VARIATION__NRF52840DK == 1)

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "lsm9ds1_reg.h"

#include "boards.h"
#include "nrf_drv_twi.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

/* TWI instance. */
nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

#define SENSOR_BUS m_twi

sensbus_t mag_bus = {&SENSOR_BUS,
                      LSM9DS1_MAG_I2C_ADD_H,
                      0
                      };
sensbus_t imu_bus = {&SENSOR_BUS,
                      LSM9DS1_IMU_I2C_ADD_H,
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
  uint8_t tbuf[64];
  uint16_t tlen = min(sizeof(tbuf) - 1, len);
  tbuf[0] = reg;
  memcpy(&tbuf[1], bufp, tlen);
  nrf_drv_twi_tx( sensbus->hbus,
                  sensbus->i2c_address >> 1,
                  tbuf,
                  tlen + 1,
                  false );
  // HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address >> 1, reg,
  //                 I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
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
  uint8_t tbuf[64];
  uint16_t tlen = min(sizeof(tbuf) - 1, len);
  tbuf[0] = reg;
  memcpy(&tbuf[1], bufp, tlen);
  nrf_drv_twi_tx( sensbus->hbus,
                  sensbus->i2c_address >> 1,
                  tbuf,
                  tlen + 1,
                  false );
  // HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address >> 1, reg,
  //                   I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
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
  nrf_drv_twi_tx( sensbus->hbus,
                  sensbus->i2c_address >> 1,
                  &reg,
                  1,
                  true );
  nrf_drv_twi_rx( sensbus->hbus,
                  sensbus->i2c_address >> 1,
                  bufp,
                  len );

  // HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
  //                  I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
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

  nrf_drv_twi_tx( sensbus->hbus,
                  sensbus->i2c_address >> 1,
                  &reg,
                  1,
                  true );

  nrf_drv_twi_rx( sensbus->hbus,
                  sensbus->i2c_address >> 1,
                  bufp,
                  len );
  // HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
  //                  I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
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
    ret_code_t nrf_err;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    do
    {
        nrf_err = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
        if (nrf_err != NRF_SUCCESS) {
            break;
        }

        nrf_drv_twi_enable(&m_twi);

        break;
    } while (1);
}

#endif /* (UBINOS__BSP__BOARD_VARIATION__NRF52840DK == 1) */
#endif /* (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NRF52840DK) */
