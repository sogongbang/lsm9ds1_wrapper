/*
 * Copyright (c) 2009 Sung Ho Park
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LSM9DS1_PLATFORM_H_
#define LSM9DS1_PLATFORM_H_

#ifdef	__cplusplus
extern "C"
{
#endif


typedef struct {
  void   *hbus;
  uint8_t i2c_address;
  uint16_t cs_pin;
} sensbus_t;

extern sensbus_t mag_bus;
extern sensbus_t imu_bus;

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
int32_t platform_write_imu(void *handle, uint8_t reg,
                           const uint8_t *bufp, uint16_t len);
int32_t platform_read_imu(void *handle, uint8_t reg,
                          uint8_t *bufp, uint16_t len);
int32_t platform_write_mag(void *handle, uint8_t reg,
                           const uint8_t *bufp, uint16_t len);
int32_t platform_read_mag(void *handle, uint8_t reg,
                          uint8_t *bufp, uint16_t len);
void tx_com( uint8_t *tx_buffer, uint16_t len );
void platform_delay(uint32_t ms);
void platform_init(void);

#ifdef	__cplusplus
}
#endif

#endif /* LSM9DS1_PLATFORM_H_ */
