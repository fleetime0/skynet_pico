#ifndef BSP_ICM_I2C_H
#define BSP_ICM_I2C_H

#include <stdint.h>

#include "hardware/i2c.h"

#define SKYNET_ICM_I2C i2c0
#define SKYNET_ICM_I2C_SDA_PIN 0
#define SKYNET_ICM_I2C_SCL_PIN 1
#define SKYNET_ICM_I2C_ADDR 0x69
#define SKYNET_ICM_I2C_CLK 400000

void bsp_icm_i2c_init(void);
int icm_i2c_write(uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);
int icm_i2c_read(uint8_t reg, uint8_t *rbuffer, uint32_t rlen);

#endif // BSP_ICM_I2C_H
