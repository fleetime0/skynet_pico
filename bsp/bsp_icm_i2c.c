#include "bsp_icm_i2c.h"

#include "pico/binary_info.h"
#include "pico/stdlib.h"

void bsp_icm_i2c_init(void) {
  i2c_init(SKYNET_ICM_I2C, SKYNET_ICM_I2C_CLK);
  gpio_set_function(SKYNET_ICM_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SKYNET_ICM_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SKYNET_ICM_I2C_SDA_PIN);
  gpio_pull_up(SKYNET_ICM_I2C_SCL_PIN);
  bi_decl(bi_2pins_with_func(SKYNET_ICM_I2C_SDA_PIN, SKYNET_ICM_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

int bsp_icm_i2c_write(uint8_t reg, const uint8_t *wbuffer, uint32_t wlen) {
  uint8_t buffer[1 + wlen];
  buffer[0] = reg;
  for (uint32_t i = 0; i < wlen; ++i) {
    buffer[i + 1] = wbuffer[i];
  }

  int ret = i2c_write_blocking(SKYNET_ICM_I2C, SKYNET_ICM_I2C_ADDR, buffer, wlen + 1, false);
  return (ret == (int) (wlen + 1)) ? 0 : -1;
}

int bsp_icm_i2c_read(uint8_t reg, uint8_t *rbuffer, uint32_t rlen) {
  int ret = i2c_write_blocking(SKYNET_ICM_I2C, SKYNET_ICM_I2C_ADDR, &reg, 1, true);
  if (ret != 1)
    return -1;

  ret = i2c_read_blocking(SKYNET_ICM_I2C, SKYNET_ICM_I2C_ADDR, rbuffer, rlen, false);
  return (ret == (int) rlen) ? 0 : -1;
}
