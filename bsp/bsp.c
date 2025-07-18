#include "bsp.h"

#include <stdio.h>

#include "hardware/watchdog.h"
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#include "bsp_adc.h"
#include "bsp_beep.h"
#include "bsp_encoder.h"
#include "bsp_icm_i2c.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_ssd1306.h"

extern void oled_show_waiting(void);

void bsp_init(void) {
#ifdef CYW43_WL_GPIO_LED_PIN
  cyw43_arch_init();
#endif

  printf("Firmware Version: V%d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
  printf("Firmware Compiled: %s, %s\n", __DATE__, __TIME__);

  bsp_led_init();
  bsp_beep_init();
  bsp_key_init();
  bsp_adc_init();

  sleep_ms(50);
  BEEP_OFF();

  bsp_ssd1306_init();
  oled_show_waiting();

  bsp_icm_i2c_init();

  bsp_motor_init(MOTOR_MAX_PULSE, MOTOR_FREQ_DIVIDE);

  bsp_encoder_init();
}

void bsp_reset_mcu(void) {
  printf("Reset MCU\n");
  watchdog_reboot(0, 0, 0);
}
