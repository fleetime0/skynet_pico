#include "bsp_led.h"

#include "bsp_beep.h"

void bsp_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
}

void bsp_led_show_state(void) {
  static uint8_t led_flash = 0;
  led_flash++;
  if (led_flash >= 30)
    led_flash = 0;

  if (led_flash >= 1 && led_flash < 4) {
    LED_ON();
  } else if (led_flash >= 4 && led_flash < 7) {
    LED_OFF();
  } else if (led_flash >= 7 && led_flash < 10) {
    LED_ON();
  } else if (led_flash >= 10 && led_flash < 13) {
    LED_OFF();
  }
}

void bsp_led_show_low_battery(uint8_t enable_beep) {
  static uint8_t led_flash_1 = 0;
  if (led_flash_1) {
    if (enable_beep)
      BEEP_ON();
    LED_ON();
    led_flash_1 = 0;
  } else {
    BEEP_OFF();
    LED_OFF();
    led_flash_1 = 1;
  }
}

void bsp_led_show_overvoltage_battery(uint8_t enable_beep) {
  static uint8_t time_count = 0;
  static uint8_t state = 0;
  time_count++;
  if (time_count > 5) {
    if (state == 0) {
      if (enable_beep)
        BEEP_ON();
      LED_ON();
      state = 1;
    } else {
      BEEP_OFF();
      LED_OFF();
      state = 0;
    }
    time_count = 0;
  }
}
