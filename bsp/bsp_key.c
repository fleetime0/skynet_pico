#include "bsp_key.h"

#include <stdio.h>

#include "pico/stdlib.h"

static uint16_t g_key_long_press = 0;

static uint8_t key_is_press(void) {
  if (!gpio_get(SKYNET_KEY_PIN)) {
    return KEY_PRESS;
  }
  return KEY_RELEASE;
}

void bsp_key_init(void) {
  gpio_init(SKYNET_KEY_PIN);
  gpio_set_dir(SKYNET_KEY_PIN, GPIO_IN);
  gpio_pull_up(SKYNET_KEY_PIN);
}

uint8_t key_long_press(uint16_t timeout) {
  if (g_key_long_press > 0) {
    if (g_key_long_press < timeout * 100 + 2) {
      g_key_long_press++;
      if (g_key_long_press == timeout * 100 + 2) {
        printf("key long press\n");
        return 1;
      }
      return 0;
    }
  }
  return 0;
}

uint8_t key_state(uint8_t mode) {
  static uint16_t key1_state = 0;

  if (key_is_press() == KEY_PRESS) {
    if (key1_state < (mode + 1) * 2) {
      key1_state++;
    }
  } else {
    key1_state = 0;
    g_key_long_press = 0;
  }
  if (key1_state == 2) {
    g_key_long_press = 1;
    return KEY_PRESS;
  }
  return KEY_RELEASE;
}
