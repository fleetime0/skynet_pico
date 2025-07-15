#include "bsp_beep.h"

static uint16_t g_beep_on_time = 0;
static uint8_t g_beep_state = 0;

static void beep_set_time(uint16_t time) { g_beep_on_time = time; }

static uint16_t beep_get_time(void) { return g_beep_on_time; }

static void beep_set_state(uint8_t state) { g_beep_state = state; }

static uint8_t beep_get_state(void) { return g_beep_state; }

void bsp_beep_init(void) {
  gpio_init(SKYNET_BEEP_PIN);
  gpio_set_drive_strength(SKYNET_BEEP_PIN, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_dir(SKYNET_BEEP_PIN, GPIO_OUT);
}

void long_beep_alarm(void) {
  BEEP_ON();
  sleep_ms(1000);
  BEEP_OFF();
}

void beep_on_time(uint16_t time) {
  if (time == BEEP_STATE_ON_ALWAYS) {
    beep_set_state(BEEP_STATE_ON_ALWAYS);
    beep_set_time(0);
    BEEP_ON();
  } else if (time == BEEP_STATE_OFF) {
    beep_set_state(BEEP_STATE_OFF);
    beep_set_time(0);
    BEEP_OFF();
  } else {
    if (time >= 10) {
      beep_set_state(BEEP_STATE_ON_DELAY);
      beep_set_time(time / 10);
      BEEP_ON();
    }
  }
}

void beep_timeout_close_handle(void) {
  if (beep_get_state() == BEEP_STATE_ON_DELAY) {
    if (beep_get_time()) {
      g_beep_on_time--;
    } else {
      BEEP_OFF();
      beep_set_state(BEEP_STATE_OFF);
    }
  }
}
