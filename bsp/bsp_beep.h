#ifndef SKYNET_BSP_BEEP_H
#define SKYNET_BSP_BEEP_H

#include "pico/stdlib.h"

#define SKYNET_BEEP_PIN 26

#define BEEP_ON() gpio_put(SKYNET_BEEP_PIN, true)
#define BEEP_OFF() gpio_put(SKYNET_BEEP_PIN, false)

#define BEEP_STATE_OFF 0
#define BEEP_STATE_ON_ALWAYS 1
#define BEEP_STATE_ON_DELAY 2

void bsp_beep_init(void);
void bsp_long_beep_alarm(void);
void bsp_beep_on_time(uint16_t time);
void bsp_beep_timeout_close_handle(void);

#endif // SKYNET_BSP_BEEP_H
