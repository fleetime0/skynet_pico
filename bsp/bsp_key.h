#ifndef SKYNET_BSP_KEY_H
#define SKYNET_BSP_KEY_H

#include <stdint.h>

#define SKYNET_KEY_PIN 16

#define KEY_PRESS 1
#define KEY_RELEASE 0

#define KEY_MODE_ONE_TIME 1
#define KEY_MODE_ALWAYS 0

void bsp_key_init(void);
uint8_t bsp_key_state(uint8_t mode);
uint8_t bsp_key_long_press(uint16_t timeout);

#endif // SKYNET_BSP_KEY_H
