#ifndef BSP_LED_H
#define BSP_LED_H

#include "pico/stdlib.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#if defined(PICO_DEFAULT_LED_PIN)
#define LED_ON() gpio_put(PICO_DEFAULT_LED_PIN, true)
#define LED_OFF() gpio_put(PICO_DEFAULT_LED_PIN, false)
#elif defined(CYW43_WL_GPIO_LED_PIN)
#define LED_ON() cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true)
#define LED_OFF() cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false)
#else
#define LED_ON() ((void) 0)
#endif

void bsp_led_init(void);
void bsp_led_show_state(void);
void bsp_led_show_low_battery(uint8_t enable_beep);
void bsp_led_show_overvoltage_battery(uint8_t enable_beep);

#endif // BSP_LED_H
