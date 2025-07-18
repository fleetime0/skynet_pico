#ifndef APP_BAT_H
#define APP_BAT_H

#include <stdint.h>

typedef enum Battery_State { BATTERY_LOW, BATTERY_NORMAL, BATTERY_OVER_VOLTAGE } Battery_State_t;

void app_bat_init(void);
uint8_t system_enable(void);
uint8_t bat_state(void);
int bat_voltage_z10(void);
uint8_t bat_get_low_voltage(void);
uint8_t bat_get_over_voltage(void);
uint8_t bat_show_led_handle(uint8_t enable_beep);

#endif // APP_BAT_H
