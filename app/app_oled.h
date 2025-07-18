#ifndef APP_OLED_H
#define APP_OLED_H

#include <stdint.h>

#include "pico/stdlib.h"

typedef enum en_OLED_FLAG {
  OLED_FLAG_NO_DISPLAY = 0,
  OLED_FLAG_IMU,
  OLED_FLAG_VOLTAGE,
  OLED_FLAG_MOTOR_SPEED,

  OLED_MAX_FLAG
} OLED_FLAG;

void oled_clear(void);
void oled_refresh(void);
void oled_draw_string(char *data, uint8_t x, uint8_t y, bool clear, bool refresh);
void oled_draw_line(char *data, uint8_t line, bool clear, bool refresh);
void oled_show_voltage(uint16_t bat_voltage);
void oled_show_cartype(uint8_t v_major, uint8_t v_minor, uint8_t v_patch);
void oled_show_imu(float yaw, float roll, float pitch);
void oled_show_motor_speed(float m1, float m2);
void oled_show_waiting(void);
void oled_show_error(void);

#endif // APP_OLED_H
