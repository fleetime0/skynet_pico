#include "app_oled.h"

#include <stdio.h>

#include "bsp.h"
#include "bsp_beep.h"
#include "bsp_led.h"
#include "bsp_ssd1306.h"

void oled_clear(void) { ssd1306_fill(SSD1306_COLOR_BLACK); }

void oled_refresh(void) { ssd1306_update_screen(); }

void oled_draw_string(char *data, uint8_t x, uint8_t y, bool clear, bool refresh) {
  if (clear)
    oled_clear();
  ssd1306_gotoxy(x, y);
  ssd1306_puts(data, &Font_7x10, SSD1306_COLOR_WHITE);
  if (refresh)
    oled_refresh();
}

void oled_draw_line(char *data, uint8_t line, bool clear, bool refresh) {
  if (line > 0 && line <= 6) {
    oled_draw_string(data, 0, 10 * (line - 1), clear, refresh);
  }
}

void oled_show_cartype(uint8_t v_major, uint8_t v_minor, uint8_t v_patch) {
  char text[20];
  oled_draw_line("SKYNET_CAR", 1, true, false);
  sprintf(text, "Version:V%d.%d.%d", v_major, v_minor, v_patch);
  oled_draw_line(text, 2, false, false);
  sprintf(text, "%s  %s", __DATE__, __TIME__);
  oled_draw_line(text, 3, false, true);
}

void oled_show_voltage(uint16_t bat_voltage) {
  char text[20];
  sprintf(text, "Voltage:%.1fV", bat_voltage / 10.0);
  oled_draw_line(text, 2, true, true);
}

void oled_show_imu(float yaw, float roll, float pitch) {
  char text[20];
  sprintf(text, "YAW:%.2f", yaw);
  oled_draw_line(text, 1, true, false);
  sprintf(text, "ROLL:%.2f", roll);
  oled_draw_line(text, 2, false, false);
  sprintf(text, "PITCH:%.2f", pitch);
  oled_draw_line(text, 3, false, true);
}

void oled_show_motor_speed(float m1, float m2) {
  char text[20];

  sprintf(text, "M1:%d, M2:%d", (int) (m1 / 10), (int) (m2 / 10));
  oled_draw_line(text, 2, true, true);
}

void oled_show_waiting(void) {
  char text[20];

  sprintf(text, "Version:V%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

  oled_draw_line(text, 2, true, false);

  sprintf(text, "Please Waiting..");
  oled_draw_line(text, 3, false, true);
}

void oled_show_error(void) {
  LED_OFF();
  BEEP_OFF();
  char text[20];

  sprintf(text, "Version:V%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

  oled_draw_line(text, 2, true, false);

  sprintf(text, "IMU ERROR!!!");
  oled_draw_line(text, 3, false, true);
}
