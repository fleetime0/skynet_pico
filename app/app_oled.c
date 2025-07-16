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
