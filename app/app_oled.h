#ifndef APP_OLED_H
#define APP_OLED_H

#include <stdint.h>

#include "pico/stdlib.h"

void oled_clear(void);
void oled_refresh(void);
void oled_draw_string(char *data, uint8_t x, uint8_t y, bool clear, bool refresh);
void oled_draw_line(char *data, uint8_t line, bool clear, bool refresh);
void oled_show_waiting(void);
void oled_show_error(void);

#endif // APP_OLED_H
