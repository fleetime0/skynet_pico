#include "app_oled.h"

#include "bsp_ssd1306.h"

void oled_clear(void) { ssd1306_fill(SSD1306_COLOR_BLACK); }

void oled_refresh(void) { ssd1306_update_screen(); }
