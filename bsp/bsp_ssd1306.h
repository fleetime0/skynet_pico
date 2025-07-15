#ifndef BSP_SSD1306_H
#define BSP_SSD1306_H

#include <stdint.h>

#include "hardware/spi.h"

#include "oled_fonts.h"

#define SKYNET_SSD1306_SPI spi0
#define SKYNET_SSD1306_SPI_TX_PIN 19
#define SKYNET_SSD1306_SPI_SCK_PIN 18
#define SKYNET_SSD1306_SPI_DC_PIN 21
#define SKYNET_SSD1306_SPI_RST_PIN 20

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

typedef enum {
  SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
  SSD1306_COLOR_WHITE = 0x01 /*!< Pixel is set. Color depends on LCD */
} SSD1306_COLOR_t;

void bsp_ssd1306_init(void);
void ssd1306_update_screen(void);
void ssd1306_fill(SSD1306_COLOR_t color);
void ssd1306_draw_pixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);
void ssd1306_gotoxy(uint16_t x, uint16_t y);
char ssd1306_putc(char ch, FontDef_t *Font, SSD1306_COLOR_t color);
char ssd1306_puts(char *str, FontDef_t *Font, SSD1306_COLOR_t color);

#endif // BSP_SSD1306_H
