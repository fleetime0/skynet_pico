#include "bsp_ssd1306.h"

#include <string.h>

#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define ssd1306_swap(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

#define SSD1306_MEMORYMODE 0x20 ///< See datasheet
#define SSD1306_COLUMNADDR 0x21 ///< See datasheet
#define SSD1306_PAGEADDR 0x22 ///< See datasheet
#define SSD1306_SETCONTRAST 0x81 ///< See datasheet
#define SSD1306_CHARGEPUMP 0x8D ///< See datasheet
#define SSD1306_SEGREMAP 0xA0 ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON 0xA5 ///< Not currently used
#define SSD1306_NORMALDISPLAY 0xA6 ///< See datasheet
#define SSD1306_INVERTDISPLAY 0xA7 ///< See datasheet
#define SSD1306_SETMULTIPLEX 0xA8 ///< See datasheet
#define SSD1306_DISPLAYOFF 0xAE ///< See datasheet
#define SSD1306_DISPLAYON 0xAF ///< See datasheet
#define SSD1306_COMSCANINC 0xC0 ///< Not currently used
#define SSD1306_COMSCANDEC 0xC8 ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET 0xD3 ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5 ///< See datasheet
#define SSD1306_SETPRECHARGE 0xD9 ///< See datasheet
#define SSD1306_SETCOMPINS 0xDA ///< See datasheet
#define SSD1306_SETVCOMDETECT 0xDB ///< See datasheet

#define SSD1306_SETLOWCOLUMN 0x00 ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE 0x40 ///< See datasheet

#define SSD1306_EXTERNALVCC 0x01 ///< External display voltage source
#define SSD1306_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26 ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27 ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL 0x2E ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL 0x2F ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3 ///< Set scroll range

#define SSD1306_MODE_COMMAND gpio_put(SKYNET_SSD1306_SPI_DC_PIN, 0)
#define SSD1306_MODE_DATA gpio_put(SKYNET_SSD1306_SPI_DC_PIN, 1)

static uint8_t ssd1306_buffer[SCREEN_WIDTH * ((SCREEN_HEIGHT + 7) / 8)];

typedef struct {
  uint16_t current_x;
  uint16_t current_y;
  uint8_t inverted;
  uint8_t initialized;
} SSD1306_t;

static SSD1306_t ssd1306;

static inline void spi_write(uint8_t d) { spi_write_blocking(SKYNET_SSD1306_SPI, &d, 1); }

static void ssd1306_command1(uint8_t c) {
  SSD1306_MODE_COMMAND;
  spi_write(c);
}

void ssd1306_commandList(const uint8_t *c, uint8_t n) {
  SSD1306_MODE_COMMAND;
  for (uint8_t i = 0; i < n; ++i) {
    spi_write(c[i]);
  }
}

void bsp_ssd1306_init(void) {
  spi_init(SKYNET_SSD1306_SPI, 10 * 1000 * 1000);
  gpio_set_function(SKYNET_SSD1306_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SKYNET_SSD1306_SPI_TX_PIN, GPIO_FUNC_SPI);

  bi_decl(bi_2pins_with_func(SKYNET_SSD1306_SPI_TX_PIN, SKYNET_SSD1306_SPI_SCK_PIN, GPIO_FUNC_SPI));

  gpio_init(SKYNET_SSD1306_SPI_DC_PIN);
  gpio_set_dir(SKYNET_SSD1306_SPI_DC_PIN, GPIO_OUT);

  gpio_init(SKYNET_SSD1306_SPI_RST_PIN);
  gpio_set_dir(SKYNET_SSD1306_SPI_RST_PIN, GPIO_OUT);

  gpio_put(SKYNET_SSD1306_SPI_RST_PIN, true);
  sleep_ms(1);
  gpio_put(SKYNET_SSD1306_SPI_RST_PIN, false);
  sleep_ms(10);
  gpio_put(SKYNET_SSD1306_SPI_RST_PIN, true);

  static const uint8_t init1[] = {SSD1306_DISPLAYOFF, // 0xAE
                                  SSD1306_SETDISPLAYCLOCKDIV, // 0xD5
                                  0x80, // the suggested ratio 0x80
                                  SSD1306_SETMULTIPLEX}; // 0xA8
  ssd1306_commandList(init1, sizeof(init1));
  ssd1306_command1(SCREEN_HEIGHT - 1);

  static const uint8_t init2[] = {SSD1306_SETDISPLAYOFFSET, // 0xD3
                                  0x0, // no offset
                                  SSD1306_SETSTARTLINE | 0x0, // line #0
                                  SSD1306_CHARGEPUMP}; // 0x8D
  ssd1306_commandList(init2, sizeof(init2));

  ssd1306_command1(0x14);

  static const uint8_t init3[] = {SSD1306_MEMORYMODE, // 0x20
                                  0x00, // 0x0 act like ks0108
                                  SSD1306_SEGREMAP | 0x1, SSD1306_COMSCANDEC};
  ssd1306_commandList(init3, sizeof(init3));

  uint8_t com_pins = 0x02;
  uint8_t contrast = 0x8F;

  if ((SCREEN_WIDTH == 128) && (SCREEN_HEIGHT == 32)) {
    com_pins = 0x02;
    contrast = 0x8F;
  } else if ((SCREEN_WIDTH == 128) && (SCREEN_HEIGHT == 64)) {
    com_pins = 0x12;
    contrast = 0xCF;
  } else if ((SCREEN_WIDTH == 96) && (SCREEN_HEIGHT == 16)) {
    com_pins = 0x2; // ada x12
    contrast = 0xAF;
  } else if ((SCREEN_WIDTH == 64) && (SCREEN_HEIGHT == 32)) {
    com_pins = 0x12; // ada x12
    contrast = 0xCF;
  } else {
    // Other screen varieties -- TBD
  }

  ssd1306_command1(SSD1306_SETCOMPINS);
  ssd1306_command1(com_pins);
  ssd1306_command1(SSD1306_SETCONTRAST);
  ssd1306_command1(contrast);

  ssd1306_command1(SSD1306_SETPRECHARGE); // 0xd9
  ssd1306_command1(0xF1);
  static const uint8_t init5[] = {SSD1306_SETVCOMDETECT, // 0xDB
                                  0x40,
                                  SSD1306_DISPLAYALLON_RESUME, // 0xA4
                                  SSD1306_NORMALDISPLAY, // 0xA6
                                  SSD1306_DEACTIVATE_SCROLL,
                                  SSD1306_DISPLAYON}; // Main screen turn on
  ssd1306_commandList(init5, sizeof(init5));

  ssd1306.current_x = 0;
  ssd1306.current_y = 0;

  ssd1306.initialized = 1;
}

void ssd1306_update_screen(void) {
  static const uint8_t dlist1[] = {SSD1306_PAGEADDR,
                                   0, // Page start address
                                   0xFF, // Page end (not really, but works here)
                                   SSD1306_COLUMNADDR}; // Column start address
  ssd1306_commandList(dlist1, sizeof(dlist1));

  if (SCREEN_WIDTH == 64) {
    ssd1306_command1(0x20); // Column start
    ssd1306_command1(0x20 + SCREEN_WIDTH - 1); // Column end address
  } else {
    ssd1306_command1(0); // Column start
    ssd1306_command1((SCREEN_WIDTH - 1)); // Column end address
  }

  SSD1306_MODE_DATA;
  spi_write_blocking(SKYNET_SSD1306_SPI, ssd1306_buffer, SCREEN_WIDTH * ((SCREEN_HEIGHT + 7) / 8));
}

void ssd1306_fill(SSD1306_COLOR_t color) {
  memset(ssd1306_buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xff, SCREEN_WIDTH * ((SCREEN_HEIGHT + 7) / 8));
}

void ssd1306_draw_pixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
  if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
    return; // 出错，超出范围
  }

  /* 检查像素是否倒置 */
  if (ssd1306.inverted) {
    color = (SSD1306_COLOR_t) !color;
  }

  /* 设置颜色 */
  if (color == SSD1306_COLOR_WHITE) {
    ssd1306_buffer[x + (y / 8) * SCREEN_WIDTH] |= 1 << (y % 8);
  } else {
    ssd1306_buffer[x + (y / 8) * SCREEN_WIDTH] &= ~(1 << (y % 8));
  }
}

void ssd1306_gotoxy(uint16_t x, uint16_t y) {
  ssd1306.current_x = x;
  ssd1306.current_y = y;
}

char ssd1306_putc(char ch, FontDef_t *Font, SSD1306_COLOR_t color) {
  uint32_t i, b, j;

  if (SCREEN_WIDTH <= (ssd1306.current_x + Font->FontWidth) ||
      SCREEN_HEIGHT <= (ssd1306.current_y + Font->FontHeight)) {
    return 0; // 出错，超出范围
  }

  for (i = 0; i < Font->FontHeight; i++) {
    b = Font->data[(ch - 32) * Font->FontHeight + i];
    for (j = 0; j < Font->FontWidth; j++) {
      if ((b << j) & 0x8000) {
        ssd1306_draw_pixel(ssd1306.current_x + j, (ssd1306.current_y + i), (SSD1306_COLOR_t) color);
      } else {
        ssd1306_draw_pixel(ssd1306.current_x + j, (ssd1306.current_y + i), (SSD1306_COLOR_t) !color);
      }
    }
  }

  /* Increase pointer */
  ssd1306.current_x += Font->FontWidth;

  /* Return character written */
  return ch;
}

char ssd1306_puts(char *str, FontDef_t *Font, SSD1306_COLOR_t color) {
  /* Write characters */
  while (*str) {
    /* Write character by character */
    if (ssd1306_putc(*str, Font, color) != *str) {
      /* Return error */
      return *str;
    }

    /* Increase string pointer */
    str++;
  }

  /* Everything OK, zero should be returned */
  return *str;
}
