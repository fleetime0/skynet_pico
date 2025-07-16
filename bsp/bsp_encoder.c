#include "bsp_encoder.h"

#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "quadrature_encoder.pio.h"

static int g_encoder_m1_now = 0;
static int g_encoder_m2_now = 0;

void bsp_encoder_init(void) {
  PIO pio = pio0;

  pio_add_program(pio, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio, 0, M1_AB_PIN, 0);
  quadrature_encoder_program_init(pio, 1, M2_AB_PIN, 0);
}

void encoder_update_count(void) {
  PIO pio = pio0;
  g_encoder_m1_now = quadrature_encoder_get_count(pio, 0);
  g_encoder_m2_now = quadrature_encoder_get_count(pio, 1);
}

void encoder_get_all(int *encoder_all) {
  encoder_all[0] = g_encoder_m1_now;
  encoder_all[1] = -g_encoder_m2_now;
}
