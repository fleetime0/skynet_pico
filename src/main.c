#include <stdio.h>

#include "pico/stdlib.h"

#include "app.h"
#include "bsp.h"

int main() {
  stdio_init_all();
  bsp_init();
  app_init();

  sleep_ms(2);
  printf("Hello skynet!\n");

  app_start_freertos();

  while (true)
    ;
}
