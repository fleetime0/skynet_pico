#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "app.h"
#include "bsp.h"

int main() {
  stdio_init_all();
  bsp_init();
  app_init();

  while (true)
    ;
}
