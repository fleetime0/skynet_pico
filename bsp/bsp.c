#include "bsp.h"

#include <stdio.h>

void bsp_init(void) {
  printf("Firmware Version: V%d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
  printf("Firmware Compiled: %s, %s\n", __DATE__, __TIME__);

  bsp_led_init();
  bsp_beep_init();
  bsp_key_init();
  bsp_adc_init();
}
