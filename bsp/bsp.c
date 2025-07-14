#include "bsp.h"

#include <stdio.h>

#include "bsp_adc.h"
#include "bsp_beep.h"
#include "bsp_icm_i2c.h"
#include "bsp_key.h"
#include "bsp_led.h"

void bsp_init(void) {
  printf("Firmware Version: V%d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
  printf("Firmware Compiled: %s, %s\n", __DATE__, __TIME__);

  bsp_led_init();
  bsp_beep_init();
  bsp_key_init();
  bsp_adc_init();
  bsp_icm_i2c_init();
}
