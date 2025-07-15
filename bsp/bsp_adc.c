#include "bsp_adc.h"

#include "hardware/adc.h"

void bsp_adc_init(void) {
  adc_init();
  adc_gpio_init(SKYNET_BAT_ADC_PIN);
  adc_select_input(SKYNET_BAT_ADC_CH);
}

uint16_t adc_get_average(uint8_t times) {
  uint16_t temp_val = 0;
  uint8_t t;
  for (t = 0; t < times; t++) {
    temp_val += adc_read();
  }
  if (times == 4) {
    temp_val = temp_val >> 2;
  } else {
    temp_val = temp_val / times;
  }
  return temp_val;
}

float adc_get_measure_volotage(void) {
  uint16_t adcx;
  float temp;
  adcx = adc_get_average(4);
  temp = (float) adcx * (3.30f / 4096);
  return temp;
}

float adc_get_battery_volotage(void) {
  float temp;
  temp = adc_get_measure_volotage();
  temp = temp * 11.0f;
  return temp;
}
