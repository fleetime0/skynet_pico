#ifndef SKYNET_BSP_ADC_H
#define SKYNET_BSP_ADC_H

#include <stdint.h>

#define SKYNET_BAT_ADC_PIN 27
#define SKYNET_BAT_ADC_CH 1

void bsp_adc_init(void);
float bsp_adc_get_measure_volotage(void);
float bsp_adc_get_battery_volotage(void);

uint16_t bsp_adc_get_average(uint8_t times);

#endif // SKYNET_BSP_ADC_H
