#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#define SKYNET_BAT_ADC_PIN 27
#define SKYNET_BAT_ADC_CH 1

void bsp_adc_init(void);
float adc_get_measure_volotage(void);
float adc_get_battery_volotage(void);

uint16_t adc_get_average(uint8_t times);

#endif // BSP_ADC_H
