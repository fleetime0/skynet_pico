#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#define M1_AB_PIN 6
#define M2_AB_PIN 8

void bsp_encoder_init(void);
void encoder_update_count(void);
void encoder_get_all(int *encoder_all);

#endif // BSP_ENCODER_H
