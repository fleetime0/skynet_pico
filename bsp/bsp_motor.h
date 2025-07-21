#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

#define PWM_M1_PIN 14
#define PWM_M2_PIN 15

#define M1_IN1_PIN 10
#define M1_IN2_PIN 11

#define M2_IN1_PIN 12
#define M2_IN2_PIN 13

#define MOTOR_MAX_PULSE 25000
#define MOTOR_FREQ_DIVIDE 1

// MOTOR: M1 M2
// MOTOR: L1 L2
typedef enum { MOTOR_ID_M1 = 0, MOTOR_ID_M2, MAX_MOTOR } Motor_ID;

void bsp_motor_init(uint16_t arr, uint16_t psc);
void motor_set_pwm(uint8_t id, int16_t speed);
void motor_stop(uint8_t brake);

#endif // BSP_MOTOR_H
