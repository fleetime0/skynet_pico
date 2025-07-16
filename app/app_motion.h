#ifndef APP_MOTION_H
#define APP_MOTION_H

#include <stdint.h>

#define ENCODER_PRECISION 60000.0f
#define WHEEL_PERIMETER 204.203f

#define WHEEL_SPACE 177

typedef enum _stop_mode { STOP_FREE = 0, STOP_BRAKE } stop_mode_t;

typedef struct _car_data {
  int16_t vx;
  int16_t vy;
  int16_t vz;
} car_data_t;

void motion_stop(uint8_t brake);
void motion_set_pwm(int16_t motor_1, int16_t motor_2);
void motion_ctrl(int16_t v_x, int16_t v_y, int16_t v_z);
void motion_get_encoder(void);
void motion_get_speed(car_data_t *car);
void motion_set_speed(int16_t speed_m1, int16_t speed_m2);
void motion_handle(void);

#endif // APP_MOTION_H
