#ifndef APP_PID_H
#define APP_PID_H

#include <stdint.h>

#define PID_DEF_KP 0.0f
#define PID_DEF_KI 1.0f
#define PID_DEF_KD 0.0f

typedef struct _pid {
  float target_val;
  float pwm_output;
  float Kp, Ki, Kd;
  float err;
  float err_last;
  float err_next;
} Pid_t;

typedef struct _motor_data_t {
  float speed_mm_s[2];
  float speed_pwm[2];
  int16_t speed_set[2];
} motor_data_t;

void pid_param_init(void);
float pid_calc_motor(uint8_t motor_id, float speed_mm_s);
void pid_set_motor_target(uint8_t motor_id, float target);
void pid_clear_motor(uint8_t motor_id);
void pid_set_motor_parm(uint8_t motor_id, float kp, float ki, float kd);
void pid_get_motor_param(uint8_t motor_id, float *kp, float *ki, float *kd);

#endif // APP_PID_H
