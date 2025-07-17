#include "app_pid.h"

#include "bsp_motor.h"

#define PI 3.1415926f

static Pid_t pid_motor[2];

static float pid_incre_calc(Pid_t *pid, float actual_val) {
  pid->err = pid->target_val - actual_val;
  pid->pwm_output += pid->Kp * (pid->err - pid->err_next) + pid->Ki * pid->err +
                     pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);

  pid->err_last = pid->err_next;
  pid->err_next = pid->err;

  if (pid->pwm_output > MOTOR_MAX_PULSE)
    pid->pwm_output = MOTOR_MAX_PULSE;
  if (pid->pwm_output < -MOTOR_MAX_PULSE)
    pid->pwm_output = -MOTOR_MAX_PULSE;

  return pid->pwm_output;
}

void pid_param_init(void) {
  for (int i = 0; i < MAX_MOTOR; i++) {
    pid_motor[i].target_val = 0.0;
    pid_motor[i].pwm_output = 0.0;
    pid_motor[i].err = 0.0;
    pid_motor[i].err_last = 0.0;
    pid_motor[i].err_next = 0.0;

    pid_motor[i].Kp = PID_DEF_KP;
    pid_motor[i].Ki = PID_DEF_KI;
    pid_motor[i].Kd = PID_DEF_KD;
  }
}

void pid_calc_motor(motor_data_t *motor) {
  int i;

  for (i = 0; i < MAX_MOTOR; i++) {
    motor->speed_pwm[i] = pid_incre_calc(&pid_motor[i], motor->speed_mm_s[i]);
  }
}

void pid_clear_motor(uint8_t motor_id) {
  if (motor_id > MAX_MOTOR)
    return;

  if (motor_id == MAX_MOTOR) {
    for (int i = 0; i < MAX_MOTOR; i++) {
      pid_motor[i].pwm_output = 0.0;
      pid_motor[i].err = 0.0;
      pid_motor[i].err_last = 0.0;
      pid_motor[i].err_next = 0.0;
    }
  } else {
    pid_motor[motor_id].pwm_output = 0.0;
    pid_motor[motor_id].err = 0.0;
    pid_motor[motor_id].err_last = 0.0;
    pid_motor[motor_id].err_next = 0.0;
  }
}

void pid_set_motor_target(uint8_t motor_id, float target) {
  if (motor_id > MAX_MOTOR)
    return;

  if (motor_id == MAX_MOTOR) {
    for (int i = 0; i < MAX_MOTOR; i++) {
      pid_motor[i].target_val = target;
    }
  } else {
    pid_motor[motor_id].target_val = target;
  }
}

void pid_set_motor_parm(uint8_t motor_id, float kp, float ki, float kd) {
  if (motor_id > MAX_MOTOR)
    return;

  if (motor_id == MAX_MOTOR) {
    for (int i = 0; i < MAX_MOTOR; i++) {
      pid_motor[i].Kp = kp;
      pid_motor[i].Ki = ki;
      pid_motor[i].Kd = kd;
    }
    // DEBUG("PID Set:%.3f, %.3f, %.3f\n", kp, ki, kd);
  } else {
    pid_motor[motor_id].Kp = kp;
    pid_motor[motor_id].Ki = ki;
    pid_motor[motor_id].Kd = kd;
    // DEBUG("PID Set M%d:%.3f, %.3f, %.3f\n", motor_id + 1, kp, ki, kd);
  }
}
