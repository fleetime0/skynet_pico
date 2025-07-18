#include "app_motion.h"

#include "app_pid.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"

#include "FreeRTOS.h"
#include "semphr.h"

static int g_encoder_all_now[MAX_MOTOR] = {0};
static int g_encoder_all_last[MAX_MOTOR] = {0};

static int g_encoder_all_offset[MAX_MOTOR] = {0};

static float speed_fb = 0;
static float speed_spin = 0;
static int speed_l_setup = 0;
static int speed_r_setup = 0;

static uint8_t g_start_ctrl = 0;

static motor_data_t motor_data;
static SemaphoreHandle_t motor_data_mutex;

static car_data_t car_data;
static SemaphoreHandle_t car_data_mutex;

static void motion_update_speed() {
  int i = 0;
  float speed_mm[MAX_MOTOR] = {0};
  float speed_pwm[MAX_MOTOR] = {0};

  motion_get_encoder();

  for (i = 0; i < MAX_MOTOR; i++) {
    speed_mm[i] = (g_encoder_all_offset[i]) * 100 * WHEEL_PERIMETER / ENCODER_PRECISION;
  }

  xSemaphoreTake(car_data_mutex, portMAX_DELAY);
  car_data.vx = (speed_mm[0] + speed_mm[1]) / 2;
  car_data.vy = 0;
  car_data.vz = (speed_mm[0] - speed_mm[1]) / WHEEL_SPACE * 1000.0f;
  xSemaphoreGive(car_data_mutex);

  if (g_start_ctrl) {
    xSemaphoreTake(motor_data_mutex, portMAX_DELAY);
    for (i = 0; i < MAX_MOTOR; i++) {
      motor_data.speed_mm_s[i] = speed_mm[i];
    }
    xSemaphoreGive(motor_data_mutex);

    for (i = 0; i < MAX_MOTOR; i++) {
      speed_pwm[i] = pid_calc_motor(i, motor_data.speed_mm_s[i]);
    }
    xSemaphoreTake(motor_data_mutex, portMAX_DELAY);
    for (i = 0; i < MAX_MOTOR; i++) {
      motor_data.speed_pwm[i] = speed_pwm[i];
    }
    xSemaphoreGive(motor_data_mutex);
  }
}

void motion_init(void) {
  motor_data_mutex = xSemaphoreCreateMutex();
  car_data_mutex = xSemaphoreCreateMutex();
}

void motion_stop(uint8_t brake) {
  motion_set_speed(0, 0);
  pid_clear_motor(MAX_MOTOR);
  g_start_ctrl = 0;
  motor_stop(brake);
}

void motion_set_pwm(int16_t motor_1, int16_t motor_2) {
  if (motor_1 >= -MOTOR_MAX_PULSE && motor_1 <= MOTOR_MAX_PULSE) {
    motor_set_pwm(MOTOR_ID_M1, motor_1);
  }
  if (motor_2 >= -MOTOR_MAX_PULSE && motor_2 <= MOTOR_MAX_PULSE) {
    motor_set_pwm(MOTOR_ID_M2, motor_2);
  }
}

void motion_ctrl(int16_t v_x, int16_t v_y, int16_t v_z) {
  float robot_APB = WHEEL_SPACE / 2;
  speed_fb = v_x;
  v_y = 0;
  speed_spin = (v_z / 1000.0f) * robot_APB;
  if (v_x == 0 && v_y == 0 && v_z == 0) {
    motion_stop(STOP_BRAKE);
    return;
  }

  speed_l_setup = speed_fb - speed_spin;
  speed_r_setup = speed_fb + speed_spin;

  if (speed_l_setup > 1000)
    speed_l_setup = 1000;
  if (speed_l_setup < -1000)
    speed_l_setup = -1000;
  if (speed_r_setup > 1000)
    speed_r_setup = 1000;
  if (speed_r_setup < -1000)
    speed_r_setup = -1000;

  motion_set_speed(speed_l_setup, speed_r_setup);
}

void motion_get_encoder(void) {
  encoder_get_all(g_encoder_all_now);

  for (uint8_t i = 0; i < MAX_MOTOR; i++) {
    g_encoder_all_offset[i] = g_encoder_all_now[i] - g_encoder_all_last[i];
    g_encoder_all_last[i] = g_encoder_all_now[i];
  }
}

void motion_get_speed(car_data_t *car) {
  xSemaphoreTake(car_data_mutex, portMAX_DELAY);
  car->vx = car_data.vx;
  car->vy = car_data.vy;
  car->vz = car_data.vz;
  xSemaphoreGive(car_data_mutex);
}

void motion_set_speed(int16_t speed_m1, int16_t speed_m2) {
  g_start_ctrl = 1;
  xSemaphoreTake(motor_data_mutex, portMAX_DELAY);
  motor_data.speed_set[0] = speed_m1;
  motor_data.speed_set[1] = speed_m2;
  xSemaphoreGive(motor_data_mutex);
  for (uint8_t i = 0; i < MAX_MOTOR; i++) {
    pid_set_motor_target(i, motor_data.speed_set[i] * 1.0);
  }
}

void motion_get_motor_speed(float *speed) {
  xSemaphoreTake(motor_data_mutex, portMAX_DELAY);
  for (int i = 0; i < MAX_MOTOR; i++) {
    speed[i] = motor_data.speed_mm_s[i];
  }
  xSemaphoreGive(motor_data_mutex);
}

void motion_handle(void) {
  motion_update_speed();

  if (g_start_ctrl) {
    motion_set_pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1]);
  }
}
