#include "app_motion.h"

#include "app_pid.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"

static int g_encoder_all_now[MAX_MOTOR] = {0};
static int g_encoder_all_last[MAX_MOTOR] = {0};

static int g_encoder_all_offset[MAX_MOTOR] = {0};

static float speed_fb = 0;
static float speed_spin = 0;
static int speed_l_setup = 0;
static int speed_r_setup = 0;

static uint8_t g_start_ctrl = 0;

static car_data_t car_data;
static motor_data_t motor_data;

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
//   // ROS 单位：m/s 和 rad/s
//   float vx_f = msg->linear.x;    // m/s
//   float vy_f = msg->linear.y;    // m/s （可忽略）
//   float wz_f = msg->angular.z;   // rad/s

//   // 转为整数速度，单位一致
//   int16_t vx = (int16_t)(vx_f * 1000.0f);     // m/s → mm/s 确实需要乘上1000
//   int16_t vy = 0;                             // 忽略横向速度
//   int16_t wz = (int16_t)(wz_f * 1000.0f);     // rad/s → 1000-scale

//   // 可选：限幅（保护机制）
//   if (vx > 1000) vx = 1000;
//   if (vx < -1000) vx = -1000;
//   if (wz > 3000) wz = 3000;      // 约 3rad/s
//   if (wz < -3000) wz = -3000;

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

//   geometry_msgs__msg__Twist twist_msg;

//   twist_msg.linear.x  = car->vx / 1000.0f;   // mm/s → m/s
//   twist_msg.linear.y  = car->vy / 1000.0f;
//   twist_msg.linear.z  = 0;

//   twist_msg.angular.x = 0;
//   twist_msg.angular.y = 0;
//   twist_msg.angular.z = car->vz / 1000.0f;            // 已是 rad/s，直接用

//   rcl_publish(&twist_publisher, &twist_msg, NULL);

void motion_get_speed(car_data_t *car) {
  int i = 0;
  float speed_mm[MAX_MOTOR] = {0};

  motion_get_encoder();

  for (i = 0; i < MAX_MOTOR; i++) {
    speed_mm[i] = (g_encoder_all_offset[i]) * 100 * WHEEL_PERIMETER / ENCODER_PRECISION;
  }

  car->vx = (speed_mm[0] + speed_mm[1]) / 2;
  car->vy = 0;
  car->vz = (speed_mm[0] - speed_mm[1]) / WHEEL_SPACE * 1000.0f;

  if (g_start_ctrl) {
    for (i = 0; i < MAX_MOTOR; i++) {
      motor_data.speed_mm_s[i] = speed_mm[i];
    }

    pid_calc_motor(&motor_data);
  }
}

void motion_set_speed(int16_t speed_m1, int16_t speed_m2) {
  g_start_ctrl = 1;
  motor_data.speed_set[0] = speed_m1;
  motor_data.speed_set[1] = speed_m2;
  for (uint8_t i = 0; i < MAX_MOTOR; i++) {
    pid_set_motor_target(i, motor_data.speed_set[i] * 1.0);
  }
}

void motion_handle(void) {
  motion_get_speed(&car_data);

  if (g_start_ctrl) {
    motion_set_pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1]);
  }
}
