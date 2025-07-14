#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "app.h"
#include "bsp.h"

#include "MadgwickAHRS.h"
#include "icm45686.h"

static float gyro_bias[3] = {0}; // 零偏
static bool bias_initialized = false;

void update_imu_pose(const imu_data_t *imu_data, float *roll_deg, float *pitch_deg, float *yaw_deg) {
  // === 1. 参数缩放因子 ===
  const float accel_scale_mps2 = 4.0f / 32768.0f * 9.80665f; // ±4g 转 m/s²
  const float gyro_scale_dps = 1000.0f / 32768.0f; // ±1000dps
  const float deg2rad = M_PI / 180.0f;

  // === 2. 加速度值（转成 g） ===
  float ax = imu_data->accel_data[0] * accel_scale_mps2 / 9.80665f;
  float ay = imu_data->accel_data[1] * accel_scale_mps2 / 9.80665f;
  float az = imu_data->accel_data[2] * accel_scale_mps2 / 9.80665f;

  // === 3. 陀螺仪值（转成 rad/s） ===
  float gx = (imu_data->gyro_data[0] - gyro_bias[0]) * gyro_scale_dps * deg2rad;
  float gy = (imu_data->gyro_data[1] - gyro_bias[1]) * gyro_scale_dps * deg2rad;
  float gz = (imu_data->gyro_data[2] - gyro_bias[2]) * gyro_scale_dps * deg2rad;

  // === 4. 获取时间差 dt（秒） ===
  static absolute_time_t last_time;
  static bool initialized = false;
  float dt = 0.01f; // 默认 100Hz
  absolute_time_t now = get_absolute_time();
  if (initialized) {
    dt = absolute_time_diff_us(last_time, now) / 1e6f;
  } else {
    initialized = true;
  }
  last_time = now;

  // === 5. 姿态融合 ===
  MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);

  // === 7. 转换为欧拉角（单位：度） ===
  *roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
  *pitch_deg = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
  *yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}



void calibrate_gyro_bias(void) {
  const int samples = 200;
  int32_t sum[3] = {0};

  for (int i = 0; i < samples; i++) {
    imu_data_t imu_data;
    get_data_from_reg(&imu_data);
    sum[0] += imu_data.gyro_data[0];
    sum[1] += imu_data.gyro_data[1];
    sum[2] += imu_data.gyro_data[2];
    sleep_ms(10); // 100Hz
  }

  gyro_bias[0] = sum[0] / (float) samples;
  gyro_bias[1] = sum[1] / (float) samples;
  gyro_bias[2] = sum[2] / (float) samples;

  bias_initialized = true;
  printf("Gyro bias calibrated: %.2f, %.2f, %.2f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

int main() {
  stdio_init_all();

  bsp_init();
  app_init();

  printf("Calibrating gyro bias...\n");
  calibrate_gyro_bias(); // 增加这行

  imu_data_t imu_data;
  float roll, pitch, yaw;
  int cnt = 0;
  while (true) {
    get_data_from_reg(&imu_data);
    update_imu_pose(&imu_data, &roll, &pitch, &yaw);

    if (cnt == 9) {
      printf("Euler Angle: R=%.2f P=%.2f Y=%.2f\n", roll, pitch, yaw);
      cnt = 0;
    } else {
      cnt++;
    }
    sleep_ms(10); // 100Hz
  }
}
