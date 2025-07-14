#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "app.h"
#include "bsp.h"
#include "MadgwickAHRS.h"
#include "icm45686.h"

// === 全局变量 ===
static float gyro_bias[3] = {0}; // 陀螺仪零偏
static float roll_offset = 0, pitch_offset = 0, yaw_offset = 0;
static bool pose_offset_initialized = false;

// === 姿态归零重置 ===
void reset_pose_offset(void) {
    pose_offset_initialized = false;
}

// === 姿态估计 ===
void update_imu_pose(const imu_data_t *imu_data, float *roll_deg, float *pitch_deg, float *yaw_deg) {
    // 缩放系数
    const float accel_scale_mps2 = 4.0f / 32768.0f * 9.80665f;
    const float gyro_scale_dps = 1000.0f / 32768.0f;
    const float deg2rad = M_PI / 180.0f;

    // 加速度 -> g
    float ax = imu_data->accel_data[0] * accel_scale_mps2 / 9.80665f;
    float ay = imu_data->accel_data[1] * accel_scale_mps2 / 9.80665f;
    float az = imu_data->accel_data[2] * accel_scale_mps2 / 9.80665f;

    // 陀螺仪 -> rad/s（减去零偏）
    float gx = (imu_data->gyro_data[0] - gyro_bias[0]) * gyro_scale_dps * deg2rad;
    float gy = (imu_data->gyro_data[1] - gyro_bias[1]) * gyro_scale_dps * deg2rad;
    float gz = (imu_data->gyro_data[2] - gyro_bias[2]) * gyro_scale_dps * deg2rad;

    // 时间差 dt
    static absolute_time_t last_time;
    static bool initialized = false;
    float dt = 0.01f; // 默认值（初次）
    absolute_time_t now = get_absolute_time();
    if (initialized)
        dt = absolute_time_diff_us(last_time, now) / 1e6f;
    else
        initialized = true;
    last_time = now;

    // 姿态融合
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);

    // 获取欧拉角（度）
    float raw_roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                            1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    float raw_pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    float raw_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                           1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;

    // 姿态初始归零处理
    if (!pose_offset_initialized) {
        roll_offset = raw_roll;
        pitch_offset = raw_pitch;
        yaw_offset = raw_yaw;
        pose_offset_initialized = true;
    }

    *roll_deg = raw_roll - roll_offset;
    *pitch_deg = raw_pitch - pitch_offset;
    *yaw_deg = raw_yaw - yaw_offset;
}

// === 陀螺仪零偏校准 ===
void calibrate_gyro_bias(void) {
    const int samples = 300;
    int32_t sum[3] = {0};

    printf("Please keep IMU still...\n");
    for (int i = 0; i < samples; i++) {
        imu_data_t imu_data;
        get_data_from_reg(&imu_data);
        sum[0] += imu_data.gyro_data[0];
        sum[1] += imu_data.gyro_data[1];
        sum[2] += imu_data.gyro_data[2];
        sleep_ms(10); // 100Hz
    }

    gyro_bias[0] = sum[0] / (float)samples;
    gyro_bias[1] = sum[1] / (float)samples;
    gyro_bias[2] = sum[2] / (float)samples;

    printf("Gyro bias calibrated: %.2f, %.2f, %.2f\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

// === 主函数 ===
int main() {
    stdio_init_all();
    bsp_init();
    app_init();

    // 延迟等待稳定
    sleep_ms(5000);

    // 陀螺仪零偏校准
    calibrate_gyro_bias();

    // 先跑几次滤掉初始突变
    imu_data_t imu_data;
    float roll, pitch, yaw;
    for (int i = 0; i < 20; i++) {
        get_data_from_reg(&imu_data);
        update_imu_pose(&imu_data, &roll, &pitch, &yaw);
        sleep_ms(10);
    }

    // 姿态初始归零
    reset_pose_offset();

    // 主循环
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

    return 0;
}