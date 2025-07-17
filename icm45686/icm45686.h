#ifndef ICM45686_H
#define ICM45686_H

#include <stdint.h>

#define ICM45686_ACCEL_SCALE_G 4
#define ICM45686_GYRO_SCALE_DPS 1000
#define ICM45686_ODR_HZ 100

#define ICM45686_CALIB_SAMPLES 300
#define ICM45686_CALIB_WAIT_MS (1000 / ICM45686_ODR_HZ)

typedef struct {
  int16_t accel_data[3];
  int16_t gyro_data[3];
  int16_t temp_data;
} imu_data_t;

int icm45686_init(void);
int start_accel(uint16_t odr, uint16_t fsr);
int start_gyro(uint16_t odr, uint16_t fsr);
int get_raw_data(imu_data_t *imu_data);

void get_quaternion(float *q0_out, float *q1_out, float *q2_out, float *q3_out);
void get_euler_angle(float *roll_deg, float *pitch_deg, float *yaw_deg);

void icm45686_calibrate_gyro_bias(void);

#endif // ICM45686_H
