#ifndef ICM45686_H
#define ICM45686_H

#include <stdint.h>

typedef struct {
  int16_t accel_data[3];
  int16_t gyro_data[3];
  int16_t temp_data;
} imu_data_t;

int icm45686_init(void);
int start_accel(uint16_t odr, uint16_t fsr);
int start_gyro(uint16_t odr, uint16_t fsr);
int get_data_from_reg(imu_data_t *imu_data);

#endif // ICM45686_H
