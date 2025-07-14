#include "icm45686.h"

#include "pico/stdlib.h"

#include "bsp_icm_i2c.h"

#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"

static inv_imu_device_t icm_driver;
static inv_imu_sensor_data_t inv_imu_data;

static void us_sleep(uint32_t us) { sleep_us(us); }

static accel_config0_accel_ui_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g) {
  accel_config0_accel_ui_fs_sel_t ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;

  switch (accel_fsr_g) {
    case 2:
      ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G;
      break;
    case 4:
      ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;
      break;
    case 8:
      ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G;
      break;
    case 16:
      ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;
      break;
#if INV_IMU_HIGH_FSR_SUPPORTED
    case 32:
      ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G;
      break;
#endif
    default:
      /* Unknown accel FSR. Set to default 16G */
      break;
  }
  return ret;
}

static gyro_config0_gyro_ui_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps) {
  gyro_config0_gyro_ui_fs_sel_t ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;

  switch (gyro_fsr_dps) {
    case 15:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS;
      break;
    case 31:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS;
      break;
    case 62:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS;
      break;
    case 125:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS;
      break;
    case 250:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS;
      break;
    case 500:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;
      break;
    case 1000:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS;
      break;
    case 2000:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;
      break;
#if INV_IMU_HIGH_FSR_SUPPORTED
    case 4000:
      ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS;
      break;
#endif
    default:
      /* Unknown gyro FSR. Set to default 2000dps" */
      break;
  }
  return ret;
}

static accel_config0_accel_odr_t accel_freq_to_param(uint16_t accel_freq_hz) {
  accel_config0_accel_odr_t ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;

  switch (accel_freq_hz) {
    case 1:
      ret = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;
      break;
    case 3:
      ret = ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ;
      break;
    case 6:
      ret = ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ;
      break;
    case 12:
      ret = ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ;
      break;
    case 25:
      ret = ACCEL_CONFIG0_ACCEL_ODR_25_HZ;
      break;
    case 50:
      ret = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;
      break;
    case 100:
      ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;
      break;
    case 200:
      ret = ACCEL_CONFIG0_ACCEL_ODR_200_HZ;
      break;
    case 400:
      ret = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
      break;
    case 800:
      ret = ACCEL_CONFIG0_ACCEL_ODR_800_HZ;
      break;
    case 1600:
      ret = ACCEL_CONFIG0_ACCEL_ODR_1600_HZ;
      break;
    case 3200:
      ret = ACCEL_CONFIG0_ACCEL_ODR_3200_HZ;
      break;
    case 6400:
      ret = ACCEL_CONFIG0_ACCEL_ODR_6400_HZ;
      break;
    default:
      /* Unknown accel frequency. Set to default 100Hz */
      break;
  }
  return ret;
}

static gyro_config0_gyro_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz) {
  gyro_config0_gyro_odr_t ret = GYRO_CONFIG0_GYRO_ODR_100_HZ;

  switch (gyro_freq_hz) {
    case 1:
      ret = GYRO_CONFIG0_GYRO_ODR_1_5625_HZ;
      break;
    case 3:
      ret = GYRO_CONFIG0_GYRO_ODR_3_125_HZ;
      break;
    case 6:
      ret = GYRO_CONFIG0_GYRO_ODR_6_25_HZ;
      break;
    case 12:
      ret = GYRO_CONFIG0_GYRO_ODR_12_5_HZ;
      break;
    case 25:
      ret = GYRO_CONFIG0_GYRO_ODR_25_HZ;
      break;
    case 50:
      ret = GYRO_CONFIG0_GYRO_ODR_50_HZ;
      break;
    case 100:
      ret = GYRO_CONFIG0_GYRO_ODR_100_HZ;
      break;
    case 200:
      ret = GYRO_CONFIG0_GYRO_ODR_200_HZ;
      break;
    case 400:
      ret = GYRO_CONFIG0_GYRO_ODR_400_HZ;
      break;
    case 800:
      ret = GYRO_CONFIG0_GYRO_ODR_800_HZ;
      break;
    case 1600:
      ret = GYRO_CONFIG0_GYRO_ODR_1600_HZ;
      break;
    case 3200:
      ret = GYRO_CONFIG0_GYRO_ODR_3200_HZ;
      break;
    case 6400:
      ret = GYRO_CONFIG0_GYRO_ODR_6400_HZ;
      break;
    default:
      /* Unknown gyro ODR. Set to default 100Hz */
      break;
  }
  return ret;
}

int icm45686_init(void) {
  int rc = 0;
  uint8_t who_am_i;

  icm_driver.transport.serif_type = UI_I2C;
  icm_driver.transport.read_reg = bsp_icm_i2c_read;
  icm_driver.transport.write_reg = bsp_icm_i2c_write;
  icm_driver.transport.sleep_us = us_sleep;

  sleep_us(3000);

  return inv_imu_adv_init(&icm_driver);
}

int start_accel(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  rc |= inv_imu_set_accel_fsr(&icm_driver, accel_fsr_g_to_param(fsr));
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_freq_to_param(odr));
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  return rc;
}

int start_gyro(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  rc |= inv_imu_set_gyro_fsr(&icm_driver, gyro_fsr_dps_to_param(fsr));
  rc |= inv_imu_set_gyro_frequency(&icm_driver, gyro_freq_to_param(odr));
  rc |= inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_LN);
  return rc;
}

int get_data_from_reg(imu_data_t *imu_data) {
  int ret = inv_imu_get_register_data(&icm_driver, (inv_imu_sensor_data_t *) imu_data);
  return ret;
}
