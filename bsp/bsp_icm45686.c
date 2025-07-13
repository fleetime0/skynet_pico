#include "bsp_icm45686.h"

#define ICM45686_REG_ACCEL_DATA_X1_UI 0x00
#define ICM45686_REG_PWR_MGMT0 0x10
#define ICM45686_REG_ACCEL_CONFIG 0x1B
#define ICM45686_REG_GYRO_CONFIG 0x1C
#define ICM45686_REG_WHO_AM_I 0x72

static void icm45686_i2c_write(uint8_t addr, uint8_t* tx, uint8_t len) {
    // i2c_write_blocking(SKYNET_ICM45686_I2C, );
}

static void icm45686_i2c_read() {

}

uint8_t bsp_icm45686_connect() {

}
