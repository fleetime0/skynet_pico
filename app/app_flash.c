#include "app_flash.h"

#include <stdint.h>
#include <stdio.h>

#include "app_pid.h"
#include "bsp_flash.h"
#include "bsp_motor.h"

static uint32_t flash_read_reset(void) {
  uint32_t flag = 0;
  flash_read(FLASH_RESET_OFFSET, &flag, sizeof(flag));
  printf("FRead Init Value:0x%X\n", flag);

  return flag;
}

static void flash_read_pid(float *kp, float *ki, float *kd) {
  float pid_data[3] = {0};
  flash_read(FLASH_PID_OFFSET, pid_data, sizeof(pid_data));
  *kp = pid_data[0];
  *ki = pid_data[1];
  *kd = pid_data[2];
}

static void flash_pid_init(void) {
  float kp = 0, ki = 0, kd = 0;
  flash_read_pid(&kp, &ki, &kd);
  printf("FRead PID:%.3f, %.3f, %.3f\n", kp, ki, kd);

  pid_set_motor_parm(MAX_MOTOR, kp, ki, kd);
}

void flash_reset(void) {
  flash_write_pid(PID_DEF_KP, PID_DEF_KI, PID_DEF_KD);

  const uint32_t reset_flag = FLASH_RESET_FLAG;
  flash_write(FLASH_RESET_OFFSET, &reset_flag, sizeof(reset_flag));

  // DEBUG("FReset OK\n");
}

void flash_init(void) {
  if (flash_read_reset() != FLASH_RESET_FLAG) {
    flash_reset();
  }
  flash_pid_init();
}

void flash_write_pid(float kp, float ki, float kd) {
  float pid_data[3] = {kp, ki, kd};
  flash_write(FLASH_PID_OFFSET, pid_data, sizeof(pid_data));
}
