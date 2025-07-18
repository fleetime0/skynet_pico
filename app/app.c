#include "app.h"

#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "geometry_msgs/msg/twist.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/int32.h"

#include "app_bat.h"
#include "app_flash.h"
#include "app_motion.h"
#include "app_oled.h"
#include "app_pid.h"
#include "bsp_beep.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "icm45686.h"
#include "my_micro_ros.h"

#define CORE(n) (1U << (n))

#define OLED_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define IMU_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define APP_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define KEY_TASK_PRIORITY (tskIDLE_PRIORITY + 4UL)
#define SKYNET_NODE_TASK_PRIORITY (tskIDLE_PRIORITY + 5UL)
#define SPEED_TASK_PRIORITY (tskIDLE_PRIORITY + 6UL)

#define OLED_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define IMU_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define APP_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define KEY_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define SKYNET_NODE_STACK_SIZE 1024
#define SPEED_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

static TaskHandle_t speed_task_handle = NULL;
static TaskHandle_t skynet_task_handle = NULL;
static TaskHandle_t key_task_handle = NULL;
static TaskHandle_t app_task_handle = NULL;
static TaskHandle_t imu_task_handle = NULL;
static TaskHandle_t oled_task_handle = NULL;

static uint8_t g_enable_beep = 1;

static void speed_task(__unused void *params) {
  uint32_t last_wake_time = xTaskGetTickCount();
  while (system_enable()) {
    vTaskDelayUntil(&last_wake_time, 10);
    encoder_update_count();
    motion_handle();
  }
  motion_stop(STOP_FREE);
  vTaskDelay(40);
  printf("Finish vTask_Speed\n");
  while (true) {
    vTaskDelay(1000);
  }
}

static void app_loop(void) { beep_timeout_close_handle(); }

static void app_task(__unused void *params) {
  beep_on_time(100);
  while (true) {
    vTaskDelay(10);
    if (bat_show_led_handle(g_enable_beep))
      app_loop();
  }
}

static void key_task(__unused void *params) {
  while (true) {
    if (key_state(KEY_MODE_ONE_TIME)) {
      beep_on_time(50);
      printf("KEY1 PRESS\n");
      if (system_enable()) {
        if (skynet_task_handle != NULL) {
          xTaskNotifyGive(skynet_task_handle);
        } else {
          printf("skynet_task_handle is NULL!\n");
        }
      } else {
        g_enable_beep = 0;
        BEEP_OFF();
      }
    }

    if (key_long_press(10)) {
      if (system_enable()) {
        beep_on_time(1000);
        motion_stop(STOP_BRAKE);

        vTaskDelay(500);
        printf("Reset MCU\n");
      }
    }
    vTaskDelay(10);
  }
}

static void imu_task(__unused void *params) {
  uint32_t last_wake_time = xTaskGetTickCount();
  while (system_enable()) {
    // get_raw_data();
    // ICM20948_Read_Data_Handle();
    vTaskDelayUntil(&last_wake_time, 10);
  }
  vTaskDelay(30);
  printf("Finish vTask_IMU\n");
  while (true) {
    vTaskDelay(1000);
  }
}

void app_init(void) {
  pid_param_init();
  flash_init();

  // DEBUG("Start ICM45686 Init\n");
  int result = icm45686_init();
  if (result != 0) {
    printf("ICM_INIT ERROR:%d\n", result);
    oled_show_error();
    long_beep_alarm();
    while (true)
      ;
  }
  icm45686_calibrate_gyro_bias();
  // DEBUG("ICM_INIT OK\n");
}

void app_start_freertos(void) {
  xTaskCreate(speed_task, "SpeedTaskThread", SPEED_TASK_STACK_SIZE, NULL, SPEED_TASK_PRIORITY, &speed_task_handle);
  printf("start SpeedTaskThread\n");
  // xTaskCreate(skynet_node_task, "SkynetNodeThread", SKYNET_NODE_STACK_SIZE, NULL, SKYNET_NODE_TASK_PRIORITY,
  //             &skynet_task_handle);
  xTaskCreate(key_task, "KeyTaskThread", KEY_TASK_STACK_SIZE, NULL, KEY_TASK_PRIORITY, &key_task_handle);
  printf("start KeyTaskThread\n");
  xTaskCreate(app_task, "AppTaskThread", APP_TASK_STACK_SIZE, NULL, APP_TASK_PRIORITY, &app_task_handle);
  printf("start AppTaskThread\n");
  xTaskCreate(imu_task, "ImuTaskThread", IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, &imu_task_handle);
  printf("start ImuTaskThread\n");
  // xTaskCreate();

#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
  vTaskCoreAffinitySet(speed_task_handle, CORE(0));
  vTaskCoreAffinitySet(key_task_handle, CORE(0));
  vTaskCoreAffinitySet(app_task_handle, CORE(0));
  vTaskCoreAffinitySet(imu_task_handle, CORE(0));
  // vTaskCoreAffinitySet(skynet_task_handle, CORE(1));
#endif

  printf("start vTaskStartScheduler\n");
  vTaskStartScheduler();
}
