#include "app.h"

#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "app_bat.h"
#include "app_flash.h"
#include "app_motion.h"
#include "app_oled.h"
#include "app_pid.h"
#include "bsp.h"
#include "bsp_beep.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "icm45686.h"
#include "skynet_node.h"

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
static uint32_t g_oled_count = 0;

uint8_t g_oled_flag = OLED_MAX_FLAG;
SemaphoreHandle_t oled_flag_mutex;

imu_norm_data_t norm_data;
SemaphoreHandle_t norm_data_mutex;

static void speed_task(__unused void *params) {
  uint32_t last_wake_time = xTaskGetTickCount();
  while (system_enable()) {
    encoder_update_count();
    motion_handle();
    vTaskDelayUntil(&last_wake_time, 10);
  }
  motion_stop(STOP_FREE);
  vTaskDelay(40);
  // DEBUG("Finish vTask_Speed\n");
  while (true) {
    vTaskDelay(1000);
  }
}

static void skynet_node_task(__unused void *params) { skynet_node_run(); }

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
      // DEBUG("KEY1 PRESS\n");
      if (!system_enable()) {
        g_enable_beep = 0;
        BEEP_OFF();
      }
    }

    if (key_long_press(10)) {
      if (system_enable()) {
        beep_on_time(1000);
        motion_stop(STOP_BRAKE);

        flash_reset();
        vTaskDelay(500);
        bsp_reset_mcu();
      }
    }
    vTaskDelay(10);
  }
}

static void imu_task(__unused void *params) {
  uint32_t last_wake_time = xTaskGetTickCount();
  imu_data_t imu_data;
  while (system_enable()) {
    get_raw_data(&imu_data);
    xSemaphoreTake(norm_data_mutex, portMAX_DELAY);
    norm_data = normalize_imu_data(&imu_data);
    update_attitude(&norm_data);
    xSemaphoreGive(norm_data_mutex);
    vTaskDelayUntil(&last_wake_time, 10);
  }
  vTaskDelay(30);
  // DEBUG("Finish vTask_IMU\n");
  while (true) {
    vTaskDelay(1000);
  }
}

static void oled_task(__unused void *params) {
  oled_show_cartype(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

  uint8_t oled_flag = g_oled_flag;
  while (system_enable()) {
    xSemaphoreTake(oled_flag_mutex, portMAX_DELAY);
    oled_flag = g_oled_flag;
    xSemaphoreGive(oled_flag_mutex);
    g_oled_count++;
    switch (oled_flag) {
      case OLED_FLAG_NO_DISPLAY: {
        oled_clear();
        oled_refresh();
        g_oled_flag = OLED_MAX_FLAG;
        break;
      }
      case OLED_FLAG_IMU: {
        if (g_oled_count >= 5) {
          g_oled_count = 0;
          float yaw = 0, roll = 0, pitch = 0;
          xSemaphoreTake(norm_data_mutex, portMAX_DELAY);
          get_euler_angle(&roll, &pitch, &yaw);
          xSemaphoreGive(norm_data_mutex);
          oled_show_imu(yaw, roll, pitch);
          // DEBUG("YAW:%.2f, ROLL:%.2f, Pitch:%.2f\n", yaw, roll, pitch);
        }
        break;
      }
      case OLED_FLAG_VOLTAGE: {
        if (g_oled_count >= 100) {
          g_oled_count = 0;
          oled_show_voltage(bat_voltage_z10());
        }
        break;
      }
      case OLED_FLAG_MOTOR_SPEED: {
        float speed_f[2] = {0};
        motion_get_motor_speed(speed_f);
        oled_show_motor_speed(speed_f[0], speed_f[1]);
        break;
      }
      case OLED_MAX_FLAG: {
        g_oled_count = 100;
        break;
      }
      default: {
        g_oled_flag = OLED_FLAG_NO_DISPLAY;
        break;
      }
    }
    vTaskDelay(10);
  }
  vTaskDelay(20);
  // DEBUG("Finish vTask_OLED\n");
  while (1) {
    vTaskDelay(1000);
  }
}

void app_init(void) {
  app_bat_init();
  pid_param_init();
  motion_init();
  flash_init();

  norm_data_mutex = xSemaphoreCreateMutex();
  oled_flag_mutex = xSemaphoreCreateMutex();

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
  xTaskCreate(skynet_node_task, "SkynetNodeThread", SKYNET_NODE_STACK_SIZE, NULL, SKYNET_NODE_TASK_PRIORITY,
              &skynet_task_handle);
  xTaskCreate(key_task, "KeyTaskThread", KEY_TASK_STACK_SIZE, NULL, KEY_TASK_PRIORITY, &key_task_handle);
  printf("start KeyTaskThread\n");
  xTaskCreate(app_task, "AppTaskThread", APP_TASK_STACK_SIZE, NULL, APP_TASK_PRIORITY, &app_task_handle);
  printf("start AppTaskThread\n");
  xTaskCreate(imu_task, "ImuTaskThread", IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, &imu_task_handle);
  printf("start ImuTaskThread\n");
  xTaskCreate(oled_task, "OledTaskThread", OLED_TASK_STACK_SIZE, NULL, OLED_TASK_PRIORITY, &oled_task_handle);
  printf("start OledTaskThread\n");

#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
  vTaskCoreAffinitySet(speed_task_handle, CORE(0));
  vTaskCoreAffinitySet(key_task_handle, CORE(0));
  vTaskCoreAffinitySet(app_task_handle, CORE(0));
  vTaskCoreAffinitySet(imu_task_handle, CORE(0));
  vTaskCoreAffinitySet(oled_task_handle, CORE(0));
  vTaskCoreAffinitySet(skynet_task_handle, CORE(1));
#endif

  printf("start vTaskStartScheduler\n");
  vTaskStartScheduler();
}
