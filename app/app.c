#include "app.h"

#include <stdio.h>

#include "pico/cyw43_arch.h"
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

#include "app_bat.h"
#include "app_motion.h"
#include "app_oled.h"
#include "app_pid.h"
#include "bsp_beep.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "icm45686.h"
#include "my_micro_ros.h"

#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
// #define OLED_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define APP_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define KEY_TASK_PRIORITY (tskIDLE_PRIORITY + 4UL)
#define SKYNET_NODE_TASK_PRIORITY (tskIDLE_PRIORITY + 5UL)
#define SPEED_TASK_PRIORITY (tskIDLE_PRIORITY + 6UL)

#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
// #define OLED_TASK_PRIORITY configMINIMAL_STACK_SIZE
#define APP_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define KEY_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define SKYNET_NODE_STACK_SIZE 1024
#define SPEED_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

static TaskHandle_t skynet_task_handle = NULL;

static uint8_t g_enable_beep = 1;

static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_timer_t timer;
static rclc_executor_t executor;

static rcl_subscription_t buzzer_subscriber;
static std_msgs__msg__Bool buzzer_msg;

static rcl_subscription_t cmd_vel_subscriber;
static geometry_msgs__msg__Twist cmd_vel_msg;

static rcl_publisher_t vel_raw_subscriber;
static geometry_msgs__msg__Twist vel_raw_msg;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  
  // rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  // msg.data++;
}

static void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

  float vx_f = msg->linear.x;
  float vy_f = msg->linear.y;
  float vz_f = msg->angular.z;

  int16_t vx = (int16_t) (vx_f * 1000.0f);
  int16_t vy = 0;
  int16_t vz = (int16_t) (vz_f * 1000.0f);

  motion_ctrl(vx, vy, vz);
}

static void buzzer_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
  beep_on_time((uint16_t) (msg->data));
}

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

static void skynet_node_task(__unused void *params) {
  cyw43_arch_enable_sta_mode();
  cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);

  printf("Connecting to Wi-Fi...\n");
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
    printf("failed to connect.\n");
    skynet_task_handle = NULL;
    vTaskDelete(NULL);
  }
  printf("Connected.\n");

  if (!my_micro_ros_init(AGENT_IP, AGENT_PORT)) {
    skynet_task_handle = NULL;
    vTaskDelete(NULL);
  }

  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  printf("Ping the agent...\n");
  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK) {
    printf("Unreachable agent, exiting program.\n");
    skynet_task_handle = NULL;
    vTaskDelete(NULL);
  }
  printf("Agent is reachable.\n");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, NODE_NAME, "", &support);

  rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                 "cmd_vel");
  rclc_subscription_init_default(&buzzer_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "buzzer");

  rclc_publisher_init_default(&vel_raw_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                              "vel_raw");

  rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100), timer_callback, true);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &buzzer_subscriber, &buzzer_msg, &buzzer_callback, ON_NEW_DATA);

  rclc_executor_add_timer(&executor, &timer);

  geometry_msgs__msg__Twist__init(&vel_raw_msg);
  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    taskYIELD();
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
        if (skynet_task_handle == NULL) {
          BaseType_t result = xTaskCreate(skynet_node_task, "SkynetNodeThread", SKYNET_NODE_STACK_SIZE, NULL,
                                          SKYNET_NODE_TASK_PRIORITY, &skynet_task_handle);

          if (result == pdPASS) {
            printf("SkynetNodeThread started\n");
          } else {
            printf("Failed to start SkynetNodeThread\n");
          }
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

static void main_task(__unused void *params) {
  if (cyw43_arch_init()) {
    printf("failed to initialise\n");
    vTaskDelete(NULL);
  }

  xTaskCreate(speed_task, "SpeedTaskThread", SPEED_TASK_STACK_SIZE, NULL, SPEED_TASK_PRIORITY, NULL);
  xTaskCreate(key_task, "KeyTaskThread", KEY_TASK_STACK_SIZE, NULL, KEY_TASK_PRIORITY, NULL);
  xTaskCreate(app_task, "AppTaskThread", APP_TASK_STACK_SIZE, NULL, APP_TASK_PRIORITY, NULL);

  vTaskDelete(NULL);
}

void app_init(void) {
  pid_param_init();

  printf("Start ICM45686 Init\n");
  int result = icm45686_init();
  if (result != 0) {
    printf("ICM_INIT ERROR:%d\n", result);
    oled_show_error();
    long_beep_alarm();
    while (true)
      ;
  }
  printf("ICM_INIT OK\n");
}

void app_start_freertos(void) {
  TaskHandle_t task;
  xTaskCreate(main_task, "MainThread", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
  vTaskCoreAffinitySet(task, 1);
#endif

  printf("start vTaskStartScheduler\n");
  vTaskStartScheduler();
}
