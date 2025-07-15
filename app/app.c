#include "app.h"

#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/int32.h"

#include "bsp_beep.h"
#include "bsp_key.h"
#include "icm45686.h"
#include "my_micro_ros.h"

#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define APP_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define KEY_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define SKYNET_NODE_TASK_PRIORITY (tskIDLE_PRIORITY + 4UL)

#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define APP_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define KEY_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define SKYNET_NODE_STACK_SIZE 1024

rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
// rcl_timer_t timer;
rclc_executor_t executor;
rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;

// static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//   rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//   msg.data++;
// }

static void skynet_node_task(__unused void *params) {
  if (!my_micro_ros_init(AGENT_IP, AGENT_PORT)) {
    vTaskDelete(NULL);
  }

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  printf("Ping the agent...\n");
  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK) {
    // Unreachable agent, exiting program.
    printf("Unreachable agent, exiting program.\n");
    vTaskDelete(NULL);
  }
  printf("Agent is reachable.\n");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, NODE_NAME, "", &support);
  // rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  // "pico_publisher");

  // rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(50), timer_callback, true);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  // rclc_executor_add_timer(&executor, &timer);

  // msg.data = 0;
  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    taskYIELD();
  }
}

static void app_loop(void) { beep_timeout_close_handle(); }

static void app_task(__unused void *params) {
  beep_on_time(100);
  while (true) {
    vTaskDelay(10);
    app_loop();
  }
}

static void key_task(__unused void *params) {
  while (true) {
    if (key_state(KEY_MODE_ONE_TIME)) {
      beep_on_time(50);
      printf("KEY1 PRESS\n");
    }
    vTaskDelay(10);
  }
}

static void main_task(__unused void *params) {
  if (cyw43_arch_init()) {
    printf("failed to initialise\n");
    vTaskDelete(NULL);
  }

  cyw43_arch_enable_sta_mode();
  cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);

  printf("Connecting to Wi-Fi...\n");
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
    printf("failed to connect.\n");
    vTaskDelete(NULL);
  }
  printf("Connected.\n");

  // xTaskCreate(skynet_node_task, "SkynetNodeThread", SKYNET_NODE_STACK_SIZE, NULL, SKYNET_NODE_TASK_PRIORITY, NULL);
  xTaskCreate(key_task, "KeyTaskThread", KEY_TASK_STACK_SIZE, NULL, KEY_TASK_PRIORITY, NULL);
  xTaskCreate(app_task, "AppTaskThread", APP_TASK_STACK_SIZE, NULL, APP_TASK_PRIORITY, NULL);

  vTaskDelete(NULL);
}

void app_init(void) {
  int result = icm45686_init();
  if (result != 0) {
    printf("ICM45686 initialization failed: %d\n", result);
    // bsp_long_beep_alarm();
    while (true)
      ;
  }
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
