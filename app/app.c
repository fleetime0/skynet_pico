#include "app.h"

#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/int32.h"

#include "my_micro_ros.h"

#define SKYNET_NODE_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define SKYNET_NODE_STACK_SIZE 1024

rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rclc_executor_t executor;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  msg.data++;
}

static void skynet_node_task(__unused void *params) {
  if (!my_micro_ros_init()) {
    return;
  }
  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  printf("Ping the agent...\n");
  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK) {
    // Unreachable agent, exiting program.
    printf("Unreachable agent, exiting program.\n");
    return;
  }
  printf("Agent is reachable.\n");

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, NODE_NAME, "", &support);
  rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pico_publisher");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  msg.data = 0;
  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    taskYIELD();
  }
}

void app_start_freertos(void) {
  TaskHandle_t task;
  xTaskCreate(skynet_node_task, "SkynetNodeThread", SKYNET_NODE_STACK_SIZE, NULL, SKYNET_NODE_TASK_PRIORITY, &task);
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
  vTaskCoreAffinitySet(task, 1);
#endif

  printf("start vTaskStartScheduler\n");
  vTaskStartScheduler();
}
