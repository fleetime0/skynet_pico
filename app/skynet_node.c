#include "skynet_node.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "geometry_msgs/msg/twist.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/float32.h"

#include "app_bat.h"
#include "app_motion.h"
#include "bsp_beep.h"
#include "my_micro_ros.h"

static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_timer_t timer;
static rclc_executor_t executor;

static rcl_subscription_t buzzer_subscriber;
static std_msgs__msg__Bool buzzer_msg;

static rcl_subscription_t cmd_vel_subscriber;
static geometry_msgs__msg__Twist cmd_vel_msg;

static rcl_publisher_t vel_raw_publisher;
static geometry_msgs__msg__Twist vel_raw_msg;

static rcl_publisher_t voltage_publisher;
static std_msgs__msg__Float32 voltage_msg;

static car_data_t car_speed;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  motion_get_speed(&car_speed);
  vel_raw_msg.linear.x = car_speed.vx / 1000.f;
  vel_raw_msg.linear.y = car_speed.vy / 1000.f;
  vel_raw_msg.linear.z = 0;

  vel_raw_msg.angular.x = 0;
  vel_raw_msg.angular.y = 0;
  vel_raw_msg.angular.z = car_speed.vz / 1000.0f;

  voltage_msg.data = Bat_Voltage_Z10() / 10.0f;

  

  rcl_ret_t ret = rcl_publish(&vel_raw_publisher, &vel_raw_msg, NULL);
  ret = rcl_publish(&voltage_publisher, &voltage_publisher, NULL);
}

static void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

  float vx_f = msg->linear.x;
  float vy_f = msg->linear.y;
  float vz_f = msg->angular.z;

  int16_t vx = (int16_t) (vx_f * 1000.0f);
  int16_t vy = (int16_t) (vy_f * 1000.0f);
  int16_t vz = (int16_t) (vz_f * 1000.0f);

  motion_ctrl(vx, vy, vz);
}

static void buzzer_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
  beep_on_time((uint16_t) (msg->data));
}

void skynet_node_run(void) {
  while (true) {
    if (!my_micro_ros_init()) {
      printf("micro-ROS transport init failed.\n");
      continue;
    }

    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    printf("Pinging the agent...\n");
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
      printf("Agent unreachable. Wait for next trigger.\n");
      continue;
    }

    printf("Agent reachable. Setting up ROS 2...\n");

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, NODE_NAME, "", &support);

    rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                   "cmd_vel");
    rclc_subscription_init_default(&buzzer_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                   "buzzer");

    rclc_publisher_init_default(&vel_raw_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                "vel_raw");

    rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100), timer_callback, true);

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &buzzer_subscriber, &buzzer_msg, &buzzer_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    geometry_msgs__msg__Twist__init(&vel_raw_msg);
    voltage_msg.data = 0.0f;
    while (true) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
  }
}
