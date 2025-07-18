#ifndef SKYNET_NODE_H
#define SKYNET_NODE_H

#define NODE_NAME "skynet_robot"

void skynet_node_run(void);
//   // ROS 单位：m/s 和 rad/s
//   float vx_f = msg->linear.x;    // m/s
//   float vy_f = msg->linear.y;    // m/s （可忽略）
//   float wz_f = msg->angular.z;   // rad/s

//   // 转为整数速度，单位一致
//   int16_t vx = (int16_t)(vx_f * 1000.0f);     // m/s → mm/s 确实需要乘上1000
//   int16_t vy = 0;                             // 忽略横向速度
//   int16_t wz = (int16_t)(wz_f * 1000.0f);     // rad/s → 1000-scale

//   // 可选：限幅（保护机制）
//   if (vx > 1000) vx = 1000;
//   if (vx < -1000) vx = -1000;
//   if (wz > 3000) wz = 3000;      // 约 3rad/s
//   if (wz < -3000) wz = -3000;

// static rcl_node_t node;
// static rcl_allocator_t allocator;
// static rclc_support_t support;
// static rcl_timer_t timer;
// static rclc_executor_t executor;

// static rcl_subscription_t buzzer_subscriber;
// static std_msgs__msg__Bool buzzer_msg;

// static rcl_subscription_t cmd_vel_subscriber;
// static geometry_msgs__msg__Twist cmd_vel_msg;

// static rcl_publisher_t vel_raw_subscriber;
// static geometry_msgs__msg__Twist vel_raw_msg;

// static rcl_publisher_t publisher;
// static std_msgs__msg__Int32 msg;

// static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//   rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//   msg.data++;
// }

// static void cmd_vel_callback(const void *msgin) {
//   const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

//   float vx_f = msg->linear.x;
//   float vy_f = msg->linear.y;
//   float vz_f = msg->angular.z;

//   int16_t vx = (int16_t) (vx_f * 1000.0f);
//   int16_t vy = 0;
//   int16_t vz = (int16_t) (vz_f * 1000.0f);

//   motion_ctrl(vx, vy, vz);
// }

// static void buzzer_callback(const void *msgin) {
//   const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
//   beep_on_time((uint16_t) (msg->data));
// }

// static void skynet_node_task(__unused void *params) {
//   while (true) {
//     printf("[skynet] Waiting for activation...\n");
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     printf("[skynet] Activated! Starting Wi-Fi and micro-ROS setup...\n");

//     if (!my_micro_ros_init()) {
//       printf("micro-ROS transport init failed.\n");
//       continue;
//     }

//     const int timeout_ms = 1000;
//     const uint8_t attempts = 120;

//     printf("Pinging the agent...\n");
//     rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
//     if (ret != RCL_RET_OK) {
//       printf("Agent unreachable. Wait for next trigger.\n");
//       continue;
//     }

//     printf("Agent reachable. Setting up ROS 2...\n");

//     allocator = rcl_get_default_allocator();
//     rclc_support_init(&support, 0, NULL, &allocator);
//     rclc_node_init_default(&node, NODE_NAME, "", &support);

//     rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg,
//     Twist),
//                                    "cmd_vel");
//     rclc_subscription_init_default(&buzzer_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
//                                    "buzzer");
//     rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//     "pico_publisher"); rclc_publisher_init_default(&vel_raw_subscriber, &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
//                                 "vel_raw");

//     rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100), timer_callback, true);

//     rclc_executor_init(&executor, &support.context, 3, &allocator);
//     rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
//     rclc_executor_add_subscription(&executor, &buzzer_subscriber, &buzzer_msg, &buzzer_callback, ON_NEW_DATA);
//     rclc_executor_add_timer(&executor, &timer);

//     msg.data = 0;
//     geometry_msgs__msg__Twist__init(&vel_raw_msg);

//     printf("[skynet] Entering ROS2 main loop...\n");

//     while (true) {
//       rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//     }
//   }
// }

#endif // SKYNET_NODE_H
