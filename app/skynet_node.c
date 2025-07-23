#include "skynet_node.h"

#include <time.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "geometry_msgs/msg/twist.h"
#include "geometry_msgs/msg/twist_stamped.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/imu.h"
#include "skynet_msgs/srv/get_pid.h"
#include "skynet_msgs/srv/set_oled.h"
#include "skynet_msgs/srv/set_pid.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/float32.h"
#include "std_srvs/srv/set_bool.h"

#include "app_bat.h"
#include "app_motion.h"
#include "app_pid.h"
#include "bsp_beep.h"
#include "icm45686.h"
#include "my_micro_ros.h"

static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_timer_t timer;
static rclc_executor_t executor;

static rcl_service_t buzzer_service;
static std_srvs__srv__SetBool_Request buzzer_req;
static std_srvs__srv__SetBool_Response buzzer_res;

static rcl_service_t set_pid_service;
static skynet_msgs__srv__SetPID_Request set_pid_req;
static skynet_msgs__srv__SetPID_Response set_pid_res;

static rcl_service_t get_pid_service;
static skynet_msgs__srv__GetPID_Request get_pid_req;
static skynet_msgs__srv__GetPID_Response get_pid_res;

static rcl_service_t set_oled_service;
static skynet_msgs__srv__SetOled_Request set_oled_req;
static skynet_msgs__srv__SetOled_Response set_oled_res;

static rcl_subscription_t cmd_vel_subscriber;
static geometry_msgs__msg__Twist cmd_vel_msg;

static rcl_publisher_t vel_raw_publisher;
static geometry_msgs__msg__TwistStamped vel_raw_msg;

static rcl_publisher_t voltage_publisher;
static std_msgs__msg__Float32 voltage_msg;

static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__Imu imu_msg;

static car_data_t car_speed;

extern imu_norm_data_t norm_data;
extern SemaphoreHandle_t norm_data_mutex;

extern uint8_t g_oled_flag;
extern SemaphoreHandle_t oled_flag_mutex;

extern int clock_gettime(clockid_t unused, struct timespec *tp);

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  static uint8_t tick = 0;
  struct timespec tv = {0};
  switch (tick % 4) {
    case 0: {
      clock_gettime(0, &tv);
      motion_get_speed(&car_speed);
      vel_raw_msg.header.stamp.sec = tv.tv_sec;
      vel_raw_msg.header.stamp.nanosec = tv.tv_nsec;

      vel_raw_msg.twist.linear.x = car_speed.vx / 1000.f;
      vel_raw_msg.twist.linear.y = car_speed.vy / 1000.f;
      vel_raw_msg.twist.linear.z = 0;

      vel_raw_msg.twist.angular.x = 0;
      vel_raw_msg.twist.angular.y = 0;
      vel_raw_msg.twist.angular.z = car_speed.vz / 1000.0f;
      rcl_ret_t ret = rcl_publish(&vel_raw_publisher, &vel_raw_msg, NULL);
      (void) ret;
      break;
    }
    case 1: {
      voltage_msg.data = bat_voltage_z10() / 10.0f;
      rcl_ret_t ret = rcl_publish(&voltage_publisher, &voltage_msg, NULL);
      (void) ret;
      break;
    }
    case 2: {
      clock_gettime(0, &tv);
      xSemaphoreTake(norm_data_mutex, portMAX_DELAY);
      imu_msg.header.stamp.nanosec = tv.tv_nsec;
      imu_msg.header.stamp.sec = tv.tv_sec;
      imu_msg.linear_acceleration.x = norm_data.ax;
      imu_msg.linear_acceleration.y = norm_data.ay;
      imu_msg.linear_acceleration.z = norm_data.az;
      imu_msg.angular_velocity.x = norm_data.gx;
      imu_msg.angular_velocity.y = norm_data.gy;
      imu_msg.angular_velocity.z = norm_data.gz;
      imu_msg.orientation_covariance[0] = -1.0f;
      xSemaphoreGive(norm_data_mutex);
      rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
      (void) ret;
      break;
    }
    case 3: {
      break;
    }
    default:
      break;
  }

  tick = (tick + 1) % 4;
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

void buzzer_callback(const void *request, void *response) {
  const std_srvs__srv__SetBool_Request *req = (const std_srvs__srv__SetBool_Request *) request;
  std_srvs__srv__SetBool_Response *res = (std_srvs__srv__SetBool_Response *) response;

  beep_on_time((uint16_t) req->data);
  res->success = true;
}

void set_pid_callback(const void *request, void *response) {
  const skynet_msgs__srv__SetPID_Request *req = (const skynet_msgs__srv__SetPID_Request *) request;
  skynet_msgs__srv__SetPID_Response *res = (skynet_msgs__srv__SetPID_Response *) response;

  pid_set_motor_parm(2, req->kp, req->ki, req->kd);
  res->success = true;
}

void get_pid_callback(const void *request, void *response) {
  (void) request;
  skynet_msgs__srv__GetPID_Response *res = (skynet_msgs__srv__GetPID_Response *) response;

  pid_get_motor_param(2, &res->kp, &res->ki, &res->kd);
}

void set_oled_callback(const void *request, void *response) {
  const skynet_msgs__srv__SetOled_Request *req = (const skynet_msgs__srv__SetOled_Request *) request;
  skynet_msgs__srv__SetOled_Response *res = (skynet_msgs__srv__SetOled_Response *) response;

  if (req->oled_flag >= 0 && req->oled_flag < 4) {
    xSemaphoreTake(oled_flag_mutex, portMAX_DELAY);
    g_oled_flag = req->oled_flag;
    xSemaphoreGive(oled_flag_mutex);
    res->success = true;
  } else {
    res->success = false;
  }
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

    rclc_service_init_default(&buzzer_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                              "set_buzzer");
    rclc_service_init_default(&set_pid_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(skynet_msgs, srv, SetPID),
                              "set_pid");
    rclc_service_init_default(&get_pid_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(skynet_msgs, srv, GetPID),
                              "get_pid");
    rclc_service_init_default(&set_oled_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(skynet_msgs, srv, SetOled),
                              "set_oled");

    rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                   "cmd_vel");

    rclc_publisher_init_default(&vel_raw_publisher, &node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "vel_raw");
    rclc_publisher_init_default(&voltage_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                "voltage");
    rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                "imu/data_raw");

    rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(25), timer_callback, true);

    rclc_executor_init(&executor, &support.context, 6, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &buzzer_service, &buzzer_req, &buzzer_res, &buzzer_callback);
    rclc_executor_add_service(&executor, &set_pid_service, &set_pid_req, &set_pid_res, &set_pid_callback);
    rclc_executor_add_service(&executor, &get_pid_service, &get_pid_req, &get_pid_res, &get_pid_callback);
    rclc_executor_add_service(&executor, &set_oled_service, &set_oled_req, &set_oled_res, &set_oled_callback);
    rclc_executor_add_timer(&executor, &timer);

    geometry_msgs__msg__TwistStamped__init(&vel_raw_msg);
    sensor_msgs__msg__Imu__init(&imu_msg);
    voltage_msg.data = 0.0f;
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    rosidl_runtime_c__String__assign(&vel_raw_msg.header.frame_id, "base_link");
    while (true) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      taskYIELD();
    }
  }
}
