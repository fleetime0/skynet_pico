#ifndef SKYNET_MY_MICRO_ROS_H
#define SKYNET_MY_MICRO_ROS_H

#include "pico/stdlib.h"

#ifdef MICRO_ROS_WIFI_SUPPORTED
#define WIFI_SSID "OpenWrt"
#define WIFI_PASSWORD "zxc147258369"
#define AGENT_IP "192.168.3.2"
#define AGENT_PORT 8888
#endif

bool my_micro_ros_init(void);

#endif // SKYNET_MY_MICRO_ROS_H
