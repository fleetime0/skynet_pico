#ifndef MICRO_ROS_PICO_WIFI_H
#define MICRO_ROS_PICO_WIFI_H

#include <stdint.h>
#include <stdio.h>

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

struct micro_ros_agent_locator {
  ip_addr_t address;
  int port;
};

bool pico_wifi_transport_open(struct uxrCustomTransport *transport);
bool pico_wifi_transport_close(struct uxrCustomTransport *transport);
size_t pico_wifi_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t pico_wifi_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                                uint8_t *err);

#endif // MICRO_ROS_PICO_WIFI_H
