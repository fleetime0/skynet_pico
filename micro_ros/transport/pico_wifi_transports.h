#ifndef MICRO_ROS_PICO_WIFI_H
#define MICRO_ROS_PICO_WIFI_H

#ifndef PICO_CYW43_SUPPORTED
#include <uxr/client/profile/transport/custom/custom_transport.h>

bool pico_wifi_transport_open(struct uxrCustomTransport *transport);
bool pico_wifi_transport_close(struct uxrCustomTransport *transport);
size_t pico_wifi_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t pico_wifi_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                                uint8_t *err);

#endif // PICO_CYW43_SUPPORTED

#endif // MICRO_ROS_PICO_WIFI_H
