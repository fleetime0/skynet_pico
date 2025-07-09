#ifndef MICRO_ROS_PICO_USB_H
#define MICRO_ROS_PICO_USB_H

#include <stdint.h>
#include <stdio.h>

#include "uxr/client/profile/transport/custom/custom_transport.h"

bool pico_usb_transport_open(struct uxrCustomTransport *transport);
bool pico_usb_transport_close(struct uxrCustomTransport *transport);
size_t pico_usb_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t pico_usb_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                               uint8_t *err);

#endif // MICRO_ROS_PICO_USB_H
