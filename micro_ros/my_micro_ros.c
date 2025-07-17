#include "my_micro_ros.h"

#include <stdio.h>

#include "rmw_microros/rmw_microros.h"

#include "pico_uart_transports.h"

bool my_micro_ros_init() {
  rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                pico_serial_transport_write, pico_serial_transport_read);

  return true;
}
