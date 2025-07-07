#include "my_micro_ros.h"

#ifdef MICRO_ROS_WIFI_SUPPORTED
#include "pico_wifi_transports.h"

#include "pico/cyw43_arch.h"
#include "rmw_microros/rmw_microros.h"

bool my_micro_ros_init(void) {
  stdio_init_all();

  if (cyw43_arch_init()) {
    printf("failed to initialise\n");
    return false;
  }

  cyw43_arch_enable_sta_mode();

  sleep_ms(1000);

  printf("Connecting to Wi-Fi...\n");
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
    printf("failed to connect.\n");
    return false;
  } else {
    printf("Connected.\n");
  }

  static struct micro_ros_agent_locator locator;
  ipaddr_aton(AGENT_IP, &locator.address);
  locator.port = AGENT_PORT;

  rmw_uros_set_custom_transport(false, (void *) &locator, pico_wifi_transport_open, pico_wifi_transport_close,
                                pico_wifi_transport_write, pico_wifi_transport_read);

  return true;
}
#else
#include "pico_uart_transports.h"

// void my_micro_ros_init(void) {
//   rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
//                                 pico_serial_transport_write, pico_serial_transport_read);
// }
#endif
