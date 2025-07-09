#include "my_micro_ros.h"

#include <stdio.h>

#include "pico/cyw43_arch.h"

#include "rcutils/allocator.h"
#include "rmw_microros/rmw_microros.h"

#include "FreeRTOS.h"

static void *freertos_allocate(size_t size, void *state) { return (void *) pvPortMalloc(size); }

static void freertos_deallocate(void *pointer, void *state) { vPortFree(pointer); }

static void *freertos_reallocate(void *pointer, size_t size, void *state) {
  if (NULL == pointer) {
    return (void *) pvPortMalloc(size);
  } else {
    vPortFree(pointer);
    return (void *) pvPortMalloc(size);
  }
}

static void *freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state) {
  void *res = (void *) pvPortMalloc(number_of_elements * size_of_element);
  memset(res, 0, number_of_elements * size_of_element);
  return res;
}

#ifdef MICRO_ROS_WIFI_SUPPORTED
#include "pico_wifi_transports.h"

bool my_micro_ros_init(void) {
  rcutils_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = freertos_allocate;
  freeRTOS_allocator.deallocate = freertos_deallocate;
  freeRTOS_allocator.reallocate = freertos_reallocate;
  freeRTOS_allocator.zero_allocate = freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
    return false;
  }

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
  }
  printf("Connected.\n");

  static struct micro_ros_agent_locator locator;
  ipaddr_aton(AGENT_IP, &locator.address);
  locator.port = AGENT_PORT;

  rmw_uros_set_custom_transport(false, (void *) &locator, pico_wifi_transport_open, pico_wifi_transport_close,
                                pico_wifi_transport_write, pico_wifi_transport_read);

  return true;
}
#else
#include "pico_usb_transports.h"

bool my_micro_ros_init(void) {
  rcutils_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = freertos_allocate;
  freeRTOS_allocator.deallocate = freertos_deallocate;
  freeRTOS_allocator.reallocate = freertos_reallocate;
  freeRTOS_allocator.zero_allocate = freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
    return false;
  }
  rmw_uros_set_custom_transport(true, NULL, pico_usb_transport_open, pico_usb_transport_close, pico_usb_transport_write,
                                pico_usb_transport_read);
  return true;
}
#endif
