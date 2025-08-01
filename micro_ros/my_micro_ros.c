#include "my_micro_ros.h"

#include <stdio.h>

#include "FreeRTOS.h"

#include "rmw_microros/rmw_microros.h"

#include "pico_uart_transports.h"

static void *microros_freertos_allocate(size_t size, void *state) { return (void *) pvPortMalloc(size); }

static void microros_freertos_deallocate(void *pointer, void *state) { vPortFree(pointer); }

static void *microros_freertos_reallocate(void *pointer, size_t size, void *state) {
  if (NULL == pointer) {
    return (void *) pvPortMalloc(size);
  } else {
    vPortFree(pointer);
    return (void *) pvPortMalloc(size);
  }
}

static void *microros_freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state) {
  void *res = (void *) pvPortMalloc(number_of_elements * size_of_element);
  memset(res, 0, number_of_elements * size_of_element);
  return res;
}


bool my_micro_ros_init() {
  rcutils_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_freertos_allocate;
  freeRTOS_allocator.deallocate = microros_freertos_deallocate;
  freeRTOS_allocator.reallocate = microros_freertos_reallocate;
  freeRTOS_allocator.zero_allocate = microros_freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
    return false;
  }

  rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                pico_serial_transport_write, pico_serial_transport_read);

  return true;
}
