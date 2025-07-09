#include "pico_usb_transports.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uxr/client/profile/transport/custom/custom_transport.h"

void usleep(uint64_t us) {
  if (us < 1000) {
    busy_wait_us(us);
    return;
  }
  TickType_t t = pdMS_TO_TICKS(us / 1000);
  if (t < 1) {
    t = 1;
  }
  vTaskDelay(t);
}

int clock_gettime(clockid_t unused, struct timespec *tp) {
  uint64_t m = time_us_64();
  tp->tv_sec = m / 1000000;
  tp->tv_nsec = (m % 1000000) * 1000;
  return 0;
}

bool pico_usb_transport_open(struct uxrCustomTransport *transport) {
  // Checks we have USB CDC connection
  return stdio_usb_connected();
}

bool pico_usb_transport_close(struct uxrCustomTransport *transport) { return true; }

size_t pico_usb_transport_write(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, uint8_t *errcode) {
  stdio_usb.out_chars(buf, len);
  return len;
}

size_t pico_usb_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                               uint8_t *errcode) {
  uint64_t until_time_us = time_us_64() + timeout;

  size_t read = 0;
  while (time_us_64() < until_time_us) {
    read = stdio_usb.in_chars(buf, len);
    if (read != 0) {
      vTaskDelay(1);
      return read;
    }
    taskYIELD();
  }

  return 0;
}
