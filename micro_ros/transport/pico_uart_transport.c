#include <stdio.h>
#include <time.h>

#include "hardware/dma.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uxr/client/profile/transport/custom/custom_transport.h"


#define UART_ID uart1
#define BAUD_RATE 921600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

static int dma_chan_tx;

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

void init_uart1(void) {
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  dma_chan_tx = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_chan_tx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_dreq(&c, uart_get_dreq(UART_ID, true));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);

  dma_channel_configure(dma_chan_tx, &c, &uart_get_hw(UART_ID)->dr, NULL, 0, false);
}

bool pico_serial_transport_open(struct uxrCustomTransport *transport) {
  init_uart1();
  return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport *transport) {
  dma_channel_unclaim(dma_chan_tx);
  return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, uint8_t *errcode) {
  (void) transport;
  (void) errcode;

  dma_channel_set_read_addr(dma_chan_tx, buf, false);
  dma_channel_set_trans_count(dma_chan_tx, len, true);
  dma_channel_wait_for_finish_blocking(dma_chan_tx);
  return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                                  uint8_t *errcode) {
  (void) transport;
  size_t count = 0;
  uint64_t start_time_us = time_us_64();

  while (count < len) {
    while (uart_is_readable(UART_ID)) {
      buf[count++] = uart_getc(UART_ID);
      if (count >= len)
        break;
    }

    if (count >= len) {
      break;
    }

    uint64_t now = time_us_64();
    if (now - start_time_us > (uint64_t) timeout * 1000) {
      if (errcode != NULL) {
        *errcode = 1;
      }
      break;
    }

    taskYIELD();
  }

  return count;
}
