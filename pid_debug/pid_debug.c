#include "pid_debug.h"

#include "pico/stdlib.h"

#include "protocol.h"

#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5

static uint8_t uart_rx_buf[PROT_FRAME_LEN_RECV];
static volatile uint16_t rx_index = 0;

static bool g_start = false;
static float g_target_speed = 0;
static float actual_speed[2];

extern void pid_set_motor_parm(uint8_t motor_id, float kp, float ki, float kd);

extern void pid_set_motor_target(uint8_t motor_id, float target);

extern void pid_get_motor_param(uint8_t motor_id, float *kp, float *ki, float *kd);

extern void motion_set_speed(int16_t speed_m1, int16_t speed_m2);

extern void motion_get_motor_speed(float *speed);

extern void motion_stop(uint8_t brake);

extern void bsp_reset_mcu(void);

static void on_uart_rx() {
  while (uart_is_readable(UART_ID)) {
    uint8_t ch = uart_getc(UART_ID);
    if (rx_index < PROT_FRAME_LEN_RECV) {
      uart_rx_buf[rx_index++] = ch;
    }
  }
  protocol_data_recv(uart_rx_buf, rx_index);
  rx_index = 0;
}

void set_pid_paramter_cmd(float p, float i, float d) { pid_set_motor_parm(2, p, i, d); }

void set_pid_target_val_cmd(int target_val) {
  g_target_speed = (float) target_val;
  pid_set_motor_target(2, g_target_speed);
}

void set_pid_period_cmd(uint32_t period) { (void) period; }

void pid_start_cmd(void) {
  motion_set_speed(0, 0);
  pid_set_motor_target(2, g_target_speed);

  g_start = true;
}

void pid_stop_cmd(void) {
  motion_stop(1);
  g_start = false;
}

void pid_reset_cmd(void) { bsp_reset_mcu(); }

void port_send_data_to_computer(void *data, uint8_t num) { uart_write_blocking(UART_ID, data, num); }

void pid_debug_init(void) {
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
  irq_set_enabled(UART1_IRQ, true);
  uart_set_irq_enables(UART_ID, true, false);

  protocol_init();

  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);

  float kp, ki, kd;
  pid_get_motor_param(2, &kp, &ki, &kd);
  float pid_temp[3] = {kp, ki, kd};
  set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);

  uint8_t cycle = 10;
  set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &cycle, 1);
}

void pid_debug_run(void) { receiving_process(); }

void pid_debug_send(void) {
  motion_get_motor_speed(actual_speed);
  int local_actual_speed = (int) actual_speed[0];
  if (g_start)
    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &local_actual_speed, 1);
}
