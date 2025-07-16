#include "bsp_motor.h"

#include "hardware/pwm.h"
#include "pico/stdlib.h"

void bsp_motor_init(uint16_t arr, uint16_t psc) {
  gpio_init(M1_IN1_PIN);
  gpio_set_dir(M1_IN1_PIN, GPIO_OUT);
  gpio_put(M1_IN1_PIN, false);

  gpio_init(M1_IN2_PIN);
  gpio_set_dir(M1_IN2_PIN, GPIO_OUT);
  gpio_put(M1_IN2_PIN, false);

  gpio_init(M2_IN1_PIN);
  gpio_set_dir(M2_IN1_PIN, GPIO_OUT);
  gpio_put(M2_IN1_PIN, false);

  gpio_init(M2_IN2_PIN);
  gpio_set_dir(M2_IN2_PIN, GPIO_OUT);
  gpio_put(M2_IN2_PIN, false);

  gpio_set_function(PWM_M1_PIN, GPIO_FUNC_PWM);
  uint slice1 = pwm_gpio_to_slice_num(PWM_M1_PIN);
  pwm_config cfg1 = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg1, (float) psc);
  pwm_config_set_wrap(&cfg1, arr - 1);
  pwm_init(slice1, &cfg1, true);
  pwm_set_gpio_level(PWM_M1_PIN, 0);

  gpio_set_function(PWM_M2_PIN, GPIO_FUNC_PWM);
  uint slice2 = pwm_gpio_to_slice_num(PWM_M2_PIN);
  pwm_config cfg2 = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg2, (float) psc);
  pwm_config_set_wrap(&cfg2, arr - 1);
  pwm_init(slice2, &cfg2, true);
  pwm_set_gpio_level(PWM_M2_PIN, 0);
}

void motor_set_pwm(uint8_t id, int16_t speed) {
  if (speed >= MOTOR_MAX_PULSE)
    speed = MOTOR_MAX_PULSE;
  if (speed <= -MOTOR_MAX_PULSE)
    speed = -MOTOR_MAX_PULSE;

  uint in1 = 0, in2 = 0;
  uint slice;
  uint chan;
  uint pwm_pin;

  if (speed > 0) {
    in1 = 1;
    in2 = 0;
  } else if (speed < 0) {
    in1 = 0;
    in2 = 1;
    speed = -speed;
  } else {
    in1 = 0;
    in2 = 0;
  }

  switch (id) {
    case MOTOR_ID_M1:
      gpio_put(M1_IN1_PIN, in1);
      gpio_put(M1_IN2_PIN, in2);
      pwm_pin = PWM_M1_PIN;
      break;
    case MOTOR_ID_M2:
      gpio_put(M2_IN1_PIN, in1);
      gpio_put(M2_IN2_PIN, in2);
      pwm_pin = PWM_M2_PIN;
      break;
    default:
      return;
  }

  slice = pwm_gpio_to_slice_num(pwm_pin);
  chan = pwm_gpio_to_channel(pwm_pin);
  pwm_set_chan_level(slice, chan, speed);
}

void motor_stop(uint8_t brake) {
  if (brake) {
    gpio_put(M1_IN1_PIN, 1);
    gpio_put(M1_IN2_PIN, 1);
    gpio_put(M2_IN1_PIN, 1);
    gpio_put(M2_IN2_PIN, 1);
    pwm_set_gpio_level(PWM_M1_PIN, 1);
    pwm_set_gpio_level(PWM_M2_PIN, 1);
  } else {
    gpio_put(M1_IN1_PIN, 0);
    gpio_put(M1_IN2_PIN, 0);
    gpio_put(M2_IN1_PIN, 0);
    gpio_put(M2_IN2_PIN, 0);
    pwm_set_gpio_level(PWM_M1_PIN, 1);
    pwm_set_gpio_level(PWM_M2_PIN, 1);
  }
}
