#include "app_bat.h"

#include "app_motion.h"
#include "app_oled.h"
#include "bsp_adc.h"
#include "bsp_led.h"

#define BAT_CHECK_COUNT 20

static uint8_t g_system_enable = 1;
static uint8_t g_bat_state = 1;
static int voltage_z10 = 0;
static int voltage_unusual_count = 0;

static uint8_t bat_check_voltage(int voltage) {

  if (voltage > bat_get_over_voltage()) {
    voltage_unusual_count++;
    if (voltage_unusual_count > BAT_CHECK_COUNT) {
      return BATTERY_OVER_VOLTAGE;
    }
  } else if (voltage < bat_get_low_voltage()) {
    if (voltage > 85 || voltage < 65) {
      voltage_unusual_count++;
      if (voltage_unusual_count > BAT_CHECK_COUNT) {
        return BATTERY_LOW;
      }
    }
  } else {
    voltage_unusual_count = 0;
  }
  return BATTERY_NORMAL;
}

uint8_t bat_get_low_voltage(void) { return 96; }

uint8_t bat_get_over_voltage(void) { return 130; }

uint8_t bat_state(void) {
  if (g_bat_state == BATTERY_NORMAL) {
    voltage_z10 = (int) (adc_get_battery_volotage() * 10);
    g_bat_state = bat_check_voltage(voltage_z10);
    if (g_bat_state != BATTERY_NORMAL) {
      g_system_enable = 0;
    }
  }
  return g_bat_state;
}

int bat_voltage_z10(void) { return voltage_z10; }

uint8_t system_enable(void) { return g_system_enable; }

uint8_t bat_show_led_handle(uint8_t enable_beep) {
  static uint16_t bat_led_state = 0;
  bat_led_state++;
  if (bat_led_state >= 10) {
    static uint8_t alarm = 1;
    uint8_t battery_state = bat_state();
    bat_led_state = 0;
    if (battery_state == BATTERY_LOW) {
      bsp_led_show_low_battery(enable_beep);
      if (alarm) {
        alarm = 0;
        oled_draw_line("Battery Low", 2, true, true);
      }
      motion_stop(STOP_BRAKE);
    } else if (battery_state == BATTERY_OVER_VOLTAGE) {
      bsp_led_show_overvoltage_battery(enable_beep);
      if (alarm) {
        alarm = 0;
        oled_draw_line("Battery", 2, true, false);
        oled_draw_line("Over Voltage", 3, false, true);
      }
      motion_stop(STOP_BRAKE);
    } else {
      bsp_led_show_state();
    }
  }
  return g_system_enable;
}
