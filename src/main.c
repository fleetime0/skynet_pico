#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "app.h"
#include "bsp.h"
#include "icm45686.h"

// === 主函数 ===
int main() {
  stdio_init_all();
  bsp_init();
  app_init();

  icm45686_calibrate_gyro_bias();

  int cnt = 0;
  float roll, pitch, yaw;
  while (true) {
    get_euler_angle(&roll, &pitch, &yaw);

    if (cnt == 9) {
      printf("Euler Angle: R=%.2f P=%.2f Y=%.2f\n", roll, pitch, yaw);
      cnt = 0;
    } else {
      cnt++;
    }
    sleep_ms(10);
  }
}
