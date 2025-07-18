#include <stdint.h>

#include "pico/time.h"

uint64_t inv_imu_get_time_us(void) { return time_us_64(); }
