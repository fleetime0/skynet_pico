#ifndef APP_FLASH_H
#define APP_FLASH_H

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#endif

#ifndef FLASH_SECTOR_SIZE
#define FLASH_SECTOR_SIZE (1u << 12)
#endif

#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - 2 * FLASH_SECTOR_SIZE)
#define FLASH_RESET_OFFSET (FLASH_TARGET_OFFSET)
#define FLASH_PID_OFFSET (FLASH_TARGET_OFFSET + FLASH_SECTOR_SIZE)

#define FLASH_RESET_FLAG 0xDEADBEEF

void flash_init(void);
void flash_reset(void);
void flash_write_pid(float kp, float ki, float kd);

#endif // APP_FLASH_H
