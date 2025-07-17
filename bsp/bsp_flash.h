#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stddef.h>
#include <stdint.h>

void flash_write(uint32_t offset, const void *data, size_t size);

void flash_read(uint32_t offset, void *data, size_t size);

#endif // BSP_FLASH_H
