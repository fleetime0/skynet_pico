#include "bsp_flash.h"

#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"

static uint8_t page_buffer[FLASH_PAGE_SIZE] = {0};

void flash_write(uint32_t offset, const void *data, size_t size) {
  if (size > FLASH_PAGE_SIZE || offset % FLASH_PAGE_SIZE != 0) {
    // DEBUG("flash_write: invalid size or offset\n");
    return;
  }

  memcpy(page_buffer, data, size);

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(offset, FLASH_SECTOR_SIZE);
  flash_range_program(offset, page_buffer, FLASH_PAGE_SIZE);
  restore_interrupts(ints);
}

void flash_read(uint32_t offset, void *data, size_t size) { memcpy(data, (const void *) (XIP_BASE + offset), size); }
