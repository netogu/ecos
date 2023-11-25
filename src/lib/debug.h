#pragma once

#include <stdint.h>
#include "external/printf.h"

//print range of memory in 32 byte chunks
void debug_print_memory_range(uint8_t *start, uint8_t *end) {
  printf("Memory range: %p - %p\r\n", start, end);
  uint8_t *memloc = start;
  while (memloc < end) {
    printf("%p: ", memloc);
    for (uint8_t i = 0; i < 32; i++) {
      printf("%02X ", *(memloc + i));
    }
    printf("\r\n");
    memloc += 32;
  }
}