#include <stdint.h>
#include "external/printf.h"

//print range of memory in 32 byte chunks
void debug_print_memory_range(uint8_t *start, uint8_t *end, uint8_t table_width) {
  printf("Memory range: %p - %p\r\n", start, end);
  for (int i=0; i < end - start; i++) {
    if (i % table_width == 0) {
        printf("\r\n");
        printf("%p: %02X ", (start + i), *(start + i));

    } else {
        printf("%02X ", *(start + i));
    }
  }
  printf("\r\n");
}