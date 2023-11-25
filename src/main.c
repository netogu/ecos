#include <stdint.h>
// #include <stdio.h>
#include "external/printf.h"
#include "board/config.h"
#include "lib/delay.h"
#include "lib/debug.h"


uint32_t counter = 0;
float val = 0.0;
int main(void) {

  board_clock_setup();
  board_gpio_setup();
  board_serial_setup();

  debug_print_memory_range((uint8_t *)0x08000000, (uint8_t *)0x08000000 + 0x400);
  
  while (1) {
    gpio_pin_toggle(&gpios.led_green);
    printf("Hello World! %ld %f\r\n", counter, val);
    delay_ms(500);
    counter++;
    val += 0.1;
  }
}