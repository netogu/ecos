#include <stdint.h>
// #include <string.h>
#include "external/printf.h"
#include "board/config.h"
#include "lib/delay.h"
#include "lib/debug.h"


float counter = 0;

int main(void) {

  board_clock_setup();
  board_gpio_setup();
  board_serial_setup();

  debug_print_memory_range((uint8_t *)0x08000000, (uint8_t *)0x08000000 + 0x400);
  
  while (1) {
    gpio_pin_toggle(&gpios.led_green);
    printf("Hello World! %f\r\n", counter);
    delay_ms(1000);
    counter += 0.1;
  }
}