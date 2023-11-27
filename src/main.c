#include <stdint.h>
// #include <string.h>
#include "external/printf.h"
#include "board/config.h"
#include "lib/delay.h"
#include "lib/debug.h"

// USB Device IRQ Monitor
extern int usb_lp_irq_counter;

int main_loop_counter = 0;
int main(void) {

  board_clock_setup();
  board_gpio_setup();
  board_serial_setup();
  board_usb_setup();
  
  while (1) {
    gpio_pin_toggle(&gpios.led_green);

    printf("cnt_main, cnt_usb_irq: %d,%d \r\n", 
      main_loop_counter, 
      usb_lp_irq_counter);

    delay_ms(1000);
    main_loop_counter++;
  }
}