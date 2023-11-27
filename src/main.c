#include <stdint.h>
// #include <string.h>
#include "external/printf.h"
#include "board/config.h"
#include "lib/delay.h"
#include "lib/debug.h"
#include "tusb.h"

// USB Device IRQ Monitor
extern int usb_lp_irq_counter;


int main_loop_counter = 0;
int main(void) {

  board_clock_setup();
  board_gpio_setup();
  board_serial_setup();
  board_usb_setup();
  tusb_init();
  printf("Waiting for USB\r\n");
  while (!tud_cdc_connected()) {
    for (int i = 0; i < 80; i++)
      printf(".");
      delay_ms(100);
    printf("\r\n");
  }
  printf("\r\nUSB connected\r\n");
  
  while (1) {
    tud_task();
    gpio_pin_toggle(&gpios.led_green);

    printf("cnt_main, cnt_usb_irq: %d,%d \r\n", 
      main_loop_counter, 
      usb_lp_irq_counter);

    delay_ms(1000);
    main_loop_counter++;
  }
}

//--------------------------------------------------------------------+
// USB Mount Callback API (Optional)
//--------------------------------------------------------------------+
// Invoked when device is mounted
void tud_mount_cb(void) {
  printf("USB mounted\r\n");
}
// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {
}