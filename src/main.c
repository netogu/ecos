#include <stdint.h>
// #include <stdio.h>
// #include <string.h>
#include "board/config.h"

void lpuart_write_str(char * buffer, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
      lpuart_write(buffer[i]);
    }
}


volatile uint32_t msTicks = 0;

void delay_ms(uint32_t ms);
uint32_t counter = 0;

int main(void) {

  board_clock_setup();
  board_gpio_setup();
  board_serial_setup();

  while (1) {
    gpio_pin_toggle(&gpios.led_green);

    char msg[] = "Hello World!\r\n";
    // sprintf(msg, "SystemCoreClock = %d\r\n", SystemCoreClock);
    // lpuart_write(msg, strlen(msg));
    lpuart_write(msg, 14);
    delay_ms(100);
    counter++;
  }
}

void SysTick_Handler(void) { msTicks++; }

void delay_ms(uint32_t ms) {
  uint32_t start = msTicks;
  uint32_t end = start + ms;

  if (end < start) {
    while (msTicks > start)
      ;
  }

  while (msTicks < end)
    ;
}

// void USB_LP_IRQHandler() {
// }

// void USB_HP_IRQHandler() {
// }
