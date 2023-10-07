#include <stdint.h>
#include "board/config.h"
#include "hardware/stm32g4/rcc.h"



volatile uint32_t msTicks = 0;

void delay_ms(uint32_t ms);

int main(void) {

  board_clock_setup();
  board_gpio_setup();

  while (1) {
    GPIOA->ODR ^= (1 << LED_PIN);
    delay_ms(500);
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
