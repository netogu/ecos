#pragma once

#include <stdint.h>

volatile uint32_t msTicks = 0;

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