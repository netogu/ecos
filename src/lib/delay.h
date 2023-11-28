#pragma once

#include <stdint.h>

volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) { system_ticks++; }

uint32_t board_millis(void){
  return system_ticks;
}

void delay_ms(uint32_t ms) {
  uint32_t start = system_ticks;
  uint32_t end = start + ms;

  if (end < start) {
    while (system_ticks > start)
      ;
  }

  while (system_ticks < end)
    ;
}