#include "hal.h"

static union {
  struct {
    uint32_t low_u32;
    uint32_t high_u32;
  };
  uint64_t cnt;
} timer_us_ticks;



void timer_us_init(void) {
  // Initialize the timer

  timer_us_ticks.cnt = 0;

  // Configuring TIM2 as a 32-bit microsecond timer

    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Set the prescaler to 1MHz
    TIM2->PSC = 170-1;
    // Set Period to max value
    TIM2->ARR = 0xFFFFFFFF;

    // Enable Periodic Interrupt
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable the timer
    TIM2->CR1 |= TIM_CR1_CEN;
}


uint64_t timer_us_get(void) {
  // Get the TIM2 value
  timer_us_ticks.low_u32 = TIM2->CNT;
  return timer_us_ticks.cnt;
}

// ISR for TIM2
void TIM2_IRQHandler(void) {
  // Clear the interrupt
  TIM2->SR &= ~TIM_SR_UIF;
  timer_us_ticks.high_u32++;
}
