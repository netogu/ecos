#include "drv_rcc.h"
#include "drv_usb.h"
#include "stm32g4xx.h"
#include <stdint.h>

#define LED_PIN 5

volatile uint32_t msTicks = 0;

void delay_ms(uint32_t ms);

int main(void) {

  rcc_clock_init(&rcc_hsi_pll_170MHz);

  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);
  drv_usb_init();
  __enable_irq();

  RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);

  // Two dummy reads after enabling the peripheral clock
  __attribute__((unused)) uint32_t dummy;
  dummy = RCC->AHB2ENR;
  dummy = RCC->AHB2ENR;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk);
  GPIOA->MODER |= (1 << GPIO_MODER_MODE5_Pos);
  GPIOA->BSRR |= (1 << LED_PIN);

  while (1) {
    GPIOA->ODR ^= (1 << LED_PIN);
    delay_ms(100);
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
