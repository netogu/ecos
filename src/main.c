#include <stdint.h>
#include "stm32g4xx.h"
#include "rcc.h"
#include "pwr.h"
#include "flash.h"

#define LED_PIN   5

volatile uint32_t msTicks = 0;

void delay_ms(uint32_t ms);

void main(void)
{

  rcc_clock_init(&rcc_Hsi_Pll_170MHz);

  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock/1000);
  __enable_irq();

  RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);

  // Two dummy reads after enabling the peripheral clock
  volatile uint32_t dummy;
  dummy = RCC->AHB2ENR;
  dummy = RCC->AHB2ENR;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk);
  GPIOA->MODER |= (1 << GPIO_MODER_MODE5_Pos);
  GPIOA->BSRR |= (1 << LED_PIN);

  uint32_t counter = 0;
  while(1)
  {
    GPIOA->ODR ^= (1 << LED_PIN);
    delay_ms(500);
  }

}

void SysTick_Handler(void) {
  msTicks++;
}

void delay_ms(uint32_t ms) {
  uint32_t start = msTicks;
  uint32_t end = start + ms;

  if ( end < start ) {
    while ( msTicks > start);
  }

  while( msTicks < end );
}