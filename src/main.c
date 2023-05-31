#include "stm32g4xx.h"

#define LED_PIN   5

void inline __attribute__((always_inline)) delay(uint32_t delay)
{
  while(delay--) __asm("");
}

void main(void)
{

  RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);

  // Two dummy reads after enabling the peripheral clock
  volatile uint32_t dummy;
  dummy = RCC->AHB1ENR;
  dummy = RCC->AHB1ENR;

  GPIOA->MODER |= (1 << GPIO_MODER_MODE5_Pos);

  while(1)
  {
    GPIOA->ODR ^= (1 << LED_PIN);
    delay(100);
  }

}