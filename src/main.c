#include <stdint.h>
#include "stm32g4xx.h"

#define LED_PIN   5


void main(void)
{

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

    for (uint32_t i = 0; i < 1000000; i++)
    {
      counter++;
    }

    GPIOA->ODR ^= (1 << LED_PIN);
  }

}