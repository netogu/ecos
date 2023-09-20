#include "drv_rcc.h"
#include "drv_usb.h"
#include "stm32g474xx.h"
#include "stm32g4xx.h"
#include <stdint.h>

#define LED_PIN 5

rcc_clock_config_t clock_170MHz_pll_hsi = {
    .sysclk_source = RCC_SYSCLK_SOURCE_PLL,
    .pll_source = RCC_PLL_SOURCE_HSI,
    .usbckl_source = RCC_USBCLK_SOURCE_HSI48,
    .pllm = 4,
    .plln = 85,
    .pllp = 2,
    .pllq = 2,
    .pllr = 2,
    .sysclk_scale = RCC_CLK_DIV2,
    .pclk1_scale = RCC_CLK_DIV1,
    .pclk2_scale = RCC_CLK_DIV1,
    .flash_wait_states = 4,
    .vos_range = 1,
    .boost_mode = 1,
};

volatile uint32_t msTicks = 0;

void SysTick_Init(void) {
  SysTick->CTRL = 0;
  msTicks = 0;
  SysTick->VAL = 0; /* Load the SysTick Counter Value */
  SysTick->CTRL = (SysTick_CTRL_TICKINT_Msk | /* Enable SysTick exception */
                   SysTick_CTRL_ENABLE_Msk) | /* Enable SysTick system timer */
                  SysTick_CTRL_CLKSOURCE_Msk; /* Use processor clock source */
}

void delay_ms(uint32_t ms);

int main(void) {

  rcc_clock_init(&clock_170MHz_pll_hsi);
  SystemCoreClockUpdate();
  SysTick_Init();
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
    for (int i = 0; i < 17000000; i++) {
      __ASM("nop");
    }
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
