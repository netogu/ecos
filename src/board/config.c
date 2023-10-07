
#include <stdint.h>
#include "board/config.h"
#include "hardware/stm32g4/rcc.h"




void board_clock_setup(void) {

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

  rcc_clock_init(&clock_170MHz_pll_hsi);
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);

}

void board_gpio_setup(void) {

  RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);

  // Two dummy reads after enabling the peripheral clock
  __attribute__((unused)) uint32_t dummy;
  dummy = RCC->AHB2ENR;
  dummy = RCC->AHB2ENR;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk);
  GPIOA->MODER |= (1 << GPIO_MODER_MODE5_Pos);
  GPIOA->BSRR |= (1 << LED_PIN);

}
