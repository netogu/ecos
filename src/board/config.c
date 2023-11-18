
#include <stdint.h>
#include "board/config.h"
#include "hardware/stm32g4/rcc.h"
#include "hardware/stm32g4/gpio.h"

//------------------------------------------------------
// GPIOs
//------------------------------------------------------
gpio_t gpio_led_green;

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

    rcc_crs_config_t crs_config = {
        .sync_source = RCC_CRS_SYNC_SOURCE_USB,
        .sync_polarity = RCC_CRS_SYNC_POLARITY_RISING,
        .sync_scale = RCC_CRS_SYNC_DIV1,
        .error_limit_value = 34,
        .hsi48_calibration_value = 32,
    };

  rcc_clock_init(&clock_170MHz_pll_hsi);
  rcc_crs_init(&crs_config);
  
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);

}

void board_gpio_setup(void) {

  gpio_led_green = (gpio_t){
    .port = GPIO_PORT_A,
    .pin = 5,
    .mode = GPIO_MODE_OUTPUT,
    .type = GPIO_TYPE_PUSH_PULL,
    .pull = GPIO_PULL_UP,
    .speed = GPIO_SPEED_HIGH,
    .af = 0,
  };

  gpio_pin_init(&gpio_led_green);

}
