#pragma once

#include <stdint.h>

enum rcc_clk_scales {
  RCC_CLK_DIV1,
  RCC_CLK_DIV2,
  RCC_CLK_DIV4,
  RCC_CLK_DIV8,
  RCC_CLK_DIV16,
  RCC_CLK_DIV_64,
  RCC_CLK_DIV128,
  RCC_CLK_DIV256,
  RCC_CLK_DIV512,
  RCC_CLK_DIV_END
};

enum rcc_pll_sources {
  RCC_PLL_SOURCE_NONE1,
  RCC_PLL_SOURCE_NONE2,
  RCC_PLL_SOURCE_HSI,
  RCC_PLL_SOURCE_HSE,
  RCC_PLL_SOURCE_END,
};

enum rcc_sysclk_sources {
  RCC_SYSCLK_SOURCE_RESERVED,
  RCC_SYSCLK_SOURCE_HSI16,
  RCC_SYSCLK_SOURCE_HSE,
  RCC_SYSCLK_SOURCE_PLL,
  RCC_SYSCLK_SOURCE_END,
};

struct rcc_clock_config {
  uint8_t pllm;
  uint8_t plln;
  uint8_t pllp;
  uint8_t pllq;
  uint8_t pllr;
  uint8_t flash_wait_states;
  uint8_t vos_range;
  uint8_t boost_mode;
  enum rcc_pll_sources pll_source;
  enum rcc_sysclk_sources sysclk_source;
  enum rcc_clk_scales mco_scale;
  enum rcc_clk_scales sysclk_scale;
  enum rcc_clk_scales pclk1_scale;
  enum rcc_clk_scales pclk2_scale;
  uint32_t sysclk_frequency;
  uint32_t pclk1_frequency;
  uint32_t pclk2_frequency;
};

// Clock Configurations
extern const struct rcc_clock_config rcc_hsi_pll_170MHz;

void rcc_clock_init(const struct rcc_clock_config *cfg);
