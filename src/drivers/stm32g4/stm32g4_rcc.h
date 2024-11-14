#ifndef STM32G4_RCC_H
#define STM32G4_RCC_H

#include "stm32g4_common.h"

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
    RCC_PLL_SOURCE_HSE_BYPASS,
    RCC_PLL_SOURCE_END,
};

enum rcc_sysclk_sources {
    RCC_SYSCLK_SOURCE_RESERVED,
    RCC_SYSCLK_SOURCE_HSI16,
    RCC_SYSCLK_SOURCE_HSE,
    RCC_SYSCLK_SOURCE_PLL,
    RCC_SYSCLK_SOURCE_END,
};

enum rcc_usbclk_sources {
    RCC_USBCLK_SOURCE_HSI48,
    RCC_USBCLK_SOURCE_PPLQ,
    RCC_USBCLK_SOURCE_END,
};

enum rcc_crs_sync_polarities {
    RCC_CRS_SYNC_POLARITY_RISING,
    RCC_CRS_SYNC_POLARITY_FALLING,
    RCC_CRS_SYNC_POLARITY_END,
};

enum rcc_crs_sync_sources {
    RCC_CRS_SYNC_SOURCE_GPIO,
    RCC_CRS_SYNC_SOURCE_USB,
    RCC_CRS_SYNC_SOURCE_LSE,
    RCC_CRS_SYNC_SOURCE_END,
};

enum rcc_crs_sync_scales {
    RCC_CRS_SYNC_DIV1,
    RCC_CRS_SYNC_DIV2,
    RCC_CRS_SYNC_DIV4,
    RCC_CRS_SYNC_DIV8,
    RCC_CRS_SYNC_DIV16,
    RCC_CRS_SYNC_DIV32,
    RCC_CRS_SYNC_DIV64,
    RCC_CRS_SYNC_DIV128,
    RCC_CRS_SYNC_END,
};

enum rcc_adcclk_sources {
    RCC_ADC_CLK_SOURCE_NONE,
    RCC_ADC_CLK_SOURCE_PLLP,
    RCC_ADC_CLK_SOURCE_SYSCLK,
    RCC_ADC_CLK_SOURCE_END,
};


typedef const struct {
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
    enum rcc_usbclk_sources usbckl_source;
    enum rcc_clk_scales mco_scale;
    enum rcc_clk_scales hclk_scale;
    enum rcc_clk_scales pclk1_scale;
    enum rcc_clk_scales pclk2_scale;
    enum rcc_adcclk_sources adc12clk_source;
    enum rcc_adcclk_sources adc345clk_source;
} rcc_clock_config_t;

typedef const struct {
    enum rcc_crs_sync_polarities sync_polarity;
    enum rcc_crs_sync_sources sync_source;
    enum rcc_crs_sync_scales sync_scale;
    uint32_t reload_value;
    uint32_t error_limit_value;
    uint32_t hsi48_calibration_value;
} rcc_crs_config_t;

/**
  * @brief  Macro to calculate reload value to be set in CRS register according to target and sync frequencies
  * @note   The RELOAD value should be selected according to the ratio between the target frequency and the frequency
  *             of the synchronization source after prescaling. It is then decreased by one in order to
  *             reach the expected synchronization on the zero value. The formula is the following:
  *             RELOAD = (fTARGET / fSYNC) -1
  * @param  __FTARGET__ Target frequency (value in Hz)
  * @param  __FSYNC__ Synchronization signal frequency (value in Hz)
  * @retval None
  */
#define RCC_CRS_ReloadValue_Calculate(__FTARGET__, __FSYNC__)  (((__FTARGET__) / (__FSYNC__)) - 1U)

void rcc_clock_init(rcc_clock_config_t *cfg);
void rcc_crs_init(rcc_crs_config_t *cfg);

#endif