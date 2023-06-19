#ifndef _RCC_H
#define _RCC_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx.h"
#include "pwr.h"
#include "flash.h"


typedef enum RCC_CLOCK {
    RCC_CLOCK_hsi,
    RCC_CLOCK_hse,
    RCC_CLOCK_pll,
    RCC_CLOCK_lsi,
    RCC_CLOCK_lse,
    RCC_CLOCK_hsi48,
} rcc_ClockType;

typedef enum RCC_MCO_SOURCE {
    RCC_MCO_SOURCE_disabled,
    RCC_MCO_SOURCE_sysclk,
    RCC_MCO_SOURCE_hsi = 3,
    RCC_MCO_SOURCE_hse,
    RCC_MCO_SOURCE_pll,
    RCC_MCO_SOURCE_lsi,
    RCC_MCO_SOURCE_lse,
    RCC_MCO_SOURCE_hsi48,
} rcc_McoSourceType;

typedef enum RCC_MCO_SCALE {
    RCC_MCO_SCALE_Div1 = RCC_CFGR_MCOPRE_DIV1,
    RCC_MCO_SCALE_Div2 = RCC_CFGR_MCOPRE_DIV2,
    RCC_MCO_SCALE_Div4 = RCC_CFGR_MCOPRE_DIV4,
    RCC_MCO_SCALE_Div8 = RCC_CFGR_MCOPRE_DIV8,
    RCC_MCO_SCALE_Div16 = RCC_CFGR_MCOPRE_DIV16,
} rcc_McoScaleType;

typedef enum RCC_PLL_SOURCE {
    RCC_PLL_SOURCE_none = RCC_PLLCFGR_PLLSRC_0,
    RCC_PLL_SOURCE_hsi = RCC_PLLCFGR_PLLSRC_HSI,
    RCC_PLL_SOURCE_hse = RCC_PLLCFGR_PLLSRC_HSE,
} rcc_PllSourceType;

typedef enum RCC_SYSCLK_SOURCE {
    RCC_SYSCLK_SOURCE_hsi = RCC_CFGR_SW_HSI,
    RCC_SYSCLK_SOURCE_hse = RCC_CFGR_SW_HSE,
    RCC_SYSCLK_SOURCE_pll = RCC_CFGR_SW_PLL,
} rcc_SysClkSourceType;

typedef enum RCC_AHB_SCALE {
    RCC_AHB_SCALE_Div1 = RCC_CFGR_HPRE_DIV1,
    RCC_AHB_SCALE_Div2 = RCC_CFGR_HPRE_DIV2,
    RCC_AHB_SCALE_Div4 = RCC_CFGR_HPRE_DIV4,
    RCC_AHB_SCALE_Div8 = RCC_CFGR_HPRE_DIV8,
    RCC_AHB_SCALE_Div16 = RCC_CFGR_HPRE_DIV16,
    RCC_AHB_SCALE_Div64 = RCC_CFGR_HPRE_DIV64,
    RCC_AHB_SCALE_Div128 = RCC_CFGR_HPRE_DIV128,
    RCC_AHB_SCALE_Div256 = RCC_CFGR_HPRE_DIV256,
    RCC_AHB_SCALE_Div512 = RCC_CFGR_HPRE_DIV512,
} rcc_AhbScaleType;

typedef enum RCC_APB_SCALE {
    RCC_APB_SCALE_Div1 = RCC_CFGR_PPRE1_DIV1,
    RCC_APB_SCALE_Div2 = RCC_CFGR_PPRE1_DIV2,
    RCC_APB_SCALE_Div4 = RCC_CFGR_PPRE1_DIV4,
    RCC_APB_SCALE_Div8 = RCC_CFGR_PPRE1_DIV8,
    RCC_APB_SCALE_Div16 = RCC_CFGR_PPRE1_DIV16
} rcc_ApbScaleType;

typedef struct rcc_Clock {
    uint8_t pll_source;
    uint8_t pllm;
    uint8_t plln;
    uint8_t pllp;
    uint8_t pllq;
    uint8_t pllr;
    uint8_t sysclk_source;
    uint8_t flash_waitstates; 
    rcc_McoSourceType mco_source;
    rcc_McoScaleType mco_scale;
    rcc_AhbScaleType ahb_scale;
    rcc_ApbScaleType apb1_scale;
    rcc_ApbScaleType apb2_scale;
    pwr_VosRangeType vos_range;
    bool boost_en;
    uint32_t ahb_frequency; 
    uint32_t apb1_frequency; 
    uint32_t apb2_frequency;
} rcc_Clock;


// Clock Configurations
extern const rcc_Clock rcc_Hsi_Pll_170MHz;

void rcc_clock_init(const rcc_Clock * clk);
void rcc_enable_pll(void);
void rcc_disable_pll(void);
bool rcc_pll_is_ready(void);
void rcc_enable_hsi(void);
void rcc_disable_hsi(void);
bool rcc_hsi_is_ready(void);
void rcc_enable_hse(void);
void rcc_disable_hse(void);
bool rcc_hse_is_ready(void);
void rcc_set_sysclk_source(rcc_SysClkSourceType source);
rcc_SysClkSourceType rcc_get_sysclk_source(void);
void rcc_set_pll_source(rcc_PllSourceType source);
void rcc_set_pll_scale(const rcc_Clock * clk );
void rcc_set_mco_source(rcc_McoSourceType source);
void rcc_set_mco_scale(rcc_McoScaleType mcopre_scale );
void rcc_set_ahb_scale(rcc_AhbScaleType hpre_scale);
void rcc_set_apb1_scale(rcc_ApbScaleType ppre1_scale);
void rcc_set_apb2_scale(rcc_ApbScaleType ppre2_scale);
void rcc_css_on(void);
void rcc_css_off(void);
void rcc_osc_hse_ext_bypass_enable(void);
void rcc_osc_hse_ext_bypass_disable(void);

#endif // _RCC_H

