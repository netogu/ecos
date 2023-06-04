#include "rcc.h"


const struct rcc_clock_setup rcc_clock_setup_hsi_170mhz = {
    .pllm = 4,
    .plln = 85,
    .pllp = 0,
    .pllq = 0,
    .pllr = 2,
    .pll_source = RCC_PLLCFGR_PLLSRC_HSI,
    .hpre = RCC_CFGR_HPRE_DIV1,
    .ppre1 = RCC_CFGR_PPRE1_DIV1,
    .ppre2 = RCC_CFGR_PPRE2_DIV1,
    .vos_scale = PWR_CR1_VOS_0,
    .boost = 0,
    .flash_config = 0,
    .flash_waitstates = 1,
    .ahb_frequency = 170e6,
    .apb1_frequency = 170e6,
    .apb2_frequency = 170e6
    
};
