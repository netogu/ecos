#include "rcc.h"


const rcc_Clock rcc_Hsi_Pll_170MHz = {
    .sysclk_source = RCC_SYSCLK_SOURCE_pll,
    .pll_source = RCC_PLL_SOURCE_hsi,
    .pllm = 4,
    .plln = 85,
    .pllp = 0,
    .pllr = 2,
    .ahb_scale = RCC_AHB_SCALE_Div1,
    .apb1_scale = RCC_APB_SCALE_Div1,
    .apb2_scale = RCC_APB_SCALE_Div1,
    .vos_range = PWR_VOS_RANGE_1,
    .boost_en = true,
    .flash_waitstates = 4,
    .ahb_frequency = 170e6,
    .apb1_frequency = 170e6,
    .apb2_frequency = 170e6
};

void rcc_enable_peripheral_clk(uint32_t peripheral_clk) {

}

void rcc_clock_init(const rcc_Clock * clk) {

    rcc_enable_hsi();
    while (!rcc_hsi_is_ready());

    rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

    if (clk->pll_source == RCC_PLL_SOURCE_hse || clk->sysclk_source == RCC_SYSCLK_SOURCE_hse) {
        rcc_enable_hse();

        while (!rcc_hse_is_ready());
        
    }
    // Voltage Regulator Confiruation
    //enable peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    pwr_set_vos_range(clk->vos_range);

    if (clk->boost_en) {
        pwr_enable_boost();
    } else {
        pwr_disable_boost();
    }

    // Pre-scaler Configuration
    rcc_set_ahb_scale(clk->ahb_scale);
    rcc_set_apb1_scale(clk->apb1_scale);
    rcc_set_apb2_scale(clk->apb2_scale);
    
    // PLL Configuration
    if (clk->sysclk_source == RCC_SYSCLK_SOURCE_pll) {

        rcc_disable_pll();

        while (rcc_pll_is_ready()); // Wait for PLL to unlock

        rcc_set_pll_source(clk->pll_source);
        rcc_set_pll_scale(clk);
        rcc_enable_pll();

        while (!rcc_pll_is_ready()); // Wait for PLL to lock

    }

    // Flash Configuration
    flash_enable_data_cache();
    flash_enable_instruction_cache();
    flash_set_latency(clk->flash_waitstates);


    rcc_set_sysclk_source(clk->sysclk_source);

   while (rcc_get_sysclk_source() != clk->sysclk_source); // Wait for selected clock to be locked

    // If system is using HSE, disable HSI
    if (clk->pll_source == RCC_PLL_SOURCE_hse) {
        rcc_disable_hsi();
    }
}

void rcc_set_pll_scale(const rcc_Clock * clk) {

    // freq(VCO clk) = freq(PLL clk in) * (plln / pllm)
    // freq(PLL_P) = f(VCO clk) / pllp
    // freq(PLL_Q) = f(VCO clk) / pllq
    // freq(PLL_R) = f(VCO clk) / pllr


    // Clear PLLM, PLLN , PLLP and PLLR
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM |
                      RCC_PLLCFGR_PLLN |
                      RCC_PLLCFGR_PLLP |
                      RCC_PLLCFGR_PLLR);

    // PLL M
    RCC->PLLCFGR |= (clk->pllm << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM_Msk;
    // PLL N
    RCC->PLLCFGR |= (clk->plln << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN_Msk;

    // PLL P
    if (clk->pllp > 0) {
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
        switch (clk->pllp) {
            case 17:
                RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLPDIV;
                RCC->PLLCFGR |= RCC_PLLCFGR_PLLP;
                break;
            case 7:
                RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLPDIV;
                RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
                break;
            default:
                RCC->PLLCFGR |= (clk->pllp << RCC_PLLCFGR_PLLPDIV_Pos) & RCC_PLLCFGR_PLLPDIV_Msk;
                break;
        }
    }

    // PLL Q
    if (clk->pllq > 0) {
        uint32_t pllq = clk->pllq;
        pllq = (pllq >> 1) - 1;
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN;
        RCC->PLLCFGR |= (pllq << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk;
    }
    
    // PLL R
    if (clk->pllr > 0) {
        uint32_t pllr = clk->pllr;
        pllr = (pllr >> 1) - 1;
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
        RCC->PLLCFGR |= (pllr << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR_Msk;
    }
} //rcc_set_pll_scale

void rcc_set_mco_scale(rcc_McoScaleType mcopre_scale) {

    uint32_t reg = RCC->CFGR;
    reg &= ~RCC_CFGR_MCOPRE;

    RCC->CFGR |= ( reg | mcopre_scale );
    
} //rcc_set_mco_scale

void rcc_set_mco_source(rcc_McoSourceType clk_source) {

    uint32_t reg = RCC->CFGR;
    reg &= ~RCC_CFGR_MCOSEL;

    RCC->CFGR |= ( reg | clk_source );
}

void rcc_set_ahb_scale(rcc_AhbScaleType hpre1_scale) {

    uint32_t reg = RCC->CFGR;
    reg &= ~RCC_CFGR_HPRE;
    RCC->CFGR |= ( reg | hpre1_scale );

}

void rcc_set_apb1_scale(rcc_ApbScaleType ppre1_scale) {

    uint32_t reg = RCC->CFGR;
    reg &= ~RCC_CFGR_PPRE1;
    RCC->CFGR |= ( reg | ppre1_scale );

}

void rcc_set_apb2_scale(rcc_ApbScaleType ppre2_scale) {

    uint32_t reg = RCC->CFGR;
    reg &= ~RCC_CFGR_PPRE2;
    RCC->CFGR |= ( reg | ppre2_scale );
}

void rcc_enable_pll(void) {

    RCC->CR |= RCC_CR_PLLON;
}

void rcc_enable_hsi(void) {

    RCC->CR |= RCC_CR_HSION;
}

void rcc_enable_hse(void) {

    RCC->CR |= RCC_CR_HSEON;
}

void rcc_disable_pll(void) {

    RCC->CR &= ~RCC_CR_PLLON;
}

void rcc_disable_hsi(void) {

    RCC->CR &= ~RCC_CR_HSION;
}

void rcc_disable_hse(void) {

    RCC->CR &= ~RCC_CR_HSEON;
}

bool rcc_pll_is_ready(void) {

    return RCC->CR & RCC_CR_PLLRDY;
}

bool rcc_hsi_is_ready(void) {

    return RCC->CR & RCC_CR_HSIRDY;
}

bool rcc_hse_is_ready(void) {

    return RCC->CR & RCC_CR_HSERDY;
}


void rcc_set_sysclk_source(rcc_SysClkSourceType clk_src) {

    uint32_t reg = RCC->CFGR;
    reg &= ~RCC_CFGR_SW;

    RCC->CFGR |= ( reg | clk_src );
}

rcc_SysClkSourceType rcc_get_sysclk_source(void) {

    rcc_SysClkSourceType sysclk = ( RCC->CFGR & RCC_CFGR_SWS ) >> RCC_CFGR_SWS_Pos;
    
    return sysclk;
}


void rcc_set_pll_source(rcc_PllSourceType clk_src){

    uint32_t reg = RCC->PLLCFGR;
    reg &= ~(RCC_PLLCFGR_PLLSRC);

    RCC->PLLCFGR |= ( reg | clk_src );
}


void rcc_css_on(void) {

    RCC->CR |= RCC_CR_CSSON;
}

void rcc_css_off(void) {

    RCC->CR &= ~RCC_CR_CSSON;
}

void rcc_osc_hse_ext_bypass_enable(void) {

    RCC->CR |= RCC_CR_HSEBYP;
}

void rcc_osc_hse_ext_bypass_disable(void) {

    RCC->CR |= ~RCC_CR_HSEBYP;
}
