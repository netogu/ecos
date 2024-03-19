#include "drivers/stm32g4/common.h"
#include "drivers/stm32g4/rcc.h"
#include "drivers/stm32g4/flash.h"
#include "drivers/stm32g4/pwr.h"
#include "stm32g4xx.h"

enum rcc_oscs {
  RCC_OSC_HSI,
  RCC_OSC_HSE,
  RCC_OSC_LSI,
  RCC_OSC_LSE,
  RCC_OSC_PLL,
  RCC_OSC_HSI48,
  RCC_OSC_END
};

enum rcc_clocks {
  RCC_CLK_SYSCLK,
  RCC_CLK_HCLK,
  RCC_CLK_PCLK1,
  RCC_CLK_PCLK2,
  RCC_CLK_MCO,
  RCC_CLK_END
};

enum rcc_mco_sources {
  RCC_MCO_SOURCE_DISABLED,
  RCC_MCO_SOURCE_SYSCLK,
  RCC_MCO_SOURCE_RESERVED,
  RCC_MCO_SOURCE_HSI16,
  RCC_MCO_SOURCE_HSE,
  RCC_MCO_SOURCE_PLL,
  RCC_MCO_SOURCE_LSI,
  RCC_MCO_SOURCE_LSE,
  RCC_MCO_SOURCE_HSI48,
  RCC_MCO_SOURCE_END,
};

#define RCC_ERROR -1
#define RCC_SUCCESS 0

#define _rcc_enable_pll() (RCC->CR |= RCC_CR_PLLON)
#define _rcc_disable_pll() (RCC->CR &= ~RCC_CR_PLLON)
#define _rcc_pll_is_ready() (RCC->CR & RCC_CR_PLLRDY)

#define _rcc_enable_hse() (RCC->CR |= RCC_CR_HSEON)
#define _rcc_disable_hse() (RCC->CR &= ~RCC_CR_HSEON)
#define _rcc_hse_is_ready() (RCC->CR & RCC_CR_HSERDY)

#define _rcc_enable_hsi() (RCC->CR |= RCC_CR_HSION)
#define _rcc_disable_hsi() (RCC->CR &= ~RCC_CR_HSION)
#define _rcc_hsi_is_ready() (RCC->CR & RCC_CR_HSIRDY)

#define _rcc_enable_hse_css() (RCC->CR |= RCC_CR_CSSON)
#define _rcc_disable_hse_css() (RCC->CR &= ~RCC_CR_CSSON)
#define _rcc_hse_css_is_ready() (RCC->CR & RCC_CR_CSSF)

#define _rcc_enable_hse_bypass() (RCC->CR |= RCC_CR_HSEBYP)
#define _rcc_disable_hse_bypass() (RCC->CR &= ~RCC_CR_HSEBYP)

static uint32_t rcc_set_hclk_div(enum rcc_clk_scales scale) {
  if (!In_range(scale, RCC_CLK_DIV1, RCC_CLK_DIV512))
    return RCC_ERROR;
  uint32_t sysclk_scale;
  sysclk_scale = (scale == RCC_CLK_DIV1 ? scale : scale + 7);
  Modify_register_field(RCC->CFGR, RCC_CFGR_HPRE, sysclk_scale);
  return RCC_SUCCESS;
}

static uint32_t rcc_set_pclk1_div(enum rcc_clk_scales scale) {
  if (!In_range(scale, RCC_CLK_DIV1, RCC_CLK_DIV16))
    return RCC_ERROR;
  uint32_t pclk_scale;
  pclk_scale = (scale == RCC_CLK_DIV1 ? scale : scale + 3);
  Modify_register_field(RCC->CFGR, RCC_CFGR_PPRE1, pclk_scale);
  return RCC_SUCCESS;
}

static uint32_t rcc_set_pclk2_div(enum rcc_clk_scales scale) {
  if (!In_range(scale, RCC_CLK_DIV1, RCC_CLK_DIV16))
    return RCC_ERROR;
  uint32_t pclk_scale;
  pclk_scale = (scale == RCC_CLK_DIV1 ? scale : scale + 3);
  Modify_register_field(RCC->CFGR, RCC_CFGR_PPRE2, pclk_scale);
  return RCC_SUCCESS;
}

/* static void rcc_set_mco_source(enum rcc_mco_sources clk_src) { */
/*   Modify_register_field(RCC->CFGR, RCC_CFGR_MCOSEL, clk_src); */
/* } */

static void rcc_set_sysclk_source(enum rcc_sysclk_sources clk_src) {
  Modify_register_field(RCC->CFGR, RCC_CFGR_SW, clk_src);
}

static uint32_t rcc_get_sysclk_source(void) {
  uint32_t sysclk_src = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;
  return sysclk_src;
}

static void rcc_set_pll_source(enum rcc_pll_sources clk_src) {
  Modify_register_field(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, clk_src);
}

static void rcc_set_usbclk_source(enum rcc_usbclk_sources clk_src) {
  uint32_t usbclk_src;
  if (clk_src == RCC_USBCLK_SOURCE_HSI48) {
    usbclk_src = 0b00;
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
      ;
  }
  if (clk_src == RCC_USBCLK_SOURCE_PPLQ)
    usbclk_src = 0b10;
  Modify_register_field(RCC->CCIPR, RCC_CCIPR_CLK48SEL, usbclk_src);
}

static void rcc_set_adcclk_source(rcc_clock_config_t *cfg) {
  uint32_t adc12clk_src = cfg->adc12clk_source;
  uint32_t adc345clk_src = cfg->adc345clk_source;

  Modify_register_field(RCC->CCIPR, RCC_CCIPR_ADC12SEL, adc12clk_src);
  Modify_register_field(RCC->CCIPR, RCC_CCIPR_ADC345SEL, adc345clk_src);
}

static void rcc_set_pll_scale(rcc_clock_config_t *cfg) {

  /*
   * PLL Configuration
   * freq(VCO clk) = freq(PLL clk in) * (plln / pllm)
   * freq(PLL_P) = f(VCO clk) / pllp
   * freq(PLL_Q) = f(VCO clk) / pllq
   * freq(PLL_R) = f(VCO clk) / pllr
   */

  // Clear PLLM, PLLN , PLLP and PLLR
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP |
                    RCC_PLLCFGR_PLLR);

  // PLL M
  uint32_t pllm = Limit(cfg->pllm - 1, 1, 16);
  Modify_register_field(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, pllm);
  // PLL N
  Modify_register_field(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, cfg->plln);

  // PLL P
  if (cfg->pllp > 0) {
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
    switch (cfg->pllp) {
    case 17:
      RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLPDIV;
      RCC->PLLCFGR |= RCC_PLLCFGR_PLLP;
      break;
    case 7:
      RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLPDIV | ~RCC_PLLCFGR_PLLP);
      break;
    default:
      Modify_register_field(RCC->PLLCFGR, RCC_PLLCFGR_PLLPDIV, cfg->pllp);
      break;
    }
  }

  // PLL Q
  if (cfg->pllq > 0) {
    uint32_t pllq = cfg->pllq;
    pllq = (pllq >> 1) - 1;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN;
    RCC->PLLCFGR |= (pllq << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk;
  }

  // PLL R
  if (cfg->pllr > 0) {
    uint32_t pllr = cfg->pllr;
    pllr = (pllr >> 1) - 1;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
    Modify_register_field(RCC->PLLCFGR, RCC_PLLCFGR_PLLR, pllr);
  }
} // rcc_set_pll_scale

void rcc_clock_init(rcc_clock_config_t *cfg) {

  _rcc_enable_hsi();
  while (!_rcc_hsi_is_ready())
    ;

  rcc_set_sysclk_source(RCC_SYSCLK_SOURCE_HSI16);

  if (cfg->pll_source == RCC_PLL_SOURCE_HSE_BYPASS) {
    _rcc_enable_hse_bypass();
  } else {
    _rcc_disable_hse_bypass();
  }

  if (cfg->pll_source == RCC_PLL_SOURCE_HSE ||
      cfg->pll_source == RCC_PLL_SOURCE_HSE_BYPASS ||
      cfg->sysclk_source == RCC_SYSCLK_SOURCE_HSE) {

    _rcc_enable_hse();

    while (!_rcc_hse_is_ready())
      ;
  }
  // Voltage Regulator Confiruation
  // enable peripheral clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;

  pwr_set_vos_range(cfg->vos_range);

  if (cfg->boost_mode) {
    pwr_enable_boost();
  } else {
    pwr_disable_boost();
  }

  // Pre-scaler Configuration
  rcc_set_hclk_div(cfg->hclk_scale);
  rcc_set_pclk1_div(cfg->pclk1_scale);
  rcc_set_pclk2_div(cfg->pclk2_scale);

  // PLL Configuration
  if (cfg->sysclk_source == RCC_SYSCLK_SOURCE_PLL) {

    _rcc_disable_pll();

    while (_rcc_pll_is_ready())
      ; // Wait for PLL to unlock

    rcc_set_pll_source(cfg->pll_source);
    rcc_set_pll_scale(cfg);
    _rcc_enable_pll();

    while (!_rcc_pll_is_ready())
      ; // Wait for PLL to lock

    // USB Clock Configuration
    rcc_set_usbclk_source(cfg->usbckl_source);
    rcc_set_adcclk_source(cfg);
  }

  //  Flash Configuration
  flash_enable_data_cache();
  flash_enable_instruction_cache();
  flash_set_latency(cfg->flash_wait_states);

  rcc_set_sysclk_source(cfg->sysclk_source);

  uint32_t sysclk_src = rcc_get_sysclk_source();
  while (sysclk_src != cfg->sysclk_source)
    ; // Wait for selected clock to be locked

  // If system is using HSE, disable HSI
  if (cfg->pll_source == RCC_PLL_SOURCE_HSE) {
    _rcc_disable_hsi();
  }
}

void rcc_crs_init(rcc_crs_config_t *cfg) {
  
  // Enable CRS peripheral clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;

  /* Before configuration, reset CRS registers to their default values*/
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_CRSRST;
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CRSRST;

  uint32_t crs_cfgr_reg = 0;

  /* CRS CFGR */
  // Set CRS sync polarity
  crs_cfgr_reg |= (cfg->sync_polarity << CRS_CFGR_SYNCPOL_Pos);
  // Set CRS sync source
  crs_cfgr_reg |= (cfg->sync_source << CRS_CFGR_SYNCSRC_Pos);
  // Set CRS sync divider
  crs_cfgr_reg |= (cfg->sync_scale << CRS_CFGR_SYNCDIV_Pos);
  // Set CRS frequency error limit
  crs_cfgr_reg |= (cfg->error_limit_value << CRS_CFGR_FELIM_Pos);
  // Set CRS reload value
  crs_cfgr_reg |= (cfg->reload_value << CRS_CFGR_RELOAD_Pos);

  CRS->CFGR = crs_cfgr_reg;

  /* CRS CR */
  // Set CRS trim value
  Modify_register_field(CRS->CR, CRS_CR_TRIM, cfg->hsi48_calibration_value);
  // Enable CRS auto trimming and frequency error counter
  Set_register_bit(CRS->CR, CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
}
