#include <stdint.h>
#include "stm32g4xx.h"

struct rcc_clock_config  {

    uint8_t pll_source;

    uint8_t pllm;
    uint8_t plln;
    uint8_t pllp;
    uint8_t pllq;
    uint8_t pllr;
    uint8_t hpre;
    uint8_t ppre1;
    uint8_t ppre2;

    uint8_t vos_scale;
    uint8_t boost;

    uint32_t flash_config; 
    uint8_t flash_waitstates; 

    uint32_t ahb_frequency; 
    uint32_t apb1_frequency; 
    uint32_t apb2_frequency;
};



// Clock Configurations
extern const struct rcc_clock_config rcc_clock_hsi_170mhz;


void rcc_osc_on(void);
void rcc_osc_off(void);
void rcc_set_sysclk_source(void);
void rcc_set_pll_source(void);
void rcc_set_pll_scale(void);
void rcc_setup_clock_pll(const struct rcc_clock_config *clk_config);

