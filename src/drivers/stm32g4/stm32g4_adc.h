#ifndef STM32G4_ADC_H
#define STM32G4_ADC_H

#include "stm32g4_common.h"

// #define ADC_CLK_DOMAIN_SYSCLK_PLL 0

// ------------------------------------------------------+
// ADC Sample Hold Time
// Tconv = Sample Time + 12.5 ADC clock cycles
// ------------------------------------------------------+

typedef struct adc_input_s {
  char *name;
  uint8_t channel;
  float scale;
  float offset;
  char *units;
  __IO uint32_t *data;
} adc_input_t;

typedef struct adc_s {
  adc_input_t regular_inputs[16];
  adc_input_t injected_inputs[4];
  ADC_TypeDef *regs;

  struct adc_options_s {
    enum adc_clk_domains_e {
      ADC_CLK_DOMAIN_SYSCLK_PLL,
      ADC_CLK_DOMAIN_HCLK,
    } clk_domain;
    enum adc_sample_mode_e {
      ADC_SAMPLE_MODE_SINGLE,
      ADC_SAMPLE_MODE_CONTINUOUS,
    } sample_mode;
    enum adc_resolution_e {
      ADC_RESOLUTION_12_BIT,
      ADC_RESOLUTION_10_BIT,
      ADC_RESOLUTION_8_BIT,
      ADC_RESOLUTION_6_BIT,
    } resolution;
  } options;
} adc_t;

int adc_init(adc_t *self);
int adc_add_regular_input(adc_t *self, adc_input_t *input,
                          uint16_t sequence_order, uint16_t sample_time);
int adc_add_injected_input(adc_t *self, adc_input_t *input,
                           uint16_t sample_time);
int adc_enable_injected_input_soc_trigger(adc_t *self);

int adc_start_regular_sampling(adc_t *self);
int adc_start_injected_sampling(adc_t *self);
int adc_stop_regular_sampling(adc_t *self);
int adc_stop_injected_sampling(adc_t *self);

// int adc_add_input(struct adc *adc, struct adc_input *channel);
// int adc_remove_input(struct adc *adc, struct adc_input *channel);
// int adc_set_trigger(struct adc *adc, struct adc_input *channel, uint32_t
// trigger); int adc_start(struct adc *adc); int adc_stop(struct adc *adc); int
// adc_deinit(struct adc *adc);

#endif
