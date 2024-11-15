#ifndef STM32G4_ADC_H
#define STM32G4_ADC_H

#include "stm32g4_common.h"

// #define ADC_CLK_DOMAIN_SYSCLK_PLL 0

// ------------------------------------------------------+
// ADC Sample Hold Time
// Tconv = Sample Time + 12.5 ADC clock cycles
// ------------------------------------------------------+

#define ADC_INSTANCE_1 1
#define ADC_INSTANCE_2 2
#define ADC_INSTANCE_3 3
#define ADC_INSTANCE_4 4
#define ADC_INSTANCE_5 5

#define _FLAG(x) (1 << (x))
#define ADC_CHANNEL_1 _FLAG(0)
#define ADC_CHANNEL_2 _FLAG(1)
#define ADC_CHANNEL_3 _FLAG(2)
#define ADC_CHANNEL_4 _FLAG(3)
#define ADC_CHANNEL_5 _FLAG(4)
#define ADC_CHANNEL_6 _FLAG(5)
#define ADC_CHANNEL_7 _FLAG(6)
#define ADC_CHANNEL_8 _FLAG(7)
#define ADC_CHANNEL_9 _FLAG(8)
#define ADC_CHANNEL_10 _FLAG(9)
#define ADC_CHANNEL_11 _FLAG(10)
#define ADC_CHANNEL_12 _FLAG(11)
#define ADC_CHANNEL_13 _FLAG(12)
#define ADC_CHANNEL_14 _FLAG(13)
#define ADC_CHANNEL_15 _FLAG(14)
#define ADC_CHANNEL_16 _FLAG(15)
#define ADC_CHANNEL_OPAMP_1 _FLAG(16)
#define ADC_CHANNEL_OPAMP_2 _FLAG(17)
#define ADC_CHANNEL_OPAMP_3 _FLAG(18)
#define ADC_CHANNEL_OPAMP_4 _FLAG(19)
#define ADC_CHANNEL_OPAMP_5 _FLAG(20)
#define ADC_CHANNEL_OPAMP_6 _FLAG(21)
#define ADC_CHANNEL_VTS _FLAG(22)
#define ADC_CHANNEL_VBAT_DIV_3 _FLAG(23)
#define ADC_CHANNEL_VREFINT _FLAG(24)

#define ADC_REG_INPUT_MAX 16

#define ADC_INJ_INPUT_MAX 4

#define ADC_SAMPLE_TIME_2_5_CYCLES 0
#define ADC_SAMPLE_TIME_6_5_CYCLES 1
#define ADC_SAMPLE_TIME_12_5_CYCLES 2
#define ADC_SAMPLE_TIME_24_5_CYCLES 3
#define ADC_SAMPLE_TIME_47_5_CYCLES 4
#define ADC_SAMPLE_TIME_92_5_CYCLES 5
#define ADC_SAMPLE_TIME_247_5_CYCLES 6
#define ADC_SAMPLE_TIME_640_5_CYCLES 7

#define ADC_INPUT_TYPE_REGULAR  0
#define ADC_INPUT_TYPE_INJECTED 1

#define ADC_MAX_CHANNELS 18

typedef struct adc_input_s {
    char *name;
    uint8_t channel;
    float scale;
    float offset;
    char *units;
    __IO uint32_t *data;
} adc_input_t;

typedef struct adc_s {
    struct {
     adc_input_t *input;
     uint16_t sample_time;
    } regular_input[ADC_REG_INPUT_MAX];

    struct {
     adc_input_t *input;
     uint16_t sample_time;
    } injected_input[ADC_INJ_INPUT_MAX];

    uint16_t num_regular_inputs;
    uint16_t num_injected_inputs;

    struct adc_options_s {
        ADC_TypeDef *instance;
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
int adc_enable(adc_t *self);
int adc_register_input(adc_t *self, adc_input_t *input, uint16_t input_type, uint16_t sample_time);
int adc_add_regular_input(adc_t *self, adc_input_t *input, uint16_t sequence_order, uint16_t sample_time);
int adc_add_injected_input(adc_t *self, adc_input_t *input, uint16_t sample_time);
int adc_enable_injected_input_soc_trigger(adc_t *self);

int adc_start_regular_sampling(adc_t *self);
int adc_start_injected_sampling(adc_t *self);
int adc_stop_regular_sampling(adc_t *self);
int adc_stop_injected_sampling(adc_t *self);

// int adc_add_input(struct adc *adc, struct adc_input *channel);
// int adc_remove_input(struct adc *adc, struct adc_input *channel);
// int adc_set_trigger(struct adc *adc, struct adc_input *channel, uint32_t trigger);
// int adc_start(struct adc *adc);
// int adc_stop(struct adc *adc);
// int adc_deinit(struct adc *adc);

#endif