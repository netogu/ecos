#pragma once

#include <stdint.h>
#include "drivers/stm32g4/gpio.h"


// #define ADC_CLK_DOMAIN_SYSCLK_PLL 0

// ------------------------------------------------------+
// ADC Sample Hold Time
// Tconv = Sample Time + 12.5 ADC clock cycles
// ------------------------------------------------------+

#define ADC_OP_CLK_DOMAN_SYSCLK_PLL 0
#define ADC_OP_CLK_DOMAN_SYSCLK 1
#define ADC_OP_RESOLUTION_6BIT 0
#define ADC_OP_RESOLUTION_8BIT 1
#define ADC_OP_RESOLUTION_10BIT 2
#define ADC_OP_RESOLUTION_12BIT 3

enum adc_sample_time_e {
    ADC_SAMPLE_TIME_2_5_CYCLES,
    ADC_SAMPLE_TIME_6_5_CYCLES,
    ADC_SAMPLE_TIME_12_5_CYCLES,
    ADC_SAMPLE_TIME_24_5_CYCLES,
    ADC_SAMPLE_TIME_47_5_CYCLES,
    ADC_SAMPLE_TIME_92_5_CYCLES,
    ADC_SAMPLE_TIME_247_5_CYCLES,
    ADC_SAMPLE_TIME_640_5_CYCLES,
};

enum adc_input_type_e {
    ADC_INPUT_TYPE_SINGLE_ENDED,
    ADC_INPUT_TYPE_DIFFERENTIAL,
};

enum adc_convertion_mode_e {
    ADC_CONV_MODE_REGULAR,
    ADC_CONV_MODE_INJECTED,
};

typedef struct adc_input_s {
    char *name;
    gpio_t pin;
    uint8_t channel;
    float scale;
    float offset;
    char *units;
    uint32_t value_raw;
    float value;
} adc_input_t;

typedef struct adc_s {
    uint32_t *instance;
    struct {
        uint16_t size;
        uint16_t *array;
    } inputs;
} adc_t;

struct adc_options_s {
    enum {
        ADC_CLK_DOMAIN_SYSCLK_PLL,
    } clk_domain;

    enum {
        ADC_RESOLUTION_6BIT = 0,
        ADC_RESOLUTION_8BIT = 1,
        ADC_RESOLUTION_10BIT = 2,
        ADC_RESOLUTION_12BIT = 3,
    } resolution;

    enum {
        ADC_DATA_ALIGN_RIGHT,
        ADC_DATA_ALIGN_LEFT,
    } data_alignment;

    enum {
        ADC_SCAN_MODE_SINGLE,
        ADC_SCAN_MODE_CONTINUOUS,
    } scan_mode;

    enum {
        ADC_EOC_SEQ_CONV,
        ADC_EOC_SINGLE_CONV,
    } eoc_selection;
};

    





int adc_init(adc_t *self);
// int adc_add_input(struct adc *adc, struct adc_input *channel);
// int adc_remove_input(struct adc *adc, struct adc_input *channel);
// int adc_set_trigger(struct adc *adc, struct adc_input *channel, uint32_t trigger);
// int adc_start(struct adc *adc);
// int adc_stop(struct adc *adc);
// int adc_deinit(struct adc *adc);


