#pragma once

#include <stdint.h>
#include "drivers/stm32g4/gpio.h"


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

#define ADC_INPUT_TYPE_REGULAR 0
#define ADC_INPUT_TYPE_INJECTED 1

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

#define ADC_MAX_CHANNELS 18

typedef struct adc_input_s {
    char *name;
    uint8_t adc_instance;
    uint8_t channel;
    gpio_t pin;
    float scale;
    float offset;
    char *units;
    uint16_t *data;
} adc_input_t;
typedef struct adc_s {
    ADC_TypeDef *regs;
    uint32_t active_channels_flg;
    uint32_t status;
    uint16_t channel_data[ADC_MAX_CHANNELS];
    uint16_t channel_count;
} adc_t;


int adc_init(adc_t *self, ADC_TypeDef *instance);
int adc_configure_channel(adc_t *self, adc_input_t *channel);
// int adc_add_input(struct adc *adc, struct adc_input *channel);
// int adc_remove_input(struct adc *adc, struct adc_input *channel);
// int adc_set_trigger(struct adc *adc, struct adc_input *channel, uint32_t trigger);
// int adc_start(struct adc *adc);
// int adc_stop(struct adc *adc);
// int adc_deinit(struct adc *adc);


