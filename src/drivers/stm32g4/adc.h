#pragma once

#include <stdint.h>
#include <stm32g4xx.h>
#include "drivers/stm32g4/gpio.h"


struct adc_input {
    uint8_t *name;
    uint8_t status;
    uint8_t channel;
    uint8_t sample_time;
    uint32_t *sample_data;
    uint32_t sample_size;
};


struct adc {
    uint32_t instance;
    uint8_t channel_count;
};


int adc_init(struct adc *adc);
int adc_add_input(struct adc *adc, struct adc_input *channel);
int adc_remove_input(struct adc *adc, struct adc_input *channel);
int adc_set_trigger(struct adc *adc, struct adc_input *channel, uint32_t trigger);
int adc_start(struct adc *adc);
int adc_stop(struct adc *adc);
int adc_deinit(struct adc *adc);


int adc_add_input(struct adc *adc, struct adc_input *channel) {
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)adc->instance;


}

int adc_enable(struct adc *adc) {
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)adc->instance;
    // Enable the ADC
    // Clear ADRDY
    adc_regs->ISR |= ADC_ISR_ADRDY;
    adc_regs->CR |= ADC_CR_ADEN;

    // Wait for the ADC to be ready
    while (!(adc_regs->ISR & ADC_ISR_ADRDY));

    return 0;
}

// int adc_disable(struct adc_channel *adc)
// {
//     ADC_TypeDef *adc_regs = (ADC_TypeDef *)adc->base;
//     // Checkthat ADSTART = 0 and JDSTART = 0
//     adc_regs->CR |= ADC_CR_ADSTP;
//     adc_regs->CR |= ADC_CR_JADSTP;
//     while (adc_regs->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART));
//     // Disable the ADC
//     adc_regs->CR |= ADC_CR_ADDIS;
//     // Wait for the ADC to be disabled
//     while (adc_regs->CR & ADC_CR_ADEN);

//     return 0;
// }

int adc_init(struct adc *adc) {

    ADC_TypeDef *adc_regs = (ADC_TypeDef *)adc->instance;

    // Enable the ADC Peripheral Clock
    if (adc->instance == ADC1_BASE || ADC2_BASE) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    } else if (adc->instance == ADC3_BASE || ADC4_BASE || ADC5_BASE) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
        ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    }


    // Exit deep-power mode
    adc_regs->CR &= ~ADC_CR_DEEPPWD;

    // Enable the ADC voltage regulator
    adc_regs->CR |= ADC_CR_ADVREGEN;
    // Ensure ADEN = 0
    adc_regs->CR &= ~ADC_CR_ADEN;

    // Wait for the ADC voltage regulator to be ready
    for (int i = 0; i < 4000; i++)
        __NOP();  

    // Calibrate the ADC for single ended mode
    adc_regs->CR |= ADC_CR_ADCAL;
    while (adc_regs->CR & ADC_CR_ADCAL)
        ; // Wait for the calibration to complete

    adc_regs->CR |= ADC_CR_ADCALDIF;
    adc_regs->CR |= ADC_CR_ADCAL;
    while (adc_regs->CR & ADC_CR_ADCAL)
        ; // Wait for the calibration to complete

    return 0;
}
