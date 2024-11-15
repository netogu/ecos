/******************************************************************************
 * File: adc.c
 * Description: Implementation of Analog-to-Digital Converter (ADC) functions
 *              for the embedded system. 
 *
 * Author: Ernesto Gonzalez Urdaneta
 * Date Created: August 19, 2024
 * Last Modified: August 19, 2024
 * Version: 0.1
 *
 * Revision History:
 * -----------------------------------------------------------------------------
 * Date          Version    Author              Description
 * -----------------------------------------------------------------------------
 * 2024-08-19    0.1        EGU                 Initial creation
 *
 * -----------------------------------------------------------------------------
 * Notes:
 * - This code is designed for use with the STM32G4 series of microcontrollers.
 * - Ensure the ADC peripheral is configured correctly before calling these
 *   functions.
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "stm32g4_adc.h"
#include <string.h> // for memset
#include <stdio.h>

#define ADC_STATUS_UNINITIALIZED 0
#define ADC_STATUS_INITIALIZED 1
#define ADC_STATUS_SAMPLING 2
#define ADC_STATUS_CALIBRATED 3
#define ADC_STATUS_ENABLED 4
#define ADC_STATUS_DISABLED 5


int adc_start_regular_sampling(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;

    // Start converting the regular group
    adc_regs->CR |= ADC_CR_ADSTART;

    return 0;
}

int adc_start_injected_sampling(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;

    // Start converting the injected group
    adc_regs->CR |= ADC_CR_JADSTART;

    return 0;
}

int adc_stop_regular_sampling(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;

    // Stop converting the regular group
    adc_regs->CR |= ADC_CR_ADSTP;

    return 0;
}

int adc_stop_injected_sampling(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;

    // Stop converting the injected group
    adc_regs->CR |= ADC_CR_JADSTP;

    return 0;
}


int adc_enable(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;
    // Enable the ADC
    // Clear ADRDY
    adc_regs->ISR |= ADC_ISR_ADRDY;
    adc_regs->CR |= ADC_CR_ADEN;

    // Wait for the ADC to be ready
    while (!(adc_regs->ISR & ADC_ISR_ADRDY));

    // Clear ADRDY
    adc_regs->ISR |= ADC_ISR_ADRDY;

    // Done
    return 0;
}

int adc_disable(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;

    // Stop on-going conversions
    adc_regs->CR |= ADC_CR_ADSTP;
    adc_regs->CR |= ADC_CR_JADSTP;
    while (adc_regs->CR & (ADC_CR_ADSTP | ADC_CR_JADSTP));
    // Disable the ADC
    adc_regs->CR |= ADC_CR_ADDIS;
    // Wait for the ADC to be disabled
    while (adc_regs->CR & ADC_CR_ADEN);

    return 0;
}

int adc_calibrate_blocking(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;

    // Calibrate the ADC for single ended mode
    adc_regs->CR |= ADC_CR_ADCAL;
    while (adc_regs->CR & ADC_CR_ADCAL)
        ; // Wait for the calibration to complete
    for (volatile uint32_t delay = 0; delay < 100; delay++);

    adc_regs->CR |= ADC_CR_ADCALDIF;
    adc_regs->CR |= ADC_CR_ADCAL;
    while (adc_regs->CR & ADC_CR_ADCAL)
        ; // Wait for the calibration to complete
    for (volatile uint32_t delay = 0; delay < 100; delay++);

    return 0;
}

int adc_register_input(adc_t *self, adc_input_t *input, uint16_t input_type, uint16_t sample_time) {

    if (input_type == ADC_INPUT_TYPE_REGULAR) {

        if ((self->num_regular_inputs + 1) > ADC_REG_INPUT_MAX) {
            goto error;
        }
        self->regular_input[self->num_regular_inputs].input = input;
        self->regular_input[self->num_regular_inputs].sample_time = sample_time;
        self->num_regular_inputs++;

    } else

    if (input_type == ADC_INPUT_TYPE_INJECTED) {

        if ((self->num_injected_inputs +1) > ADC_INJ_INPUT_MAX) {
            goto error;
        }
        self->injected_input[self->num_injected_inputs].input = input;
        self->injected_input[self->num_injected_inputs].sample_time = sample_time;
        self->num_injected_inputs++;
        
    } else {
        goto error;
    }

    return 0;
    error:
    return -1;

}

static int adc_config_input_sample_time(adc_t *self) {
    // Configure Sample Time
    ADC_TypeDef *adc_regs = self->options.instance;

    uint32_t smpr1_reg = 0;
    uint32_t smpr2_reg = 0;

    if (self->num_regular_inputs > 0) {

        for (uint32_t i = 0; i < self->num_regular_inputs; i++) {
            uint8_t channel = self->regular_input[i].input->channel;
            uint16_t sample_time = self->regular_input[i].sample_time;
            if (channel <= 9) {
                smpr1_reg |= sample_time << (ADC_SMPR1_SMP0_Pos + (3 * channel));
            } else {
                smpr2_reg |= sample_time << (ADC_SMPR2_SMP10_Pos + (3 * (channel - 10)));
            }
        }
    }

    if (self->num_injected_inputs > 0) {

        for (uint32_t i = 0; i < self->num_injected_inputs; i++) {
            uint8_t channel = self->injected_input[i].input->channel;
            uint16_t sample_time = self->injected_input[i].sample_time;
            if (channel <= 9) {
                smpr1_reg |= sample_time << (ADC_SMPR1_SMP0_Pos + (3 * channel));
            } else {
                smpr2_reg |= sample_time << (ADC_SMPR2_SMP10_Pos + (3 * (channel - 10)));
            }
        }

    }

    adc_regs->SMPR1 = smpr1_reg;
    adc_regs->SMPR2 = smpr2_reg;

    return 0;
}


int adc_init(adc_t *self) {

    static bool adc12_clock_initialized = false;
    static bool adc345_clock_initialized = false;
    int status;

    ADC_TypeDef *adc_regs = self->options.instance;

    // ADC Clk domain: SYSCLK or PLL
    // Enable the ADC Peripheral Clock

    if (adc_regs == ADC1 || adc_regs == ADC2) {

        if (!adc12_clock_initialized){
            RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
            RCC->CCIPR &= ~(RCC_CCIPR_ADC12SEL);
            ADC12_COMMON->CCR &= ~(ADC_CCR_CKMODE);


            if (self->options.clk_domain == ADC_CLK_DOMAIN_SYSCLK_PLL) {
                // RCC->CCIPR |= RCC_CCIPR_ADC12SEL_0;
                
            } else {
                // RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;
                ADC12_COMMON->CCR |= (ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1); // Set ADC CLK Domain to HCLK/4
            }

            adc12_clock_initialized = true;

        }

    } else
    if (adc_regs == ADC3 || adc_regs == ADC4 || adc_regs == ADC5) { 

        if (!adc345_clock_initialized){
            RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
            RCC->CCIPR &= ~(RCC_CCIPR_ADC345SEL);
            ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;

            if (self->options.clk_domain == ADC_CLK_DOMAIN_SYSCLK_PLL) {
                // RCC->CCIPR |= RCC_CCIPR_ADC345SEL_0;
            } else {
                // RCC->CCIPR |= RCC_CCIPR_ADC345SEL_1;
                ADC345_COMMON->CCR |= (ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1); // Set ADC CLK Domain to HCLK/4
            }
            adc345_clock_initialized = true;
        }
    }  else {
        return -1;
    }


    // Exit deep-power mode
    adc_regs->CR &= ~ADC_CR_DEEPPWD;

    // Enable the ADC voltage regulator
    adc_regs->CR |= ADC_CR_ADVREGEN;
// Wait for the ADC voltage regulator to be ready
// Regulator startup time = 20us (from datasheet)
    for (volatile uint32_t delay = 0; delay < 15000; delay++);
    // Ensure ADEN = 0
    adc_regs->CR &= ~ADC_CR_ADEN;


    // Calibrate the ADC
    if(adc_calibrate_blocking(self) != 0) {
        return -1;
    }

    // Configure Inputs
    if(adc_config_input_sample_time(self) != 0) {
        return -1;
    }

    // Configure Regular Input Sequence
    // Configure Injected Input Sequence
    // Configure Triggers
    // Enable HW Triggers
    adc_regs->CFGR &= ~(ADC_CFGR_JQDIS);



    // Enable the ADC
    adc_enable(self);

    return 0;
}

int adc_enable_injected_input_soc_trigger(adc_t *self) {
    ADC_TypeDef *adc_regs = self->options.instance;
    uint32_t reg_val = adc_regs->JSQR;

    const uint32_t hrtim_adc_trig1 = 27;
    adc_regs->CFGR &= ~(ADC_CFGR_JQDIS);

    reg_val &= ~(ADC_JSQR_JEXTSEL) | ~(ADC_JSQR_JEXTEN); 
    reg_val |= (hrtim_adc_trig1 << ADC_JSQR_JEXTSEL_Pos);
    reg_val |= (2 << ADC_JSQR_JEXTEN_Pos);

    adc_regs->JSQR = reg_val;

    return 0;
}

}

int adc_add_regular_input(adc_t *self, adc_input_t *input, uint16_t sequence_order, uint16_t sample_time) {

    ADC_TypeDef *adc_regs = self->options.instance;

    // Configure Sequence Order

    if (self->num_regular_inputs >= ADC_REG_SEQ_ORDER_MAX) {
        return -1;
    }

    if (sequence_order <= 4) {
        adc_regs->SQR1 = input->channel << (ADC_SQR1_SQ1_Pos + (sequence_order - 1) * 6);
    } else if (sequence_order <= 9) {
        adc_regs->SQR2 = input->channel << (ADC_SQR2_SQ5_Pos + (sequence_order - 5) * 6);
    } else if (sequence_order <= 14) {
        adc_regs->SQR3 = input->channel << (ADC_SQR3_SQ10_Pos + (sequence_order - 10) * 6);
    } else if (sequence_order <= 16) {
        adc_regs->SQR4 = input->channel << (ADC_SQR4_SQ15_Pos + (sequence_order - 15) * 6);
    } else {
        return -1;
    }

    adc_config_input_sample_time(self, input, sample_time);


    input->data = &self->regular_result[sequence_order];

    self->num_regular_inputs++;
    adc_regs->SQR1 |= (self->num_regular_inputs - 1) << ADC_SQR1_L_Pos;


    return 0;
}

int adc_add_injected_input(adc_t *self, adc_input_t *input, uint16_t sample_time) {
    ADC_TypeDef *adc_regs = self->options.instance;
    uint32_t reg_val = adc_regs->JSQR;

    switch (self->num_injected_inputs) {
        case 0:
            reg_val |= input->channel << (ADC_JSQR_JSQ1_Pos);
            break;
        case 1:
            reg_val = input->channel << (ADC_JSQR_JSQ2_Pos);
            break;
        case 2:
            reg_val = input->channel << (ADC_JSQR_JSQ3_Pos);
            break;
        case 3:
            reg_val = input->channel << (ADC_JSQR_JSQ4_Pos);
            break;
        default:
            goto error;
            break;
    }
    
    adc_config_input_sample_time(self, input, sample_time);

    self->num_injected_inputs++;
    reg_val &= ~(ADC_JSQR_JL);
    reg_val |= (self->num_injected_inputs - 1) << ADC_JSQR_JL_Pos;
    adc_regs->JSQR = reg_val;

    return 0;
    error:
    return -1;
}
