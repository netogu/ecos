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

#include "drivers/stm32g4/adc.h"
#include <string.h> // for memset
#include "log.h"

#define NULL 0

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

    adc_regs->CR |= ADC_CR_ADCALDIF;
    adc_regs->CR |= ADC_CR_ADCAL;
    while (adc_regs->CR & ADC_CR_ADCAL)
        ; // Wait for the calibration to complete

    return 0;
}
int adc_init(adc_t *self) {

    ADC_TypeDef *adc_regs = self->options.instance;
    memset(self->regular_result, 0, sizeof(self->regular_result));

    // ADC Clk domain: SYSCLK or PLL
    // Enable the ADC Peripheral Clock

    if (adc_regs == ADC1 || ADC2) {

        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

        if (self->options.clk_domain == ADC_CLK_DOMAIN_SYSCLK_PLL) {
            ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
        } else {
            ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
            ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
        }

    } else
    if (adc_regs == ADC3 || ADC4 || ADC5) { 

        // ADC_INSTANCE_3, ADC_INSTANCE_4, ADC_INSTANCE_5
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;

        if (self->options.clk_domain == ADC_CLK_DOMAIN_SYSCLK_PLL) {
            ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;
        } else {
            ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;
            ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
        }
    }  else {
        return -1;
    }

    // Exit deep-power mode
    adc_regs->CR &= ~ADC_CR_DEEPPWD;

    // Enable the ADC voltage regulator
    adc_regs->CR |= ADC_CR_ADVREGEN;
    // Ensure ADEN = 0
    adc_regs->CR &= ~ADC_CR_ADEN;

// Wait for the ADC voltage regulator to be ready
// Regulator startup time = 20us (from datasheet)
    for (int i = 0; i < 4000; i++)
        __NOP();  

    // Calibrate the ADC
    adc_calibrate_blocking(self);

    // Enable the ADC
    adc_enable(self);

    return 0;
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

    // Configure Sample Time

    if (input->channel <= 9) {
        adc_regs->SMPR1 |= sample_time << (ADC_SMPR1_SMP0_Pos + (3 * input->channel));
    } else {
        adc_regs->SMPR2 |= sample_time << (ADC_SMPR2_SMP10_Pos + (3 * (input->channel - 10)));
    }

    input->data = &self->regular_result[sequence_order];

    self->num_regular_inputs++;
    adc_regs->SQR1 |= (self->num_regular_inputs - 1) << ADC_SQR1_L_Pos;


    return 0;
}