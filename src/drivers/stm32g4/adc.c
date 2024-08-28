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

#define NULL 0

static ADC_TypeDef *adc_get_base_address(adc_t *adc) {
    switch (adc->instance) {
        case ADC_INSTANCE_1:
            return ADC1;
        case ADC_INSTANCE_2:
            return ADC2;
        case ADC_INSTANCE_3:
            return ADC3;
        case ADC_INSTANCE_4:
            return ADC4;
        case ADC_INSTANCE_5:
            return ADC5;
        default:
            return NULL;
    }
}


int adc_start_sampling(adc_t *self) {
    ADC_TypeDef *adc_regs = adc_get_base_address(self);
    if (adc_regs == NULL) {
        return -1;
    }

    // Start converting the regular group and injected group
    adc_regs->CR |= ADC_CR_ADSTART;
    adc_regs->CR |= ADC_CR_JADSTART;
    return 0;
}

int adc_stop_sampling(adc_t *self) {
    ADC_TypeDef *adc_regs = adc_get_base_address(self);
    if (adc_regs == NULL) {
        return -1;
    }
    // Stop converting the regular group and injected group
    adc_regs->CR |= ADC_CR_ADSTP;
    adc_regs->CR |= ADC_CR_JADSTP;
    return 0;
}


int adc_enable(adc_t *self) {
    ADC_TypeDef *adc_regs = adc_get_base_address(self);
    if (adc_regs == NULL) {
        return -1;
    }
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

int adc_disable(adc_t *self)
{
    ADC_TypeDef *adc_regs = adc_get_base_address(self);
    if (adc_regs == NULL) {
        return -1;
    }
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

int adc_calibrate(adc_t *self) {
    ADC_TypeDef *adc_regs = adc_get_base_address(self);
    if (adc_regs == NULL) {
        return -1;
    }

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

    ADC_TypeDef *adc_regs = adc_get_base_address(self);
    if (adc_regs == NULL) {
        return -1;
    }

    // ADC Clk domain: SYSCLK or PLL
    // Enable the ADC Peripheral Clock

    if (self->instance == ADC_INSTANCE_1 || ADC_INSTANCE_2) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    } else { 
        // ADC_INSTANCE_3, ADC_INSTANCE_4, ADC_INSTANCE_5
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
        ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    } 

    // ADC Clk domain: HCLK
    if (self->instance == ADC_INSTANCE_1 || ADC_INSTANCE_2) {
        ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0; // Set ADC CLK Domain to HCLK/1

    } else {
        // ADC_INSTANCE_3, ADC_INSTANCE_4, ADC_INSTANCE_5
        ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;
        ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0; // Set ADC CLK Domain to HCLK/1
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
    adc_calibrate(self);

    // Enable the ADC
    adc_enable(self);

    return 0;
}

adc_init_input(adc_t *self, adc_input_t *input) {
    ADC_TypeDef *adc_regs = adc_get_base_address(self);
    if (adc_regs == NULL) {
        return -1;
    }

    // Configure the regular group
    adc_regs->SQR1 = 0;
    adc_regs->SQR2 = 0;
    adc_regs->SQR3 = 0;

    // for (int i = 0; i < self->regular_inputs.size; i++) {
    //     adc_regs->SQR1 |= (self->regular_inputs.array[i].channel << (5 * i));
    // }

    // Configure the injected group
    adc_regs->JSQR = 0;

    // for (int i = 0; i < self->injected_inputs.size; i++) {
    //     adc_regs->JSQR |= (self->injected_inputs.array[i].channel << (5 * i));
    // }

    return 0;
}
