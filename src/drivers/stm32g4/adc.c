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




int adc_start_sampling(adc_t *self) {
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->instance;
    // Start converting the regular group and injected group
    adc_regs->CR |= ADC_CR_ADSTART;
    adc_regs->CR |= ADC_CR_JADSTART;
    return 0;
}

int adc_stop_sampling(adc_t *self) {
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->instance;
    // Stop converting the regular group and injected group
    adc_regs->CR |= ADC_CR_ADSTP;
    adc_regs->CR |= ADC_CR_JADSTP;
    return 0;
}


int adc_enable(adc_t *self) {
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->instance;
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
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->instance;
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
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->instance;

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

    ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->instance;

    #if ADC_CLK_DOMAIN_SYSCLK_PLL
    // ADC Clk domain: SYSCLK or PLL
    // Enable the ADC Peripheral Clock
    if (adc->instance == ADC1_BASE || ADC2_BASE) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    } else if (adc->instance == ADC3_BASE || ADC4_BASE || ADC5_BASE) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
        ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    }
    #else
    // ADC Clk domain: HCLK
    if (self->instance == ADC1_BASE || ADC2_BASE) {
        ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0; // Set ADC CLK Domain to HCLK/1

    } else if (self->instance == ADC3_BASE || ADC4_BASE || ADC5_BASE) {
        ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;
        ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0; // Set ADC CLK Domain to HCLK/1
    }
    #endif



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

