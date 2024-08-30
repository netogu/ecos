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

#define ADC_STATUS_UNINITIALIZED 0
#define ADC_STATUS_INITIALIZED 1
#define ADC_STATUS_SAMPLING 2
#define ADC_STATUS_CALIBRATED 3
#define ADC_STATUS_ENABLED 4
#define ADC_STATUS_DISABLED 5


int adc_start_sampling(adc_t *self) {
    // Start converting the regular group and injected group
    self->regs->CR |= ADC_CR_ADSTART;
    self->regs->CR |= ADC_CR_JADSTART;

    return 0;
}


int adc_stop_sampling(adc_t *self) {
    // Stop converting the regular group and injected group
    self->regs->CR |= ADC_CR_ADSTP;
    self->regs->CR |= ADC_CR_JADSTP;
    return 0;
}


int adc_enable(adc_t *self) {
    // Enable the ADC
    // Clear ADRDY
    self->regs->ISR |= ADC_ISR_ADRDY;
    self->regs->CR |= ADC_CR_ADEN;

    // Wait for the ADC to be ready
    while (!(self->regs->ISR & ADC_ISR_ADRDY));

    // Clear ADRDY
    self->regs->ISR |= ADC_ISR_ADRDY;

    // Done
    return 0;
}

int adc_disable(adc_t *self)
{
    // Stop on-going conversions
    self->regs->CR |= ADC_CR_ADSTP;
    self->regs->CR |= ADC_CR_JADSTP;
    while (self->regs->CR & (ADC_CR_ADSTP | ADC_CR_JADSTP));
    // Disable the ADC
    self->regs->CR |= ADC_CR_ADDIS;
    // Wait for the ADC to be disabled
    while (self->regs->CR & ADC_CR_ADEN);

    return 0;
}

int adc_calibrate(adc_t *self) {

    // Calibrate the ADC for single ended mode
    self->regs->CR |= ADC_CR_ADCAL;
    while (self->regs->CR & ADC_CR_ADCAL)
        ; // Wait for the calibration to complete

    self->regs->CR |= ADC_CR_ADCALDIF;
    self->regs->CR |= ADC_CR_ADCAL;
    while (self->regs->CR & ADC_CR_ADCAL)
        ; // Wait for the calibration to complete

    return 0;
}
int adc_init(adc_t *self, ADC_TypeDef *adc_base) {

    self->regs = adc_base;


    // ADC Clk domain: SYSCLK or PLL
    // Enable the ADC Peripheral Clock

    if (self->regs == ADC1 || ADC2) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    } else { 
        // ADC_INSTANCE_3, ADC_INSTANCE_4, ADC_INSTANCE_5
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
        ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1; // Set ADC CLK Domain to HCLK/4
    } 

    // ADC Clk domain: HCLK
    if (self->regs == ADC1 || ADC2) {
        ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0; // Set ADC CLK Domain to HCLK/1

    } else {
        // ADC_INSTANCE_3, ADC_INSTANCE_4, ADC_INSTANCE_5
        ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;
        ADC345_COMMON->CCR |= ADC_CCR_CKMODE_0; // Set ADC CLK Domain to HCLK/1
    }

    // Exit deep-power mode
    self->regs->CR &= ~ADC_CR_DEEPPWD;

    // Enable the ADC voltage regulator
    self->regs->CR |= ADC_CR_ADVREGEN;
    // Ensure ADEN = 0
    self->regs->CR &= ~ADC_CR_ADEN;

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

    // Configure the regular group
    self->regs->SQR1 = 0;
    self->regs->SQR2 = 0;
    self->regs->SQR3 = 0;

    // for (int i = 0; i < self->regular_inputs.size; i++) {
    //     self->regs->SQR1 |= (self->regular_inputs.array[i].channel << (5 * i));
    // }

    // Configure the injected group
    self->regs->JSQR = 0;

    // for (int i = 0; i < self->injected_inputs.size; i++) {
    //     self->regs->JSQR |= (self->injected_inputs.array[i].channel << (5 * i));
    // }

    return 0;
}
