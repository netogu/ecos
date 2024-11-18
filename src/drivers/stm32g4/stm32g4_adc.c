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
#include <stdio.h>
#include <string.h> // for memset

#define ADC_STATUS_UNINITIALIZED 0
#define ADC_STATUS_INITIALIZED 1
#define ADC_STATUS_SAMPLING 2
#define ADC_STATUS_CALIBRATED 3
#define ADC_STATUS_ENABLED 4
#define ADC_STATUS_DISABLED 5

int adc_start_regular_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = self->regs;

  // Start converting the regular group
  adc_regs->CR |= ADC_CR_ADSTART;

  return 0;
}

int adc_start_injected_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = self->regs;

  // Start converting the injected group
  adc_regs->CR |= ADC_CR_JADSTART;

  return 0;
}

int adc_stop_regular_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = self->regs;

  // Stop converting the regular group
  adc_regs->CR |= ADC_CR_ADSTP;

  return 0;
}

int adc_stop_injected_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = self->regs;

  // Stop converting the injected group
  adc_regs->CR |= ADC_CR_JADSTP;

  return 0;
}

int adc_enable(adc_t *self) {

  ADC_TypeDef *adc_regs = self->regs;
  // Enable the ADC
  // Clear ADRDY
  adc_regs->ISR |= ADC_ISR_ADRDY;
  adc_regs->CR |= ADC_CR_ADEN;

  // Wait for the ADC to be ready
  while (!(adc_regs->ISR & ADC_ISR_ADRDY))
    ;

  // Clear ADRDY
  adc_regs->ISR |= ADC_ISR_ADRDY;

  // Done
  return 0;
}

int adc_disable(adc_t *self) {

  ADC_TypeDef *adc_regs = self->regs;

  // Stop on-going conversions
  adc_regs->CR |= ADC_CR_ADSTP;
  adc_regs->CR |= ADC_CR_JADSTP;
  while (adc_regs->CR & (ADC_CR_ADSTP | ADC_CR_JADSTP))
    ;
  // Disable the ADC
  adc_regs->CR |= ADC_CR_ADDIS;
  // Wait for the ADC to be disabled
  while (adc_regs->CR & ADC_CR_ADEN)
    ;

  return 0;
}

int adc_calibrate_blocking(adc_t *self) {

  ADC_TypeDef *adc_regs = self->regs;

  // Calibrate the ADC for single ended mode
  adc_regs->CR |= ADC_CR_ADCAL;
  while (adc_regs->CR & ADC_CR_ADCAL)
    ; // Wait for the calibration to complete
  for (volatile uint32_t delay = 0; delay < 100; delay++)
    ;

  adc_regs->CR |= ADC_CR_ADCALDIF;
  adc_regs->CR |= ADC_CR_ADCAL;
  while (adc_regs->CR & ADC_CR_ADCAL)
    ; // Wait for the calibration to complete
  for (volatile uint32_t delay = 0; delay < 100; delay++)
    ;

  return 0;
}

int adc_init(adc_t *self) {

  static bool adc12_clock_initialized = false;
  static bool adc345_clock_initialized = false;

  ADC_TypeDef *adc_regs = self->regs;

  // ADC Clk domain: SYSCLK or PLL
  // Enable the ADC Peripheral Clock

  if (adc_regs == ADC1 || adc_regs == ADC2) {

    if (!adc12_clock_initialized) {
      RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
      RCC->CCIPR &= ~(RCC_CCIPR_ADC12SEL);
      ADC12_COMMON->CCR &= ~(ADC_CCR_CKMODE);

      if (self->options.clk_domain == ADC_CLK_DOMAIN_SYSCLK_PLL) {
        // RCC->CCIPR |= RCC_CCIPR_ADC12SEL_0;

      } else {
        // RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;
        ADC12_COMMON->CCR |= (ADC_CCR_CKMODE_0 |
                              ADC_CCR_CKMODE_1); // Set ADC CLK Domain to HCLK/4
      }

      adc12_clock_initialized = true;
    }

  } else if (adc_regs == ADC3 || adc_regs == ADC4 || adc_regs == ADC5) {

    if (!adc345_clock_initialized) {
      RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
      RCC->CCIPR &= ~(RCC_CCIPR_ADC345SEL);
      ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;

      if (self->options.clk_domain == ADC_CLK_DOMAIN_SYSCLK_PLL) {
        // RCC->CCIPR |= RCC_CCIPR_ADC345SEL_0;
      } else {
        // RCC->CCIPR |= RCC_CCIPR_ADC345SEL_1;
        ADC345_COMMON->CCR |=
            (ADC_CCR_CKMODE_0 |
             ADC_CCR_CKMODE_1); // Set ADC CLK Domain to HCLK/4
      }
      adc345_clock_initialized = true;
    }
  } else {
    return -1;
  }

  // Exit deep-power mode
  adc_regs->CR &= ~ADC_CR_DEEPPWD;

  // Enable the ADC voltage regulator
  adc_regs->CR |= ADC_CR_ADVREGEN;
  // Wait for the ADC voltage regulator to be ready
  // Regulator startup time = 20us (from datasheet)
  for (volatile uint32_t delay = 0; delay < 15000; delay++)
    ;
  // Ensure ADEN = 0
  adc_regs->CR &= ~ADC_CR_ADEN;

  // Calibrate the ADC
  adc_calibrate_blocking(self);

  // Enable the ADC
  adc_enable(self);

  return 0;
}

int adc_enable_injected_input_soc_trigger(adc_t *self) {
  ADC_TypeDef *adc_regs = self->regs;
  uint32_t reg_val = adc_regs->JSQR;

  const uint32_t hrtim_adc_trig1 = 27;

  reg_val &= ~(ADC_JSQR_JEXTSEL);
  reg_val |= (hrtim_adc_trig1 << ADC_JSQR_JEXTSEL_Pos);
  reg_val |= ADC_JSQR_JEXTEN;

  adc_regs->JSQR = reg_val;

  return 0;
}

static int adc_config_input_sample_time(adc_t *self, adc_input_t *input,
                                        uint16_t sample_time) {
  // Configure Sample Time
  ADC_TypeDef *adc_regs = self->regs;

  if (input->channel <= 9) {
    adc_regs->SMPR1 |= sample_time
                       << (ADC_SMPR1_SMP0_Pos + (3 * input->channel));
  } else {
    adc_regs->SMPR2 |= sample_time
                       << (ADC_SMPR2_SMP10_Pos + (3 * (input->channel - 10)));
  }

  return 0;
}

int adc_add_regular_input(adc_t *self, adc_input_t *input,
                          uint16_t sequence_order, uint16_t sample_time) {

  ADC_TypeDef *adc_regs = self->regs;

  // Configure Sequence Order

  // if (self->num_regular_inputs >= ADC_REG_SEQ_ORDER_MAX) {
  //   return -1;
  // }

  if (sequence_order <= 4) {
    adc_regs->SQR1 = input->channel
                     << (ADC_SQR1_SQ1_Pos + (sequence_order - 1) * 6);
  } else if (sequence_order <= 9) {
    adc_regs->SQR2 = input->channel
                     << (ADC_SQR2_SQ5_Pos + (sequence_order - 5) * 6);
  } else if (sequence_order <= 14) {
    adc_regs->SQR3 = input->channel
                     << (ADC_SQR3_SQ10_Pos + (sequence_order - 10) * 6);
  } else if (sequence_order <= 16) {
    adc_regs->SQR4 = input->channel
                     << (ADC_SQR4_SQ15_Pos + (sequence_order - 15) * 6);
  } else {
    return -1;
  }

  adc_config_input_sample_time(self, input, sample_time);

  // input->data = &self->regular_result[sequence_order];

  // self->num_regular_inputs++;
  // adc_regs->SQR1 |= (self->num_regular_inputs - 1) << ADC_SQR1_L_Pos;

  return 0;
}

int adc_add_injected_input(adc_t *self, adc_input_t *input,
                           uint16_t sample_time) {

  ADC_TypeDef *adc_regs = self->regs;
  uint32_t reg_val = adc_regs->JSQR;

  // switch (self->num_injected_inputs) {
  // case 0:
  //   reg_val |= input->channel << (ADC_JSQR_JSQ1_Pos);
  //   break;
  // case 1:
  //   reg_val = input->channel << (ADC_JSQR_JSQ2_Pos);
  //   break;
  // case 2:
  //   reg_val = input->channel << (ADC_JSQR_JSQ3_Pos);
  //   break;
  // case 3:
  //   reg_val = input->channel << (ADC_JSQR_JSQ4_Pos);
  //   break;
  // default:
  //   goto error;
  //   break;
  // }

  adc_config_input_sample_time(self, input, sample_time);

  // input->data = &self->injected_result[self->num_injected_inputs];

  // self->num_injected_inputs++;
  reg_val &= ~(ADC_JSQR_JL);
  // reg_val |= (self->num_injected_inputs - 1) << ADC_JSQR_JL_Pos;
  adc_regs->JSQR = reg_val;

  return 0;
  // error:
  //   return -1;
}
