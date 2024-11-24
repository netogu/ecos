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
#include "stm32g4_common.h"

static int adc_configure_input_sample_time(adc_t *self);
static int adc_configure_regular_input_sequence(const adc_t *self);
static int adc_configure_injected_input_sequence(adc_t *self);
static int adc_configure_injected_conversion_pwm_trigger(adc_t *self);

int adc_start_regular_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

  // Start converting the regular group
  adc_regs->CR |= ADC_CR_ADSTART;

  return 0;
}

int adc_start_injected_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

  // Start converting the injected group
  adc_regs->CR |= ADC_CR_JADSTART;

  return 0;
}

int adc_stop_regular_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

  // Stop converting the regular group
  adc_regs->CR |= ADC_CR_ADSTP;

  return 0;
}

int adc_stop_injected_sampling(adc_t *self) {

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

  // Stop converting the injected group
  adc_regs->CR |= ADC_CR_JADSTP;

  return 0;
}

int adc_enable(adc_t *self) {

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;
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

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

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

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

  // Calibrate the ADC for single ended mode
  adc_regs->CR |= ADC_CR_ADCAL;
  while (adc_regs->CR & ADC_CR_ADCAL)
    ; // Wait for the calibration to complete
  for (volatile uint32_t delay = 0; delay < 100; delay++)
    ;

  // adc_regs->CR |= ADC_CR_ADCALDIF;
  // adc_regs->CR |= ADC_CR_ADCAL;
  // while (adc_regs->CR & ADC_CR_ADCAL)
  //   ; // Wait for the calibration to complete
  // for (volatile uint32_t delay = 0; delay < 100; delay++)
  //   ;

  return 0;
}

int adc_register_input(adc_t *self, adc_input_t *input, char input_type,
                       uint16_t sample_time) {

  if (input_type == 'r') {

    if ((self->num_regular_inputs + 1) > ADC_REG_INPUT_MAX) {
      goto error;
    }

    self->regular_inputs[self->num_regular_inputs].input = input;
    self->regular_inputs[self->num_regular_inputs].sample_time = sample_time;
    self->num_regular_inputs++;

  } else

      if (input_type == 'i') {

    if ((self->num_injected_inputs + 1) > ADC_INJ_INPUT_MAX) {
      goto error;
    }
    self->injected_inputs[self->num_injected_inputs].input = input;
    self->injected_inputs[self->num_injected_inputs].sample_time = sample_time;
    self->num_injected_inputs++;

  } else {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int adc_configure_input_sample_time(adc_t *self) {
  // Configure Sample Time
  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

  uint32_t smpr1_reg = 0;
  uint32_t smpr2_reg = 0;

  if (self->num_regular_inputs > 0) {

    for (uint32_t i = 0; i < self->num_regular_inputs; i++) {
      uint8_t channel = self->regular_inputs[i].input->channel;
      uint16_t sample_time = self->regular_inputs[i].sample_time;
      if (channel <= 9) {
        smpr1_reg |= sample_time << (ADC_SMPR1_SMP0_Pos + (3 * channel));
      } else {
        smpr2_reg |= sample_time
                     << (ADC_SMPR2_SMP10_Pos + (3 * (channel - 10)));
      }
    }
  }

  if (self->num_injected_inputs > 0) {

    for (uint32_t i = 0; i < self->num_injected_inputs; i++) {
      uint8_t channel = self->injected_inputs[i].input->channel;
      uint16_t sample_time = self->injected_inputs[i].sample_time;
      if (channel <= 9) {
        smpr1_reg |= sample_time << (ADC_SMPR1_SMP0_Pos + (3 * channel));
      } else {
        smpr2_reg |= sample_time
                     << (ADC_SMPR2_SMP10_Pos + (3 * (channel - 10)));
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

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;

  // ADC Clk domain: SYSCLK or PLL
  // Enable the ADC Peripheral Clock

  if (adc_regs == ADC1 || adc_regs == ADC2) {

    if (!adc12_clock_initialized) {
      RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
      RCC->CCIPR &= ~(RCC_CCIPR_ADC12SEL);
      ADC12_COMMON->CCR &= ~(ADC_CCR_CKMODE);

      ADC12_COMMON->CCR |=
          (ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1); // Set ADC CLK Domain to HCLK/4

      adc12_clock_initialized = true;
    }

  } else if (adc_regs == ADC3 || adc_regs == ADC4 || adc_regs == ADC5) {

    if (!adc345_clock_initialized) {
      RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
      RCC->CCIPR &= ~(RCC_CCIPR_ADC345SEL);
      ADC345_COMMON->CCR &= ~ADC_CCR_CKMODE;

      ADC345_COMMON->CCR |=
          (ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1); // Set ADC CLK Domain to HCLK/4
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
  status = adc_calibrate_blocking(self);

  // Configure Inputs
  status = adc_configure_input_sample_time(self);

  // Configure Input Sequence
  status = adc_configure_regular_input_sequence(self);
  status = adc_configure_injected_input_sequence(self);

  // Configure Triggers
  // status = adc_configure_injected_conversion_pwm_trigger(self);

  if (0 != status) {
    return -1;
  }

  // Enable the ADC
  adc_enable(self);

  return 0;
}

static int adc_configure_regular_input_sequence(const adc_t *self) {

  // Configure Sequence Order
  if (self->num_regular_inputs > 0) {
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;
    uint32_t sqr1 = 0;
    uint32_t sqr2 = 0;
    uint32_t sqr3 = 0;
    uint32_t sqr4 = 0;

    for (uint16_t i = 0; i < self->num_regular_inputs; i++) {

      adc_input_t *input = self->regular_inputs[i].input;

      if (i < 4) {
        sqr1 |= input->channel << (ADC_SQR1_SQ1_Pos + (i) * 6);
      } else if (i < 9) {
        sqr2 |= input->channel << (ADC_SQR2_SQ5_Pos + (i - 4) * 6);
      } else if (i < 14) {
        sqr3 |= input->channel << (ADC_SQR3_SQ10_Pos + (i - 9) * 6);
      } else if (i < 16) {
        sqr4 |= input->channel << (ADC_SQR4_SQ15_Pos + (i - 14) * 6);
      } else {
        return -1;
      }

      adc_regs->SQR1 = sqr1;
      adc_regs->SQR2 = sqr2;
      adc_regs->SQR3 = sqr3;
      adc_regs->SQR4 = sqr4;

      // TODO: Map to static reg conversion result buffer
      // self->regular_inputs[i]->input.data =
    }

    adc_regs->SQR1 |= (self->num_regular_inputs - 1) << ADC_SQR1_L_Pos;
  }
  return 0;
}

__attribute__((unused)) static int
adc_configure_injected_conversion_pwm_trigger(adc_t *self) {

  const uint32_t hrtim_adc_trig1 = 27;

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;
  uint32_t reg_val = adc_regs->JSQR;

  adc_regs->CFGR &= ~(ADC_CFGR_JQDIS);

  reg_val &= ~(ADC_JSQR_JEXTSEL) | ~(ADC_JSQR_JEXTEN);
  reg_val |= (hrtim_adc_trig1 << ADC_JSQR_JEXTSEL_Pos);
  reg_val |= (2 << ADC_JSQR_JEXTEN_Pos);

  adc_regs->JSQR = reg_val;

  return 0;
}

static int adc_configure_injected_input_sequence(adc_t *self) {

  ADC_TypeDef *adc_regs = (ADC_TypeDef *)self->regs;
  const size_t jsq_offset_bits = 6;
  const size_t jdr_offset_bytes = 4;

  uint32_t reg_val = 0;

  if (self->num_injected_inputs > 0) {

    for (uint16_t i = 0; i < self->num_injected_inputs; i++) {
      reg_val |= self->injected_inputs[i].input->channel
                 << (ADC_JSQR_JSQ1_Pos + i * jsq_offset_bits);
      self->injected_inputs[i].input->data =
          ((uint32_t *)(&adc_regs->JDR1 + i * jdr_offset_bytes));
    }
  }

  reg_val &= ~(ADC_JSQR_JL);
  reg_val |= (self->num_injected_inputs - 1) << ADC_JSQR_JL_Pos;
  adc_regs->JSQR = reg_val;

  return 0;
}
