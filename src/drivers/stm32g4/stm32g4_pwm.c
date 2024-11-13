
#include "stm32g4_pwm.h"


static void _hrtim1_init(void) {

  // Enable HRTIM clock source (RCC)
  // - check that fHRTIM won't exceed range of DLL lock
  RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;

  // Start DLL calibration by setting CAL in HRTIM_DLLCR
  HRTIM1->sCommonRegs.DLLCR |= HRTIM_DLLCR_CAL;

  // Wait for HR unit is ready by waiting for DLLRDY flag,
  // can keep doing things but must be ready before starting timers
}

static int _pwm_enable_outputs(pwm_t *self) {

  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if (pwm_timer == PWM_TIMER_HRTIM1) {
    // Enable outputs
    HRTIM1->sCommonRegs.OENR |= 1 << (HRTIM_OENR_TA1OEN_Pos + 2 * pwm_channel);
    HRTIM1->sCommonRegs.OENR |= 1 << (HRTIM_OENR_TA2OEN_Pos + 2 * pwm_channel);
  }

  return 0;
}

int pwm_set_frequency(pwm_t *self, uint32_t freq_hz) {

  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if ( pwm_timer == PWM_TIMER_HRTIM1) {
    uint32_t prescale = HRTIM1->sTimerxRegs[pwm_channel].TIMxCR & HRTIM_TIMCR_CK_PSC;
    HRTIM1->sTimerxRegs[pwm_channel].PERxR = SystemCoreClock / freq_hz * 32 >> (prescale + 1);
  }
  return 0;
}

/**
 * @brief Set PWM duty cycle 
 * 
 * @param self      pwm_t object 
 * @param duty_u    Duty cycle normalized to 1.0 
 * @return int      0 on success
 */

int pwm_set_duty(pwm_t *self, float duty_u) {

  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if ( pwm_timer == PWM_TIMER_HRTIM1) {
    const float duty_max = 0.98;
    const float duty_min = 0.02;

    if (duty_u < duty_min) {
      duty_u = duty_min;
    }else if (duty_u > duty_max) {
      duty_u = duty_max;
    }
    // } else {
    //   // Invalid duty cycle
    //   return -1;
    // }

    //read period
    uint32_t period_reg = HRTIM1->sTimerxRegs[pwm_channel].PERxR;
    float duty_period = (period_reg) * (duty_u);

    // Center edge modulation by default
    // uint32_t cmp = period_reg/2;
    uint32_t cmp = (uint32_t)(duty_period + 0.5f);
    HRTIM1->sTimerxRegs[pwm_channel].CMP1xR = cmp;
  }

  return 0;
}

int pwm_swap_output(pwm_t *self) {

  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if ( pwm_timer == PWM_TIMER_HRTIM1) {
    // Swap PWM outputs
    HRTIM1->sCommonRegs.CR2 |= (1 << (HRTIM_CR2_SWPA_Pos + pwm_channel));
    // Update registers
    HRTIM1->sCommonRegs.CR2 |= (1 << (HRTIM_CR2_TASWU_Pos + pwm_channel));
  }
  return 0;
}

int pwm_enable_fault_input(pwm_t *self, uint32_t fault) {
  (void) fault;

  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if ( pwm_timer == PWM_TIMER_HRTIM1) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm_channel];

    // Set Input Source to FLT pin
    HRTIM1->sCommonRegs.FLTINR2 &=
        ~(HRTIM_FLTINR2_FLT5SRC_0_Msk | HRTIM_FLTINR2_FLT5SRC_1_Msk);

    // Set Input Polarity
    HRTIM1->sCommonRegs.FLTINR2 &= ~HRTIM_FLTINR2_FLT5P_Msk; // Active Low

    // Configure input filter
    HRTIM1->sCommonRegs.FLTINR2 |= 2
                                  << HRTIM_FLTINR2_FLTSD_Pos; // fFLTS = fHRTIM/4
    HRTIM1->sCommonRegs.FLTINR2 &= ~HRTIM_FLTINR2_FLT5F_Msk;
    HRTIM1->sCommonRegs.FLTINR2 |=
        6 << HRTIM_FLTINR2_FLT5F_Pos; // fsampling = fFLTS/4 * 6

    // Configure Blanking sources
    // -- None --

    // Engage fault input
    HRTIM1->sCommonRegs.FLTINR2 |= HRTIM_FLTINR2_FLT5E;

    // Configure PWM faulted states
    tim_regs->OUTxR &= ~HRTIM_OUTR_FAULT1_Msk;
    tim_regs->OUTxR &= ~HRTIM_OUTR_FAULT2_Msk;
    tim_regs->OUTxR |= 0b10 << HRTIM_OUTR_FAULT1_Pos; // Inactive fault state
    tim_regs->OUTxR |= 0b10 << HRTIM_OUTR_FAULT2_Pos; // Inactive fault state

    // Enable fault
    tim_regs->FLTxR |= HRTIM_FLTR_FLT5EN;
  }

  return 0;
}

/**
 * @brief Initialize PWM object 
 * 
 * @param self 
 * @param freq_hz  Frequency in Hz
 * @param dt_ns    Deadtime in ns
 * @return int     0 on success
 */
int pwm_init(pwm_t *self, uint32_t freq_hz, uint32_t dt_ns) {
  
  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if (pwm_timer == PWM_TIMER_HRTIM1) {

    _hrtim1_init();

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm_channel];

      // Configure PWM Mode

    uint32_t period = 0;
    uint32_t prescale = 0;

    if (freq_hz < 83000) {
      prescale = 1;
    }
    if (freq_hz < 41500) {
      prescale = 2;
    }
    if (freq_hz < 20800) {
      prescale = 3;
    }
    if (freq_hz < 10400) {
      prescale = 4;
    }
    if (freq_hz < 5190) {
      prescale = 5;
    }
    if (freq_hz < 2590) {
      prescale = 6;
    }
    if (freq_hz < 1300) {
      prescale = 7;
    }

    tim_regs->TIMxCR &= ~HRTIM_TIMCR_CK_PSC;
    tim_regs->TIMxCR |= prescale << HRTIM_TIMCR_CK_PSC_Pos;

    // Set PWM Mode to Center Aligned
    tim_regs->TIMxCR2 |= HRTIM_TIMCR2_UDM;
    tim_regs->SETx1R = HRTIM_SET1R_CMP1;
    period = (SystemCoreClock / freq_hz * (32 >> prescale) + 1) / 2;

    tim_regs->PERxR = period;
    tim_regs->CMP1xR = 0;

    // Pre-load enable update on reset/roll-over, continuous mode
    tim_regs->TIMxCR |=
        (HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT);

    // Configure Timer outputs, polarity, then FAULT and IDLE states

    // Configure PWM Output : Reset on match, Set on Period

    tim_regs->RSTx2R = HRTIM_RST1R_CMP1;
    // tim_regs->SETx2R = HRTIM_SET1R_CMP1;

    // Configure Deadtime
    tim_regs->DTxR |= (1 << HRTIM_DTR_DTPRSC_Pos); // tDTG = tHRTIM/4
    // From Rm0440 table 221
    uint32_t dt_cnt = (uint32_t)(dt_ns / 1.47 + 0.5); 
    tim_regs->DTxR |= (dt_cnt << HRTIM_DTR_DTF_Pos) | (dt_cnt << HRTIM_DTR_DTR_Pos);
    // Enable Deadtime
    tim_regs->OUTxR |= HRTIM_OUTR_DTEN;

    // Configure PWM Polarity
  }

  return 0;
}

/**
 * @brief Start PWM
 * 
 * @param self pwm_t object
 * @return int 0 on success
 */
int pwm_start(pwm_t *self) {

  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if (pwm_timer == PWM_TIMER_HRTIM1) { 
  // Enable Outputs
  _pwm_enable_outputs(self);
  // Start Timer
  HRTIM1->sMasterRegs.MCR |= 1 << (HRTIM_MCR_TACEN_Pos + pwm_channel);
  }

  return 0;
}

/** 
 * @brief Stop PWM
 * 
 * @param self pwm_t object
 * @return int 0 on success
*/
int pwm_stop(pwm_t *self) {
  
  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if (pwm_timer == PWM_TIMER_HRTIM1) {
  HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN + pwm_channel);
  }

  return 0;
}

int pwm_set_n_cycle_run(pwm_t *self, uint32_t cycles) {

  enum pwm_timer_e pwm_timer = self->options.pwm_timer;
  uint16_t pwm_channel = self->options.pwm_channel;

  if (pwm_timer == PWM_TIMER_HRTIM1) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm_channel];

    // Reset Timer
    pwm_stop(self);

    // Set PWM to Continuous mode
    tim_regs->TIMxCR |= HRTIM_TIMCR_CONT;
    // Roll-over mode counter = zero
    tim_regs->TIMxCR2 |= 1 << HRTIM_TIMCR2_ROM_Pos;
    // Set Repetition counter
    tim_regs->REPxR = cycles - 2;
    // Force Update
    HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU + pwm_channel;

    // Enable REP Interrupt
    tim_regs->TIMxDIER |= HRTIM_TIMDIER_REPIE;

    switch (pwm_channel) {
    case PWM_HRTIM_TIM_A:
      NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
      break;
    case PWM_HRTIM_TIM_B:
      NVIC_EnableIRQ(HRTIM1_TIMB_IRQn);
      break;
    case PWM_HRTIM_TIM_C:
      NVIC_EnableIRQ(HRTIM1_TIMC_IRQn);
      break;
    case PWM_HRTIM_TIM_D:
      NVIC_EnableIRQ(HRTIM1_TIMD_IRQn);
      break;
    case PWM_HRTIM_TIM_E:
      NVIC_EnableIRQ(HRTIM1_TIME_IRQn);
      break;
    case PWM_HRTIM_TIM_F:
      NVIC_EnableIRQ(HRTIM1_TIMF_IRQn);
      break;
    default:
      break;
    }
    
  }

  return 0;
}

int pwm_3ph_init(pwm_3ph_t *self, uint32_t freq_hz, uint32_t dt_ns) {

  pwm_t *pwms[3] = {
    &self->pwma,
    &self->pwmb,
    &self->pwmc
  };

  if (self->mode == PWM_3PHASE_MODE_6PWM) {

    for (size_t i = 0; i < sizeof(pwms)/sizeof(pwms[0]); i++) {

      pwm_init(pwms[i], freq_hz, dt_ns);

    }

  } else if (self->mode == PWM_3PHASE_MODE_3PWM) {
    // TODO implement 3 PWM mode
  } else {
    // Invalid 3-phase mode
    return -1;
  }
  return 0;
}

int pwm_3ph_start(pwm_3ph_t *self) {

  pwm_t *pwms[3] = {
    &self->pwma,
    &self->pwmb,
    &self->pwmc
  };

  uint32_t mcr_reg = 0;
  for (size_t i = 0; i < sizeof(pwms)/sizeof(pwms[0]); i++) {
    _pwm_enable_outputs(pwms[i]);
    mcr_reg |= 1 << (HRTIM_MCR_TACEN_Pos + pwms[i]->options.pwm_channel);

  }
  // Start Timer
  HRTIM1->sMasterRegs.MCR |= mcr_reg;

  return 0;
}

int pwm_3ph_stop(pwm_3ph_t *self) {

  pwm_t *pwms[3] = {
    &self->pwma,
    &self->pwmb,
    &self->pwmc
  };

  uint32_t mcr_reg = HRTIM1->sMasterRegs.MCR;
  for (size_t i = 0; i < sizeof(pwms)/sizeof(pwms[0]); i++) {
    pwm_stop(pwms[i]);
    mcr_reg &= ~(HRTIM_MCR_TACEN + pwms[i]->options.pwm_channel);

  }
  HRTIM1->sMasterRegs.MCR = mcr_reg;

  return 0;
}

int pwm_3ph_set_frequency(pwm_3ph_t *self, uint32_t freq_hz) {

  pwm_t *pwms[3] = {
    &self->pwma,
    &self->pwmb,
    &self->pwmc
  };

  for (size_t i = 0; i < sizeof(pwms)/sizeof(pwms[0]); i++) {
    pwm_set_frequency(pwms[i], freq_hz);
  }

  return 0;
}

int pwm_3ph_set_duty(pwm_3ph_t *self, float d1_u, float d2_u, float d3_u) {

  pwm_set_duty(&self->pwma, d1_u);
  pwm_set_duty(&self->pwmb, d2_u);
  pwm_set_duty(&self->pwmc, d3_u);

  return 0;
}

__attribute__((unused))
static void pwm_dac_init(void) {
  // Enable TIM20 APB Clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
  // Enable Auto-Reload
  TIM20->CR1 |= TIM_CR1_ARPE;
  // Set count mode to up-count
  TIM20->CR1 &= ~TIM_CR1_DIR;
  // Set Prescaler
  TIM20->PSC = 0;
  // Set Period
  TIM20->ARR = SystemCoreClock / 50000;
  // Set Duty Cycle to 25%
  TIM20->CCR3 = TIM20->ARR >> 2;
  // Set CH3 output mode to PWM
  TIM20->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  // Preload disable
  TIM20->CCMR2 &= ~TIM_CCMR2_OC3PE;
  // Enable CH3 output
  TIM20->CCER |= TIM_CCER_CC3E;
  // Update registers
  TIM20->EGR |= TIM_EGR_UG;
  // Enable Counter
  TIM20->CR1 |= TIM_CR1_CEN;
  TIM20->BDTR |= TIM_BDTR_MOE;
}

