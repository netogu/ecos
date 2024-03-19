#include <stm32g4xx.h>
#include "drivers/stm32g4/hrtim.h"
#include "drivers/stm32g4/gpio.h"

void hrtim_pwm_set_frequency(struct hrtim_pwm *pwm, uint32_t freq_hz) {
    uint32_t period = 0;
    uint32_t prescale = HRTIM1->sTimerxRegs[pwm->timer].TIMxCR & HRTIM_TIMCR_CK_PSC;
    HRTIM1->sTimerxRegs[pwm->timer].PERxR = SystemCoreClock / freq_hz * 32 >> prescale + 1;
}

void hrtim_pwm_set_duty(struct hrtim_pwm *pwm, uint32_t duty_pc) {
    uint32_t cmp = 0;
    uint32_t period = HRTIM1->sTimerxRegs[pwm->timer].PERxR;
    cmp = period * (100.0 - duty_pc) / 100.0;

    HRTIM1->sTimerxRegs[pwm->timer].CMP1xR = cmp;
}   

void hrtim_pwm_swap_output(struct hrtim_pwm *pwm) {

  // Swap PWM outputs
  HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_SWPA + pwm->timer;
  // Update registers
  HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU + pwm->timer;
  
}

int hrtim_pwm_enable_fault_input(struct hrtim_pwm *pwm, uint32_t fault) {
    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm->timer];

    // Set Input Source to FLT pin
    HRTIM1->sCommonRegs.FLTINR2 &= ~(HRTIM_FLTINR2_FLT5SRC_0_Msk | HRTIM_FLTINR2_FLT5SRC_1_Msk);

    // Set Input Polarity
    HRTIM1->sCommonRegs.FLTINR2 &= ~HRTIM_FLTINR2_FLT5P_Msk; // Active Low

    // Configure input filter
    HRTIM1->sCommonRegs.FLTINR2 |= 2 << HRTIM_FLTINR2_FLTSD_Pos;  // fFLTS = fHRTIM/4
    HRTIM1->sCommonRegs.FLTINR2 &= ~HRTIM_FLTINR2_FLT5F_Msk;
    HRTIM1->sCommonRegs.FLTINR2 |= 6 << HRTIM_FLTINR2_FLT5F_Pos; // fsampling = fFLTS/4 * 6

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




    return 0;
}
int hrtim_pwm_init(struct hrtim_pwm *pwm) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm->timer];

    // Configure PWM Mode

    uint32_t period = 0;
    uint32_t cmp = 0;
    uint32_t prescale = 0;

    if (pwm->freq_hz < 83000) {
        prescale = 1;
    }
    if (pwm->freq_hz < 41500) {
        prescale = 2;
    }
    if (pwm->freq_hz < 20800) {
        prescale = 3;
    }
    if (pwm->freq_hz < 10400) {
        prescale = 4;
    }
    if (pwm->freq_hz < 5190) {
        prescale = 5;
    }
    if (pwm->freq_hz < 2590) {
        prescale = 6;
    }
    if (pwm->freq_hz < 1300) {
        prescale = 7;
    }

    
    tim_regs->TIMxCR &= ~HRTIM_TIMCR_CK_PSC;
    tim_regs->TIMxCR |= prescale << HRTIM_TIMCR_CK_PSC_Pos;

    // Set PWM Mode to Center Aligned
    tim_regs->TIMxCR2 |= HRTIM_TIMCR2_UDM;
    tim_regs->SETx1R = HRTIM_SET1R_CMP1;
    period = (SystemCoreClock / pwm->freq_hz * (32 >> prescale) + 1) / 2;
    
    tim_regs->PERxR = period;
    tim_regs->CMP1xR = 0;

    // Pre-load enable update on reset/roll-over, continuous mode
    tim_regs->TIMxCR |= ( HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT );

    // Configure Timer outputs, polarity, then FAULT and IDLE states

    // Configure PWM Output : Reset on match, Set on Period

    tim_regs->RSTx2R = HRTIM_RST1R_CMP1;
    // tim_regs->SETx2R = HRTIM_SET1R_CMP1;


    // Configure Deadtime
    tim_regs->DTxR |= (1 << HRTIM_DTR_DTPRSC_Pos); // tDTG = tHRTIM/4 
    uint32_t deadtime = (uint32_t)(pwm->deadtime_ns/1.47 + 0.5); // From Rm0440 table 221
    tim_regs->DTxR |= (deadtime << HRTIM_DTR_DTF_Pos) | (deadtime << HRTIM_DTR_DTR_Pos);
    // Enable Deadtime
    tim_regs->OUTxR |= HRTIM_OUTR_DTEN;

    // Configure PWM Polarity

    return 0;

}

void hrtim_pwm_start(struct hrtim_pwm *pwm) {

    // Enable outputs
    HRTIM1->sCommonRegs.OENR |= 1 << (HRTIM_OENR_TA1OEN_Pos + 2*pwm->timer);
    HRTIM1->sCommonRegs.OENR |= 1 << (HRTIM_OENR_TA2OEN_Pos + 2*pwm->timer);
    // Start Timer
    HRTIM1->sMasterRegs.MCR |= 1 << (HRTIM_MCR_TACEN_Pos + pwm->timer);

}

inline void hrtim_pwm_stop(struct hrtim_pwm *pwm) {
    HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN + pwm->timer);
}

void hrtim_pwm_set_n_cycle_run(struct hrtim_pwm *pwm, uint32_t cycles) {
    

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm->timer];

    // Reset Timer
    HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN + pwm->timer);

    // Set PWM to Continuous mode
    tim_regs->TIMxCR |= HRTIM_TIMCR_CONT;
    tim_regs->TIMxCR2 |= 1 << HRTIM_TIMCR2_ROM_Pos; // Roll-over mode counter = zero
    // Set Repetition counter
    tim_regs->REPxR = cycles - 2;  
    // Force Update
    HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU + pwm->timer;

    // Enable REP Interrupt
    tim_regs->TIMxDIER |= HRTIM_TIMDIER_REPIE;
    NVIC_EnableIRQ(HRTIM1_TIMA_IRQn + pwm->timer);
}



void hrtim_init(void) {

    // Enable HRTIM clock source (RCC)
    // - check that fHRTIM won't exceed range of DLL lock
    RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;

    // Start DLL calibration by setting CAL in HRTIM_DLLCR
    HRTIM1->sCommonRegs.DLLCR |= HRTIM_DLLCR_CAL;

    // Wait for HR unit is ready by waiting for DLLRDY flag, 
    // can keep doing things but must be ready before starting timers


}

	
