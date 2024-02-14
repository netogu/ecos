#include <stm32g4xx.h>
#include "drivers/stm32g4/hrtim.h"
#include "drivers/stm32g4/gpio.h"

void hrtim_pwm_set_frequency(struct hrtim_pwm *pwm, uint32_t freq_hz) {
    uint32_t period = 0;
    switch (pwm->type) {
        case HRTIM_PWM_TYPE_TRAILING_EDGE:
            period = HRTIM_FREQ_2_PER(freq_hz);
            break;
        case HRTIM_PWM_TYPE_CENTER_ALIGNED:
            period = HRTIM_FREQ_2_PER(freq_hz)/2;
            break;
        default:
            return -1;
    }

    HRTIM1->sTimerxRegs[pwm->timer].PERxR = period;   
}

void hrtim_pwm_set_duty(struct hrtim_pwm *pwm, uint32_t duty_pc) {
    uint32_t cmp = 0;
    uint32_t period = HRTIM1->sTimerxRegs[pwm->timer].PERxR;
    switch (pwm->type) {
        case HRTIM_PWM_TYPE_TRAILING_EDGE:
            cmp = period * duty_pc / 100.0;
            break;
        case HRTIM_PWM_TYPE_CENTER_ALIGNED:
            cmp = period * (100.0 - duty_pc) / 100.0;
            break;
        default:
            break;
    }

    HRTIM1->sTimerxRegs[pwm->timer].CMP1xR = cmp;
}   
int hrtim_pwm_init(struct hrtim_pwm *pwm) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm->timer];

    // Configure PWM Mode

    uint32_t period = 0;
    uint32_t cmp = 0;

    switch (pwm->type) {
        case HRTIM_PWM_TYPE_TRAILING_EDGE:
            tim_regs->TIMxCR2 &= HRTIM_TIMCR2_UDM;
            tim_regs->RSTx1R = HRTIM_SET1R_CMP1;
            tim_regs->SETx1R = HRTIM_RST1R_PER;
            period = HRTIM_FREQ_2_PER(pwm->freq_hz);
            break;
        case HRTIM_PWM_TYPE_CENTER_ALIGNED:
            tim_regs->TIMxCR2 |= HRTIM_TIMCR2_UDM;
            tim_regs->SETx1R = HRTIM_SET1R_CMP1;
            period = HRTIM_FREQ_2_PER(pwm->freq_hz)/2;
            break;
        default:
            return -1;
    }



    tim_regs->PERxR = period;
    tim_regs->CMP1xR = 0;


    // Pre-load enable update on reset/roll-over, continuous mode
    tim_regs->TIMxCR |= ( HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT );

    // Configure Timer outputs, polarity, then FAULT and IDLE states

    // Configure PWM Output : Reset on match, Set on Period

    
    
    // tim_regs->RSTx2R = HRTIM_RST1R_PER;
    // tim_regs->SETx2R = HRTIM_SET1R_CMP1;

    // Enable Deadtime
    tim_regs->OUTxR |= HRTIM_OUTR_DTEN;

    // Configure Deadtime
    uint32_t deadtime = HRTIM_DT_COUNT_PER_NS(pwm->deadtime_ns);
    tim_regs->DTxR = (deadtime << HRTIM_DTR_DTF_Pos) | (deadtime << HRTIM_DTR_DTR_Pos);
    

    // Configure PWM Polarity

    // Configure GPIOs. Timer ready to take over
    {
        gpio_t pwm_gpio_1 = {
            .port = GPIO_PORT_A,
            .pin = GPIO_PIN_8,
            .mode = GPIO_MODE_ALTERNATE,
            .speed = GPIO_SPEED_HIGH,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .af = GPIO_AF13
        };

        gpio_t pwm_gpio_2 = {
            .port = GPIO_PORT_A,
            .pin = GPIO_PIN_9,
            .mode = GPIO_MODE_ALTERNATE,
            .speed = GPIO_SPEED_HIGH,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .af = GPIO_AF13
        };

        gpio_pin_init(&pwm_gpio_1);
        gpio_pin_init(&pwm_gpio_2);

    }

    return 0;

}

void hrtim_pwm_start(struct hrtim_pwm *pwm) {

    // Enable outputs
    HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TA1OEN + pwm->timer; 
    HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TA2OEN + pwm->timer; 
    // Start Timer
    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN + pwm->timer;

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



void hrtim_init() {

    // Enable HRTIM clock source (RCC)
    // - check that fHRTIM won't exceed range of DLL lock
    RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;

    // Start DLL calibration by setting CAL in HRTIM_DLLCR
    HRTIM1->sCommonRegs.DLLCR |= HRTIM_DLLCR_CAL;

    // Wait for HR unit is ready by waiting for DLLRDY flag, 
    // can keep doing things but must be ready before starting timers


}

	
void HRTIM1_TIMA_IRQHandler(void) {

    // Disable TIMER A
    // HRTIM1->sCommonRegs.ODISR |= HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
    // HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN);

    // HRTIM1->sTimerxRegs[HRTIM_TIM_A].CMP1CxR = 0;
    // HRTIM1->sTimerxRegs[HRTIM_TIM_A].RSTx1R = HRTIM_RST1R_PER;
    HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxCR &= ~(HRTIM_TIMCR_CONT);
    // // HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxCR2 |= HRTIM_CR2_TARST;
    // HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU;

    // HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN); // Disable TIMER A
    HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxICR |= HRTIM_TIMICR_REPC; // Clear REP interrupt

    // for (int pwm = 0; pwm < 5; pwm++) {
    //     if (HRTIM1->sTimerxRegs[pwm].TIMxISR & HRTIM_TIMISR_REP) {
    //         HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN + pwm);
    //         HRTIM1->sTimerxRegs[pwm].TIMxICR = HRTIM_TIMICR_REPC; // Clear REP interrupt
    //     }
    // }
}
