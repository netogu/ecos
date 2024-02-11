#pragma once

#include <stm32g4xx.h>
#include "drivers/stm32g4/gpio.h"

#define HRTIM_PERCLK_HZ 170000000L 

#define HRTIM_FREQ_2_PER(__FREQ__) ((HRTIM_PERCLK_HZ / __FREQ__) * 32 + 1)
#define HRTIM_COUNT_PER_NS(__DT_NS__) (__DT_NS__ / 0.73 + 0.5) // From RM0440 Table 221

enum hrtim_timer {
    HRTIM_TIMER_A,
    HRTIM_TIMER_B,
    HRTIM_TIMER_C,
    HRTIM_TIMER_D,
    HRTIM_TIMER_E
};

enum hrtim_pwm_type {
    HRTIM_PWM_TYPE_TRAILING_EDGE,
    HRTIM_PWM_TYPE_CENTER_ALIGNED
};

enum hrtim_pwm_output {
    HRTIM_PWM_OUTPUT_SINGLE,
    HRTIM_PWM_OUTPUT_COMPLEMENTARY,
};

enum hrtim_pwm_polarity {
    HRTIM_PWM_POLARITY_NORMAL,
    HRTIM_PWM_POLARITY_INVERTED
};

struct hrtim_pwm {
    enum hrtim_timer timer;
    enum hrtim_pwm_type type;
    enum hrtim_pwm_output output;
    enum hrtim_pwm_polarity polarity;
    uint32_t freq_hz;
    float duty_pc;
    float deadtime_ns;
};


int hrtim_pwm_init(struct hrtim_pwm *pwm) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm->timer];

    // Configure PWM Mode

    switch (pwm->type) {
        case HRTIM_PWM_TYPE_TRAILING_EDGE:
            tim_regs->TIMxCR2 &= HRTIM_TIMCR2_UDM;
            break;
        case HRTIM_PWM_TYPE_CENTER_ALIGNED:
            tim_regs->TIMxCR |= HRTIM_TIMCR2_UDM;
            break;
    }

    // Configure PWM Period and Compare
    uint32_t period = HRTIM_FREQ_2_PER(pwm->freq_hz);
    uint32_t cmp = period * pwm->duty_pc / 100.0;

    tim_regs->PERxR = period;
    tim_regs->CMP1xR = cmp;


    // Pre-load enable update on reset/roll-over, continuous mode
    tim_regs->TIMxCR |= ( HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT );

    // Configure Timer outputs, polarity, then FAULT and IDLE states

    // Configure PWM Output

    tim_regs->RSTx1R = HRTIM_SET1R_CMP1;
    tim_regs->SETx1R = HRTIM_RST1R_PER;
    
    // tim_regs->RSTx2R = HRTIM_RST1R_PER;
    // tim_regs->SETx2R = HRTIM_SET1R_CMP1;

    // Enable Deadtime
    tim_regs->OUTxR |= HRTIM_OUTR_DTEN;

    // Configure Deadtime
    uint32_t deadtime = HRTIM_COUNT_PER_NS(pwm->deadtime_ns);
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


    // Enable outputs
    HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TA1OEN + pwm->timer; 
    HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TA2OEN + pwm->timer; 


    return 0;

}

inline void hrtim_pwm_start(struct hrtim_pwm *pwm) {
    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN + pwm->timer;
}

inline void hrtim_pwm_stop(struct hrtim_pwm *pwm) {
    HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN + pwm->timer);
}

void hrtim_set_pwm_pulse_count(struct hrtim_pwm *pwm, uint32_t pulse_count) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm->timer];

    // Set Repetition counter
    tim_regs->REPxR = pulse_count; 

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
    // HRTIM1->sCommonRegs.ODISR |= HRTIM_ODISR_TA1ODIS;

    for (int pwm = 0; pwm < 5; pwm++) {
        if (HRTIM1->sTimerxRegs[pwm].TIMxISR & HRTIM_TIMISR_REP) {
            HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN + pwm);
            HRTIM1->sTimerxRegs[pwm].TIMxICR = HRTIM_TIMICR_REPC; // Clear REP interrupt
        }
    }
}

