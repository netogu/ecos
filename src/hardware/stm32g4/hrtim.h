#pragma once

#include <stm32g4xx.h>
#include "hardware/stm32g4/gpio.h"

#define HRTIM_PERCLK 170000000L 
#define HRTIM_FREQ_2_PER(__FREQ__) ((HRTIM_PERCLK/__FREQ__)*32 + 1)

void hrtim_init() {

    // 1. enable HRTIM clock source (RCC)
    // - check that fHRTIM won't exceed range of DLL lock
    RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;

    // 2. start DLL calibration by setting CAL in HRTIM_DLLCR
    HRTIM1->sCommonRegs.DLLCR |= HRTIM_DLLCR_CAL;

    // 3. wait for HR unit is ready by waiting for DLLRDY flag, 
    // can keep doing things but must be ready before starting timers
    /* Obot Procedure:
        set UDM in TIMxCR2
        set PREEN, TRSTU, CONT in TIMxCR
        
    */

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[0];

    tim_regs->TIMxCR = (HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT );
    const uint32_t period = HRTIM_FREQ_2_PER(100000); 
    tim_regs->PERxR = period;
    tim_regs->CMP1xR = period >> 1;

    // 4. configure Timer inputs

    // 5. configure Timer outputs, polarity, then FAULT and IDLE states
    tim_regs->SETx1R = HRTIM_SET1R_CMP1;
    tim_regs->RSTx1R = HRTIM_RST1R_PER;

    // Preload enabled. PER and CMP1 loaded into active register
    // tim_regs->TIMxCR |= HRTIM_TIMCR_PREEN;

    // 6. configure GPIOs. Timer ready to take over

    gpio_t pwm = {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_8,
        .mode = GPIO_MODE_ALTERNATE,
        .speed = GPIO_SPEED_HIGH,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_UP,
        .af = GPIO_AF13
    };


    gpio_pin_init(&pwm);


    // Set Repetition counter to 5
    tim_regs->REPxR = 4; 

    // Enable REP Interrupt
    tim_regs->TIMxDIER |= HRTIM_TIMDIER_REPIE;
    NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);

    // 7. enable outputs TxyOEN in HRTIM_OENR
    HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TA1OEN; 

    // 8. Start timer by setting TxCEN or MCEN in HRTIM_MCR
    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN;

}

	
void HRTIM1_TIMA_IRQHandler(void) {
    // Disable TIMER A
    HRTIM1->sCommonRegs.ODISR |= HRTIM_ODISR_TA1ODIS;
    HRTIM1->sMasterRegs.MCR &= ~HRTIM_MCR_TACEN;
    // On interrupt
    HRTIM1->sTimerxRegs[0].TIMxICR = HRTIM_TIMICR_REPC; // Clear REP interrupt
}

