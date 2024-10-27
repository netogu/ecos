#include "encoder.h"
#include "stm32g4xx.h"

static uint32_t encoder_read(void) {
    return TIM5->CNT;
}

// Encoder Implementation using 32bit TIM5
int encoder_init(encoder_t *self) {
    self->read = encoder_read;
    TIM_TypeDef *timer = TIM5;

    //Enable peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;

    //Select TI1 and TI2 Source to TI1|2 as Inputs
    timer->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

    //Set edge polarity in CCER - No filter
    // CC1P & CC1NP = 0  : Non-Inverted Rising-Edge
    // CC2P & CC2NP = 0  : Non-Inverted Rising-Edge
    timer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);

    //Initialize TIM5 - Slave mode selection 
    timer->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
    // timer->SMCR |= 0b1111 << TIM_SMCR_SMS_Pos;

    //Limit Range to 1024 points
    timer->ARR = 400;

    // Enable TIM5
    timer->CR1 |= TIM_CR1_CEN;

    return 0;
}
