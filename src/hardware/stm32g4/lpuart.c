#include "hardware/stm32g4/lpuart.h"


void lpuart_init(lpuart_config_t *config) {
    
        // Set LPUART clock source to PCLK
        RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL);
        RCC->CCIPR |= (config->clock_source << RCC_CCIPR_LPUART1SEL_Pos);

        // Enable LPUART clock
        RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
        uint32_t tmpreg = RCC->APB1ENR2;
        (void)tmpreg;

        // Set LPUART clock prescaler
        LPUART1->PRESC = 0x00;
        LPUART1->PRESC |= (config->clock_prescale & 0x0F);
        
        // Disable LPUART
        LPUART1->CR1 &= ~(USART_CR1_UE);
        LPUART1->CR1 = 0x00;

        // Set baudrate
        // TODO: Calculate baudrate based on prescaler and clock source
        uint32_t usartdiv = SystemCoreClock / config->baudrate * 256; 
        LPUART1->BRR = usartdiv & 0x0FFFFF; // 20 bits
        
        // Set data bits
        // 8 bits by default
        LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
        if (config->word_length == LPUART_DATA_BITS_9) {
            LPUART1->CR1 |= (USART_CR1_M0);
        } else if (config->word_length == LPUART_DATA_BITS_7) {
            LPUART1->CR1 |= (USART_CR1_M1);
        } 

        // Set parity
        LPUART1->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
        if (config->parity != LPUART_PARITY_NONE) {
            LPUART1->CR1 |= (USART_CR1_PCE);
            LPUART1->CR1 |= (config->parity << USART_CR1_PS_Pos);
        } // else LPUART_PARITY_NONE
        
        // Set stop bits
        LPUART1->CR2 &= ~(USART_CR2_STOP);
        LPUART1->CR2 |= (config->stop_bits << USART_CR2_STOP_Pos);
        
        // Set flow control
        LPUART1->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
        if (config->flow_control & LPUART_FLOW_CONTROL_RTS) {
            LPUART1->CR3 |= (USART_CR3_RTSE);
        } else if (config->flow_control & LPUART_FLOW_CONTROL_CTS) {
            LPUART1->CR3 |= (USART_CR3_CTSE);
        } else if (config->flow_control & LPUART_FLOW_CONTROL_RTS_CTS) {
            LPUART1->CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);
        } // else LPUART_FLOW_CONTROL_NONE
        
        // Set mode
        LPUART1->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);
        if (config->mode == LPUART_MODE_RX) {
            LPUART1->CR1 |= (USART_CR1_RE);
        } else if (config->mode == LPUART_MODE_TX) {
            LPUART1->CR1 |= (USART_CR1_TE);
        } else if (config->mode == LPUART_MODE_RX_TX) {
            LPUART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);
        } // else LPUART_MODE_NONE

        // Enable LPUART
        LPUART1->CR1 |= (USART_CR1_UE);
        
}

#define lpuart_write_byte_blocking(data) do{ \
    while (!(LPUART1->ISR & USART_ISR_TC)); \
    LPUART1->TDR = data; \
}while(0);
#define lpuart_write_byte(data) do{ \
    LPUART1->TDR = data; \
}while(0);

void lpuart_write(uint8_t *data, uint32_t len) {

    for (uint32_t i = 0; i < len; i++) {
        lpuart_write_byte_blocking(data[i]);
    }
    
}
