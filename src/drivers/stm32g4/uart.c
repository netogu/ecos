#include <stdint.h>
#include "stm32g4xx.h"
#include "stm32g4/uart.h"


void uart_init(uart_t *self) {


        // UART Base
        
        // Configure GPIOs
        gpio_pin_init(&self->tx_pin);
        gpio_pin_init(&self->rx_pin);
    
        // Set LPUART clock source to PCLK
        RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL);
        RCC->CCIPR |= (LPUART_CLOCK_SOURCE_PCLK << RCC_CCIPR_LPUART1SEL_Pos);

        // Enable LPUART clock
        RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
        uint32_t tmpreg = RCC->APB1ENR2;
        (void)tmpreg;

        // Set LPUART clock prescaler
        LPUART1->PRESC = 0x00;
        LPUART1->PRESC |= (LPUART_CLOCK_PRESCALER_1 & 0x0F);
        
        // Disable LPUART
        LPUART1->CR1 &= ~(USART_CR1_UE);
        LPUART1->CR1 = 0x00;

        // Set baudrate
        // TODO: Calculate baudrate based on prescaler and clock source
        uint32_t usartdiv = SystemCoreClock / self->config.baudrate * 256; 
        LPUART1->BRR = usartdiv & 0x0FFFFF; // 20 bits
        
        // Set data bits
        // 8 bits by default
        LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
        if (self->config.word_length == LPUART_DATA_BITS_9) {
            LPUART1->CR1 |= (USART_CR1_M0);
        } else if (self->config.word_length == LPUART_DATA_BITS_7) {
            LPUART1->CR1 |= (USART_CR1_M1);
        } 

        // Set parity
        LPUART1->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
        if (self->config.parity != LPUART_PARITY_NONE) {
            LPUART1->CR1 |= (USART_CR1_PCE);
            LPUART1->CR1 |= (self->config.parity << USART_CR1_PS_Pos);
        } // else LPUART_PARITY_NONE
        
        // Set stop bits
        LPUART1->CR2 &= ~(USART_CR2_STOP);
        LPUART1->CR2 |= (self->config.stop_bits << USART_CR2_STOP_Pos);
        
        // Set flow control
        LPUART1->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
        if (self->config.flow_control & LPUART_FLOW_CONTROL_RTS) {
            LPUART1->CR3 |= (USART_CR3_RTSE);
        } else if (self->config.flow_control & LPUART_FLOW_CONTROL_CTS) {
            LPUART1->CR3 |= (USART_CR3_CTSE);
        } else if (self->config.flow_control & LPUART_FLOW_CONTROL_RTS_CTS) {
            LPUART1->CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);
        } // else LPUART_FLOW_CONTROL_NONE
        
        // Set mode
        LPUART1->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);
        if (self->config.mode == LPUART_MODE_RX) {
            LPUART1->CR1 |= (USART_CR1_RE);
        } else if (self->config.mode == LPUART_MODE_TX) {
            LPUART1->CR1 |= (USART_CR1_TE);
        } else if (self->config.mode == LPUART_MODE_RX_TX) {
            LPUART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);
        } // else LPUART_MODE_NONE

        // Enable LPUART
        LPUART1->CR1 |= (USART_CR1_UE);

        // Enable LPUART RXNE interrupt
        LPUART1->CR1 |= (USART_CR1_RXNEIE);
        // Enable LPUART TXE interrupt
        LPUART1->CR1 |= (USART_CR1_TXEIE);
        NVIC_EnableIRQ(LPUART1_IRQn);
        
}



void uart_write_byte(uart_t *self, uint8_t byte) {

    while (!(LPUART1->ISR & USART_ISR_TXE));
    LPUART1->TDR = byte;
    while (!(LPUART1->ISR & USART_ISR_TC)){
        // wait for transmission complete
    }
    
}
// non-blocking write byte
int uart_write_byte_nb(uart_t *self, uint8_t byte) {

    if (LPUART1->ISR & USART_ISR_TXE) {
        LPUART1->TDR = byte;
        return 1;
    }
    return 0;
}
// non-blocking read byte
int uart_read_byte_nb(uart_t *self, uint8_t *byte) {
    if (LPUART1->ISR & USART_ISR_RXNE) {
        *byte = LPUART1->RDR;
        return 1;
    }
    return 0;
}

// check if uart is busy
int uart_is_busy(uart_t *self) {
    if (LPUART1->ISR & USART_ISR_BUSY) {
        return 1;
    }
    return 0;
}

int uart_write(uart_t *self, uint8_t *data, uint16_t len) {
    uint16_t next_head;

    for (uint16_t i = 0; i < len; i++) {
        next_head = (self->tx_head + 1) % UART_TX_BUFFER_SIZE;
        if (next_head == self->tx_tail) {
            // buffer is full 
            return i;
        }
        self->tx_buffer[self->tx_head] = data[i];
        self->tx_head = next_head;
    }
    // Enable TXE interrupt
    LPUART1->CR1 |= USART_CR1_TXEIE;
    return len;
}