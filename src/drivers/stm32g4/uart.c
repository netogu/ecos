#include <stdint.h>
#include "stm32g4xx.h"
#include "stm32g4/uart.h"

#define UART_STATUS_OK 0
#define UART_STATUS_ERROR 1
#define UART_STATUS_BUSY 2
#define UART_STATUS_TIMEOUT 3

#define UART_CLOCK_SOURCE_PCLK 0
#define UART_CLOCK_SOURCE_SYSCLK 1
#define UART_CLOCK_SOURCE_HSI 2
#define UART_CLOCK_SOURCE_LSE 3

#define UART_INPUT_CLOCK_PRESCALER_NONE 0
#define UART_INPUT_CLOCK_PRESCALER_DIV_2 1
#define UART_INPUT_CLOCK_PRESCALER_DIV_4 2
#define UART_INPUT_CLOCK_PRESCALER_DIV_6 3
#define UART_INPUT_CLOCK_PRESCALER_DIV_8 4
#define UART_INPUT_CLOCK_PRESCALER_DIV_10 5
#define UART_INPUT_CLOCK_PRESCALER_DIV_12 6
#define UART_INPUT_CLOCK_PRESCALER_DIV_16 7
#define UART_INPUT_CLOCK_PRESCALER_DIV_32 8
#define UART_INPUT_CLOCK_PRESCALER_DIV_64 9
#define UART_INPUT_CLOCK_PRESCALER_DIV_128 10
#define UART_INPUT_CLOCK_PRESCALER_DIV_256 11

static void lpuart_init(uart_t *self) {

    
        // Set LPUART clock source to PCLK
        RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL);
        RCC->CCIPR |= (UART_CLOCK_SOURCE_PCLK << RCC_CCIPR_LPUART1SEL_Pos);

        // Enable LPUART clock
        RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
        uint32_t tmpreg = RCC->APB1ENR2;
        (void)tmpreg;
        
        NVIC_EnableIRQ(LPUART1_IRQn);
        
}

static void usart3_init(uart_t *self) {
    // Enable USART3 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
    uint32_t tmpreg = RCC->APB1ENR1;
    (void)tmpreg;
    // Enable USART3 interrupt
    NVIC_EnableIRQ(USART3_IRQn);
}

void uart_init(uart_t *self) {

    // Clear Buffers
    self->rx_head = 0;
    self->rx_tail = 0;
    self->tx_head = 0;
    self->tx_tail = 0;
    
    // Configure GPIOs
    gpio_pin_init(&self->tx_pin);
    gpio_pin_init(&self->rx_pin);

    if (self->instance == LPUART1) {
        lpuart_init(self);
    } else if (self->instance == USART3) {
        usart3_init(self);
    }
    
    // Set UART Prescaler
    self->instance->PRESC = 0x00;
    self->instance->PRESC |= ( UART_INPUT_CLOCK_PRESCALER_NONE & 0x0F);

    // Disable UART
    self->instance->CR1 &= ~(USART_CR1_UE);
    self->instance->CR1 = 0x00;

    // Set baudrate
    uint32_t usartdiv = SystemCoreClock / self->config.baudrate * 256;
    self->instance->BRR = usartdiv & 0x0FFFFF; // 20 bits

    // Set data bits
    // 8 bits by default
    self->instance->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
    if (self->config.word_length == UART_DATA_BITS_9) {
        self->instance->CR1 |= (USART_CR1_M0);
    } else if (self->config.word_length == UART_DATA_BITS_7) {
        self->instance->CR1 |= (USART_CR1_M1);
    }

    // Set parity
    self->instance->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
    if (self->config.parity == UART_PARITY_EVEN) {
        self->instance->CR1 |= (USART_CR1_PCE);
        self->instance->CR1 |= (self->config.parity << USART_CR1_PS_Pos);
    } // else LPUART_PARITY_NONE

    // Set stop bits
    self->instance->CR2 &= ~(USART_CR2_STOP);
    self->instance->CR2 |= (self->config.stop_bits << USART_CR2_STOP_Pos);

    // Set Flow Control
    self->instance->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
    if (self->config.flow_control & UART_FLOW_CONTROL_RTS) {
        self->instance->CR3 |= (USART_CR3_RTSE);
    } else if (self->config.flow_control & UART_FLOW_CONTROL_CTS) {
        self->instance->CR3 |= (USART_CR3_CTSE);
    } else if (self->config.flow_control & UART_FLOW_CONTROL_RTS_CTS) {
        self->instance->CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);
    } // else LPUART_FLOW_CONTROL_NONE

    // Set mode
    self->instance->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);  
    if (self->config.mode == UART_MODE_RX) {
        self->instance->CR1 |= (USART_CR1_RE);
    } else if (self->config.mode == UART_MODE_TX) {
        self->instance->CR1 |= (USART_CR1_TE);
    } else if (self->config.mode == UART_MODE_RX_TX) {
        self->instance->CR1 |= (USART_CR1_RE | USART_CR1_TE);
    } // else LPUART_MODE_NONE

    // Enable UART
    self->instance->CR1 |= (USART_CR1_UE);

    // Enable UART RXNE interrupt
    self->instance->CR1 |= (USART_CR1_RXNEIE);
    // Enable UART TXE interrupt
    self->instance->CR1 |= (USART_CR1_TXEIE);
}



int uart_get_tx_buffer_count(uart_t *self) {
    return (self->tx_head - self->tx_tail) % UART_TX_BUFFER_SIZE;
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

int uart_read(uart_t *self, uint8_t *data, uint16_t size) {
    uint16_t bytes_read = 0;

    while (bytes_read < size && self->rx_head != self->rx_tail) {
        data[bytes_read] = self->rx_buffer[self->rx_tail];
        self->rx_tail = (self->rx_tail + 1) % UART_RX_BUFFER_SIZE;
        bytes_read++;
    }

    // Return the number of bytes successfully read from the buffer
    return bytes_read;
}