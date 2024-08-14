#pragma once

#include <stdint.h>
#include "stm32g4/gpio.h"


typedef enum {
    LPUART_PARITY_EVEN = 0,
    LPUART_PARITY_ODD = 1,
    LPUART_PARITY_NONE = 2,
} lpuart_parity_t;

typedef enum {
    LPUART_FLOW_CONTROL_NONE = 0,
    LPUART_FLOW_CONTROL_RTS = 1,
    LPUART_FLOW_CONTROL_CTS = 2,
    LPUART_FLOW_CONTROL_RTS_CTS = 3,
} lpuart_flow_control_t;

typedef enum {
    LPUART_DATA_BITS_8 = 0,
    LPUART_DATA_BITS_9 = 1,
    LPUART_DATA_BITS_7 = 2,
} lpuart_word_length_t;

typedef enum {
    LPUART_STOP_BITS_1 = 0,
    LPUART_STOP_BITS_2 = 2,
} lpuart_stop_bits_t;

typedef enum {
    LPUART_MODE_RX = 0,
    LPUART_MODE_TX = 1,
    LPUART_MODE_RX_TX = 2,
} lpuart_mode_t;

typedef enum {
    LPUART_STATUS_OK = 0,
    LPUART_STATUS_ERROR = 1,
    LPUART_STATUS_BUSY = 2,
    LPUART_STATUS_TIMEOUT = 3,
} lpuart_status_t;

typedef enum {
    LPUART_CLOCK_SOURCE_PCLK = 0,
    LPUART_CLOCK_SOURCE_SYSCLK = 1,
    LPUART_CLOCK_SOURCE_HSI = 2,
    LPUART_CLOCK_SOURCE_LSE = 3,
} lpuart_clock_source_t;

typedef enum { 
    LPUART_CLOCK_PRESCALER_1 = 0,
    LPUART_CLOCK_PRESCALER_2 = 1,
    LPUART_CLOCK_PRESCALER_4 = 2,
    LPUART_CLOCK_PRESCALER_6 = 3,
    LPUART_CLOCK_PRESCALER_8 = 4,
    LPUART_CLOCK_PRESCALER_10 = 5,
    LPUART_CLOCK_PRESCALER_12 = 6,
    LPUART_CLOCK_PRESCALER_16 = 7,
    LPUART_CLOCK_PRESCALER_32 = 8,
    LPUART_CLOCK_PRESCALER_64 = 9,
    LPUART_CLOCK_PRESCALER_128 = 10,
    LPUART_CLOCK_PRESCALER_256 = 11,
} lpaurt_clock_prescaler_t;

#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128

typedef struct {
    volatile uint32_t *uart_instance;
    gpio_t tx_pin;
    gpio_t rx_pin;
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t tx_head;
    uint8_t tx_tail;
    struct {
        uint32_t baudrate;
        lpuart_word_length_t word_length;
        lpuart_stop_bits_t stop_bits;
        lpuart_parity_t parity;
        lpuart_mode_t mode;
        lpuart_flow_control_t flow_control;
    }config;
} uart_t;


void uart_init(uart_t *self);
void uart_write_byte(uart_t *self, uint8_t byte);
int uart_write_byte_nb(uart_t *self, uint8_t byte);
int uart_read_byte_nb(uart_t *self, uint8_t *byte);
int uart_is_busy(uart_t *self);
int uart_write(uart_t *self, uint8_t *data, uint16_t len);