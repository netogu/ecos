#pragma once

#include <stdint.h>
#include "stm32g4xx.h"


typedef enum {
    UART_PARITY_EVEN = 0,
    UART_PARITY_ODD = 1,
    UART_PARITY_NONE = 2,
} uart_parity_t;

typedef enum {
    UART_FLOW_CONTROL_NONE = 0,
    UART_FLOW_CONTROL_RTS = 1,
    UART_FLOW_CONTROL_CTS = 2,
    UART_FLOW_CONTROL_RTS_CTS = 3,
} uart_flow_control_t;

typedef enum {
    UART_DATA_BITS_8 = 0,
    UART_DATA_BITS_9 = 1,
    UART_DATA_BITS_7 = 2,
} uart_word_length_t;

typedef enum {
    UART_STOP_BITS_1 = 0,
    UART_STOP_BITS_2 = 2,
} uart_stop_bits_t;

typedef enum {
    UART_MODE_RX = 0,
    UART_MODE_TX = 1,
    UART_MODE_RX_TX = 2,
} uart_mode_t;

typedef enum {
    UART_STATUS_OK = 0,
    UART_STATUS_ERROR = 1,
    UART_STATUS_BUSY = 2,
    UART_STATUS_TIMEOUT = 3,
} uart_status_t;

typedef enum {
    UART_CLOCK_SOURCE_PCLK = 0,
    UART_CLOCK_SOURCE_SYSCLK = 1,
    UART_CLOCK_SOURCE_HSI = 2,
    UART_CLOCK_SOURCE_LSE = 3,
} uart_clock_source_t;

typedef enum { 
    UART_CLOCK_PRESCALER_1 = 0,
    UART_CLOCK_PRESCALER_2 = 1,
    UART_CLOCK_PRESCALER_4 = 2,
    UART_CLOCK_PRESCALER_6 = 3,
    UART_CLOCK_PRESCALER_8 = 4,
    UART_CLOCK_PRESCALER_10 = 5,
    UART_CLOCK_PRESCALER_12 = 6,
    UART_CLOCK_PRESCALER_16 = 7,
    UART_CLOCK_PRESCALER_32 = 8,
    UART_CLOCK_PRESCALER_64 = 9,
    UART_CLOCK_PRESCALER_128 = 10,
    UART_CLOCK_PRESCALER_256 = 11,
} lpaurt_clock_prescaler_t;


typedef struct {
    USART_TypeDef *instance;
    uint32_t baudrate;
    uint8_t clock_source;
    uint8_t clock_prescale;
    uint8_t word_length;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t mode;
    uint8_t flow_control;
    uint8_t one_bit_sample;
} uart_config_t;


void uart_init(uart_config_t *config);
void uart_write(uint8_t *data, uint32_t len);

