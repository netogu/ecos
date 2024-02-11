#pragma once

#include <stdint.h>
#include "stm32g4xx.h"


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


typedef struct {
    uint32_t baudrate;
    uint8_t clock_source;
    uint8_t clock_prescale;
    uint8_t word_length;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t mode;
    uint8_t flow_control;
    uint8_t one_bit_sample;
} lpuart_config_t;


void lpuart_init(lpuart_config_t *config);
void lpuart_write(uint8_t *data, uint32_t len);

