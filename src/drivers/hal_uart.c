#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "hal.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include "stm32g4/uart.h"

#define NOCHAR '\0'

static StaticSemaphore_t uart_mutex_buffer;
SemaphoreHandle_t uart_mutex;

static uart_t *serial_port = NULL;


int cli_uart_init(uart_t *port) {
    
    uart_mutex = xSemaphoreCreateMutexStatic(&uart_mutex_buffer);
    if (uart_mutex == NULL) {
        // Error Creating UART Mutex
        while(1);
    }

    serial_port = port;

    return 0;
}

int cli_uart_putc(char tx_char) {
    
    int status = 0;
    if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
        status = uart_write(serial_port, (uint8_t *) &tx_char, 1);
        xSemaphoreGive(uart_mutex);
    }
    return status;
}

int cli_uart_puts(const char *str) {
    int status = 0;
    if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
        status = uart_write(serial_port, (uint8_t *) str, strlen(str));
        xSemaphoreGive(uart_mutex);
    }
    return status;
}

int cli_printf(const char *format, ...) {

    int status = 0;
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
        status = uart_write(serial_port, (uint8_t *) buffer, strlen(buffer));
        xSemaphoreGive(uart_mutex);
    }
    return status;
}


char cli_uart_getc(void) {
    int status = 0;
    char readchar = NOCHAR;
    if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
        status = uart_read(serial_port, (uint8_t *) &readchar, 1);
        xSemaphoreGive(uart_mutex);
    }
    return readchar;
}