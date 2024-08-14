#include "hal.h"
#include <FreeRTOS.h>
#include <semphr.h>

#define NOCHAR '\0'

static StaticSemaphore_t uart_mutex_buffer;
SemaphoreHandle_t uart_mutex;

static volatile uint32_t *serial_port;


int cli_uart_init(void) {
    

    uart_mutex = xSemaphoreCreateMutexStatic(&uart_mutex_buffer);
    if (uart_mutex == NULL) {
        // Error Creating UART Mutex
        while(1);
    }

    return 0;
}

int cli_uart_putc(char tx_char) {
    int status = 0;
    if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
        status = lpuart_write(serial_port, &tx_char, 1);
        xSemaphoreGive(uart_mutex);
    }
    return status;
}

char cli_uart_getc(void) {
    char readchar = NOCHAR;
    if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
        readchar = lpuart_read(serial_port, &readchar, 1);
        xSemaphoreGive(uart_mutex);
    }
    return readchar;
}