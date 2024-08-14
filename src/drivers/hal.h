#ifndef DRIVER_STM32G4_HAL_H
#define DRIVER_STM32G4_HAL_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include "stm32g4/uart.h"



#define UART_RX_BUFFER_SIZE  64
#define UART_TX_BUFFER_SIZE  64

// ----------------------------- USB -----------------------------

extern SemaphoreHandle_t usb_mutex;

void cli_usb_init(void);
int cli_usb_putc(char tx_char);
char cli_usb_getc(void);

// ----------------------------- UART -----------------------------

extern SemaphoreHandle_t uart_mutex;

int cli_uart_init(void);
int cli_uart_putc(char tx_char);
char cli_uart_getc(void);

#endif // DRIVER_STM32G4_HAL_H