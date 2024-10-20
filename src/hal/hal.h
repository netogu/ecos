#ifndef DRIVER_STM32G4_HAL_H
#define DRIVER_STM32G4_HAL_H

#include <stdint.h>
#include "rtos.h"

#include "encoder.h"

#include "stm32g4/rcc.h"
#include "stm32g4/uart.h"
#include "stm32g4/adc.h"
#include "stm32g4/gpio.h"
#include "stm32g4/pwm.h"
#include "stm32g4/spi.h"
#include "stm32g4/uart.h"
#include "stm32g4/cordic.h"



#define NOCHAR                  '\0'

// ----------------------------- USB -----------------------------
#define USB_STR_SERIALNO_LEN    8

extern SemaphoreHandle_t usb_mutex;

void cli_usb_init(void);
int cli_usb_putc(char tx_char);
char cli_usb_getc(void);

// ----------------------------- UART -----------------------------

extern SemaphoreHandle_t uart_mutex;

int cli_uart_init(uart_t *port);
int cli_uart_putc(char tx_char);
char cli_uart_getc(void);
int cli_uart_puts(const char *str);
int cli_printf(const char *format, ...);
uint32_t cli_uart_tx_pending(uart_t *port);


// ----------------------------- TIMERS -----------------------------
// FreeRTOS Stats Timer
void timer_us_init(void);
uint64_t timer_us_get(void);




#endif // DRIVER_STM32G4_HAL_H