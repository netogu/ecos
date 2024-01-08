/**
 * @file      uart.c
 * @author    Shinichiro Nakamura (CuBeatSystems)
 * ===============================================================
 * MicroShell (Version 0.0.2)
 * Copyright (c) 2016, 2017 Shinichiro Nakamura (CuBeatSystems)
 * ===============================================================
 * The MIT License : https://opensource.org/licenses/MIT
 *
 * Copyright (c) 2016, 2017 Shinichiro Nakamura (CuBeatSystems)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "chip.h"
#include <string.h>

#define UART_RB_SIZE    (64)
#define CONFIG_BAUDRATE (115200)

static RINGBUFF_T txring, rxring;
static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];

void UART0_IRQHandler(void)
{
    Chip_UART_IRQRBHandler(LPC_USART0, &rxring, &txring);
}

void uart_init()
{
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
    Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 4);
    Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 0);
    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

    Chip_Clock_SetUARTClockDiv(1);
    Chip_UART_Init(LPC_USART0);
    Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
    Chip_Clock_SetUSARTNBaseClockRate((115200 * 16), true);
    Chip_UART_SetBaud(LPC_USART0, CONFIG_BAUDRATE);
    Chip_UART_Enable(LPC_USART0);
    Chip_UART_TXEnable(LPC_USART0);
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);
    Chip_UART_IntEnable(LPC_USART0, UART_INTEN_RXRDY);
    Chip_UART_IntDisable(LPC_USART0, UART_INTEN_TXRDY);
    NVIC_EnableIRQ(UART0_IRQn);
}

uint8_t uart_getc(void)
{
    uint8_t c = 0;
    while (1) {
        int bytes = Chip_UART_ReadRB(LPC_USART0, &rxring, &c, 1);
        if (bytes > 0) {
            return c;
        }
    }
}

void uart_putc(uint8_t c)
{
    while (1) {
        int bytes = Chip_UART_SendRB(LPC_USART0, &txring, (const uint8_t *) &c, 1);
        if (bytes > 0) {
            return;
        }
    }
}

void uart_puts(char *str)
{
    while (*str) {
        uart_putc(*str++);
    }
}

