#include "drivers/stm32g4/uart.h"

void uart_init(uart_config_t *config) {

  USART_TypeDef *uart_regs = config->instance;

  uint32_t tmpreg = 0x00;
  // Configure Clocks
  if (config->instance == USART1) {
    RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL_Msk);
    RCC->CCIPR |= (config->clock_source << RCC_CCIPR_USART1SEL_Pos);
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
    tmpreg = RCC->APB2ENR;
  } else if (config->instance == USART2) {
    RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL_Msk);
    RCC->CCIPR |= (config->clock_source << RCC_CCIPR_USART2SEL_Pos);
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN_Msk;
    tmpreg = RCC->APB1ENR1;
  } else if (config->instance == USART3) {
    RCC->CCIPR &= ~(RCC_CCIPR_USART3SEL_Msk);
    RCC->CCIPR |= (config->clock_source << RCC_CCIPR_USART3SEL_Pos);
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN_Msk;
    tmpreg = RCC->APB1ENR1;
  } else if (config->instance == UART4) {
    RCC->CCIPR &= ~(RCC_CCIPR_UART4SEL_Msk);
    RCC->CCIPR |= (config->clock_source << RCC_CCIPR_UART4SEL_Pos);
    RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN_Msk;
    tmpreg = RCC->APB1ENR1;
  } else if (config->instance == UART5) {
    RCC->CCIPR &= ~(RCC_CCIPR_UART5SEL_Msk);
    RCC->CCIPR |= (config->clock_source << RCC_CCIPR_UART5SEL_Pos);
    RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN_Msk;
    tmpreg = RCC->APB1ENR1;
  } else if (config->instance == LPUART1) {
    RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL_Msk);
    RCC->CCIPR |= (config->clock_source << RCC_CCIPR_LPUART1SEL_Pos);
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN_Msk;
    tmpreg = RCC->APB1ENR2;
  } else {
    return;
  }
  (void)tmpreg;

  // Set uart clock prescaler
  uart_regs->PRESC = 0x00;
  uart_regs->PRESC |= (config->clock_prescale & 0x0F);

  // Disable uart
  uart_regs->CR1 &= ~(USART_CR1_UE);
  uart_regs->CR1 = 0x00;

  // Set baudrate
  // TODO: Calculate baudrate based on prescaler and clock source
  uint32_t usartdiv = SystemCoreClock / config->baudrate * 256;
  uart_regs->BRR = usartdiv & 0x0FFFFF; // 20 bits

  // Set data bits
  // 8 bits by default
  uart_regs->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
  if (config->word_length == UART_DATA_BITS_9) {
    uart_regs->CR1 |= (USART_CR1_M0);
  } else if (config->word_length == UART_DATA_BITS_7) {
    uart_regs->CR1 |= (USART_CR1_M1);
  }

  // Set parity
  uart_regs->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
  if (config->parity != UART_PARITY_NONE) {
    uart_regs->CR1 |= (USART_CR1_PCE);
    uart_regs->CR1 |= (config->parity << USART_CR1_PS_Pos);
  } // else uart_PARITY_NONE

  // Set stop bits
  uart_regs->CR2 &= ~(USART_CR2_STOP);
  uart_regs->CR2 |= (config->stop_bits << USART_CR2_STOP_Pos);

  // Set flow control
  uart_regs->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
  if (config->flow_control & UART_FLOW_CONTROL_RTS) {
    uart_regs->CR3 |= (USART_CR3_RTSE);
  } else if (config->flow_control & UART_FLOW_CONTROL_CTS) {
    uart_regs->CR3 |= (USART_CR3_CTSE);
  } else if (config->flow_control & UART_FLOW_CONTROL_RTS_CTS) {
    uart_regs->CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);
  } // else uart_FLOW_CONTROL_NONE

  // Set mode
  uart_regs->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);
  if (config->mode == UART_MODE_RX) {
    uart_regs->CR1 |= (USART_CR1_RE);
  } else if (config->mode == UART_MODE_TX) {
    uart_regs->CR1 |= (USART_CR1_TE);
  } else if (config->mode == UART_MODE_RX_TX) {
    uart_regs->CR1 |= (USART_CR1_RE | USART_CR1_TE);
  } // else uart_MODE_NONE

  // Enable uart
  uart_regs->CR1 |= (USART_CR1_UE);
}

#define uart_write_byte_blocking(data)                                         \
  do {                                                                         \
    while (!(uart_regs->ISR & USART_ISR_TC))                                   \
      ;                                                                        \
    uart_regs->TDR = data;                                                     \
  } while (0);
#define uart_write_byte(data)                                                  \
  do {                                                                         \
    uart_regs->TDR = data;                                                     \
  } while (0);

void uart_write(uint8_t *data, uint32_t len) {

  for (uint32_t i = 0; i < len; i++) {
    // uart_write_byte_blocking(data[i]);
  }
}
