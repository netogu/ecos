#include "board/bsp.h"
#include "tusb.h"

//------------------------------------------------------+
// HRTIM Interrupt Handler
//------------------------------------------------------+

volatile uint32_t g_isr_count_uart_tx = 0;
volatile uint32_t g_isr_count_uart_rx = 0;

void HRTIM1_TIMA_IRQHandler(void) {

  /* REP Interrupt Routine - TIMA */
  if (HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_A].TIMxISR & HRTIM_TIMISR_REP) {
    // Disable continuous mode
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_A].TIMxCR &= ~(HRTIM_TIMCR_CONT);
    // Clear REP interrupt
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_A].TIMxICR |= HRTIM_TIMICR_REPC;
  }

  /* RESET Roll-Over Interupt */
  if (HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_A].TIMxISR & HRTIM_TIMISR_RST) {
    // Togle test pin
    // gpio_pin_set(&io.test_pin1);
    // gpio_pin_clear(&io.test_pin1);
    // Clear RST interrupt
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_A].TIMxICR |= HRTIM_TIMICR_RSTC;
  }
}

void HRTIM1_TIME_IRQHandler(void) {

  /* REP Interrupt Routine - TIME */
  if (HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_E].TIMxISR & HRTIM_TIMISR_REP) {
    // Disable continuous mode
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_E].TIMxCR &= ~(HRTIM_TIMCR_CONT);
    // Clear REP interrupt
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_E].TIMxICR |= HRTIM_TIMICR_REPC;
  }

  /* RESET Roll-Over Interupt */
  if (HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_E].TIMxISR & HRTIM_TIMISR_RST) {
    // Togle test pin
    // gpio_pin_set(&io.test_pin1);
    // gpio_pin_clear(&io.test_pin1);
    // Clear RST interrupt
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_E].TIMxICR |= HRTIM_TIMICR_RSTC;
  }
}

void HRTIM1_TIMF_IRQHandler(void) {

  /* REP Interrupt Routine - TIMF */
  if (HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_F].TIMxISR & HRTIM_TIMISR_REP) {
    // Disable continuous mode
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_F].TIMxCR &= ~(HRTIM_TIMCR_CONT);
    // Clear REP interrupt
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_F].TIMxICR |= HRTIM_TIMICR_REPC;
  }

  /* RESET Roll-Over Interupt */
  if (HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_F].TIMxISR & HRTIM_TIMISR_RST) {
    // Togle test pin
    // gpio_pin_set(&io.test_pin1);
    // gpio_pin_clear(&io.test_pin1);
    // Clear RST interrupt
    HRTIM1->sTimerxRegs[PWM_HRTIM_TIM_F].TIMxICR |= HRTIM_TIMICR_RSTC;
  }
}
//------------------------------------------------------+
// ADC Interrupt Handler
//------------------------------------------------------+

// void ADC1_2_IRQHandler(void) {
//     // Clear the ADC interrupt
//     ADC1->ISR |= ADC_ISR_EOC;
//     // Read the ADC value
//     uint16_t adc_value = ADC1->DR;
//     // Togle test pin
//     gpio_pin_set(&io.test_pin1);
//     gpio_pin_clear(&io.test_pin1);
// }

//--------------------------------------------------------------------+
// USB interrupt Handler
//--------------------------------------------------------------------+

void USB_HP_IRQHandler(void) { tud_int_handler(0); }

void USB_LP_IRQHandler(void) {
  tud_int_handler(0);
}

void USBWakeUp_IRQHandler(void) {
  tud_int_handler(0);
}


//--------------------------------------------------------------------+
// UART interrupt Handler
//--------------------------------------------------------------------+


static inline void uart_receive_byte(uart_t *self) {
  uint8_t data;
  uint16_t next_head;

  data = (uint8_t) self->instance->RDR & 0xFF;
  next_head = (self->rx_head + 1) % UART_RX_BUFFER_SIZE;

  if (next_head != self->rx_tail) {
    // If not full, add data to buffer
    self->rx_buffer[self->rx_head] = data;
    self->rx_head = next_head;
  }
  // Clear RXNE flag by reading data
}

static inline void uart_send_byte(uart_t *self) {
    uint16_t next_tail;

    if (self->tx_head != self->tx_tail) {
      // If not empty, send data
      next_tail = (self->tx_tail + 1) % UART_TX_BUFFER_SIZE;
      self->instance->TDR = self->tx_buffer[self->tx_tail];
      self->tx_tail = next_tail;

    } else {
      // If empty, disable TXE interrupt
      self->instance->CR1 &= ~USART_CR1_TXEIE;
    }
    // ISR Cleared by writing data to TDR

}

void LPUART1_IRQHandler(void) {

  struct board_descriptor *brd = board_get_descriptor();

  // Received a byte on LPUART1
  if (LPUART1->ISR & USART_ISR_RXNE) {
    uart_receive_byte(&brd->lpuart1);
    g_isr_count_uart_rx++;
  }


  // Ready to send byte on LPUART1
  if (LPUART1->ISR & USART_ISR_TXE) {
    uart_send_byte(&brd->lpuart1);
    g_isr_count_uart_tx++;
  }


}

void USART3_IRQHandler(void) {

  struct board_descriptor *brd = board_get_descriptor();

  // Received a byte on USART3
  if (USART3->ISR & USART_ISR_RXNE) {
    uart_receive_byte(&brd->usart3);
    g_isr_count_uart_rx++;
  }

  // Ready to send byte on USART3
  if (USART3->ISR & USART_ISR_TXE) {
    uart_send_byte(&brd->usart3);
    g_isr_count_uart_tx++;
  }

}