#include "board/bsp.h"
#include "tusb.h"

//------------------------------------------------------+
// HRTIM Interrupt Handler
//------------------------------------------------------+

void HRTIM1_TIMA_IRQHandler(void) {

  /* REP Interrupt Routine - TIMA */
  if (HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxISR & HRTIM_TIMISR_REP) {
    // Disable continuous mode
    HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxCR &= ~(HRTIM_TIMCR_CONT);
    // Clear REP interrupt
    HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxICR |= HRTIM_TIMICR_REPC;
  }

  /* RESET Roll-Over Interupt */
  if (HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxISR & HRTIM_TIMISR_RST) {
    // Togle test pin
    // gpio_pin_set(&io.test_pin1);
    // gpio_pin_clear(&io.test_pin1);
    // Clear RST interrupt
    HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxICR |= HRTIM_TIMICR_RSTC;
  }
}

void HRTIM1_TIME_IRQHandler(void) {

  /* REP Interrupt Routine - TIME */
  if (HRTIM1->sTimerxRegs[HRTIM_TIM_E].TIMxISR & HRTIM_TIMISR_REP) {
    // Disable continuous mode
    HRTIM1->sTimerxRegs[HRTIM_TIM_E].TIMxCR &= ~(HRTIM_TIMCR_CONT);
    // Clear REP interrupt
    HRTIM1->sTimerxRegs[HRTIM_TIM_E].TIMxICR |= HRTIM_TIMICR_REPC;
  }

  /* RESET Roll-Over Interupt */
  if (HRTIM1->sTimerxRegs[HRTIM_TIM_E].TIMxISR & HRTIM_TIMISR_RST) {
    // Togle test pin
    // gpio_pin_set(&io.test_pin1);
    // gpio_pin_clear(&io.test_pin1);
    // Clear RST interrupt
    HRTIM1->sTimerxRegs[HRTIM_TIM_E].TIMxICR |= HRTIM_TIMICR_RSTC;
  }
}

void HRTIM1_TIMF_IRQHandler(void) {

  /* REP Interrupt Routine - TIMF */
  if (HRTIM1->sTimerxRegs[HRTIM_TIM_F].TIMxISR & HRTIM_TIMISR_REP) {
    // Disable continuous mode
    HRTIM1->sTimerxRegs[HRTIM_TIM_F].TIMxCR &= ~(HRTIM_TIMCR_CONT);
    // Clear REP interrupt
    HRTIM1->sTimerxRegs[HRTIM_TIM_F].TIMxICR |= HRTIM_TIMICR_REPC;
  }

  /* RESET Roll-Over Interupt */
  if (HRTIM1->sTimerxRegs[HRTIM_TIM_F].TIMxISR & HRTIM_TIMISR_RST) {
    // Togle test pin
    // gpio_pin_set(&io.test_pin1);
    // gpio_pin_clear(&io.test_pin1);
    // Clear RST interrupt
    HRTIM1->sTimerxRegs[HRTIM_TIM_F].TIMxICR |= HRTIM_TIMICR_RSTC;
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

void LPUART1_IRQHandler(void) {
  uint16_t next_head;
  uint8_t data;
  struct board_descriptor *brd = board_get_handler();

  // Received a byte
  if (LPUART1->ISR & USART_ISR_RXNE) {
    data = (uint8_t) LPUART1->RDR & 0xFF;
    next_head = (brd->lpuart1.rx_head + 1) % UART_RX_BUFFER_SIZE;

    if (next_head != brd->lpuart1.rx_tail) {
      // If not full, add data to buffer
      brd->lpuart1.rx_buffer[brd->lpuart1.rx_head] = data;
      brd->lpuart1.rx_head = next_head;
    }
    // Clear RXNE flag by reading data
  }

  // Empty data register
  if (LPUART1->ISR & USART_ISR_TXE) {
    if (brd->lpuart1.tx_head != brd->lpuart1.tx_tail) {
      // If not empty, send data
      LPUART1->TDR = brd->lpuart1.tx_buffer[brd->lpuart1.tx_tail];
      brd->lpuart1.tx_tail = (brd->lpuart1.tx_tail + 1) % UART_TX_BUFFER_SIZE;
    } else {
      // If empty, disable TXE interrupt
      LPUART1->CR1 &= ~USART_CR1_TXEIE;
    }
    // ISR Cleared by writing data to TDR
  }
}