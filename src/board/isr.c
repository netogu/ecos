#include "bsp.h"
#include "tusb.h"

//------------------------------------------------------+
// HRTIM Interrupt Handler
//------------------------------------------------------+

struct global_isr_counter {
  uint32_t usart3_tx;
  uint32_t usart3_rx;
  uint32_t usart3_idle;
  uint32_t lpuart1_tx;
  uint32_t lpuart1_rx;
  uint32_t lpuart1_idle;
} g_isr_counter;


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

  data = (uint8_t) self->instance->RDR & 0xFF;
  uart_fifo_push(&self->rx_fifo, data);
  // Clear RXNE flag by reading data
}

static inline void uart_send_byte(uart_t *self) {
  uint16_t next_tail;

  if (self->tx_fifo.size == 0) {
    // If empty, disable TXE interrupt
    self->instance->CR1 &= ~USART_CR1_TXEIE;
    return;
  } else {
    // If not empty, send data
    uint8_t tx_byte;
    uart_fifo_pop(&self->tx_fifo, &tx_byte);
    self->instance->TDR = tx_byte;
    // ISR Cleared by writing data to TDR
  }

}

void LPUART1_IRQHandler(void) {

  struct board_descriptor *brd = board_get_descriptor();

  // Received a byte on LPUART1
  if (LPUART1->ISR & USART_ISR_RXNE) {
    // uart_receive_byte(&brd->lpuart1);
    g_isr_counter.lpuart1_rx++;
  }


  // Ready to send byte on LPUART1
  if (LPUART1->ISR & USART_ISR_TXE) {
    // uart_send_byte(&brd->lpuart1);
    // uart_set_tx_dma_transfer(&brd->lpuart1, DMA1_Channel3);
    g_isr_counter.lpuart1_tx++;
  }

  // Idle line detected
  if (LPUART1->ISR & USART_ISR_IDLE) {
    // Clear the IDLE flag
    LPUART1->ICR |= USART_ICR_IDLECF;
    uart_service_rx_dma(&brd->lpuart1);
    g_isr_counter.lpuart1_idle++;
  }


}

void USART3_IRQHandler(void) {

  struct board_descriptor *brd = board_get_descriptor();

  // Received a byte on USART3
  if (USART3->ISR & USART_ISR_RXNE) {
    uart_receive_byte(&brd->usart3);
    g_isr_counter.usart3_rx++;
  }

  // Ready to send byte on USART3
  if (USART3->ISR & USART_ISR_TXE) {
    uart_send_byte(&brd->usart3);
    g_isr_counter.usart3_tx++;
  }

  // Idle line detected
  if (USART3->ISR & USART_ISR_IDLE) {
    // Clear the IDLE flag
    USART3->ICR |= USART_ICR_IDLECF;
    uart_service_rx_dma(&brd->usart3);
    g_isr_counter.usart3_idle++;
  }

}

void DMA1_Channel2_IRQHandler(void) {

  struct board_descriptor *brd = board_get_descriptor();

  if (DMA1->ISR & DMA_ISR_TCIF2) {
    // Service the RX DMA buffer
    DMA1->IFCR |= DMA_IFCR_CTCIF2;
    uart_service_rx_dma(&brd->lpuart1);
  }

  if (DMA1->ISR & DMA_ISR_HTIF2) {
    // Service the RX DMA buffer
    DMA1->IFCR |= DMA_IFCR_CHTIF2;
    uart_service_rx_dma(&brd->lpuart1);
  }

}

void DMA1_Channel3_IRQHandler(void) {
  
    struct board_descriptor *brd = board_get_descriptor();
  
    if (DMA1->ISR & DMA_ISR_TCIF3) {
      // Service the TX DMA buffer
      DMA1->IFCR |= DMA_IFCR_CTCIF3;
      // Advance the tail pointer to account for the data sent
      brd->lpuart1.tx_fifo.tail = (brd->lpuart1.tx_fifo.tail + brd->lpuart1.tx_dma_current_transfer_size) % (UART_BUFFER_SIZE);
      // Update the current transfer size
      brd->lpuart1.tx_dma_current_transfer_size = 0;
      // Transfer what may be left in the buffer
      uart_start_dma_tx_transfer(&brd->lpuart1, DMA1_Channel3);
    }

    if (DMA1->ISR & DMA_ISR_HTIF3) {
      // Service the TX DMA buffer
      DMA1->IFCR |= DMA_IFCR_CHTIF3;
    }

    if (DMA1->ISR & DMA_ISR_TEIF3) {
      // Service the TX DMA buffer
      DMA1->IFCR |= DMA_IFCR_CTEIF3;
    }
  
}