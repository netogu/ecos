/**
 * @file drv_usb.c
 * @brief USB Peripheral driver
 * @details This file contains the USB Peripheral driver implementation.
 */

#include <stdint.h>
#include "hardware/stm32g4/usb.h"

void drv_usb_init(void) {

  // Enable IO Clock & GPIO Port
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;

  NVIC_SetPriority(USB_LP_IRQn, 8);
  NVIC_EnableIRQ(USB_LP_IRQn);

  // Enable macrocell
  USB->CNTR &= ~USB_CNTR_PDWN;

  // Wait 1us for stabilization
  for (int i = 0; i < 170; i++) {
    __ASM("nop");
  }

  /* SysTick->LOAD = 200; */
  /* SysTick->VAL = 0; */
  /* SysTick->CTRL = 1; */
  /* while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) */
  /*   ; */
  /* SysTick->CTRL = 0; */

  // Enable all interrupts & activate D+ 1.5kohm pull-up (FullSpeed USB)
  USB->CNTR |=
      USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM;
  USB->BCDR |= USB_BCDR_DPPU;

  // Clear the USB Reset (D+ & D- low) to start enumeration
  USB->CNTR &= ~USB_CNTR_FRES;
}

/**
 * @brief Packet Memory Area macros
 */
#define __USB_TABLE __attribute__((section(".usbtable")))
#define __USB_BUF __attribute__((section(".usbbuf")))
#define __USBBUF_BEGIN 0x40006000
#define __MEM2USB(x) ((uint32_t)x - __USBBUF_BEGIN);
#define __USB2MEM(x) ((uint32_t)x + __USBBUF_BEGIN);

typedef struct {
  uint16_t addr_tx;
  uint16_t count_tx;
  uint16_t addr_rx;
  uint16_t count_rx;
} usb_btable_entry_t;

__ALIGNED(8)
__USB_TABLE
__IO static usb_btable_entry_t btable[8] = {0};

__ALIGNED(2)
__USB_BUF
__IO static uint8_t ep0_buff_tx[64] = {0};
__ALIGNED(2)
__USB_BUF
__IO static uint8_t ep0_buff_rx[64] = {0};

#define USB_MAX_CTRL_DATA 64

typedef struct {
  uint16_t length;
  uint16_t bytes_sent;
  uint8_t *buffer;
} usb_transfer_t;

typedef struct {
  usb_setup_packet_t setup;
  usb_transfer_t transfer;
} usb_control_state_t;

typedef struct {
  uint32_t *buffer;
  uint32_t size;
  void (*update)(uint8_t ep, uint16_t length);
} usb_endpoint_buffer_t;

static void usb_set_endpoint(__IO uint16_t *ep, uint16_t value, uint16_t mask) {
  uint16_t toggle = 0b0111000001110000;
  uint16_t rc_w0 = 0b1000000010000000;
  uint16_t rw = 0b0000011100001111;

  uint16_t wr0 = rc_w0 & (~mask | value);
  uint16_t wr1 = (mask & toggle) & (*ep ^ value);
  uint16_t wr2 = rw & ((*ep & ~mask) | value);

  *ep = wr0 | wr1 | wr2;
}

static void usb_clear_sram(void) {
  uint8_t *buffer = (uint8_t *)__USBBUF_BEGIN;

  for (uint32_t i = 0; i < 1024; i++) {
    buffer[i] = 0;
  }
}

static void usb_copy_memory(uint16_t *source, uint16_t *target,
                            uint16_t length) {
  for (uint32_t i = 0; i < length / 2; i++) {
    target[i] = source[i];
  }
  if (length % 2 == 1) {
    ((uint8_t *)target)[length - 1] = ((uint8_t *)source)[length - 1];
  }
}

static void usb_prepare_transfer(usb_transfer_t *transfer, uint16_t *ep,
                                 uint8_t *buf, uint16_t *buf_count,
                                 uint16_t buf_size) {
  *buf_count = Min(buf_size, transfer->length - transfer->bytes_sent);

  if (*buf_count > 0) {
    usb_copy_memory((uint16_t *)(transfer->buffer + transfer->bytes_sent),
                    (uint16_t *)buf, *buf_count);
    transfer->bytes_sent += *buf_count;
    usb_set_endpoint(ep, USB_EP_TX_VALID, USB_EP_TX_VALID);
  } else {
    usb_set_endpoint(ep, USB_EP_TX_NAK, USB_EP_TX_VALID);
  }
}
