/**
 * @file drv_usb.c
 * @brief USB Peripheral driver
 * @details This file contains the USB Peripheral driver implementation.
 */

#include "drv_common.h"
#include "stm32g4xx.h"
#include <stdint.h>

/**
 * @brief Packet Memory Area macros
 */
#define __USBPMA __attribute__((section(".usb_pma")))
#define __USBPMA_BEGIN 0x40006000
#define __MEM2USB(x) ((uint32_t)x - __USBPMA_BEGIN);
#define __USB2MEM(x) ((uint32_t)x + __USBPMA_BEGIN);

typedef struct {
  uint16_t addr_tx;
  uint16_t count_tx;
  uint16_t addr_rx;
  uint16_t count_rx;
} usb_btable_entry_t;

typedef struct {
  uint16_t length;
  uint16_t bytes_sent;
  uint16_t *buffer;
} usb_transfer_state_t;

typedef struct {
  usb_setup_packet_t setup;
  usb_transfer_state_t tx;
  usb_transfer_state_t rx;
} usb_control_state_t;

typedef struct {
  uint8_t *buffer;
  uint8_t size;
  void (*update)(uint8_t ep, uint16_t length);
} usb_ep_buffer_t;

__ALIGNED(8)
__USBPMA
__IO usb_btable_entry_t usb_btable[8];

__ALIGNED(2)
__USBPMA
__IO static uint8_t usb_ep0_buffer[2][64] = {0};

static usb_ep_buffer_t usb_ep_buffers[16] = {0};

static usb_control_state_t usb_control_state = {0};
static usb_transfer_state_t usb_transfers[7] = {0};
static uint8_t usb_active_config = 0;
static uint8_t usb_device_state = 0; // 0 - Default, 1 - Address, 2 - Configured
static uint8_t usb_endpoint_state[USB_NUM_ENDPOINTS] = {0};
static uint8_t usb_control_data_buffer[USB_MAX_CTRL_DATA] = {0};

static void usb_copy_memory(volatile uint16_t *src, volatile uint16_t *dst,
                            uint16_t length);

static void usb_clear_pma(void);

static void usb_set_endpoint(uint16_t *endpoint, uint16_t value, uint16_t mask);

static void usb_handle_control(void);

static void usb_handle_setup(usb_setup_packet_t *setup);

static void usb_prepare_transfer(usb_transfer_state_t *transfer,
                                 uint16_t *endpoint, uint8_t *tx_buf,
                                 uint16_t *tx_buf_count, uint16_t tx_buf_size);

void usb_init(void) {
  NVIC_SetPriority(USB_LP_IRQn, 8);
  NVIC_EnableIRQ(USB_LP_IRQn);
  NVIC_SetPriority(USB_HP_IRQn, 8);
  NVIC_EnableIRQ(USB_HP_IRQn);

  usb_control_state.rx.buffer = usb_control_data_buffer;

  // Enable macrocell
  USB->CNTR &= ~USB_CNTR_PDWN;

  // Wait 1us for stabilization
  SysTick->LOAD = 100;
  SysTick->VAL = 0;
  SysTick->CTRL = 1;
  while (~(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
    ;
  SysTick->CTRL = 0;

  // Enable all interrupts & activate D+ 1.5kohm pull-up (FullSpeed USB)
  USB->CNTR |=
      USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM;
  USB->BCDR |= USB_BCDR_DPPU;

  // Clear the USB Reset (D+ & D- low) to start enumeration
  USB->CNTR &= ~USB_CNTR_FRES;
}

void USB_HP_IRQHandler() {
  if (USB->ISTR & USB_ISTR_CTR) {

    uint8_t ep_id = USB->ISTR & USB_ISTR_EP_ID;
    __IO uint16_t *ep_reg = &USB->EP0R + ep_id * 2;

    if (ep_id > 0 && ep_id < 8) {

      if (*ep_reg & USB_EP_CTR_RX) {
        // Successful EPn host OUT device RX transfer
        usb_transfer_state_t *transfer = &usb_transfers[ep_id - 1];
        if (transfer->length > 0) {
          if (transfer->length > transfer->bytes_sent) {
            usb_prepare_transfer(transfer, ep_reg, uint8_t * tx_buf,
                                 uint16_t * tx_buf_count, uint16_t tx_buf_size)
          }
        }
      }
    }
  }
}
