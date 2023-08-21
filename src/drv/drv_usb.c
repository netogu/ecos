/**
 * @file drv_usb.c
 * @brief USB Peripheral driver
 * @details This file contains the USB Peripheral driver implementation.
 */

#include "drv_usb.h"
#include "drv_common.h"
#include "stm32g474xx.h"
#include "stm32g4xx.h"
#include <stdint.h>

/**
 * @brief Packet Memory Area macros
 */
#define __USBPMA __attribute__((section(".usb_pma")))
#define __USBPMA_BEGIN 0x40006000
#define __MEM2USB(x) ((uint32_t)x - __USBPMA_BEGIN);
#define __USB2MEM(x) ((uint32_t)x + __USBPMA_BEGIN);

struct usb_buffer_table_entry {
  uint16_t addr_tx;
  uint16_t count_tx;
  uint16_t addr_rx;
  uint16_t count_rx;
};

typedef struct {
  uint16_t length;
  uint16_t bytes_sent;
  uint16_t *buffer;
} usb_transfer_t;

/* struct usb_control_state { */
/*   usb_setup_packet_t setup; */
/*   usb_transfer_t tx_transfer; */
/*   usb_transfer_t rx_transfer; */
/* }; */

typedef struct {
  uint32_t *buffer;
  uint32_t size;
  void (*update)(uint8_t ep, uint16_t length);
} usb_endpoint_buffer_t;

__ALIGNED(8)
__USBPMA
__IO struct usb_buffer_table_entry usb_btable[8];

__ALIGNED(2)
__USBPMA
__IO uint8_t EP0_buf[2][64];

#define USB_NUM_ENDPOINTS 8
#define USB_MAX_CTRL_DATA 64

/* static usb_endpoint_buffer_t usb_endpoint_buffers[16] = {0}; */
/**/
/* static struct usb_control_state usb_control = {0}; */
/* static usb_transfer_t usb_transfers[7] = {0}; */
/* static uint8_t usb_active_config = 0; */
/* static uint8_t usb_device_state = 0; // 0 - Default, 1 - Address, 2 -
 * Configured */
/* static uint8_t usb_endpoint_state[USB_NUM_ENDPOINTS] = {0}; */
/* static uint8_t usb_control_data_buffer[USB_MAX_CTRL_DATA] = {0}; */
/**/
/* static void usb_copy_memory(volatile uint16_t *src, volatile uint16_t *dst,
 */
/*                             uint16_t length); */
/**/
/* static void usb_clear_pma(void); */
/**/
/* static void usb_set_endpoint(uint16_t *endpoint, uint16_t value, uint16_t
 * mask); */
/**/
/* static void usb_handle_control(void); */
/**/
/* static void usb_handle_setup(usb_setup_packet_t *setup); */
/**/
/* static void usb_prepare_transfer(usb_transfer_state_t *transfer, */
/*                                  uint16_t *endpoint, uint8_t *tx_buf, */
/*                                  uint16_t *tx_buf_count, uint16_t
 * tx_buf_size); */
/**/

void drv_usb_init(void) {

  // Enable IO Clock & GPIO Port
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;

  // NVIC_SetPriority(USB_LP_IRQn, 8);
  //  NVIC_EnableIRQ(USB_LP_IRQn);

  // Enable macrocell
  USB->CNTR &= ~USB_CNTR_PDWN;

  // Wait 1us for stabilization
  SysTick->LOAD = 200;
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
  // USB->CNTR &= ~USB_CNTR_FRES;
}

static void usb_clear_sram(void) {
  uint8_t *sram = (uint8_t *)__USBPMA_BEGIN;

  for (uint32_t i = 0; i < 1024; i++) {
    sram[i] = 0;
  }
}

void USB_LP_IRQHandler() {
  if (USB->ISTR & USB_ISTR_RESET) {
    // Clear interrupt
    USB->ISTR = ~USB_ISTR_RESET;

    // Clear USB-SRAM
    usb_clear_sram();

    // Prepare Buffer Table
    USB->BTABLE = __MEM2USB(usb_btable);

    usb_btable[0].addr_rx = __MEM2USB(EP0_buf[0]);
    usb_btable[0].addr_tx = __MEM2USB(EP0_buf[1]);
    usb_btable[0].count_tx = 0;
    usb_btable[0].count_rx = (1 << 15) | (1 << 10);

    // Prepare for a setup packet (RX = Valid, TX = NAK)
    // TODO: Write function to setup an endpoint

    // Enable USB functionality and set address to 0
    USB->DADDR = USB_DADDR_EF;
  } else if (USB->ISTR & USB_ISTR_CTR) {
    __BKPT();
  }
}
