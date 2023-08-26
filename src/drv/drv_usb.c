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

#define Min(x, y) ((x) < (y) ? (x) : (y))

typedef struct {
  uint8_t length;
  uint8_t type;
  uint16_t total_length;
  uint8_t num_interfaces;
  uint8_t config_id;
  uint8_t config_index;
  uint8_t attributes;
  uint8_t max_power;
} usb_config_descriptor_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t interface_id;
  uint8_t alternate_setting;
  uint8_t num_endpoints;
  uint8_t class;
  uint8_t subclass;
  uint8_t protocol;
  uint8_t interface_index;
} usb_interface_descriptor_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t address;
  uint8_t attributes;
  uint16_t max_packet_size;
  uint8_t interval;
} usb_endpoint_descriptor_t;

typedef struct {
  uint8_t bmRequestType;
  uint8_t bRequest;
  union {
    uint16_t wValue;
    struct {
      uint8_t descriptor_index;
      uint8_t descriptor_type;
    };
  };
  uint16_t wIndex;
  uint16_t wLength;
} usb_setup_packet_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint16_t usb_version;
  uint8_t device_class;
  uint8_t device_subclass;
  uint8_t device_protocol;
  uint8_t max_packet_size;
  uint16_t vendor_id;
  uint16_t product_id;
  uint16_t device_release;
  uint8_t manufacturer_index;
  uint8_t product_index;
  uint8_t serial_index;
  uint8_t num_configurations;
} usb_device_descriptor_t;

static const usb_device_descriptor_t device_descriptor = {
    .length = 18,
    .type = 0x01,
    .usb_version = 0x0200,
    .device_class = 0x00,
    .device_subclass = 0x00,
    .device_protocol = 0x00,
    .max_packet_size = 64,
    .vendor_id = 0x0483,
    .product_id = 0x5740,
    .device_release = 0x0100,
    .manufacturer_index = 0x00,
    .product_index = 0x00,
    .serial_index = 0x00,
    .num_configurations = 0x01};

static const usb_config_descriptor_t config_descriptor = {.length = 9,
                                                          .type = 0x02,
                                                          .total_length = 0x20,
                                                          .num_interfaces =
                                                              0x01,
                                                          .config_id = 0x01,
                                                          .config_index = 0x00,
                                                          .attributes = 0x80,
                                                          .max_power = 0x32};

static const usb_interface_descriptor_t interface_descriptor[] = {
    {.length = 7,
     .type = 0x05,
     .interface_id = 0x00,
     .alternate_setting = 0x00,
     .num_endpoints = 0x02,
     .class = 0x0A,
     .subclass = 0x00,
     .protocol = 0x00,
     .interface_index = 0x00}};

static const usb_endpoint_descriptor_t endpoint_descriptor[] = {
    {.length = 7,
     .type = 0x05,
     .address = 0x81,
     .attributes = 0x03,
     .max_packet_size = 0x40,
     .interval = 0xFF},
    {.length = 7,
     .type = 0x05,
     .address = 0x01,
     .attributes = 0x03,
     .max_packet_size = 0x40,
     .interval = 0xFF}};

static uint8_t configuration_buffer[32] = {0};

static void usb_add_to_descriptor(uint8_t *data, uint16_t *offset) {
  uint16_t descriptor_length = data[0];
  for (uint32_t i = 0; i < descriptor_length; i++) {
    configuration_buffer[i + *offset] = data[i];
  }

  *offset += descriptor_length;
}

uint8_t *usb_get_config_descriptor(uint16_t *length) {
  if (configuration_buffer[0] == 0) {
    uint16_t offset = 0;
    usb_add_to_descriptor((uint8_t *)&config_descriptor, &offset);
    usb_add_to_descriptor((uint8_t *)&interface_descriptor[0], &offset);
    usb_add_to_descriptor((uint8_t *)&endpoint_descriptor[0], &offset);
    usb_add_to_descriptor((uint8_t *)&endpoint_descriptor[1], &offset);
  }
  *length = sizeof(configuration_buffer);
  return configuration_buffer;
}

const usb_device_descriptor_t *usb_get_device_descriptor() {
  return &device_descriptor;
}

/**
 * @brief Packet Memory Area macros
 */
#define __USB_MEM __attribute__((section(".usbbuf")))
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
__USB_MEM
__IO static usb_btable_entry_t btable[8] = {0};

__ALIGNED(2)
__USB_MEM
__IO static uint8_t EP0_buf[2][64] = {0};

#define USB_NUM_ENDPOINTS 8
#define USB_MAX_CTRL_DATA 64

typedef struct {
  uint16_t length;
  uint16_t bytes_sent;
  uint16_t *buffer;
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

static usb_control_state_t control_state = {0};

void drv_usb_init(void) {

  // Enable IO Clock & GPIO Port
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;

  NVIC_SetPriority(USB_LP_IRQn, 8);
  NVIC_EnableIRQ(USB_LP_IRQn);

  // Enable macrocell
  USB->CNTR &= ~USB_CNTR_PDWN;

  // Wait 1us for stabilization
  SysTick->LOAD = 200;
  SysTick->VAL = 0;
  SysTick->CTRL = 1;
  while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    ;
  SysTick->CTRL = 0;

  // Enable all interrupts & activate D+ 1.5kohm pull-up (FullSpeed USB)
  USB->CNTR |=
      USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM;
  USB->BCDR |= USB_BCDR_DPPU;

  // Clear the USB Reset (D+ & D- low) to start enumeration
  USB->CNTR &= ~USB_CNTR_FRES;
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
    usb_copy_memory(transfer->buffer + transfer->bytes_sent, buf, *buf_count);
    transfer->bytes_sent += *buf_count;
    usb_set_endpoint(ep, USB_EP_TX_VALID, USB_EP_TX_VALID);
  } else {
    usb_set_endpoint(ep, USB_EP_TX_NAK, USB_EP_TX_VALID);
  }
}

static void usb_handle_setup(usb_setup_packet_t *setup) {
  usb_copy_memory(setup, &control_state.setup, sizeof(usb_setup_packet_t));
  control_state.transfer.length = 0;

  if ((setup->bmRequestType & 0x0F) == 0) {
    ;
    switch (setup->bRequest) {
    case 0x05: { // Set Address
      btable[0].count_tx = 0;
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
      break;
    }
    case 0x06: { // Get Descriptor
    case 0x01: { // Device Descriptor
      const usb_device_descriptor_t *descriptor = usb_get_device_descriptor();
      usb_copy_memory(descriptor, EP0_buf[1], sizeof(usb_device_descriptor_t));
      btable[0].count_tx = sizeof(usb_device_descriptor_t);
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    } break;
    case 0x02: { // Configuration Descriptor
      uint16_t length = 0;
      uint8_t *descriptor = usb_get_config_descriptor(&length);
      control_state.transfer.buffer = descriptor;
      control_state.transfer.bytes_sent = 0;
      control_state.transfer.length = Min(length, setup->wLength);
      usb_prepare_transfer(&control_state.transfer, &USB->EP0R, &EP0_buf[1],
                           &btable[0].count_tx, 64);
    } break;
    }
    }
  }
}

static void usb_handle_control() {
  if (USB->EP0R & USB_EP_CTR_RX) {
    // Received control message
    if (USB->EP0R & USB_EP_SETUP) {
      usb_setup_packet_t *setup = EP0_buf[0];
      usb_handle_setup(setup);
    }

    usb_set_endpoint(&USB->EP0R, USB_EP_RX_VALID,
                     USB_EP_CTR_RX | USB_EP_RX_VALID);
  }
  // times out after enabling USB macrocell
  if (USB->EP0R & USB_EP_CTR_TX) {
    // Just sent a control message
    if (control_state.setup.bRequest == 0x05) {
      USB->DADDR = USB_DADDR_EF | control_state.setup.wValue;
    }

    // Check for ongoing usb_transfers
    if (control_state.transfer.length > 0) {
      if (control_state.transfer.length > control_state.transfer.bytes_sent) {
        usb_prepare_transfer(&control_state.transfer, &USB->EP0R, &EP0_buf[1],
                             &btable[0].count_tx, 64);
      }
    }
    usb_set_endpoint(&USB->EP0R, 0x00, USB_EP_CTR_TX);
  }
}

void USB_LP_IRQHandler() {
  if (USB->ISTR & USB_ISTR_RESET) {
    // Clear interrupt
    USB->ISTR = ~USB_ISTR_RESET;

    // Clear USB-SRAM
    usb_clear_sram();

    // Prepare Buffer Table
    USB->BTABLE = __MEM2USB(btable);

    btable[0].addr_rx = __MEM2USB(EP0_buf[0]);
    btable[0].addr_tx = __MEM2USB(EP0_buf[1]);
    btable[0].count_tx = 0;
    btable[0].count_rx = (1 << 15) | (1 << 10);

    // Prepare for a setup packet (RX = Valid, TX = NAK)
    usb_set_endpoint(&USB->EP0R,
                     USB_EP_CONTROL | USB_EP_RX_VALID | USB_EP_TX_NAK,
                     USB_EP_TYPE_MASK | USB_EP_RX_VALID | USB_EP_TX_VALID);

    // Enable USB functionality and set address to 0
    USB->DADDR = USB_DADDR_EF;
  } else if (USB->ISTR & USB_ISTR_CTR) {
    if ((USB->ISTR & USB_ISTR_EP_ID) == 0) {
      usb_handle_control();
    }
  }
}
