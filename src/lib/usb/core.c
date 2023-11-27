/**
 * @file drv_usb.c
 * @brief USB Peripheral driver
 * @details This file contains the USB Peripheral driver implementation.
 */

#include <stdint.h>
#include "lib/usb/descriptors.h"

#define Min(x, y) ((x) < (y) ? (x) : (y))

#define USB_SELF_POWERED 0
#define USB_NUM_INTERFACES 1
#define USB_NUM_ENDPOINTS 2

static uint8_t usb_active_config = 0;
static uint8_t usb_device_state = 0;
static uint8_t usb_endpoint_state[USB_NUM_ENDPOINTS] = {0};




static uint8_t configuration_buffer[32] = {0};

const usb_device_descriptor_t *usb_get_device_descriptor() {
  return &device_descriptor;
}

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


static usb_control_state_t control_state;


void usb_reset_class(uint8_t interface, uint8_t alt_id){};


usb_get_status_req_handler(){
  ep0_buff_tx[0] = USB_SELF_POWERED;
  ep0_buff_tx[1] = 0x00;
  btable[0].count_tx = 2;
  usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

usb_clear_feature_req_handler(){
  usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
}

usb_set_feature_req_handler(){
  btable[0].count_tx = 0;
  usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

usb_set_addr_req_handler(){
  btable[0].count_tx = 0;
  usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}


usb_set_descriptor_req_handler(){
  // Allows the Host to alter the descriptor. Not supported
  usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
}

usb_get_config_req_handler(){
  if (usb_device_state == 1 || usb_device_state == 2) {
    if (usb_device_state == 1) {
      usb_active_config = 0;
    }

    ep0_buff_tx[0] = usb_active_config;
    btable[0].count_tx = 1;
    usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
  } else {
    usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
  }
}

usb_set_config_req_handler(){
  if (usb_device_state == 1 || usb_device_state == 2) {
    btable[0].count_tx = 0;
    switch (setup->wValue & 0xFF) {
    case 0:
      usb_device_state = 1;
      usb_active_config = 0;
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
      break;
    case 1:
      usb_device_state = 2;
      usb_active_config = 1;
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
      break;
    default:
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      break;
    }

    if (usb_device_state == 2) {
      usb_set_endpoint(&USB->EP1R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usb_set_endpoint(&USB->EP2R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usb_set_endpoint(&USB->EP3R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usb_set_endpoint(&USB->EP4R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usb_set_endpoint(&USB->EP5R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usb_set_endpoint(&USB->EP6R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usb_set_endpoint(&USB->EP7R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
    }
  } else {
    usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
  }
}

