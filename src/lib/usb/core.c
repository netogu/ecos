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

//--------------------------------------------------------------------+
// Standard USB Requests
//--------------------------------------------------------------------+

#define USB_REQ_TYPE_STANDARD 0x00
#define USB_REQ_TYPE_CLASS 0x20
#define USB_REQ_TYPE_VENDOR 0x40
#define USB_REQ_TYPE_MASK 0x60

#define USB_REQ_GET_STATUS 0x00
#define USB_REQ_CLEAR_FEATURE 0x01
#define USB_REQ_SET_FEATURE 0x03
#define USB_REQ_SET_ADDRESS 0x05
#define USB_REQ_GET_DESCRIPTOR 0x06
#define USB_REQ_SET_DESCRIPTOR 0x07
#define USB_REQ_GET_CONFIGURATION 0x08
#define USB_REQ_SET_CONFIGURATION 0x09
#define USB_REQ_GET_INTERFACE 0x0A
#define USB_REQ_SET_INTERFACE 0x0B
#define USB_REQ_SYNCH_FRAME 0x0C



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

usb_get_device_descriptor_req_handler(setup){

  switch (setup->descriptor_type) {
    case 0x01: { // Device Descriptor
    const usb_device_descriptor_t *descriptor = usb_get_device_descriptor();
    usb_copy_memory((uint16_t *)descriptor, (uint16_t *)ep0_buff_tx,
                  sizeof(usb_device_descriptor_t));
    btable[0].count_tx = sizeof(usb_device_descriptor_t);
    usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    } break;
  case 0x02: { // Configuration Descriptor
    uint16_t length = 0;
    uint8_t *descriptor = usb_get_config_descriptor(&length);
    control_state.transfer.buffer = descriptor;
    control_state.transfer.bytes_sent = 0;
    control_state.transfer.length = Min(length, setup->wLength);
    usb_prepare_transfer(&control_state.transfer, &USB->EP0R, &ep0_buff_tx,
                        &btable[0].count_tx, 64);
    } break;
  case 0x06: // Device Qualifier Descriptor
    usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
    break;
  }
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

static void usb_handle_setup(usb_setup_packet_t *setup) {

  usb_copy_memory((uint16_t *)setup, (uint16_t *)&control_state.setup,
                  sizeof(usb_setup_packet_t));
  control_state.transfer.length = 0;

  if ((setup->bmRequestType & 0x0F) == 0) { // Device Requests

    switch (setup->bRequest) {
    case USB_REQ_GET_STATUS:
      usb_get_status_req_handler(setup);
      break;
    case USB_REQ_CLEAR_FEATURE:
      usb_clear_feature_req_handler(setup);
      break;
    case USB_REQ_SET_FEATURE:
      usb_set_feature_req_handler(setup);
      break;
    case USB_REQ_SET_ADDRESS: 
      usb_set_addr_req_handler(setup);
      break;
    case USB_REQ_GET_DESCRIPTOR: 
      usb_get_descriptor_req_handler(setup);
      break;
    case USB_REQ_SET_DESCRIPTOR:
      usb_set_descriptor_req_handler(setup);
      break;
    case USB_REQ_GET_CONFIGURATION: 
      usb_get_config_req_handler(setup);
      break;
    case USB_REQ_SET_CONFIGURATION: 
      usb_set_config_req_handler(setup);
      break;
    }
    
  //TODO: Add support for other requests
  } else if ((setup->bmRequestType & 0x0F) == 0x01) { // Interface requests
    switch (setup->bRequest) {
    case USB_REQ_GET_STATUS: 
      if (usb_device_state == 2) {
        ep0_buff_tx[0] = 0x00;
        ep0_buff_tx[1] = 0x00;
        btable[0].count_tx = 2;
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
      } else {
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x01: // Clear Feature
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      break;
    case 0x03: // Set Feature
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      break;
    case 0x0A: // Get Interface
      if (usb_device_state == 2 && setup->wIndex < USB_NUM_INTERFACES) {
        ep0_buff_tx[0] = 0x00;
        btable[0].count_tx = 1;
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
      } else {
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x0B: // Set Interface /!\ USB InANutshell is wrong here, it confuses
               // the decimal value (11) with the hex one (0x0B)
      if (usb_device_state == 2 && setup->wIndex < USB_NUM_INTERFACES) {
        btable[0].count_tx = 0;
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        usb_reset_class(setup->wIndex, setup->wValue);
      } else {
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    }

  } else if ((setup->bmRequestType & 0x0F) == 0x02) { // Endpoint requests
    switch (setup->bRequest) {
    case 0x00: // Get Status
      if ((usb_device_state == 2 ||
           (usb_device_state == 1 && setup->wIndex == 0x00)) &&
          setup->wIndex < USB_NUM_ENDPOINTS) {
        if (setup->wValue == 0x00) {
          ep0_buff_tx[0] = usb_endpoint_state[setup->wIndex];
          ep0_buff_tx[0] = 0x00;
          btable[0].count_tx = 2;
          usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        } else {
          usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        }
      } else {
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x01: // Clear Feature
      if ((usb_device_state == 2 ||
           (usb_device_state == 1 && setup->wIndex == 0x00)) &&
          setup->wIndex < USB_NUM_ENDPOINTS) {
        if (setup->wValue == 0x00) {
          usb_endpoint_state[setup->wIndex] = 0;
          btable[0].count_tx = 0;
          usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        } else {
          usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        }
      } else {
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x03: // Set Feature
      if ((usb_device_state == 2 ||
           (usb_device_state == 1 && setup->wIndex == 0x00)) &&
          setup->wIndex < USB_NUM_ENDPOINTS) {
        if (setup->wValue == 0x00) {
          usb_endpoint_state[setup->wIndex] = 1;
          btable[0].count_tx = 0;
          usb_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        } else {
          usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        }
      } else {
        usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x0C: // Sync Frame /!\ USB InANutshell is wrong here again, as it
               // confuses the decimal value (12) with the hex one (0x0C)
      usb_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      break;
    }
  }
}
