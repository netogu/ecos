#pragma once
#include <stdint.h>

#define USB_SETUP_REQUEST_TYPE_STANDARD 0x00
#define USB_SETUP_REQUEST_TYPE_CLASS 0x20
#define USB_SETUP_REQUEST_TYPE_VENDOR 0x40
#define USB_SETUP_REQUEST_TYPE_RESERVED 0x60
#define USB_SETUP_REQUEST_TYPE_DEVICE_TO_HOST 0x80
#define USB_SETUP_REQUEST_TYPE_HOST_TO_DEVICE 0x00
#define USB_SETUP_REQUEST_TYPE_MASK 0x80
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_DEVICE 0x00
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE 0x01
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_ENDPOINT 0x02
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_OTHER 0x03
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_MASK 0x03

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


// void usb_setup_handler(void);
// void usb_config_handler(void);
// void usb_reset_handler(void);
// void usb_sof_handler(void);
// void usb_suspend_handler(void);
// void usb_resume_handler(void);
