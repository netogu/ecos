#pragma once
#include <stdint.h>

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

void drv_usb_init(void);
void usb_setup_handler(void);
void usb_config_handler(void);
void usb_reset_handler(void);
void usb_sof_handler(void);
void usb_suspend_handler(void);
void usb_resume_handler(void);
