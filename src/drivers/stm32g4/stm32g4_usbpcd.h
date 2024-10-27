#pragma once
#ifndef STM32G4_USBPCD_H
#define STM32G4_USBPCD_H

#include "stm32g4_common.h"

//--------------------------------------------------------------------+
// USB Peripheral Controller Memory Map
//--------------------------------------------------------------------+
#define USB_EP0_BUFF_SIZE 64
#define USB_PMA_START 0x40006000
#define MEM2PMA(x) ((uint32_t)x - USB_PMA_START);
#define PMA2MEM(x) ((uint32_t)x + USB_PMA_START);

typedef struct {

  struct {
    __IO uint16_t addr_tx;
    __IO uint16_t count_tx;
    __IO uint16_t addr_rx;
    __IO uint16_t count_rx;
  } usb_btable[8];

  struct {
    __IO uint8_t tx[USB_EP0_BUFF_SIZE];
    __IO uint8_t rx[USB_EP0_BUFF_SIZE];
  } ep0_buffer;

} usbpd_pma_t;
#define USB_PMA ((usbpd_pma_t *)USB_PMAADDR)


int usbpcd_init(void);
void usbpcd_clear_pma(void);
void usbpcd_set_endpoint(__IO uint16_t *ep, uint16_t value, uint16_t mask);
void usbpcd_copy_memory(uint16_t *source, uint16_t *target, uint16_t length);


#endif // STM32G4_USBPCD_H