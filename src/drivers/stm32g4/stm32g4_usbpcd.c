#include "stm32g4_usbpcd.h"


// #define __USB_TABLE __attribute__((section(".usbtable")))
// #define __USB_BUF __attribute__((section(".usbbuf")))

int usbpcd_init(void) {

  // Enable IO Clock & GPIO Port
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;


  // Enable macrocell
  USB->CNTR &= ~USB_CNTR_PDWN;

  // Wait 1us for stabilization @ 170Mhz Sysclk
  // TODO: use DWT?
  for (int i = 0; i < 180; i++) {
    __ASM("nop");
  }

  // Enable all interrupts & activate D+ 1.5kohm pull-up (FullSpeed USB)
  USB->CNTR |=
      USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM;
  USB->BCDR |= USB_BCDR_DPPU;

  // Clear the USB Reset (D+ & D- low) to start enumeration
  USB->CNTR &= ~USB_CNTR_FRES;

  USB->ISTR = 0;

  NVIC_EnableIRQ(USB_HP_IRQn);
  NVIC_EnableIRQ(USB_LP_IRQn);
  NVIC_EnableIRQ(USBWakeUp_IRQn);

  return 0;
}

void usbpcd_set_endpoint(__IO uint16_t *ep, uint16_t value, uint16_t mask) {
  uint16_t toggle = 0b0111000001110000;
  uint16_t rc_w0 = 0b1000000010000000;
  uint16_t rw = 0b0000011100001111;

  uint16_t wr0 = rc_w0 & (~mask | value);
  uint16_t wr1 = (mask & toggle) & (*ep ^ value);
  uint16_t wr2 = rw & ((*ep & ~mask) | value);

  *ep = wr0 | wr1 | wr2;
}

void usbpcd_clear_pma(void) {
  uint8_t *buffer = (uint8_t *)USB_PMAADDR;

  for (uint32_t i = 0; i < 1024; i++) {
    buffer[i] = 0;
  }
}

void usbpcd_copy_memory(uint16_t *source, uint16_t *target,
                            uint16_t length) {
  for (uint32_t i = 0; i < length / 2; i++) {
    target[i] = source[i];
  }
  if (length % 2 == 1) {
    ((uint8_t *)target)[length - 1] = ((uint8_t *)source)[length - 1];
  }
}
