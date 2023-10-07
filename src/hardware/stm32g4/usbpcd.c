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
