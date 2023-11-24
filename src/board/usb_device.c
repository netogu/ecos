#include "board/usb_device.h"
#include "hardware/stm32g4/usbpcd.h"

void usbd_init() { return; }

#define USB_Correct_Transfer_Event (USB->ISTR & USB_ISTR_CTR)
#define USB_Memory_Overflow_Event (USB->ISTR & USB_ISTR_PMAOVR)
#define USB_Error_Event (USB->ISTR & USB_ISTR_ERR)
#define USB_Wakeup_Event (USB->ISTR & USB_ISTR_WKUP)
#define USB_Suspend_Event (USB->ISTR & USB_ISTR_SUSP)
#define USB_Reset_Event (USB->ISTR & USB_ISTR_RESET)
#define USB_SOF_Event (USB->ISTR & USB_ISTR_SOF)
#define USB_Expected_Start_Of_Frame (USB->ISTR & USB_ISTR_ESOF)
#define USB_LPM_Event (USB->ISTR & USB_ISTR_L1REQ)
#define USB_Transaction_Dir (USB->ISTR & USB_ISTR_DIR)
#define USB_Endpoint_Identifier (USB->ISTR & USB_ISTR_EP_ID)

//--------------------------------------------------------------------+
// USB Request Handlers
//--------------------------------------------------------------------+

void usb_handle_reset() {

  // Clear interrupt
  USB->ISTR = ~USB_ISTR_RESET;

  // Clear USB-SRAM
  usb_clear_sram();

  // Prepare Buffer Table
  USB->BTABLE = __MEM2USB(btable);

  btable[0].addr_tx = __MEM2USB(ep0_buff_tx);
  btable[0].count_tx = 0;
  btable[0].addr_rx = __MEM2USB(ep0_buff_rx);
  btable[0].count_rx = (1 << 15) | (1 << 10);

  // Prepare for a setup packet (RX = Valid, TX = NAK)
  usb_set_endpoint(&USB->EP0R, USB_EP_CONTROL | USB_EP_RX_VALID | USB_EP_TX_NAK,
                   USB_EP_TYPE_MASK | USB_EP_RX_VALID | USB_EP_TX_VALID);

  // Enable USB functionality and set address to 0
  USB->DADDR = USB_DADDR_EF;
}

static void usb_handle_control() {

  if (USB->EP0R & USB_EP_CTR_RX) {
    // Received control message
    if (USB->EP0R & USB_EP_SETUP) {
      usb_setup_packet_t *setup = (usb_setup_packet_t *)ep0_buff_rx;
      usb_handle_setup(setup);
    }

    usb_set_endpoint(&USB->EP0R, USB_EP_RX_VALID,
                     USB_EP_CTR_RX | USB_EP_RX_VALID);
  }

  if (USB->EP0R & USB_EP_CTR_TX) {
    // Just sent a control message
    if (control_state.setup.bRequest == 0x05) {
      USB->DADDR = USB_DADDR_EF | control_state.setup.wValue;
    }

    // Check for ongoing usb_transfers
    if (control_state.transfer.length > 0) {
      if (control_state.transfer.length > control_state.transfer.bytes_sent) {
        usb_prepare_transfer(&control_state.transfer, &USB->EP0R, &ep0_buff_tx,
                             &btable[0].count_tx, 64);
      }
    }
    usb_set_endpoint(&USB->EP0R, 0x00, USB_EP_CTR_TX);
  }
}

void usb_suspend_device() {}
void usb_wakeup_device() {}

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void USB_LP_IRQHandler() {

  if (USB_Reset_Event) {
    usb_handle_reset();

  } else if (USB_Correct_Transfer_Event) {
    if ((USB_Endpoint_Identifier) == 0) {
      usb_handle_control();
    }
  } else if (USB_Wakeup_Event) {
    USB->ISTR = ~USB_ISTR_WKUP;
    // Resume peripheral
    USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LPMODE);
    usb_wakeup_device();

  } else if (USB_Suspend_Event) {
    USB->ISTR = ~USB_ISTR_SUSP;
    usb_suspend_device();
    // On Suspend, the device should enter low power mode and turn off the
    // USB-Peripheral
    USB->CNTR |= USB_CNTR_FSUSP;
    // If the device still needs power from the USB Host
    USB->CNTR |= USB_CNTR_LPMODE;
  }
}
