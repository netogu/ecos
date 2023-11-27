#include "hardware/stm32g4/usbpcd.h"
#include "external/printf.h"
#include "lib/usb/usb.h"
#include "lib/usb/descriptors.h"
#include "lib/debug.h"


enum device_states {
  USB_UNCONNECTED = 0,
  USB_DEFAULT,
  USB_ADDRESS,
  USB_CONFIGURED,
  USB_SUSPENDED
};

static uint8_t usb_device_state = USB_UNCONNECTED;
static uint8_t usb_device_active_configuration = 0;

#define USB_NUM_INTERFACES 1
#define USB_NUM_ENDPOINTS 8

static uint8_t usb_endpoint_state[USB_NUM_ENDPOINTS] = {0};

typedef struct {
  uint8_t length;
  uint8_t bytes_sent;
  uint8_t *buffer;
} usb_transfer_state_t;

typedef struct {
  usb_setup_packet_t setup;
  usb_transfer_state_t transfer;
} usb_control_state_t;

static usb_control_state_t control_state;

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

//--------------------------------------------------------------------+
// USB Device Descriptors
//--------------------------------------------------------------------+

static const usb_dev_desc_t device_descriptor = {
    .bLength = 18,
    .bDescriptorType = 0x01,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0001,
    .idProduct = 0x7778,
    .bcdDevice = 0x0001,
    .iManufacturer = 0,
    .iProduct = 0,
    .iSerialNumber = 0,
    .bNumConfigurations = 1};

static const usb_cfg_desc_t cfg_descriptor = {
    .bLength = 9,
    .bDescriptorType = 0x02,
    .wTotalLength = 32,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 50};

static const usb_intf_desc_t intf_descriptors[] = {
    {.bLength = 9,
    .bDescriptorType = 0x04,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0x0A,
    .bInterfaceSubClass = 0x00,
    .bInterfaceProtocol = 0x00,
    .iInterface = 0}};

static const usb_ep_desc_t ep_descriptors[] = {
    {.bLength = 7,
    .bDescriptorType = 0x05,
    .bEndpointAddress = 0x81,
    .bmAttributes = 0x03,
    .wMaxPacketSize = 64,
    .bInterval = 0xFF},
    {.bLength = 7,
    .bDescriptorType = 0x05,
    .bEndpointAddress = 0x01,
    .bmAttributes = 0x03,
    .wMaxPacketSize = 64,
    .bInterval = 0xFF}};

static uint8_t _configuration_buffer[32] = {0};

static void _add_to_descriptor(uint8_t *data, uint8_t *offset) {
  uint8_t length = data[0];

  for (int i =0; i < length; i++) {
    _configuration_buffer[i + *offset] = data[i];
  }
  *offset += length;
}

uint8_t *usb_get_config_descriptor(uint8_t *length) {
  if (_configuration_buffer[0] == 0) {
    uint8_t offset = 0;
    _add_to_descriptor(&cfg_descriptor, &offset);
    _add_to_descriptor(&intf_descriptors[0], &offset);
    _add_to_descriptor(&ep_descriptors[0], &offset);
    _add_to_descriptor(&ep_descriptors[1], &offset);
  }

  *length = sizeof(_configuration_buffer);
  return _configuration_buffer;
}

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


static void usb_prepare_transfer(usb_transfer_state_t *transfer, uint8_t *ep, uint8_t *txBuffer, uint8_t *txBufferCount, uint8_t txBufferSize) {
    *txBufferCount = MIN(txBufferSize, transfer->length - transfer->bytes_sent);
    if (*txBufferCount > 0) {
        usbpcd_copy_memory(transfer->buffer + transfer->bytes_sent, txBuffer, *txBufferCount);
        transfer->bytes_sent += *txBufferCount;
        usbpcd_set_endpoint(ep, USB_EP_TX_VALID, USB_EP_TX_VALID);
    } else {
        usbpcd_set_endpoint(ep, USB_EP_TX_NAK, USB_EP_TX_VALID);
    }
}

//--------------------------------------------------------------------+
// USB Reset Handler
//--------------------------------------------------------------------+
static void usb_reset_handler() {


  // Clear USB- Packet Memory
  usbpcd_clear_pma();

  // Prepare Buffer Table
  // USB->BTABLE = __MEM2USB(btable);
  USB->BTABLE = 0x00;

  USB_PMA->usb_btable[0].addr_tx = MEM2PMA(USB_PMA->ep0_buffer.tx);
  USB_PMA->usb_btable[0].count_tx = 0x00;
  USB_PMA->usb_btable[0].addr_rx = MEM2PMA(USB_PMA->ep0_buffer.rx);
  // BL_SIZE = 0x01, NUM_BLOCK = 0x01 (64 bytes)
  USB_PMA->usb_btable[0].count_rx = ( 1 << 15 ) | ( 1 << 10 );


  // Prepare for a setup packet (RX = Valid, TX = NAK)
  usbpcd_set_endpoint(&USB->EP0R, USB_EP_CONTROL | USB_EP_RX_VALID | USB_EP_TX_NAK,
                   USB_EP_TYPE_MASK | USB_EP_RX_VALID | USB_EP_TX_VALID);

  // Enable USB functionality and set address to 0
  USB->DADDR = USB_DADDR_EF;

  // Clear interrupt
  USB->ISTR = ~USB_ISTR_RESET;
}

//--------------------------------------------------------------------+
// USB Device Request Handlers
//--------------------------------------------------------------------+

void usb_get_descriptor_req_handler(usb_setup_packet_t *setup){

  switch (setup->descriptor_type) {

    case 0x01: { // Device Descriptor
    const usb_dev_desc_t *descriptor = &device_descriptor;
    usbpcd_copy_memory((uint16_t *)descriptor, (uint16_t *)USB_PMA->ep0_buffer.tx,
                    sizeof(usb_dev_desc_t));
    USB_PMA->usb_btable[0].count_tx = sizeof(usb_dev_desc_t);
    usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    } break;

  case 0x02: { // Configuration Descriptor
    uint16_t length = 0;
    uint8_t *descriptor = usb_get_config_descriptor(&length);
    control_state.transfer.buffer = descriptor;
    control_state.transfer.bytes_sent = 0;
    control_state.transfer.length = MIN(length, setup->wLength);
    usb_prepare_transfer(&control_state.transfer, &USB->EP0R, &USB_PMA->ep0_buffer.tx,
                        &USB_PMA->usb_btable[0].count_tx, 64);
    } break;
  case 0x03: // String Descriptor
    switch (setup->descriptor_index) {
      case 0x00: // Language ID
        USB_PMA->ep0_buffer.tx[0] = 0x04;
        USB_PMA->ep0_buffer.tx[1] = 0x03;
        USB_PMA->usb_btable[0].count_tx = 2;
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        break;
      case 0x01: // Manufacturer
        USB_PMA->ep0_buffer.tx[0] = 0x0C;
        USB_PMA->ep0_buffer.tx[1] = 0x03;
        USB_PMA->ep0_buffer.tx[2] = 'S';
        USB_PMA->ep0_buffer.tx[3] = 0x00;
        USB_PMA->ep0_buffer.tx[4] = 'T';
        USB_PMA->ep0_buffer.tx[5] = 0x00;
        USB_PMA->ep0_buffer.tx[6] = 'M';
        USB_PMA->ep0_buffer.tx[7] = 0x00;
        USB_PMA->ep0_buffer.tx[8] = '3';
        USB_PMA->ep0_buffer.tx[9] = 0x00;
        USB_PMA->ep0_buffer.tx[10] = '2';
        USB_PMA->ep0_buffer.tx[11] = 0x00;
        USB_PMA->usb_btable[0].count_tx = 12;
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        break;
      case 0x02: // Product
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        break;
      case 0x03: // Serial Number 
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        break;
      case 0x04: // Configuration
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        break;
      case 0x05: // Interface
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        break;
      case 0x06: // String Descriptor
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        break;
      default:
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        break;
    }
    usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
    break;
  
  case 0x06: // Device Qualifier Descriptor
    usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
    break;
  default:
    usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
    break;
  }
}

void usb_set_addr_req_handler(usb_setup_packet_t *setup) {
  USB_PMA->usb_btable[0].count_tx = 0;
  usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);

}


void usb_get_status_req_handler(usb_setup_packet_t *setup) {

  USB_PMA->ep0_buffer.tx[0] = 0x00; //Self Powered
  USB_PMA->ep0_buffer.tx[1] = 0x00;
  USB_PMA->usb_btable[0].count_tx = 2;
  usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);

}

void usb_clear_feature_req_handler(usb_setup_packet_t *setup) {
  USB_PMA->usb_btable[0].count_tx = 0;
  usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

void usb_set_feature_req_handler(usb_setup_packet_t *setup) {
  USB_PMA->usb_btable[0].count_tx = 0;
  usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

void usb_set_descriptor_req_handler(usb_setup_packet_t *setup) {
  // Not Supported
  usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
}

void usb_get_config_req_handler(usb_setup_packet_t *setup) {

  if (usb_device_state == USB_DEFAULT) {
    USB_PMA->ep0_buffer.tx[0] = 0x00;
    USB_PMA->usb_btable[0].count_tx = 1;
    usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
  } else if (usb_device_state == USB_CONFIGURED) {
    USB_PMA->ep0_buffer.tx[0] = 0x01;
    USB_PMA->usb_btable[0].count_tx = 1;
    usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
  } else {
    usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
  }
}

void usb_set_config_req_handler(usb_setup_packet_t *setup) {

  if (usb_device_state == USB_DEFAULT || usb_device_state == USB_ADDRESS) {

    USB_PMA->usb_btable[0].count_tx = 0;

    switch(setup->wValue & 0xFF) {
      case 0x00:
        usb_device_state = USB_DEFAULT;
        usb_device_active_configuration = 0;
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        break;
      case 0x01:
        usb_device_state = USB_ADDRESS;
        usb_device_active_configuration = 1;
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        break;
      default:
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        break;
    }

    if (usb_device_state == USB_ADDRESS) {
      usbpcd_set_endpoint(&USB->EP1R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usbpcd_set_endpoint(&USB->EP2R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usbpcd_set_endpoint(&USB->EP3R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usbpcd_set_endpoint(&USB->EP4R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usbpcd_set_endpoint(&USB->EP5R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usbpcd_set_endpoint(&USB->EP6R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
      usbpcd_set_endpoint(&USB->EP7R, 0x00, USB_EP_DTOG_RX | USB_EP_DTOG_TX);
                
    } else {
      usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
    }
  }
}

void usb_reset_class(uint8_t interface, uint8_t configuration) {
  // do nothing
}

//--------------------------------------------------------------------+   
// USB Setup Request Handler
//--------------------------------------------------------------------+
void usb_handle_setup(usb_setup_packet_t *setup) {

  usbpcd_copy_memory((uint16_t *)setup, (uint16_t *)&control_state.setup,
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
      if (usb_device_state == USB_ADDRESS) {
        USB_PMA->ep0_buffer.tx[0] = 0x00;
        USB_PMA->ep0_buffer.tx[1] = 0x00;
        USB_PMA->usb_btable[0].count_tx = 2;
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
      } else {
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x01: // Clear Feature
      usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      break;
    case 0x03: // Set Feature
      usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      break;
    case 0x0A: // Get Interface
      if (usb_device_state == USB_ADDRESS && setup->wIndex < USB_NUM_INTERFACES) {
        USB_PMA->ep0_buffer.tx[0] = 0x00;
        USB_PMA->usb_btable[0].count_tx = 1;
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
      } else {
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x0B: // Set Interface /!\ USB InANutshell is wrong here, it confuses
               // the decimal value (11) with the hex one (0x0B)
      if (usb_device_state == USB_ADDRESS && setup->wIndex < USB_NUM_INTERFACES) {
        USB_PMA->usb_btable[0].count_tx = 0;
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        usb_reset_class(setup->wIndex, setup->wValue);
      } else {
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    }

  } else if ((setup->bmRequestType & 0x0F) == 0x02) { // Endpoint requests
    switch (setup->bRequest) {
    case 0x00: // Get Status
      if ((usb_device_state == USB_ADDRESS ||
           (usb_device_state == USB_DEFAULT && setup->wIndex == 0x00)) &&
          setup->wIndex < USB_NUM_ENDPOINTS) {
        if (setup->wValue == 0x00) {
          USB_PMA->ep0_buffer.tx[0] = usb_endpoint_state[setup->wIndex];
          USB_PMA->ep0_buffer.tx[1] = 0x00;
          USB_PMA->usb_btable[0].count_tx = 2;
          usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        } else {
          usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        }
      } else {
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x01: // Clear Feature
      if ((usb_device_state == USB_ADDRESS ||
           (usb_device_state == USB_DEFAULT && setup->wIndex == 0x00)) &&
          setup->wIndex < USB_NUM_ENDPOINTS) {
        if (setup->wValue == 0x00) {
          usb_endpoint_state[setup->wIndex] = 0;
          USB_PMA->usb_btable[0].count_tx = 0;
          usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        } else {
          usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        }
      } else {
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x03: // Set Feature
      if ((usb_device_state == USB_ADDRESS ||
           (usb_device_state == USB_DEFAULT && setup->wIndex == 0x00)) &&
          setup->wIndex < USB_NUM_ENDPOINTS) {
        if (setup->wValue == 0x00) {
          usb_endpoint_state[setup->wIndex] = 1;
          USB_PMA->usb_btable[0].count_tx = 0;
          usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
        } else {
          usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
        }
      } else {
        usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      }
      break;
    case 0x0C: // Sync Frame /!\ USB InANutshell is wrong here again, as it
               // confuses the decimal value (12) with the hex one (0x0C)
      usbpcd_set_endpoint(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
      break;
    }
  }
}

//--------------------------------------------------------------------+
// USB Control Request Handler
//--------------------------------------------------------------------+

static void usb_handle_control_transfer() {

  if (USB->EP0R & USB_EP_CTR_RX) {
    // Received control message
    if (USB->EP0R & USB_EP_SETUP) {
      usb_setup_packet_t *setup = (usb_setup_packet_t *)USB_PMA->ep0_buffer.rx;
      usb_handle_setup(setup);
    }

    usbpcd_set_endpoint(&USB->EP0R, USB_EP_RX_VALID,
                     USB_EP_CTR_RX | USB_EP_RX_VALID);
  }

  if (USB->EP0R & USB_EP_CTR_TX) {
    // Just sent a control message
    if (control_state.setup.bRequest == USB_REQ_SET_ADDRESS) {
      USB->DADDR = USB_DADDR_EF | control_state.setup.wValue;
      printf("USB Address set to %d\r\n", USB->DADDR);
    }

    // Check for ongoing usb_transfers
    if (control_state.transfer.length > 0) {
      if (control_state.transfer.length > control_state.transfer.bytes_sent) {
        usb_prepare_transfer(&control_state.transfer, &USB->EP0R, &USB_PMA->ep0_buffer.tx,
                             &USB_PMA->usb_btable[0].count_tx, 64);
      }
    }
    usbpcd_set_endpoint(&USB->EP0R, 0x00, USB_EP_CTR_TX);
  }
}

//--------------------------------------------------------------------+
// USB Event Handlers
//--------------------------------------------------------------------+
void usb_suspend_device() {
  USB->ISTR = ~USB_ISTR_SUSP;
}
void usb_wakeup_device() {}


//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+

int usb_lp_irq_counter = 0;
void USB_LP_IRQHandler() {

  usb_lp_irq_counter++;

  if (USB_Reset_Event) {
    usb_reset_handler();

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
