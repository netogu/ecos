
#include <stdint.h>
#include "lib/usb/descriptors.h"

static const usb_dev_desc_t device_descriptor = {
    .bLength = 18,
    .bDescriptorType = 0x01,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
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

static const usb_intf_desc_t intf_descriptor = {
    .bLength = 9,
    .bDescriptorType = 0x04,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0x0A,
    .bInterfaceSubClass = 0x00,
    .bInterfaceProtocol = 0x00,
    .iInterface = 0};

static const usb_ep_desc_t ep_descriptor[] = {
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