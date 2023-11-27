#pragma once
#include <stdint.h>

//-----------------------------------------------------------------
// USB Descriptors:
//-----------------------------------------------------------------
enum usb_descriptor_type {
    USB_DESC_DEVICE = 1,
    USB_DESC_CONFIGURATION,
    USB_DESC_STRING,
    USB_DESC_INTERFACE,
    USB_DESC_ENDPOINT,
    USB_DESC_DEVICE_QUALIFIER,
    USB_DESC_OTHER_SPEED_CONFIGURATION,
    USB_DESC_INTERFACE_POWER,
};

// USB Device Descriptor
typedef struct {
    uint8_t bLength;            // Size of this descriptor in bytes
    uint8_t bDescriptorType;    // DEVICE descriptor type
    uint16_t bcdUSB;            // USB Spec Release Number in BCD
    uint8_t bDeviceClass;       // Class Code
    uint8_t bDeviceSubClass;    // Subclass code
    uint8_t bDeviceProtocol;    // Protocol code
    uint8_t bMaxPacketSize0;    // Max packet size for EP0
    uint16_t idVendor;          // Vendor ID
    uint16_t idProduct;         // Product ID
    uint16_t bcdDevice;         // Device release number in BCD
    uint8_t iManufacturer;      // Manufacturer string index
    uint8_t iProduct;           // Product string index
    uint8_t iSerialNumber;      // Serial number string index
    uint8_t bNumConfigurations; // Number of possible configurations
} __attribute__((packed, aligned(2))) usb_dev_desc_t;

// USB Configuration Descriptor
typedef struct {
    uint8_t bLength;             // Size of this descriptor in bytes
    uint8_t bDescriptorType;     // CONFIGURATION descriptor type
    uint16_t wTotalLength;       // Total length of data for this cfg
    uint8_t bNumInterfaces;      // Number of interfaces in this cfg
    uint8_t bConfigurationValue; // Value for SetConfiguration resquest
    uint8_t iConfiguration;      // Index of string descriptor for this cfg
    uint8_t bmAttributes;        // Configuration characteristics
    uint8_t bMaxPower;           // Max power consumption in 2mA units
} __attribute__((packed, aligned(2))) usb_cfg_desc_t;

// USB Interface Descriptor
typedef struct {
    uint8_t bLength;            // Size of this descriptor in bytes
    uint8_t bDescriptorType;    // INTERFACE descriptor type
    uint8_t bInterfaceNumber;   // Number of this interface
    uint8_t bAlternateSetting;  // Value for SetInterface request
    uint8_t bNumEndpoints;      // Number of endpoints in this intf
    uint8_t bInterfaceClass;    // Class code
    uint8_t bInterfaceSubClass; // Subclass code
    uint8_t bInterfaceProtocol; // Protocol code
    uint8_t iInterface;         // Index of string descriptor for this intf
} __attribute__((packed, aligned(2))) usb_intf_desc_t;

// USB Endpoint Descriptor
typedef struct {
    uint8_t bLength;            // Size of this descriptor in bytes
    uint8_t bDescriptorType;    // ENDPOINT descriptor type
    uint8_t bEndpointAddress;   // Endpoint address (number & direction)
    uint8_t bmAttributes;       // Endpoint attributes
    uint16_t wMaxPacketSize;    // Max packet size this endpoint is capable of sending or receiving
    uint8_t bInterval;          // Interval for polling endpoint for data transfers
} __attribute__((packed, aligned(2))) usb_ep_desc_t;

// USB String Descriptor
typedef struct {
    uint8_t bLength;            // Size of this descriptor in bytes
    uint8_t bDescriptorType;    // STRING descriptor type
    uint16_t wString[];         // Unicode string
} __attribute__((packed, aligned(2))) usb_str_desc_t;
