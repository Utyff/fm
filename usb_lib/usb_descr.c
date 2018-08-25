#include "usb_descr.h"
/*
const uint8_t USB_DeviceDescriptor[] = {
        0x12,    // bLength
        0x01,    // bDescriptorType
        0x10,    // bcdUSB_L
        0x01,    // bcdUSB_H
        0x00,    // bDeviceClass
        0x00,    // bDeviceSubClass
        0x00,    // bDeviceProtocol
        0x40,    // bMaxPacketSize
        0x83,    // idVendor_L
        0x04,    // idVendor_H
        0x11,    // idProduct_L
        0x57,    // idProduct_H
        0x01,    // bcdDevice_Ver_L
        0x00,    // bcdDevice_Ver_H
        0x00,    // iManufacturer
        0x00,    // iProduct
        0x03,    // iSerialNumber
        0x01     // bNumConfigurations
}; //*/

#define LOBYTE(x) ((uint8_t)(x & 0x00FF))
#define HIBYTE(x) ((uint8_t)((x & 0xFF00) >>8))
//#define USB_DESC_TYPE_DEVICE 1
#define USB_MAX_EP0_SIZE 64
#define USBD_IDX_MFC_STR 0x01
#define USBD_IDX_PRODUCT_STR 0x02
#define USBD_IDX_SERIAL_STR 0x03
#define USBD_MAX_NUM_CONFIGURATION 1
#define USBD_VID 0x0483     //1155
#define USBD_PID_FS 0x5740  //22336

#define  USB_DESC_TYPE_DEVICE                              1
#define  USB_DESC_TYPE_CONFIGURATION                       2
#define  USB_DESC_TYPE_STRING                              3
#define  USB_DESC_TYPE_INTERFACE                           4
#define  USB_DESC_TYPE_ENDPOINT                            5
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                    6
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION           7
#define  USB_DESC_TYPE_BOS                                 0x0F


//__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
const uint8_t USB_DeviceDescriptor[] = {
        0x12,                      // bLength
        USB_DESC_TYPE_DEVICE,      // bDescriptorType
        0x00,                      // bcdUSB
        0x02,
        0x02,                      // bDeviceClass
        0x02,                      // bDeviceSubClass
        0x00,                      // bDeviceProtocol
        USB_MAX_EP0_SIZE,          // bMaxPacketSize
        LOBYTE(USBD_VID),          // idVendor
        HIBYTE(USBD_VID),          // idVendor
        LOBYTE(USBD_PID_FS),       // idProduct
        HIBYTE(USBD_PID_FS),       // idProduct
        0x00,                      // bcdDevice rel. 2.00
        0x02,
        USBD_IDX_MFC_STR,          // Index of manufacturer string
        USBD_IDX_PRODUCT_STR,      // Index of product string
        USBD_IDX_SERIAL_STR,       // Index of serial number string
        USBD_MAX_NUM_CONFIGURATION // bNumConfigurations
};

const uint8_t USB_DeviceQualifierDescriptor[] = {
        0x0A,    // bLength
        0x06,    // bDescriptorType
        0x00,    // bcdUSB_L
        0x02,    // bcdUSB_H
        0x00,    // bDeviceClass
        0x00,    // bDeviceSubClass
        0x00,    // bDeviceProtocol
        0x40,    // bMaxPacketSize0
        0x01,    // bNumConfigurations
        0x00     // Reserved
};


const uint8_t USB_ConfigDescriptor[] = {
//Дескриптор конфигурации
        0x09,    // bLength
        USB_DESC_TYPE_CONFIGURATION,    // bDescriptorType
        CONFIG_DESCRIPTOR_SIZE_BYTE,    // wTotalLength_L
        0x00,    // wTotalLength_H
        0x01,    // bNumInterfaces
        0x01,    // bConfigurationValue
        0x00,    // iConfiguration
        0x80,    // bmAttributes
        0x32,    // bMaxPower
//Дескриптор интерфейса
        0x09,    // bLength
        0x04,    // bDescriptorType
        0x00,    // bInterfaceNumber
        0x00,    // bAlternateSetting
        0x02,    // bNumEndpoints
        0x08,    // bInterfaceClass(0x03 - HID, 0xFF - custom)
        0x06,    // bInterfaceSubClass
        0x50,    // bInterfaceProtocol
        0x00,    // iInterface
//Дескриптор конечной точки 1 IN
        0x07,    // bLength
        0x05,    // bDescriptorType
        0x81,    // bEndpointAddress
        0x02,    // bmAttributes
        0x40,    // wMaxPacketSize_L
        0x00,    // wMaxPacketSize_H
        0x00,    // bInterval
//Дескриптор конечной точки 1 OUT
        0x07,    // bLength
        0x05,    // bDescriptorType
        0x01,    // bEndpointAddress
        0x02,    // bmAttributes
        0x40,    // wMaxPacketSize_L
        0x00,    // wMaxPacketSize_H
        0x00     // bInterval
}; // */
/*
#define USB_CDC_CONFIG_DESC_SIZ 67   // Total size of USB CDC device Configuration Descriptor
#define CDC_IN_EP               0x81 // EP1 for data IN
#define CDC_OUT_EP              0x01 // EP1 for data OUT
#define CDC_CMD_EP              0x82 // EP2 for CDC commands IN
#define CDC_CMD_PACKET_SIZE     8    // Control Endpoint Packet size
#define CDC_DATA_FS_MAX_PACKET_SIZE 64  // Endpoint IN & OUT Packet size

// USB CDC device Configuration Descriptor
const uint8_t USB_ConfigDescriptor[USB_CDC_CONFIG_DESC_SIZ] = {
        // Configuration Descriptor
        0x09,   // bLength: Configuration Descriptor size
        USB_DESC_TYPE_CONFIGURATION,     // bDescriptorType: Configuration
        USB_CDC_CONFIG_DESC_SIZ,         // wTotalLength:no of returned bytes
        0x00,
        0x02,   // bNumInterfaces: 2 interface
        0x01,   // bConfigurationValue: Configuration value
        0x00,   // iConfiguration: Index of string descriptor describing the configuration
        0xC0,   // bmAttributes: self powered
        0x4B,   // MaxPower 150 mA /2

        // Interface Descriptor
        0x09,   // bLength: Interface Descriptor size
        USB_DESC_TYPE_INTERFACE,  // bDescriptorType: Interface
        // Interface descriptor type
        0x00,   // bInterfaceNumber: Number of Interface
        0x00,   // bAlternateSetting: Alternate setting
        0x01,   // bNumEndpoints: One endpoints used
        0x02,   // bInterfaceClass: Communication Interface Class
        0x02,   // bInterfaceSubClass: Abstract Control Model
        0x01,   // bInterfaceProtocol: Common AT commands
        0x00,   // iInterface:

        // Header Functional Descriptor
        0x05,   // bLength: Endpoint Descriptor size
        0x24,   // bDescriptorType: CS_INTERFACE
        0x00,   // bDescriptorSubtype: Header Func Desc
        0x10,   // bcdCDC: spec release number
        0x01,

        // Call Management Functional Descriptor
        0x05,   // bFunctionLength
        0x24,   // bDescriptorType: CS_INTERFACE
        0x01,   // bDescriptorSubtype: Call Management Func Desc
        0x00,   // bmCapabilities: D0+D1
        0x01,   // bDataInterface: 1

        // ACM Functional Descriptor
        0x04,   // bFunctionLength
        0x24,   // bDescriptorType: CS_INTERFACE
        0x02,   // bDescriptorSubtype: Abstract Control Management desc
        0x02,   // bmCapabilities

        // Union Functional Descriptor
        0x05,   // bFunctionLength
        0x24,   // bDescriptorType: CS_INTERFACE
        0x06,   // bDescriptorSubtype: Union func desc
        0x00,   // bMasterInterface: Communication class interface
        0x01,   // bSlaveInterface0: Data Class Interface

        // Endpoint 2 Descriptor
        0x07,                        // bLength: Endpoint Descriptor size
        USB_DESC_TYPE_ENDPOINT,      // bDescriptorType: Endpoint
        CDC_CMD_EP,                  // bEndpointAddress
        0x03,                        // bmAttributes: Interrupt
        LOBYTE(CDC_CMD_PACKET_SIZE), // wMaxPacketSize:
        HIBYTE(CDC_CMD_PACKET_SIZE),
        0x10,                        // bInterval:

        // Data class interface descriptor
        0x09,   // bLength: Endpoint Descriptor size
        USB_DESC_TYPE_INTERFACE,  // bDescriptorType:
        0x01,   // bInterfaceNumber: Number of Interface
        0x00,   // bAlternateSetting: Alternate setting
        0x02,   // bNumEndpoints: Two endpoints used
        0x0A,   // bInterfaceClass: CDC
        0x00,   // bInterfaceSubClass:
        0x00,   // bInterfaceProtocol:
        0x00,   // iInterface:

        // Endpoint OUT Descriptor
        0x07,   // bLength: Endpoint Descriptor size
        USB_DESC_TYPE_ENDPOINT,            // bDescriptorType: Endpoint
        CDC_OUT_EP,                        // bEndpointAddress
        0x02,                              // bmAttributes: Bulk
        LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), // wMaxPacketSize:
        HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
        0x00,                              // bInterval: ignore for Bulk transfer

        // Endpoint IN Descriptor
        0x07,   // bLength: Endpoint Descriptor size
        USB_DESC_TYPE_ENDPOINT,            // bDescriptorType: Endpoint
        CDC_IN_EP,                         // bEndpointAddress
        0x02,                              // bmAttributes: Bulk
        LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), // wMaxPacketSize:
        HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
        0x00                               // bInterval: ignore for Bulk transfer
}; // */


const uint8_t USB_StringLangDescriptor[] = {
        0x04,    // bLength
        0x03,    // bDescriptorType
        0x09,    // wLANGID_L
        0x04     // wLANGID_H
};

const uint8_t USB_StringManufacturingDescriptor[] = {
        STRING_MANUFACTURING_DESCRIPTOR_SIZE_BYTE,       // bLength
        0x03,                                            // bDescriptorType
        'U', 0x00,                                       // bString...
        'T', 0x00,
        'Y', 0x00,
        'F', 0x00
};

const uint8_t USB_StringProdDescriptor[] = {
        STRING_PRODUCT_DESCRIPTOR_SIZE_BYTE,        // bLength
        0x03,                                       // bDescriptorType
        'U', 0x00,                                  // bString...
        'T', 0x00,
        'Y', 0x00,
        'F', 0x00,
        '-', 0x00,
        'C', 0x00,
        'D', 0x00,
        'C', 0x00
};

const uint8_t USB_StringSerialDescriptor[STRING_SERIAL_DESCRIPTOR_SIZE_BYTE] = {
        STRING_SERIAL_DESCRIPTOR_SIZE_BYTE, // bLength
        0x03,                               // bDescriptorType
        'q', 0,
        'r', 0,
        '3', 0,
        '4', 0,
        '5', 0,
        '6', 0,
        '7', 0,
        '8', 0
};
