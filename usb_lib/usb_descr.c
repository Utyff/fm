#include "usb_descr.h"

const uint8_t USB_DeviceDescriptor[] = {
		0x12,	//bLength
		0x01,	//bDescriptorType
		0x10,	//bcdUSB_L
		0x01,	//bcdUSB_H
		0x00,	//bDeviceClass
		0x00,	//bDeviceSubClass
		0x00,	//bDeviceProtocol
		0x40,	//bMaxPacketSize
		0x83,	//idVendor_L
		0x04,	//idVendor_H
		0x11,	//idProduct_L
		0x57,	//idProduct_H
		0x01,	//bcdDevice_Ver_L
		0x00,	//bcdDevice_Ver_H
		0x00,	//iManufacturer
		0x00,	//iProduct
		0x03,	//iSerialNumber
		0x01	//bNumConfigurations
};

const uint8_t USB_DeviceQualifierDescriptor[] = {
		0x0A,	//bLength
		0x06,	//bDescriptorType
		0x00,	//bcdUSB_L
		0x02,	//bcdUSB_H
		0x00,	//bDeviceClass
		0x00,	//bDeviceSubClass
		0x00,	//bDeviceProtocol
		0x40,	//bMaxPacketSize0
		0x01,	//bNumConfigurations
		0x00	//Reserved
};

const uint8_t USB_ConfigDescriptor[] = {
//Дескриптор конфигурации
		0x09,	//bLength
		0x02,	//bDescriptorType
		CONFIG_DESCRIPTOR_SIZE_BYTE,	//wTotalLength_L
		0x00,	//wTotalLength_H
		0x01,	//bNumInterfaces
		0x01,	//bConfigurationValue
		0x00,	//iConfiguration
		0x80,	//bmAttributes
		0x32,	//bMaxPower
//Дескриптор интерфейса
		0x09,	//bLength
		0x04,	//bDescriptorType
		0x00,	//bInterfaceNumber
		0x00,	//bAlternateSetting
		0x02,	//bNumEndpoints
		0x08,	//bInterfaceClass(0x03 - HID, 0xFF - custom)
		0x06,	//bInterfaceSubClass
		0x50,	//bInterfaceProtocol
		0x00,	//iInterface
//Дескриптор конечной точки 1 IN
		0x07,	//bLength
		0x05,	//bDescriptorType
		0x81,	//bEndpointAddress
		0x02,	//bmAttributes
		0x40,	//wMaxPacketSize_L
		0x00,	//wMaxPacketSize_H
		0x00,	//bInterval
//Дескриптор конечной точки 1 OUT
		0x07,	//bLength
		0x05,	//bDescriptorType
		0x01,	//bEndpointAddress
		0x02,	//bmAttributes
		0x40,	//wMaxPacketSize_L
		0x00,	//wMaxPacketSize_H
		0x00	//bInterval
};

const uint8_t USB_StringLangDescriptor[] = {
	0x04,	//bLength
	0x03,	//bDescriptorType
	0x09,	//wLANGID_L
	0x04	//wLANGID_H
};

const uint8_t USB_StringManufacturingDescriptor[] = {
	STRING_MANUFACTURING_DESCRIPTOR_SIZE_BYTE,		//bLength
	0x03,											//bDescriptorType
	'S', 0x00,										//bString...
	'O', 0x00,
	'B', 0x00,
	'S', 0x00
};

const uint8_t USB_StringProdDescriptor[] = {
	STRING_PRODUCT_DESCRIPTOR_SIZE_BYTE,		//bLength
	0x03,										//bDescriptorType
	'H', 0x00,									//bString...
	'I', 0x00,
	'D', 0x00,
	' ', 0x00,
	'T', 0x00,
	'E', 0x00,
	'S', 0x00,
	'T', 0x00
};

const uint8_t USB_StringSerialDescriptor[STRING_SERIAL_DESCRIPTOR_SIZE_BYTE] =
{
	STRING_SERIAL_DESCRIPTOR_SIZE_BYTE,           /* bLength */
	0x03,        /* bDescriptorType */
	'1', 0,
	'2', 0,
	'3', 0,
	'4', 0,
	'5', 0,
	'6', 0,
	'7', 0,
	'8', 0
};
