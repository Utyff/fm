#include "usb_defs.h"
#include "usb_descr.h"

//Максимальное количесво конечных точек
#define MAX_ENDPOINTS					2
//Стандартные запросы
#define GET_DESCRIPTOR					0x06
#define SET_DESCRIPTOR					0x07	//не реализован
#define SET_ADDRESS						0x05
#define SET_FEATURE						0x03	//не реализован
#define CLEAR_FEATURE					0x01
#define GET_STATUS						0x00
#define GET_CONFIGURATION				0x08	//не реализован
#define SET_CONFIGURATION				0x09
#define GET_INTERFACE					0x0A	//не реализован
#define SET_INTERFACE					0x0B	//не реализован
#define SYNC_FRAME						0x0C	//не реализован
//Параметры запроса
#define DEVICE_DESCRIPTOR				0x100
#define CONFIGURATION_DESCRIPTOR		0x200
#define HID_REPORT_DESCRIPTOR			0x2200
#define STRING_LANG_DESCRIPTOR			0x300
#define STRING_MAN_DESCRIPTOR			0x301
#define STRING_PROD_DESCRIPTOR			0x302
#define STRING_SN_DESCRIPTOR			0x303
#define DEVICE_QALIFIER_DESCRIPTOR		0x600

#define BULK_RESET						0xFF
#define GET_MAX_LUN						0xFE

//Макросы установки/очистки битов в EPnR регистрах
#define CLEAR_DTOG_RX(R)       			(R & USB_EPnR_DTOG_RX) ? R : (R & (~USB_EPnR_DTOG_RX))
#define SET_DTOG_RX(R)         			(R & USB_EPnR_DTOG_RX) ? (R & (~USB_EPnR_DTOG_RX)) : (R | USB_EPnR_DTOG_RX)
#define TOGGLE_DTOG_RX(R)      			(R | USB_EPnR_DTOG_RX)
#define KEEP_DTOG_RX(R)        			(R & (~USB_EPnR_DTOG_RX))
#define CLEAR_DTOG_TX(R)       			(R & USB_EPnR_DTOG_TX) ? R : (R & (~USB_EPnR_DTOG_TX))
#define SET_DTOG_TX(R)         			(R & USB_EPnR_DTOG_TX) ? (R & (~USB_EPnR_DTOG_TX)) : (R | USB_EPnR_DTOG_TX)
#define TOGGLE_DTOG_TX(R)      			(R | USB_EPnR_DTOG_TX)
#define KEEP_DTOG_TX(R)        			(R & (~USB_EPnR_DTOG_TX))
#define SET_VALID_RX(R)        			((R & USB_EPnR_STAT_RX) ^ USB_EPnR_STAT_RX)   | (R & (~USB_EPnR_STAT_RX))
#define SET_NAK_RX(R)          			((R & USB_EPnR_STAT_RX) ^ USB_EPnR_STAT_RX_1) | (R & (~USB_EPnR_STAT_RX))
#define SET_STALL_RX(R)        			((R & USB_EPnR_STAT_RX) ^ USB_EPnR_STAT_RX_0) | (R & (~USB_EPnR_STAT_RX))
#define KEEP_STAT_RX(R)        			(R & (~USB_EPnR_STAT_RX))
#define SET_VALID_TX(R)        			((R & USB_EPnR_STAT_TX) ^ USB_EPnR_STAT_TX)   | (R & (~USB_EPnR_STAT_TX))
#define SET_NAK_TX(R)          			((R & USB_EPnR_STAT_TX) ^ USB_EPnR_STAT_TX_1) | (R & (~USB_EPnR_STAT_TX))
#define SET_STALL_TX(R)        			((R & USB_EPnR_STAT_TX) ^ USB_EPnR_STAT_TX_0) | (R & (~USB_EPnR_STAT_TX))
#define KEEP_STAT_TX(R)        			(R & (~USB_EPnR_STAT_TX))
#define CLEAR_CTR_RX(R)       		 	(R & (~(USB_EPnR_CTR_RX | USB_EPnR_STAT_RX | USB_EPnR_STAT_TX | USB_EPnR_DTOG_RX | USB_EPnR_DTOG_TX)))
#define CLEAR_CTR_TX(R)        			(R & (~(USB_EPnR_CTR_TX | USB_EPnR_STAT_RX | USB_EPnR_STAT_TX | USB_EPnR_DTOG_RX | USB_EPnR_DTOG_TX)))
#define CLEAR_CTR_RX_TX(R)     			(R & (~(USB_EPnR_CTR_TX | USB_EPnR_CTR_RX | USB_EPnR_STAT_RX | USB_EPnR_STAT_TX | USB_EPnR_DTOG_RX | USB_EPnR_DTOG_TX)))
//Состояния соединения USB
#define USB_DEFAULT_STATE				0
#define USB_ADRESSED_STATE				1
#define USB_CONFIGURE_STATE				2
//Типы конечных точек
#define EP_TYPE_BULK					0x00
#define EP_TYPE_CONTROL					0x01
#define EP_TYPE_ISO						0x02
#define EP_TYPE_INTERRUPT				0x03

extern uint8_t USB_DeviceDescriptor[];
extern uint8_t USB_ConfigDescriptor[];
extern uint8_t USB_StringLangDescriptor[];
extern uint8_t USB_StringManufacturingDescriptor[];
extern uint8_t USB_DeviceQualifierDescriptor[];
extern uint8_t USB_StringProdDescriptor[];
extern uint8_t HID_ReportDescriptor[];
extern uint8_t USB_StringSerialDescriptor[];

//Тип для расшифровки конфигурационного пакета
typedef struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} config_pack_t;
//Структура состояний конечных точек
typedef struct {
	uint16_t *tx_buf;
	uint8_t *rx_buf;
	uint16_t status;
	unsigned rx_cnt : 10;
	unsigned tx_flag : 1;
	unsigned rx_flag : 1;
	unsigned setup_flag : 1;
} ep_t;
//Статус и адрес соединения USB
typedef struct {
	uint8_t USB_Status;
	uint16_t USB_Addr;
} usb_dev_t;

extern ep_t endpoints[MAX_ENDPOINTS];

//Инициализация USB
void USB_Init();
//Получить статус соединения USB
uint8_t USB_GetState();
//Инициализация конечной точки
void EP_Init(uint8_t number, uint8_t type, uint16_t addr_tx, uint16_t addr_rx);
//Запись массива в буфер конечной точки извне прерывания
void EP_Write(uint8_t number, uint8_t *buf, uint16_t size);
//Чтение массива из буфера конечной точки
void EP_Read(uint8_t number, uint8_t *buf);
//Энумерация
void Enumerate(uint8_t number);
