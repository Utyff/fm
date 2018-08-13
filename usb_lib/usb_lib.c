#include "usb_lib.h"

//uint8_t set_featuring;
//uint16_t tx_len;
//uint8_t *tx_message, tx_num;

usb_dev_t USB_Dev;
ep_t endpoints[MAX_ENDPOINTS];

void EP_SendNull(uint8_t number){
	uint32_t timeout = 100000;
	uint16_t status = USB_ -> EPnR[number];
	USB_BTABLE_ -> EP[number].USB_COUNT_TX = 0;
	status = KEEP_STAT_RX(status);
	status = SET_VALID_TX(status);
	status = KEEP_DTOG_RX(status);
	status = SET_DTOG_TX(status);
	USB_ -> EPnR[number] = status;
	endpoints[number].tx_flag = 0;
	while (!endpoints[number].tx_flag){
		if (timeout) timeout--;
		else break;
	}
	endpoints[number].tx_flag = 0;
}

void EP_WaitNull(uint8_t number){
	uint32_t timeout = 100000;
	uint16_t status = USB_ -> EPnR[number];
	status = SET_VALID_RX(status);
	status = KEEP_STAT_TX(status);
	status = KEEP_DTOG_TX(status);
	status = SET_DTOG_RX(status);
	USB_ -> EPnR[number] = status;
	endpoints[number].rx_flag = 0;
	while (!endpoints[number].rx_flag){
		if (timeout) timeout--;
		else break;
	}
	endpoints[number].rx_flag = 0;
}

void Enumerate(uint8_t number){
	config_pack_t *packet = (config_pack_t *)endpoints[number].rx_buf;
	uint8_t length;
	uint16_t status;
	if ((endpoints[number].rx_flag) && (endpoints[number].setup_flag)){
		switch (packet -> bmRequestType){
		case 0x80:
			switch (packet -> bRequest){
			case GET_DESCRIPTOR:
				switch (packet -> wValue){
				case DEVICE_DESCRIPTOR:
					length = ((packet -> wLength < DEVICE_DESCRIPTOR_SIZE_BYTE) ? packet -> wLength : DEVICE_DESCRIPTOR_SIZE_BYTE);
					EP_Write(number, USB_DeviceDescriptor, length);
					EP_WaitNull(number);
					break;
				case CONFIGURATION_DESCRIPTOR:
					length = ((packet -> wLength < CONFIG_DESCRIPTOR_SIZE_BYTE) ? packet -> wLength : CONFIG_DESCRIPTOR_SIZE_BYTE);
					EP_Write(number, USB_ConfigDescriptor, length);
					EP_WaitNull(number);
					break;
				case STRING_LANG_DESCRIPTOR:
					length = ((packet -> wLength < STRING_LANG_DESCRIPTOR_SIZE_BYTE) ? packet -> wLength : STRING_LANG_DESCRIPTOR_SIZE_BYTE);
					EP_Write(number, USB_StringLangDescriptor, length);
					EP_WaitNull(number);
					break;
				case STRING_MAN_DESCRIPTOR:
					length = ((packet -> wLength < STRING_MANUFACTURING_DESCRIPTOR_SIZE_BYTE) ? packet -> wLength : STRING_MANUFACTURING_DESCRIPTOR_SIZE_BYTE);
					EP_Write(number, USB_StringManufacturingDescriptor, length);
					EP_WaitNull(number);
					break;
				case STRING_PROD_DESCRIPTOR:
					length = ((packet -> wLength < STRING_PRODUCT_DESCRIPTOR_SIZE_BYTE) ? packet -> wLength : STRING_PRODUCT_DESCRIPTOR_SIZE_BYTE);
					EP_Write(number, USB_StringProdDescriptor, length);
					EP_WaitNull(number);
					break;
				case STRING_SN_DESCRIPTOR:
					length = ((packet -> wLength < STRING_SERIAL_DESCRIPTOR_SIZE_BYTE) ? packet -> wLength : STRING_SERIAL_DESCRIPTOR_SIZE_BYTE);
					EP_Write(number, USB_StringSerialDescriptor, length);
					EP_WaitNull(number);
					break;
				case DEVICE_QALIFIER_DESCRIPTOR:
					length = ((packet -> wLength < DEVICE_QALIFIER_SIZE_BYTE) ? packet -> wLength : DEVICE_QALIFIER_SIZE_BYTE);
					EP_Write(number, USB_DeviceQualifierDescriptor, length);
					EP_WaitNull(number);
					break;
				default:
					break;
				}
				break;
			case GET_STATUS:
				status = 0;
				//отправляем состояние
				EP_Write(0, (uint8_t *)&status, 2);
				EP_WaitNull(number);
				break;
			default:
				break;
			}
			break;
		case 0x00:
			switch (packet -> bRequest){
			case SET_ADDRESS:
				//Сразу присвоить адрес в DADDR нельзя, так как хост ожидает подтверждения
				//приема со старым адресом
				USB_Dev.USB_Addr = packet -> wValue;
				EP_SendNull(number);
				//Присваиваем новый адрес устройству
				USB -> DADDR = USB_DADDR_EF | USB_Dev.USB_Addr;
				//Устанавливаем состояние в "Адресованно"
				USB_Dev.USB_Status = USB_ADRESSED_STATE;
				break;
			case SET_CONFIGURATION:
				//Устанавливаем состояние в "Сконфигурировано"
				USB_Dev.USB_Status = USB_CONFIGURE_STATE;
				EP_SendNull(number);
				break;
			default:
				break;
			}
			break;
		case 0xA1:
			switch (packet -> bRequest){
				case GET_MAX_LUN:
					status = 0;
					EP_Write(0, (uint8_t *)&status, 1);
					EP_WaitNull(number);
					break;
				default:
					break;
			}
			break;
		default:
		//	EP_SendNull(number);
			break;
		}
		status = USB_ -> EPnR[number];
		status = SET_VALID_RX(status);
		status = SET_NAK_TX(status);
		status = CLEAR_DTOG_TX(status);
		status = CLEAR_DTOG_RX(status);
		USB_ -> EPnR[number] = status;
		endpoints[number].rx_flag = 0;
		endpoints[number].tx_flag = 0;
	}
}

/*
 * Инициализация конечной точки
 * number - номер (0...7)
 * type - тип конечной точки (EP_TYPE_BULK, EP_TYPE_CONTROL, EP_TYPE_ISO, EP_TYPE_INTERRUPT)
 * addr_tx - адрес передающего буфера в периферии USB
 * addr_rx - адрес приемного буфера в периферии USB
 * Размер приемного буфера - фиксированный 64 байта
 */
void EP_Init(uint8_t number, uint8_t type, uint16_t addr_tx, uint16_t addr_rx){
	USB_ -> EPnR[number] = (type << 9) | (number & USB_EPnR_EA);
	USB_ -> EPnR[number] ^= USB_EPnR_STAT_RX | USB_EPnR_STAT_TX_1;
	USB_BTABLE_ -> EP[number].USB_ADDR_TX = addr_tx;
	USB_BTABLE_ -> EP[number].USB_COUNT_TX = 0;
	USB_BTABLE_ -> EP[number].USB_ADDR_RX = addr_rx;
	USB_BTABLE_ -> EP[number].USB_COUNT_RX = 0x8400;	//размер приемного буфера
	endpoints[number].tx_buf = (uint16_t *)(USB_BTABLE_BASE + addr_tx);
	endpoints[number].rx_buf = (uint8_t *)(USB_BTABLE_BASE + addr_rx);
}
//Инициализация USB
void USB_Init(){
	RCC -> APB1ENR |= RCC_APB1ENR_USBEN;
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	//Ремапим ноги с USB
//	SYSCFG -> CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
	//Разрешаем прерывания по RESET и CTRM
	USB -> CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM;
	//Сбрасываем флаги
	USB -> ISTR = 0;
	//Включаем подтяжку на D+
	USB -> BCDR |= USB_BCDR_DPPU;
	NVIC_EnableIRQ(USB_IRQn);
}
//Обработчик прерываний USB
void USB_IRQHandler(){
	uint8_t n;
	if (USB -> ISTR & USB_ISTR_RESET){
		//Переинициализируем регистры
		USB -> CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM;
		USB -> ISTR = 0;
		//Создаем 0 конечную точку, типа CONTROL
		EP_Init(0, EP_TYPE_CONTROL, 128, 256);
		//Создаем 1 конечную точку, типа BULK
		EP_Init(1, EP_TYPE_BULK, 384, 512);
		//Обнуляем адрес устройства
		USB -> DADDR = USB_DADDR_EF;
		//Присваиваем состояние в DEFAULT (ожидание энумерации)
		USB_Dev.USB_Status = USB_DEFAULT_STATE;
	}
	if (USB -> ISTR & USB_ISTR_CTR){
		//Определяем номер конечной точки, вызвавшей прерывание
		n = USB -> ISTR & USB_ISTR_EPID;
		//Копируем количество принятых байт
		endpoints[n].rx_cnt = USB_BTABLE_ -> EP[n].USB_COUNT_RX;
		//Копируем содержимое EPnR этой конечной точки
		endpoints[n].status = USB_ -> EPnR[n];
		//Обновляем состояние флажков
		endpoints[n].rx_flag = (endpoints[n].status & USB_EPnR_CTR_RX) ? 1 : 0;
		endpoints[n].setup_flag = (endpoints[n].status & USB_EPnR_SETUP) ? 1 : 0;
		endpoints[n].tx_flag = (endpoints[n].status & USB_EPnR_CTR_TX) ? 1 : 0;
		//Очищаем флаги приема и передачи
		endpoints[n].status = CLEAR_CTR_RX_TX(endpoints[n].status);
		USB_ -> EPnR[n] = endpoints[n].status;
	}
}
/*
 * Функция записи массива в буфер конечной точки
 * number - номер конечной точки
 * *buf - адрес массива с записываемыми данными
 * size - размер массива
 */
void EP_Write(uint8_t number, uint8_t *buf, uint16_t size){
	uint8_t i;
	uint32_t timeout = 1000000;
	uint16_t status = USB_ -> EPnR[number];
	if (size > 64) size = 64;
/*
 * ВНИМАНИЕ КОСТЫЛЬ
 * Из-за ошибки записи в область USB/CAN SRAM с 8-битным доступом
 * пришлось упаковывать массив в 16-бит, сообветственно размер делить
 * на 2, если он был четный, или делить на 2 + 1 если нечетный
 */
	uint16_t temp = (size & 0x0001) ? (size + 1) / 2 : size / 2;
	uint16_t *buf16 = (uint16_t *)buf;
	for (i = 0; i < temp; i++){
		endpoints[number].tx_buf[i] = buf16[i];
	}
	//Количество передаваемых байт
	USB_BTABLE_ -> EP[number].USB_COUNT_TX = size;

	status = KEEP_STAT_RX(status);		//RX в NAK
	status = SET_VALID_TX(status);		//TX в VALID
	status = KEEP_DTOG_TX(status);
	status = KEEP_DTOG_RX(status);
	USB_ -> EPnR[number] = status;

	endpoints[number].tx_flag = 0;
	while (!endpoints[number].tx_flag){
		if (timeout) timeout--;
		else break;
	}
}
/*
 * Функция чтения массива из буфера конечной точки
 * number - номер конечной точки
 * *buf - адрес массива куда считываем данные
 */
void EP_Read(uint8_t number, uint8_t *buf){
	uint32_t timeout = 100000;
	uint16_t status, i;
	status = USB_ -> EPnR[number];
	status = SET_VALID_RX(status);
	status = SET_NAK_TX(status);
	status = KEEP_DTOG_TX(status);
	status = KEEP_DTOG_RX(status);
	USB_ -> EPnR[number] = status;
	endpoints[number].rx_flag = 0;
	while (!endpoints[number].rx_flag){
		if (timeout) timeout--;
		else break;
	}
	for (i = 0; i < endpoints[number].rx_cnt; i++){
		buf[i] = endpoints[number].rx_buf[i];
	}
}
//Функция получения состояния соединения USB
uint8_t USB_GetState(){
	return USB_Dev.USB_Status;
}
