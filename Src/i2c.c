#include "i2c.h"

extern uint32_t stick;
uint32_t cntr;

/**
  * Процедура инициализации i2c (I2C1 или I2C2) в режиме master с заданной частотой интерфейса
  */
void i2cm_init() {
    // Enable the peripheral clock of GPIOF
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

    // (1) open drain for I2C signals
    // (2) AF1 for I2C signals
    // (3) Select AF mode (10) on PF0 and PF1
    GPIOF->OTYPER |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1; // (1)
    GPIOF->AFR[0] = (GPIOF->AFR[0] & ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1))
                    | (1 << (0 * 4)) | (1 << (1 * 4)); // (2)
    GPIOF->MODER = (GPIOF->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1))
                   | (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1); // (3)

    // Enable the peripheral clock I2C2
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Use SysClk for I2C CLK
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

    // Configure I2C2, master
    // (1) Timing register value is computed with the AN4235 xls file,
    // fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns
    // (2) Periph enable
    // (3) Slave address = 0x5A, write transfer, 1 byte to transmit, autoend
    I2C1->TIMINGR = (uint32_t) 0x20303E5D; // 100khz 0ms 0ms  // 0x00B01A4B; // (1)
    I2C1->CR1 = I2C_CR1_PE; // (2)
    //I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (I2C1_OWN_ADDRESS << 1); // (3)

    /*/ Стартуем тактирование GPIO и I2C1
    if (I2Cx == I2C1)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    else
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Настраиваем I2C
    I2C_Cmd(I2Cx, DISABLE);
    I2C_DeInit(I2Cx);
    I2C_InitTypeDef i2c_InitStruct;
    i2c_InitStruct.I2C_Mode = I2C_Mode_I2C;
    i2c_InitStruct.I2C_ClockSpeed = i2c_clock;
    i2c_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c_InitStruct.I2C_Ack = I2C_Ack_Enable;
    i2c_InitStruct.I2C_OwnAddress1 = 0;
    I2C_Cmd(I2Cx, ENABLE);
    I2C_Init(I2Cx, &i2c_InitStruct);

    // Настраиваем ноги GPIO
    GPIO_InitTypeDef InitStruct;
    InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    if (I2Cx == I2C1)
        InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    else
        InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;

    GPIO_Init(GPIOB, &InitStruct); //*/
}


/**
  * Функция стартует обмен. Выдаёт условие START, выдаёт адрес слейва с признаком R/W
  */
int8_t i2cm_Start(uint8_t slave_addr, uint8_t IsRead) {
    uint16_t TOcntr;

    // Выдаём условие START
    // I2C_GenerateSTART(I2Cx, ENABLE);
/*    I2Cx->CR2 |= I2C_CR2_START;

    cntr = stick;
    while (I2C1->ISR & I2C_ISR_BUSY) if (stick - cntr > 5) return 0;  // check busy
    cntr = stick;
    while (I2C1->CR2 & I2C_CR2_START) if (stick - cntr > 5) return 0; // check start
    // read three bytes
    I2C1->CR2 = 1 << 16 | slave_addr | 1 | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;

    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) && TOcntr) { TOcntr--; }
    if (!TOcntr)
        return I2C_ERR_HWerr;

    // Выдаём адрес слейва и ожидаем окончания выдачи
    if (IsRead) {
        I2C_Send7bitAddress(I2Cx, slave_addr << 1, I2C_Direction_Receiver);
        TOcntr = TimeOut;
        while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && TOcntr) { TOcntr--; }
    } else {
        I2C_Send7bitAddress(I2Cx, slave_addr << 1, I2C_Direction_Transmitter);
        TOcntr = TimeOut;
        while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && TOcntr) { TOcntr--; }
    }

    if (!TOcntr)
        return I2C_ERR_NotConnect;
//*/
    return I2C_Ok;
}


/**
  * Функция выдаёт условие STOP
  */
int8_t i2cm_Stop() {
/*    I2C_GenerateSTOP(I2Cx, ENABLE);
    uint16_t TOcntr = TimeOut;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) && TOcntr);
    if (!TOcntr)
        return I2C_ERR_HWerr;
//*/
    return I2C_Ok;
}


/**
  * Функция выдаёт на шину массив байт из буфера
  */
int8_t i2cm_WriteBuff(uint8_t *pbuf, uint16_t len) {
/*    uint16_t TOcntr;

    while (len--) {
        I2C_SendData(I2Cx, *(pbuf++));
        TOcntr = TimeOut;
        while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && TOcntr) { TOcntr--; }
        if (!TOcntr)
            return I2C_ERR_NotConnect;
    }
//*/
    return I2C_Ok;
}


/**
  * Функция читает массив байт с шины и выдаёт условие STOP
  */
int8_t i2cm_ReadBuffAndStop(uint8_t *pbuf, uint16_t len) {
/*    uint16_t TOcntr;

    // Разрешаем выдачу подтверждений ACK
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    while (len-- != 1) {
        TOcntr = TimeOut;
        while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) && TOcntr) { TOcntr--; }
        *pbuf++ = I2C_ReceiveData(I2Cx);
    }

    // Запрещаем выдачу ACK
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);               // Выдаём STOP

    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) && TOcntr) { TOcntr--; }
    *pbuf++ = I2C_ReceiveData(I2Cx);             // Читаем N-2 байт

    i2cm_Stop(I2Cx, TimeOut);
//*/
    return I2C_Ok;
}

uint32_t cntr;

// HAL_I2C_Mem_Read(I2Cx, RDA5807_RandAccess_Addr << 1u, 0, I2C_MEMADD_SIZE_8BIT, buf, 1, RDA5807_TO);
// return 1 if all OK, 0 if NACK
uint8_t i2c_read(uint16_t i2c_addr, uint8_t count, uint8_t *data) {

    cntr = stick;
    while (I2C1->ISR & I2C_ISR_BUSY) if (stick - cntr > 5) return 0;  // check busy
    cntr = stick;
    while (I2C1->CR2 & I2C_CR2_START) if (stick - cntr > 5) return 0; // check start
    // byte to read count | i2c addr | autoend enable | read operation
    I2C1->CR2 = count << 16 | i2c_addr | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;

    for (int i = 0; i < count; ++i) {
        cntr = stick;
        while (!(I2C1->ISR & I2C_ISR_RXNE)) { // wait for data
            if (I2C1->ISR & I2C_ISR_NACKF) {
                I2C1->ICR |= I2C_ICR_NACKCF;
                return 0;
            }
            if (stick - cntr > 5) return 0;
        }
        *data = (uint8_t) I2C1->RXDR;
        data++;
    }

    return 1;
}

uint8_t i2c_write(uint16_t i2c_addr, uint8_t count, uint8_t *data) {
    cntr = stick;
    while (I2C1->ISR & I2C_ISR_BUSY) if (stick - cntr > 5) return 0;  // check busy
    cntr = stick;
    while (I2C1->CR2 & I2C_CR2_START) if (stick - cntr > 5) return 0; // check start
    I2C1->CR2 = count << 16 | i2c_addr | I2C_CR2_AUTOEND;
    // now start transfer
    I2C1->CR2 |= I2C_CR2_START;

    for (int i = 0; i < count; ++i) {
        cntr = stick;
        while (!(I2C1->ISR & I2C_ISR_TXIS)) { // ready to transmit
            if (I2C1->ISR & I2C_ISR_NACKF) {
                I2C1->ICR |= I2C_ICR_NACKCF;
                return 0;
            }
            if (stick - cntr > 5) return 0;
        }
        I2C1->TXDR = (uint32_t)data; // send data
        data++;
    }
    return 1;
}

uint8_t i2c_mem_read(uint16_t i2c_addr, uint8_t mem_addr, uint8_t count, uint8_t *data) {
    if( !i2c_write(i2c_addr, 1, &mem_addr)) return 0;
    if( !i2c_read(i2c_addr, count, data)) return 0;
}

uint8_t i2c_mem_write(uint16_t i2c_addr, uint8_t mem_addr, uint8_t count, uint8_t *data) {
    if( !i2c_write(i2c_addr, 1, &mem_addr)) return 0;
    if( !i2c_write(i2c_addr, count, data)) return 0;
}
