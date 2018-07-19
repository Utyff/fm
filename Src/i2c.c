#include "i2c.h"

#define I2C1_OWN_ADDRESS (0x5A)

/**
  * ��������� ������������� i2c (I2C1 ��� I2C2) � ������ master � �������� �������� ����������
  */
void i2cm_init(I2C_TypeDef *I2Cx, uint32_t i2c_clock) {
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

    /*/ �������� ������������ GPIO � I2C1
    if (I2Cx == I2C1)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    else
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // ����������� I2C
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

    // ����������� ���� GPIO
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
  * ������� �������� �����. ����� ������� START, ����� ����� ������ � ��������� R/W
  */
int8_t i2cm_Start(I2C_TypeDef *I2Cx, uint8_t slave_addr, uint8_t IsRead, uint16_t TimeOut) {
    uint16_t TOcntr;

    /*/ ����� ������� START
    I2C_GenerateSTART(I2Cx, ENABLE);
    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) && TOcntr) { TOcntr--; }
    if (!TOcntr)
        return I2C_ERR_HWerr;

    // ����� ����� ������ � ������� ��������� ������
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
  * ������� ����� ������� STOP
  */
int8_t i2cm_Stop(I2C_TypeDef *I2Cx, uint16_t TimeOut) {
/*    I2C_GenerateSTOP(I2Cx, ENABLE);
    uint16_t TOcntr = TimeOut;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) && TOcntr);
    if (!TOcntr)
        return I2C_ERR_HWerr;
//*/
    return I2C_Ok;
}
//==============================================================================


/**
  * ������� ����� �� ���� ������ ���� �� ������
  */
int8_t i2cm_WriteBuff(I2C_TypeDef *I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut) {
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
//==============================================================================


/**
  * ������� ������ ������ ���� � ���� � ����� ������� STOP
  */
int8_t i2cm_ReadBuffAndStop(I2C_TypeDef *I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut) {
/*    uint16_t TOcntr;

    // ��������� ������ ������������� ACK
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    while (len-- != 1) {
        TOcntr = TimeOut;
        while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) && TOcntr) { TOcntr--; }
        *pbuf++ = I2C_ReceiveData(I2Cx);
    }

    // ��������� ������ ACK
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);               // ����� STOP

    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) && TOcntr) { TOcntr--; }
    *pbuf++ = I2C_ReceiveData(I2Cx);             // ������ N-2 ����

    i2cm_Stop(I2Cx, TimeOut);
//*/
    return I2C_Ok;
}
