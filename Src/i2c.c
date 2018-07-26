#include <main.h>
#include "i2c.h"

// I2C timeout milliseconds
#define Timeout 5

extern uint32_t stick;
static uint32_t Tickstart;
uint32_t cntr;

/**
  * Процедура инициализации I2C1 в режиме master с частотой интерфейса 100KHz
  */
void i2cm_init() {
    // Enable the peripheral clock of GPIOF
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

    // (1) open drain for I2C1 signals
    // (2) AF1 for I2C1 signals
    // (3) Select AF mode (10) on PF0 and PF1
    GPIOF->OTYPER |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1; // (1)
    GPIOF->AFR[0] = (GPIOF->AFR[0] & ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1))
                    | (1 << (0 * 4)) | (1 << (1 * 4)); // (2)
    GPIOF->MODER = (GPIOF->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1))
                   | (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1); // (3)

    // Enable the peripheral clock I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Use SysClk for I2C CLK
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

    // Configure I2C1, master
    // (1) Timing register value is computed with the AN4235 xls file,
    // fast Mode @100kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns
    // (2) Periph enable
    I2C1->TIMINGR = (uint32_t) 0x20303E5D; // 100khz 0ms 0ms  // 0x00B01A4B; // (1)
    I2C1->CR1 = I2C_CR1_PE; // (2)
}


/**
  * Функция стартует обмен. Выдаёт условие START, выдаёт адрес слейва с признаком R/W
  */
int8_t i2cm_Start(uint8_t slave_addr, uint8_t IsRead) {
    uint16_t TOcntr;
    Error_Handler();

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
    Error_Handler();

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
    Error_Handler();

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
    Error_Handler();

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

// HAL_I2C_Mem_Read(I2Cx, RDA5807_RandAccess_Addr << 1u, 0, I2C_MEMADD_SIZE_8BIT, buf, 1, RDA5807_TO);
// return 1 if all OK, 0 if NACK
uint8_t i2c_read(uint16_t i2c_addr, uint8_t count, uint8_t *data) {
    Error_Handler();

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
    Error_Handler();

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
        I2C1->TXDR = (uint32_t) (*data); // send data
        data++;
    }
    return 1;
}

// ======== HAL ==========

uint8_t I2C_WaitOnFlagUntilTimeout(uint32_t Flag, uint32_t Status) {
    while ((I2C1->ISR & Flag) == Status) {
        if ((stick - Tickstart) > Timeout) {
            return 1;
        }
    }
    return 0;
}

uint8_t I2C_WaitOnTXISFlagUntilTimeout() {
    while ((I2C1->ISR & I2C_ISR_TXIS) == RESET) {
        // Check if a NACK is detected
//        if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK) {
//            return HAL_ERROR;
//        }

        if (stick - Tickstart > Timeout) {
            return 1;
        }
    }
    return 0;
}

uint8_t I2C_WaitOnRXNEFlagUntilTimeout() {
    while ((I2C1->ISR & I2C_ISR_RXNE) == RESET) {
        // Check if a NACK is detected
//        if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK) {
//            return HAL_ERROR;
//        }
        // Check if a STOPF is detected

        if (stick - Tickstart > Timeout) {
            return 1;
        }
    }
    return 0;
}

uint8_t I2C_WaitOnSTOPFlagUntilTimeout() {
    while ((I2C1->ISR & I2C_ISR_STOPF) == RESET) {
        // Check if a NACK is detected
//        if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK) {
//            return HAL_ERROR;
//        }

        if ((stick - Tickstart) > Timeout) {
            return 1;
        }
    }
    return 0;
}

void I2C_TransferConfig(uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request) {
    // clear specific bits
    uint32_t tmp = I2C1->CR2 & ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND
                                 | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
    // set bits
    I2C1->CR2 = tmp | ((DevAddress & I2C_CR2_SADD) | (((uint32_t) Size << 16u) & I2C_CR2_NBYTES) | Mode | Request);
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval 0 - ok, other - error code
  */
uint8_t I2C_Mem_Read(uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint8_t Size) {
    Tickstart = stick;

    if (I2C_WaitOnFlagUntilTimeout(I2C_ISR_BUSY, I2C_ISR_BUSY)) Error_Handler(); //  return 1;

    I2C_TransferConfig(DevAddress, 1, 0, I2C_CR2_START); // 1 - addr size, 0 - SOFT_END_MODE

    if (I2C_WaitOnTXISFlagUntilTimeout()) Error_Handler(); // return 1;
    I2C1->TXDR = MemAddress & 0xFFu;
    if (I2C_WaitOnFlagUntilTimeout(I2C_ISR_TC, RESET)) Error_Handler(); // return 1;

    I2C_TransferConfig(DevAddress, Size, I2C_CR2_AUTOEND, I2C_CR2_START | I2C_CR2_RD_WRN);

    while (Size > 0) {
        I2C_WaitOnFlagUntilTimeout(I2C_ISR_RXNE, RESET);
        (*pData++) = (uint8_t) I2C1->RXDR;
        Size--;
    }

    if (I2C_WaitOnSTOPFlagUntilTimeout()) Error_Handler(); // return 1;
    I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN); // Clear CR2

    return 0;
}

/*
HAL_I2C_Mem_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)

    I2C_WaitOnFlagUntilTimeout(I2C_FLAG_BUSY, SET)
    I2C_RequestMemoryRead(DevAddress, MemAddress, MemAddSize)
        I2C_TransferConfig(DevAddress, MemAddSize, I2C_SOFTEND_MODE, I2C_GENERATE_START_WRITE) // 0u, I2C_CR2_START
            tmpreg = Instance->CR2;
            // clear tmpreg specific bits
            tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));
            tmpreg |= (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << 16) & I2C_CR2_NBYTES) | (uint32_t)Mode | (uint32_t)Request);
            Instance->CR2 = tmpreg;

        I2C_WaitOnTXISFlagUntilTimeout()
        Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress)
        I2C_WaitOnFlagUntilTimeout(I2C_FLAG_TC, RESET)

    I2C_TransferConfig(DevAddress, hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ)  // I2C_CR2_AUTOEND, (uint32_t)(I2C_CR2_START | I2C_CR2_RD_WRN)

 v  I2C_WaitOnFlagUntilTimeout(I2C_FLAG_RXNE, RESET)
 ^  (*hi2c->pBuffPtr++) = hi2c->Instance->RXDR;

    I2C_WaitOnSTOPFlagUntilTimeout()

    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
    // Clear Configuration Register 2
    I2C_RESET_CR2(hi2c);

 */

uint8_t I2C_Mem_Write(uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint8_t Size) {
    Tickstart = stick;

    if (I2C_WaitOnFlagUntilTimeout(I2C_ISR_BUSY, I2C_ISR_BUSY)) return 1;

    I2C_TransferConfig(DevAddress, 1, I2C_CR2_RELOAD, I2C_CR2_START); // I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE

    if (I2C_WaitOnTXISFlagUntilTimeout()) return 1;
    I2C1->TXDR = MemAddress & 0xFFu;
    if (I2C_WaitOnFlagUntilTimeout(I2C_ISR_TCR, RESET)) return 1;

    I2C_TransferConfig(DevAddress, Size, I2C_CR2_AUTOEND, 0); // I2C_AUTOEND_MODE, I2C_NO_STARTSTOP

    do {
        if (I2C_WaitOnTXISFlagUntilTimeout()) return 1;
        I2C1->TXDR = *pData++;
    } while (--Size > 0);

    if (I2C_WaitOnSTOPFlagUntilTimeout()) return 1;
    I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);

    return 0;
}

/*
HAL_I2C_Mem_Write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)

    I2C_WaitOnFlagUntilTimeout(I2C_FLAG_BUSY, SET)

    I2C_RequestMemoryWrite(DevAddress, MemAddress, MemAddSize)
        I2C_TransferConfig(hi2c, DevAddress, MemAddSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);
            tmpreg = Instance->CR2;
            // clear tmpreg specific bits
            tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));
            tmpreg |= (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << 16) & I2C_CR2_NBYTES) | (uint32_t)Mode | (uint32_t)Request);
            hi2c->Instance->CR2 = tmpreg;
        I2C_WaitOnTXISFlagUntilTimeout()
        Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress)
        I2C_WaitOnFlagUntilTimeout(I2C_FLAG_TCR, RESET)

    I2C_TransferConfig(hi2c, DevAddress, hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);

 v  I2C_WaitOnTXISFlagUntilTimeout()
        __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) == RESET
        I2C_IsAcknowledgeFailed
 ^  hi2c->Instance->TXDR = (*hi2c->pBuffPtr++);

    I2C_WaitOnSTOPFlagUntilTimeout()
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
    I2C_RESET_CR2()
      ((__HANDLE__)->Instance->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)))
*/


uint8_t I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint8_t Size) {
    Tickstart = stick;

    if (I2C_WaitOnFlagUntilTimeout(I2C_ISR_BUSY, I2C_ISR_BUSY)) return 1;

    I2C_TransferConfig(DevAddress, Size, I2C_CR2_AUTOEND, I2C_CR2_START); // I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE

    while (Size > 0) {
        if (I2C_WaitOnTXISFlagUntilTimeout()) return 1;
        I2C1->TXDR = (*pData++);
        Size--;
    }

    if (I2C_WaitOnSTOPFlagUntilTimeout()) return 1;
    I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);
    return 0;
}

uint8_t I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint8_t Size) {
    Tickstart = stick;

    if (I2C_WaitOnFlagUntilTimeout(I2C_ISR_BUSY, I2C_ISR_BUSY)) return 1;

    // I2C_AUTOEND_MODE, I2C_GENERATE_START_READ
    I2C_TransferConfig(DevAddress, Size, I2C_CR2_AUTOEND, I2C_CR2_START | I2C_CR2_RD_WRN);
    while (Size > 0U) {
        if (I2C_WaitOnRXNEFlagUntilTimeout()) return 1;

        // Read data from RXDR
        (*pData++) = (uint8_t) I2C1->RXDR;
        Size--;
    }
    if (I2C_WaitOnSTOPFlagUntilTimeout()) return 1;
    I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);

    return 0;
}
