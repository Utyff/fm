#include <main.h>
#include "i2c.h"


/**
  * Процедура инициализации I2C1 в режиме master с частотой интерфейса 100KHz
  * PB6 - I2C1_SCL
  * PB7 - I2C1_SDA
  */
void i2c_init() {
    // Enable the peripheral clock of GPIOB
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // open drain for I2C1 signals for PB6 and PB7
    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
    // AF1 for I2C1 signals for PB6 and PB7
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7))
                    | (1u << (6u * 4)) | (1u << (7u * 4));
    // Select AF mode (10) on PB6 and PB7
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7))
                   | (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);

    // Enable the peripheral clock I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Use APBCLK for I2C CLK
    RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;

    // Configure I2C1, master
    // fast Mode @100kHz with I2CCLK = 32MHz, rise time = 0ns, fall time = 0ns
    I2C1->TIMINGR = 0x00707CBBu;
    // Periph enable
    I2C1->CR1 |= I2C_CR1_PE;
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
