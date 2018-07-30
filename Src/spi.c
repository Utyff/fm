#include <main.h>
#include "spi.h"

/**
  * @brief  This function :
  *           - Enables GPIO clock
  *           - Configures the SPI1 pins on GPIO PA5 PA7
  */
void Configure_GPIO_SPI1(void) {
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Select output mode (01) on PA6 for DC and PB1 for CS
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER6)) | (GPIO_MODER_MODER6_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER1)) | (GPIO_MODER_MODER1_0);

    // (1) Select AF mode (10) on PA5, PA7
    // (2) AF0 for SPI1 signals
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER7)) |
                   (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1); // (1)
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL7)); // (2)
}

/*  PA5     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
*/

/**
  * @brief  This function configures SPI1.
  */
void Configure_SPI1(void) {
    // Enable the peripheral clock SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure SPI1 in master
    // (1) Master selection, BR: Fpclk/256 (due to C27 on the board, SPI_CLK is set to the minimum)
    //       CPOL and CPHA at zero (rising first edge)
    // (2) Slave select output enabled, RXNE IT, 8-bit Rx fifo
    // (3) Enable SPI1
//    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR; // (1)
//    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // (2)
//    SPI1->CR1 |= SPI_CR1_SPE; // (3)

    // Configure IT
    // (4) Set priority for SPI1_IRQn
    // (5) Enable SPI1_IRQn
//    NVIC_SetPriority(SPI1_IRQn, 0); // (4)
//    NVIC_EnableIRQ(SPI1_IRQn); // (5)

    // Disable the selected SPI peripheral
    SPI1->CR1 &= ~SPI_CR1_SPE;
    // SPI_RXFIFO_THRESHOLD_QF
    // Configure : SPI Mode, Communication Mode, Clock polarity and phase, NSS management,
    // Communication speed, First bit and CRC calculation state
#define SPI_POLARITY_LOW 0u
#define SPI_PHASE_1EDGE 0u
#define SPI_BAUDRATEPRESCALER_8  SPI_CR1_BR_1
#define SPI_FIRSTBIT_MSB   0u
#define SPI_DATASIZE_8BIT  0x00000700u

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_BIDIMODE | SPI_POLARITY_LOW
                | SPI_PHASE_1EDGE | SPI_CR1_SSM | SPI_BAUDRATEPRESCALER_8 | SPI_FIRSTBIT_MSB;
    // Configure : NSS management, TI Mode, NSS Pulse, Data size and Rx Fifo Threshold
    SPI1->CR2 = SPI_CR2_NSSP | SPI_DATASIZE_8BIT;
    SPI1->CR1 |= SPI_CR1_SPE; // (3)

}
/*
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
*/


/**
  * @brief Handle SPI FIFO Communication Timeout.
  * @param Fifo Fifo to check
  * @param State Fifo state to check
  * @retval 0 - ok
  */
uint8_t SPI_WaitFifoStateUntilTimeout(uint32_t Fifo, uint32_t State) {
    while ((SPI1->SR & Fifo) != State) {
        if (stick - Tickstart >= Timeout) {
            return 1;
        }
    }
    return 0;
}


/**
  * @brief Handle SPI Communication Timeout.
  * @param Flag SPI flag to check
  * @param State flag state to check
  * @retval 0 - ok
  */
uint8_t SPI_WaitFlagStateUntilTimeout(uint32_t Flag, uint32_t State) {
    while ((SPI1->SR & Flag) != State) {
        if (stick - Tickstart >= Timeout) {
            return 1;
        }
    }

    return 0;
}

#define SPI_FTLVL_EMPTY                 (0x00000000U)

/**
  * @brief  Handle the check of the RXTX or TX transaction complete.
  * @retval 0 - ok
  */
uint8_t SPI_EndRxTxTransaction() {
    // Control if the TX fifo is empty
    if (SPI_WaitFifoStateUntilTimeout(SPI_SR_FTLVL, SPI_FTLVL_EMPTY) != 0) {
        return 1;
    }

    // Control the BSY flag
    if (SPI_WaitFlagStateUntilTimeout(SPI_SR_BSY, RESET) != 0) {
        return 1;
    }

    // Control if the RX fifo is empty
//    if (SPI_WaitFifoStateUntilTimeout(SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY) != 0) {
//        return 1;
//    }
    return 0;
}

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval 0 - ok, other - error code
  */
uint8_t SPI_Transmit(uint8_t *pData, uint16_t Size) {
    Tickstart = stick;

//#define SPI_1LINE_TX(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIOE)
    SET_BIT(SPI1->CR1, SPI_CR1_BIDIOE);
    while (Size > 0U) {
        if (SPI1->SR & SPI_SR_TXE) {
            if (Size > 1U) {
                // write on the data register in packing mode
                SPI1->DR = *((uint16_t *) pData);
                pData += sizeof(uint16_t);
                Size -= 2U;
            } else {
                *((__IO uint8_t *) &SPI1->DR) = (*pData++);
                Size--;
            }
        } else {
            if (stick - Tickstart >= 5) {
                return 1;
            }
        }
    }

    if (SPI_EndRxTxTransaction()) return 1;
    // Clear overrun flag in 2 Lines communication mode because received is not read
//    if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
//    {
//        __HAL_SPI_CLEAR_OVRFLAG(hspi);
//    }
    return 0;
}
