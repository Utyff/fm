#include <main.h>
#include "spi.h"

/**
  * @brief  This function :
  *           - Enables GPIO clock
  *           - Configures the SPI1 pins on GPIO PA4 PA5 PA6 PA7
  */
void Configure_GPIO_SPI1(void) {
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // (1) Select AF mode (10) on PA5, PA7
    // (2) AF0 for SPI1 signals
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER7)) |
                   (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1); // (1)
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL7)); // (2)
}

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
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR; // (1)
    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // (2)
    SPI1->CR1 |= SPI_CR1_SPE; // (3)

    // Configure IT
    // (4) Set priority for SPI1_IRQn
    // (5) Enable SPI1_IRQn
//    NVIC_SetPriority(SPI1_IRQn, 0); // (4)
//    NVIC_EnableIRQ(SPI1_IRQn); // (5)
}

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

    if(SPI_EndRxTxTransaction()) return 1;
    // Clear overrun flag in 2 Lines communication mode because received is not read
//    if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
//    {
//        __HAL_SPI_CLEAR_OVRFLAG(hspi);
//    }
    return 0;
}
