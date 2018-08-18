#include <main.h>
#include "spi.h"

/**
  * @brief  This function :
  *           - Enables GPIO clock
  *           - Configures the SPI1 pins on GPIO PA5 PA7
  */
void Configure_GPIO_SPI1(void) {
    // Enable the peripheral clock of GPIOA
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Select output mode (01) on PA6 for DC and PB1 for CS
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE6)) | (GPIO_MODER_MODE6_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE1)) | (GPIO_MODER_MODE1_0);

    // Select AF mode (10) on PA5, PA7
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE7)) |
                   (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);
    // AF0 for SPI1 signals
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL7));
    // Very high speed PA5 PA7
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED5 | GPIO_OSPEEDER_OSPEED7;
}

/**
  * @brief  This function configures SPI1.
  */
void Configure_SPI1(void) {
    // Enable the peripheral clock SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Disable the selected SPI peripheral
    SPI1->CR1 &= ~SPI_CR1_SPE;

#define SPI_MODE_MASTER                 SPI_CR1_MSTR | SPI_CR1_SSI
#define SPI_DIRECTION_2LINES            0x00000000U
#define SPI_DATASIZE_8BIT               0x00000000U
#define SPI_POLARITY_LOW                0x00000000U
#define SPI_PHASE_1EDGE                 0x00000000U
#define SPI_NSS_SOFT                    SPI_CR1_SSM
#define SPI_BAUDRATEPRESCALER_8         SPI_CR1_BR_1
#define SPI_FIRSTBIT_MSB                0x00000000U
#define SPI_CRCCALCULATION_DISABLE      0x00000000U

    // Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
    // Communication speed, First bit and CRC calculation state
    WRITE_REG(SPI1->CR1, (SPI_MODE_MASTER | SPI_DIRECTION_2LINES | SPI_DATASIZE_8BIT |
                          SPI_POLARITY_LOW | SPI_PHASE_1EDGE | (SPI_NSS_SOFT & SPI_CR1_SSM) |
                          SPI_BAUDRATEPRESCALER_8 | SPI_FIRSTBIT_MSB | SPI_CRCCALCULATION_DISABLE) );

#define SPI_TIMODE_DISABLE  0x00000000U

    // Configure : NSS management
    WRITE_REG(SPI1->CR2, (((SPI_NSS_SOFT >> 16U) & SPI_CR2_SSOE) | SPI_TIMODE_DISABLE));

    //---------------------------- SPIx CRCPOLY Configuration ------------------
    // Configure : CRC Polynomial
    WRITE_REG(SPI1->CRCPR, 7);

    // Enable the selected SPI peripheral
    SPI1->CR1 |= SPI_CR1_SPE;
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

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval 0 - ok, other - error code
  */
uint8_t SPI_Transmit(uint8_t *pData, uint16_t Size) {
    Tickstart = stick;

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

    // wait till the BSY flag
    if (SPI_WaitFlagStateUntilTimeout(SPI_SR_BSY, RESET) != 0) {
        return 1;
    }
    return 0;
}
