#include "stm32f0xx.h"
#include "i2c.h"


#define I2C1_OWN_ADDRESS (0x5A)

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the I2C1 pins on GPIO PB6 PB7
  */
void Configure_GPIO_I2C1() {
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
}

/**
  * @brief  This function configures I2C1, master.
  */
void Configure_I2C1_Slave() {
    // Configure RCC for I2C1
    // (1) Enable the peripheral clock I2C1
    // (2) Use SysClk for I2C CLK
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // (1)
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW; // (2)

    // Configure I2C1, slave
    // (2) Timing register value is computed with the AN4235 xls file,
    // fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns
    // (3) Periph enable, receive interrupt enable
    // (4) 7-bit address = 0x5A
    // (5) Enable own address 1
    I2C1->TIMINGR = (uint32_t) 0x00B00000; // (2)
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE | I2C_CR1_ADDRIE; // (3)
    I2C1->OAR1 |= (uint32_t) (I2C1_OWN_ADDRESS << 1); // (4)
    I2C1->OAR1 |= I2C_OAR1_OA1EN; // (5)

    // Configure IT
    // (7) Set priority for I2C1_IRQn
    // (8) Enable I2C1_IRQn
    NVIC_SetPriority(I2C1_IRQn, 0); // (7)
    NVIC_EnableIRQ(I2C1_IRQn); // (8)
}

/**
  * @brief  This function configures I2C2, master.
  */
void Configure_I2C1_Master() {
    // Enable the peripheral clock I2C2
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure I2C2, master
    // (1) Timing register value is computed with the AN4235 xls file,
    // fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns
    // (2) Periph enable
    // (3) Slave address = 0x5A, write transfer, 1 byte to transmit, autoend
    I2C1->TIMINGR = (uint32_t) 0x00B01A4B; // (1)
    I2C1->CR1 = I2C_CR1_PE; // (2)
    I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (I2C1_OWN_ADDRESS << 1); // (3)
}
