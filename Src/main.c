#include "stm32f0xx.h"
#include <rda5807m.h>
#include "main.h"


#pragma clang diagnostic ignored "-Wmissing-noreturn"


void Configure_GPIO_LED(void);

void Configure_GPIO_Button(void);

void Configure_EXTI(void);

void Configure_GPIO_USART2();

void Configure_USART2(void);


uint8_t send = 0;
uint8_t string2send[32] = "STm\n";
uint32_t stick = 0;
uint32_t Tickstart; // operation start time. For detect timeout


int main(void) {

    SysTick_Config(48000);
    Configure_GPIO_LED();
    Configure_GPIO_Button();
    Configure_EXTI();
    Configure_GPIO_USART2();
    Configure_USART2();
    Configure_GPIO_SPI1();
    Configure_SPI1();

    rda5807_init();

    //ssd1306_Init();
    ssd1306_TestAll();

    LED1_TOGGLE();

    while (1) {
        LED1_TOGGLE();
        LED2_TOGGLE();

        prints("\n\rtuned freq: ");
        printi(rda5807_GetFreq_In100Khz());

        Delay(300);
    }
}

/**
  * @brief  This function :
             - Enables LEDs GPIO clock
             - Configures the Green LED pin on GPIO PA4
             - Configures the orange LED pin on GPIO PB8
  */
__INLINE void Configure_GPIO_LED(void) {
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Select output mode (01) on PA4 and PB8
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER4)) | (GPIO_MODER_MODER4_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_0);
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the Push Button GPIO PA0
  */
__INLINE void Configure_GPIO_Button(void) {
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Select mode
    // Select input mode (00) on PA0
//    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER0));
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the USART2 pins on GPIO PB6 PB7
  */
__INLINE void Configure_GPIO_USART2(void) {
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // GPIO configuration for USART2 signals
    // (1) Select AF mode (10) on PA2 and PA3
    // (2) AF1 for USART2 signals
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3))
                   | (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1); // (1)
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3))
                    | (1u << (2u * 4)) | (1u << (3u * 4)); // (2)
}

/**
  * @brief  This function configures USART2.
  */
__INLINE void Configure_USART2(void) {
    // Enable the peripheral clock USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure USART2
    // (1) oversampling by 16, 115200 baud
    // (2) 8 data bit, 1 start bit, 1 stop bit, no parity
    USART2->BRR = 480000 / 1152; // (1)
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE; // (2)

    // polling idle frame Transmission
    while ((USART2->ISR & USART_ISR_TC) != USART_ISR_TC) {
        // add time out here for a robust application
    }
    USART2->ICR |= USART_ICR_TCCF;// clear TC flag
    USART2->CR1 |= USART_CR1_TCIE;// enable TC interrupt

    // Configure IT
    // (3) Set priority for USART2_IRQn
    // (4) Enable USART2_IRQn
    NVIC_SetPriority(USART2_IRQn, 0); // (3)
    NVIC_EnableIRQ(USART2_IRQn); // (4)
}


/**
  * @brief  This function configures EXTI.
  */
__INLINE void Configure_EXTI(void) {
    // Configure Syscfg, exti and nvic for pushbutton PA0
    // (1) PA0 as source input
    // (2) unmask port 0
    // (3) Rising edge
    // (4) Set priority
    // (5) Enable EXTI0_1_IRQn
//    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PA; // (1)
//    EXTI->IMR |= EXTI_IMR_MR0; // (2)
//    EXTI->RTSR |= EXTI_RTSR_TR0; // (3)
//    NVIC_SetPriority(EXTI0_1_IRQn, 0); // (4)
//    NVIC_EnableIRQ(EXTI0_1_IRQn); // (5)
}

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
void NMI_Handler(void) {
}

/**
  * @brief  This function handles Hard Fault exception.
  */
void HardFault_Handler(void) {
    // Go to infinite loop when Hard Fault exception occurs
    while (1) {
    }
}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void) {
    stick++;
}


/**
  * @brief  This function handles EXTI 0 1 interrupt request.
  */
void EXTI0_1_IRQHandler(void) {
    EXTI->PR |= 1;
}

/**
  * @brief  This function handles USART2 interrupt request.
  */
void USART2_IRQHandler(void) {
    if ((USART2->ISR & USART_ISR_TC) == USART_ISR_TC) {
        if (string2send[send] == 0) {
            send = 0;
            USART2->ICR |= USART_ICR_TCCF; // Clear transfer complete flag
        } else {
            // clear transfer complete flag and fill TDR with a new char
            USART2->TDR = string2send[send++];
        }
    } else {
        NVIC_DisableIRQ(USART2_IRQn); // Disable USART2_IRQn
    }
}

/**
 * @param delay in milliseconds
 */
void Delay(uint32_t delay) {
    uint32_t start = stick;
    while (stick - start < delay);
}

void _strcpy(uint8_t *dst, const uint8_t *src) {
    int i = 0;
    do {
        dst[i] = src[i];
    } while (src[i++] != 0);
}

#define hex2char(hex) (uint8_t)((hex)<=9u ? (hex) + '0' : (hex) + 'a' - 10u)

void printi(uint16_t val) {
    uint8_t buf[5];

    buf[0] = hex2char(val >> 12u & 0xFu);
    buf[1] = hex2char(val >> 8u & 0xFu);
    buf[2] = hex2char(val >> 4u & 0xFu);
    buf[3] = hex2char(val & 0xFu);
    buf[4] = 0;

    prints((char *) buf);
}

void prints(const char *str) {
    // wait till end current transmission
    while (send != 0);

    _strcpy(string2send, (uint8_t *) str);
    // start USART transmission. Will initiate TC if TXE
    USART2->TDR = string2send[send++];
}

#pragma clang diagnostic ignored "-Wunused-parameter"
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  */
void _Error_Handler(char *file, int line) {
    while (1) {
    }
}
