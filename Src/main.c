#include "stm32f0xx.h"


#pragma clang diagnostic ignored "-Wmissing-noreturn"


void Configure_GPIO_LED(void);

void Configure_GPIO_Button(void);

void Configure_EXTI(void);


int main(void) {

    Configure_GPIO_LED();
    Configure_GPIO_Button();
    Configure_EXTI();

    GPIOA->ODR ^= GPIO_ODR_6;

    while (1) {
        // toggle green LED
        GPIOA->ODR ^= GPIO_ODR_6;
        GPIOA->ODR ^= GPIO_ODR_5;
        uint32_t count = 1000000;
        while (count--);
    }
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the Green LED pin on GPIO PC9
             - Configures the orange LED pin on GPIO PC8
  */
__INLINE void Configure_GPIO_LED(void) {
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Select output mode (01) on PC8 and PC9
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6))
                   | (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0);
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the Push Button GPIO PA0
  */
__INLINE void Configure_GPIO_Button(void) {
    /* Enable the peripheral clock of GPIOA */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Select mode */
    /* Select input mode (00) on PA0 */
//    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER0));
}

/**
  * @brief  This function configures EXTI.
  */
__INLINE void Configure_EXTI(void) {
    /* Configure Syscfg, exti and nvic for pushbutton PA0 */
    /* (1) PA0 as source input */
    /* (2) unmask port 0 */
    /* (3) Rising edge */
    /* (4) Set priority */
    /* (5) Enable EXTI0_1_IRQn */
//    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PA; /* (1) */
//    EXTI->IMR |= EXTI_IMR_MR0; /* (2) */
//    EXTI->RTSR |= EXTI_RTSR_TR0; /* (3) */
//    NVIC_SetPriority(EXTI0_1_IRQn, 0); /* (4) */
//    NVIC_EnableIRQ(EXTI0_1_IRQn); /* (5) */
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
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles SysTick Handler.
  */
uint32_t stick = 0;

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
  * @brief  This function handles I2C1 interrupt request.
  */
void I2C1_IRQHandler(void) {
}
