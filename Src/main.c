#include <main.h>


#pragma clang diagnostic ignored "-Wmissing-noreturn"


void Configure_GPIO_LED(void);

void Configure_GPIO_Button(void);

void Configure_EXTI(void);


int main(void) {

    SystemCoreClockUpdate();

    Configure_GPIO_LED();
    Configure_GPIO_Button();
    Configure_EXTI();

    LED1_GPIO_Port->ODR ^= LED1_Pin;

    while (1) {
        // toggle LED1
        LED1_GPIO_Port->ODR ^= LED1_Pin;
        LED2_GPIO_Port->ODR ^= LED2_Pin;
        int count = 10000;
        while (count--);
    }
}

/**
  * @brief  This function :
             - Enables GPIOA clock
             - Configures the Green LED1 pin on GPIO PA12
             - Configures the orange LED2 pin on GPIO PA11
  */
__INLINE void Configure_GPIO_LED(void) {
    // Enable the peripheral clock of GPIOx port
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Select output mode (01) on LED1 and LED2
    LED1_GPIO_Port->MODER = (LED1_GPIO_Port->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12)) |
                            (GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0);
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
/*    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PA; // (1)
    EXTI->IMR |= EXTI_IMR_MR0; // (2)
    EXTI->RTSR |= EXTI_RTSR_TR0; // (3)
    NVIC_SetPriority(EXTI0_1_IRQn, 0); // (4)
    NVIC_EnableIRQ(EXTI0_1_IRQn); // (5) */
}

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void) {
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void) {
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
uint32_t stick = 0;

void SysTick_Handler(void) {
    stick++;
}


/**
  * @brief  This function handles EXTI 0 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void) {
    EXTI->PR |= 1;
// TODO
}
