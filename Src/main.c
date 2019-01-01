#include "stm32l0xx.h"
#include <rda5807m.h>
#include <spi.h>
#include <ssd1306.h>
#include <usblib.h>
#include "main.h"


#pragma clang diagnostic ignored "-Wmissing-noreturn"

void SetClocks();

void Configure_GPIO_LED(void);

void Configure_GPIO_Button(void);

void Configure_GPIO_USART2();

void Configure_USART2(void);

void drawScreen();

void getButtons();

void getFreq(char *);

uint8_t send = 0;
uint8_t string2send[32] = "STm\n";
uint32_t stick = 0;
uint32_t Tickstart; // operation start time. For detect timeout


int main(void) {
    SetClocks();
    SysTick_Config(32000);
    Configure_GPIO_LED();
    Configure_GPIO_Button();
    Configure_GPIO_USART2();
    Configure_USART2();
    Configure_GPIO_SPI1();
    Configure_SPI1();
    USBLIB_Init();
    rda5807_init();
    ssd1306_Init();

    LED1_TOGGLE();

    while (1) {
        LED1_TOGGLE();
        LED2_TOGGLE();
        char buf[20];
//        prints("\n\rtuned freq: ");
//        printh(rda5807_GetFreq_In100Khz());
//        getFreq(buf);
//        USBLIB_Transmit((uint16_t *) buf, 6);
//        USBLIB_Transmit((uint16_t *) "\r\n", 2);

        getButtons();
        drawScreen();

        uint32_t start = stick;
        while (stick - start < 100) {
        }
    }
}

#define MODE_FREQ 0x1
#define MODE_VOL  0x2

uint16_t mode = MODE_FREQ;
uint16_t lastState = 0; // buttons state
uint8_t volume = 1;     // 0 - mote, 1-16 volume level

void getButtons() {
    uint16_t state = (uint16_t) (((~(GPIOB->IDR)) & (GPIO_IDR_ID3 | GPIO_IDR_ID4 | GPIO_IDR_ID5)) >> 3);
    if (state == lastState) {
        return;
    }

    uint16_t action = (mode << 8) | (lastState << 4) | state;
    lastState = state;

    switch (action) {
        // frequency control mode
        case 0x110: // PB3 up, set mode volume
            mode = MODE_VOL;
            break;
        case 0x120: // PB4 up, seek down
            rda5807_StartSeek(0);
            break;
        case 0x140: // PB5 up, seek up
            rda5807_StartSeek(1);
            break;
            // volume control mode
        case 0x210: // PB3 up, set mode frequency
            mode = MODE_FREQ;
            break;
        case 0x220: // PB4 up, vol down
            if (volume < 16) {
                rda5807_SetVolume(++volume);
            }
            break;
        case 0x240: // PB5 up, vol up
            if (volume > 0) {
                rda5807_SetVolume(--volume);
            }
            break;
        default:
            break;
    }
}

void getFreq(char *buf) {
    _itoa(rda5807_GetFreq_In100Khz(), buf);
    int i = 0;
    while (buf[i] != 0) i++;
    buf[i + 1] = 0;
    buf[i] = buf[i - 1];
    buf[i - 1] = '.';
}

void drawScreen() {
    char buf[50];
    getFreq(buf);
    ssd1306_Fill(0);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buf, Font_11x18, White);
    ssd1306_WriteString(" MHz", Font_7x10, White);

    if (mode == MODE_VOL) {
        _itoa(volume, buf);
        ssd1306_SetCursor(0, 40);
        ssd1306_WriteString("vol: ", Font_11x18, White);
        ssd1306_WriteString(buf, Font_11x18, White);
    }

    ssd1306_UpdateScreen();
}


void SetClocks() {
// ==============  HSI
    RCC->CR |= RCC_CR_HSION;
    while (RCC->CR & RCC_CR_HSIRDY == RESET);

// === Enable the VREF for HSI48
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    SYSCFG->CFGR3 |= 0x01;
    while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF));
    SYSCFG->CFGR3 |= SYSCFG_CFGR3_ENREF_HSI48;
    while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_REF_HSI48_RDYF));

// ============== HSI48
    SET_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON);
    while (RCC->CRRCR & RCC_CRRCR_HSI48RDY == RESET);

// ============== PLL
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2;
    // Enable the main PLL.
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    // Wait till PLL is ready
    while (RCC->CR & RCC_CR_PLLRDY == RESET);

// ============== FLASH
    FLASH->ACR |= FLASH_ACR_LATENCY;
    // Check that the new number of wait states is taken into account to access the Flash
    // memory by reading the FLASH_ACR register
    while ((FLASH->ACR & FLASH_ACR_LATENCY) == RESET);

// ============== HCLK
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

// ============== SYSCLK
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (RCC->CFGR & RCC_CFGR_SWS != RCC_CFGR_SWS_PLL);

    // set HSI48 as USB Clock source
    RCC->CCIPR |= RCC_CCIPR_HSI48SEL;

// ==========  RCCEx_CRSConfig

    // Before configuration, reset CRS registers to their default values
    // RCC_CRS_FORCE_RESET()
    SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CRSRST);
    // RCC_CRS_RELEASE_RESET()
    CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CRSRST);

// Set the SYNCDIV[2:0] bits to Prescaler value
#define RCC_CRS_SYNC_DIV1              ((uint32_t)0x00000000U) // Synchro Signal not divided (default)
// Set the SYNCSRC[1:0] bits to Source value
#define RCC_CRS_SYNC_SOURCE_USB        CRS_CFGR_SYNCSRC_1      // Synchro Signal source USB SOF (default)
// Set the SYNCSPOL bit to Polarity value
#define RCC_CRS_SYNC_POLARITY_RISING   ((uint32_t)0x00000000U) // Synchro Active on rising edge (default)
// Set the RELOAD[15:0] bits to ReloadValue value
#define RCC_CRS_RELOADVALUE_CALCULATE(__FTARGET__, __FSYNC__)  (((__FTARGET__) / (__FSYNC__)) - 1)
// Set the FELIM[7:0] bits according to ErrorLimitValue value
#define RCC_CRS_ERROR_LIMIT 34

    CRS->CFGR = RCC_CRS_SYNC_DIV1 | RCC_CRS_SYNC_SOURCE_USB | RCC_CRS_SYNC_POLARITY_RISING |
                RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000) | (RCC_CRS_ERROR_LIMIT << CRS_CFGR_FELIM_Pos);

    // Adjust HSI48 oscillator smooth trimming
    // Set the TRIM[5:0] bits according to RCC_CRS_HSI48CalibrationValue value
    //MODIFY_REG(CRS->CR, CRS_CR_TRIM, (32 << CRS_CR_TRIM_Pos));

    // Enable Automatic trimming & Frequency error counter
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
}

/**
  * @brief  This function :
             - Enables LEDs GPIO clock
             - Configures the Green LED pin on GPIO PA4
             - Configures the orange LED pin on GPIO PB8
  */
void Configure_GPIO_LED(void) {
    // Enable the peripheral clock of GPIOA
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Select output mode (01) on PA4 and PB8
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4)) | (GPIO_MODER_MODE4_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE8)) | (GPIO_MODER_MODE8_0);
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the Push Button GPIO PA0
  */
void Configure_GPIO_Button(void) {
    // Enable the peripheral clock of GPIOA
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Select mode
    // Select input mode (00) on PB3 PB4 PB5 with Pull-up (01)
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5));
    GPIOB->PUPDR = GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0;
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the USART2 pins on GPIO PA2 PA3
  */
void Configure_GPIO_USART2(void) {
    // Enable the peripheral clock of GPIOA
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    // Select AF mode (10) on PA2 and PA3
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3))
                   | (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
    // AF4 for USART2 signals
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3))
                    | (4u << (2u * 4)) | (4u << (3u * 4));
}

/**
  * @brief  This function configures USART2.
  */
void Configure_USART2(void) {
    // Enable the peripheral clock USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // oversampling by 16, 115200 baud
    USART2->BRR = 320000 / 1152;
    // 8 data bit, 1 start bit, 1 stop bit, no parity
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;

    // polling idle frame Transmission
    while ((USART2->ISR & USART_ISR_TC) != USART_ISR_TC) {
        // add time out here for a robust application
    }
    USART2->ICR |= USART_ICR_TCCF;// clear TC flag
    USART2->CR1 |= USART_CR1_TCIE;// enable TC interrupt

    // Set priority for USART2_IRQn
    NVIC_SetPriority(USART2_IRQn, 0);
    // Enable USART2_IRQn
    NVIC_EnableIRQ(USART2_IRQn);
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

void _itoa(uint16_t i, char *p) {
//    char const digit[] = "0123456789";
    if (i < 0) {
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do { //Move to where representation ends
        ++p;
        shifter = shifter / 10;
    } while (shifter);
    *p = '\0';
    do { //Move back, inserting digits as u go
        *--p = (char) ('0' + i % 10);
        i = i / (uint16_t) 10;
    } while (i);
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

void printh(uint16_t val) {
    char buf[7];
    _itoa(val, buf);
    prints(buf);
}

void prints(const char *str) {
    // wait till end current transmission
    while (send != 0);

    _strcpy(string2send, (uint8_t *) str);
    // start USART transmission. Will initiate TC if TXE
    USART2->TDR = string2send[send];
    send = 1;
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
