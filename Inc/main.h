#ifndef FM_MAIN_H
#define FM_MAIN_H

#include "stm32f0xx.h"

#define BNT1_Pin GPIO_PIN_0
#define BNT1_GPIO_Port GPIOA
#define BNT2_Pin GPIO_PIN_1
#define BNT2_GPIO_Port GPIOA

#define OLED_DC_Pin GPIO_PIN_6
#define OLED_DC_GPIO_Port GPIOA
#define OLED_CS_Pin GPIO_PIN_1
#define OLED_CS_GPIO_Port GPIOB

#define LED1_Pin GPIO_IDR_12
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_IDR_11
#define LED2_GPIO_Port GPIOA

void prints(const char *);

void printi(uint16_t);

void _strcpy(uint8_t *dst, const uint8_t *src);

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)


#endif //FM_MAIN_H
