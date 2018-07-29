#ifndef FM_SPI_H
#define FM_SPI_H

void Configure_GPIO_SPI1(void);

void Configure_SPI1(void);

uint8_t SPI_Transmit(uint8_t *pData, uint16_t Size);

#endif //FM_SPI_H
