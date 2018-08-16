#ifndef FM_I2C_H
#define FM_I2C_H

#include "stm32l0xx.h"

// Константы, обозначающие ошибки при чтении RAM из mlx9061x
#define I2C_Ok         0
#define I2C_ERR_NotConnect -1
#define I2C_ERR_BadChksum  -2
#define I2C_ERR_HWerr      -3


// Процедура инициализации i2c (I2C1 или I2C2) в режиме master с заданной частотой интерфейса
void i2c_init();

// Функция стартует обмен. Выдаёт условие START, выдаёт адрес слейва с признаком R/W
int8_t i2cm_Start(uint8_t slave_addr, uint8_t IsRead);

// Функция выдаёт условие STOP
int8_t i2cm_Stop();

// Функция выдаёт на шину массив байт из буфера
int8_t i2cm_WriteBuff(uint8_t *pbuf, uint16_t len);

// Функция читает массив байт с шины и выдаёт условие STOP
int8_t i2cm_ReadBuffAndStop(uint8_t *pbuf, uint16_t len);

uint8_t i2c_read(uint16_t i2c_addr, uint8_t count, uint8_t *data);

uint8_t i2c_write(uint16_t i2c_addr, uint8_t count, uint8_t *data);

void I2C_TransferConfig(uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);

uint8_t I2C_WaitOnFlagUntilTimeout(uint32_t Flag, uint32_t Status);

uint8_t I2C_WaitOnTXISFlagUntilTimeout();

uint8_t I2C_WaitOnSTOPFlagUntilTimeout();

uint8_t I2C_Mem_Read(uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint8_t Size);

uint8_t I2C_Mem_Write(uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint8_t Size);

uint8_t I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint8_t Size);

uint8_t I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint8_t Size);

#endif //FM_I2C_H
