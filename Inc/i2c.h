#ifndef FM_I2C_H
#define FM_I2C_H

#include "stm32f0xx.h"

// ���������, ������������ ������ ��� ������ RAM �� mlx9061x
#define I2C_Ok         0
#define I2C_ERR_NotConnect -1
#define I2C_ERR_BadChksum  -2
#define I2C_ERR_HWerr      -3


// ��������� ������������� i2c (I2C1 ��� I2C2) � ������ master � �������� �������� ����������
void i2cm_init();

// ������� �������� �����. ����� ������� START, ����� ����� ������ � ��������� R/W
int8_t i2cm_Start(uint8_t slave_addr, uint8_t IsRead);

// ������� ����� ������� STOP
int8_t i2cm_Stop();

// ������� ����� �� ���� ������ ���� �� ������
int8_t i2cm_WriteBuff(uint8_t *pbuf, uint16_t len);

// ������� ������ ������ ���� � ���� � ����� ������� STOP
int8_t i2cm_ReadBuffAndStop(uint8_t *pbuf, uint16_t len);

uint8_t i2c_read(uint16_t i2c_addr, uint8_t count, uint8_t *data);

uint8_t i2c_write(uint16_t i2c_addr, uint8_t count, uint8_t *data);

uint8_t i2c_mem_read(uint16_t i2c_addr, uint8_t mem_addr, uint8_t count, uint8_t *data);

uint8_t i2c_mem_write(uint16_t i2c_addr, uint8_t mem_addr, uint8_t count, uint8_t *data);

void I2C_TransferConfig(uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);

uint8_t I2C_WaitOnFlagUntilTimeout(uint32_t Flag, uint32_t Status, uint32_t Timeout);

#endif //FM_I2C_H
