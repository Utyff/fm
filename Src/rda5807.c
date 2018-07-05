//------------------------------------------------------------------------------
// This is Open source software. You can place this code on your site, but don't
// forget a link to my YouTube-channel: https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// ��� ����������� ����������� ���������������� ��������. �� ������ ���������
// ��� �� ����� �����, �� �� �������� ������� ������ �� ��� YouTube-����� 
// "����������� � ���������" https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// �����: �������� ������ / Nadyrshin Ruslan
//------------------------------------------------------------------------------
#include "stm32f0xx_hal.h"
#include "rda5807.h"
#include "i2cm.h"
//#include "delay.h"



//==============================================================================
// ������������� i2c ��� ������ � rda5807
//==============================================================================
void rda5807_bus_init(I2C_TypeDef* I2Cx)
{
//  i2cm_init(I2Cx, RDA5807_i2cRate);
}
//==============================================================================


//==============================================================================
// ��������� ������ ������� ����� ������� � ������ pBuff 
//==============================================================================
void rda5807_bytes_change(uint8_t *pBuff, uint8_t Count)
{
  while (Count > 1)     // ���� �������� ���� �� ���� ����
  {
    uint8_t Temp = *(pBuff+1);
    *(pBuff+1) = *pBuff;
    *pBuff = Temp;
    pBuff += 2;
    Count -= 2;
  }
}
//==============================================================================
                 
                 
//==============================================================================
// ��������� ������ �� rda5807 ������ ��������� (���-�� RegNum) ������� � 0x0A
// ������������ I2C-����� RDA5807_SeqAccess_Addr
//==============================================================================
void rda5807_read_regfile(I2C_TypeDef* I2Cx, uint16_t *pBuff, uint8_t RegNum)
{
  // ����� START �� ����
  if (i2cm_Start(I2Cx, RDA5807_SeqAccess_Addr, 1, RDA5807_TO))
  {
    i2cm_Stop(I2Cx, RDA5807_TO);
    rda5807_bus_init(I2Cx);
    return;
  }
  
  // ������
  i2cm_ReadBuffAndStop(I2Cx, (uint8_t *)pBuff, RegNum << 1, RDA5807_TO);
  rda5807_bytes_change((uint8_t *)pBuff, RegNum << 1);
}
//==============================================================================


//==============================================================================
// ��������� ����� � rda5807 ������ ��������� (���-�� RegNum) ������� � 0x02
// ������������ I2C-����� RDA5807_SeqAccess_Addr
//==============================================================================
void rda5807_write_regfile(I2C_TypeDef* I2Cx, uint16_t *pBuff, uint8_t RegNum)
{
  int8_t err;
  
  // ����� START �� ����
  err = i2cm_Start(I2Cx, RDA5807_SeqAccess_Addr, 0, RDA5807_TO);
  if (err)
  {
    i2cm_Stop(I2Cx, RDA5807_TO);
    rda5807_bus_init(I2Cx);
    return;
  }
    
  rda5807_bytes_change((uint8_t *)pBuff, RegNum << 1);

  err = i2cm_WriteBuff(I2Cx, (uint8_t *)pBuff, RegNum << 1, RDA5807_TO);
  i2cm_Stop(I2Cx, RDA5807_TO);
  
  rda5807_bytes_change((uint8_t *)pBuff, RegNum << 1);
}
//==============================================================================


//==============================================================================
// ��������� ������ �� rda5807 ������ ��������� (���-�� RegNum) ������� � RegAddr
// ������������ I2C-����� RDA5807_RandAccess_Addr (��� ������ ������������� � rda5800)
//==============================================================================
void rda5807_read(I2C_TypeDef* I2Cx, uint8_t RegAddr, uint16_t *pBuff, uint8_t RegNum)
{
  // ����� START �� ����
  if (i2cm_Start(I2Cx, RDA5807_RandAccess_Addr, 0, RDA5807_TO))
  {
    i2cm_Stop(I2Cx, RDA5807_TO);
    rda5807_bus_init(I2Cx);
    return;
  }
  
  if (i2cm_WriteBuff(I2Cx, &RegAddr, 1, RDA5807_TO))
  {
    i2cm_Stop(I2Cx, RDA5807_TO);
    rda5807_bus_init(I2Cx);
    return;
  }
  
  // ����� START �� ����
  if (i2cm_Start(I2Cx, RDA5807_RandAccess_Addr, 1, RDA5807_TO))
  {
    i2cm_Stop(I2Cx, RDA5807_TO);
    rda5807_bus_init(I2Cx);
    return;
  }

  // ������
  i2cm_ReadBuffAndStop(I2Cx, (uint8_t *)pBuff, RegNum << 1, RDA5807_TO);
  rda5807_bytes_change((uint8_t *)pBuff, RegNum << 1);
}
//==============================================================================


//==============================================================================
// ��������� ����� � rda5807 ������ ��������� (���-�� RegNum) ������� � RegAddr
// ������������ I2C-����� RDA5807_RandAccess_Addr (��� ������ ������������� � rda5800)
//==============================================================================
void rda5807_write(I2C_TypeDef* I2Cx, uint8_t RegAddr, uint16_t *pBuff, uint8_t RegNum)
{
  int8_t err;
  
  // ����� START �� ����
  err = i2cm_Start(I2Cx, RDA5807_RandAccess_Addr, 0, RDA5807_TO);
  if (err)
  {
    i2cm_Stop(I2Cx, RDA5807_TO);
    rda5807_bus_init(I2Cx);
    return;
  }
 
  if (i2cm_WriteBuff(I2Cx, &RegAddr, 1, RDA5807_TO))
  {
    i2cm_Stop(I2Cx, RDA5807_TO);
    rda5807_bus_init(I2Cx);
    return;
  }

  rda5807_bytes_change((uint8_t *)pBuff, RegNum << 1);

  err = i2cm_WriteBuff(I2Cx, (uint8_t *)pBuff, RegNum << 1, RDA5807_TO);
  i2cm_Stop(I2Cx, RDA5807_TO);
  
  rda5807_bytes_change((uint8_t *)pBuff, RegNum << 1);
}
//==============================================================================


//==============================================================================
// ��������� ������������� ������ � rda5807
//==============================================================================
void rda5807_init(I2C_TypeDef* I2Cx)
{
  rda5807_bus_init(I2Cx);
}
//==============================================================================


//==============================================================================
// ��������� ������ ����������� ����� rda5807
//==============================================================================
void rda5807_SoftReset(I2C_TypeDef* I2Cx)
{
  tReg02h Reg02;
  
  Reg02.bENABLE = 1;
  Reg02.bSOFT_RESET = 1;
  rda5807_write(I2Cx, 0x02, (uint16_t *) &Reg02, 1);

  Reg02.bENABLE = 1;
  Reg02.bSOFT_RESET = 0;
  rda5807_write(I2Cx, 0x02, (uint16_t *) &Reg02, 1);
}
//==============================================================================


//==============================================================================
// ��������� ���������� ��������� ��������� rda5807
//==============================================================================
void rda5807_SetupDefault(I2C_TypeDef* I2Cx)
{
  // ����� ��������� rda5807 ��� ������ �������� (����� 0x08, 0x09)
  struct 
  {
    tReg02h     Reg02;
    tReg03h     Reg03;
    tReg04h     Reg04;
    tReg05h     Reg05;
    tReg06h     Reg06;
    tReg07h     Reg07;
  } Buff;
  
  // ������� 0x02
  Buff.Reg02.bENABLE = 1;
  Buff.Reg02.bSOFT_RESET = 0;
  Buff.Reg02.bNEW_METHOD = 1;
  Buff.Reg02.bRDS_EN = 1;
  Buff.Reg02.bCLK_MODE = 0;        // 32.768
  Buff.Reg02.bSKMODE = 0;
  Buff.Reg02.bSEEK = 0;
  Buff.Reg02.bSEEKUP = 1;
  Buff.Reg02.bRCLK_DirectInput = 0;
  Buff.Reg02.bRCLK_NonCalibMode = 0;
  Buff.Reg02.bBASS = 0;
  Buff.Reg02.bMONO = 0;
  Buff.Reg02.bDMUTE = 1;
  Buff.Reg02.bDHIZ = 1;
  // ������� 0x03
  Buff.Reg03.bSPACE = 0;   // ��� ��������� = 100 ���
  Buff.Reg03.bBAND = 0;    // �������� 87�108 MHz (US/Europe)
  Buff.Reg03.bTUNE = 1;
  Buff.Reg03.bDIRECT_MODE = 0;
  Buff.Reg03.bCHAN = 0;
  // ������� 0x04
  Buff.Reg04.bRSVD1 = 0;
  Buff.Reg04.bAFCD = 0;
  Buff.Reg04.bSOFTMUTE_EN = 1;
  Buff.Reg04.bRSVD2 = 0;
  Buff.Reg04.bDE = 0;
  Buff.Reg04.bRSVD3 = 0;
  // ������� 0x05
  Buff.Reg05.bVOLUME = 0;
  Buff.Reg05.bANT_GAIN = 0;
  Buff.Reg05.bANT_TYPE = ANT_TYPE_Both;//ANT_TYPE_External;//ANT_TYPE_Headphones;//ANT_TYPE_Both;
  Buff.Reg05.bSEEKTH = 8;
  Buff.Reg05.bRSVD3 = 0;
  Buff.Reg05.bINT_MODE = 1;
  // ������� 0x06
  Buff.Reg06.bRSVD1 = 0;
  Buff.Reg06.bOPEN_MODE = 0;
  Buff.Reg06.bRSVD2 = 0;
  // ������� 0x07
  Buff.Reg07.bFREQ_MODE = 0;
  Buff.Reg07.bSOFTBLEND_EN = 1;
  Buff.Reg07.bSEEK_TH_OLD = 0;
  Buff.Reg07.bRSVD1 = 0;
  Buff.Reg07.b65M_50M_MODE = 1;
  Buff.Reg07.bTH_SOFRBLEND = 16;
  Buff.Reg07.bRSVD2 = 0;
    
  // ����� �������� �������� ������ ������������ �����
  rda5807_write_regfile(I2Cx, (uint16_t *) &(Buff.Reg02), 6);
}
//==============================================================================


//==============================================================================
// ��������� ������������� ������� ��������� (0..16) ������ rda5807. ��� Value=0 �������� MUTE
//==============================================================================
void rda5807_SetVolume(I2C_TypeDef* I2Cx, uint8_t Value)
{
  tReg02h Reg02;
  tReg05h Reg05;
  uint8_t Mute = Value ? 0 : 1;
  
  if (Value > 16)
    Value = 16;
  
  Value--;    // �������� ��� ���� Volume �� 1 ������, ��� ������� �������� �������

  if (!Mute)
  {
    // ������ ������� 0x05
    rda5807_read(I2Cx, 0x05, (uint16_t *) &Reg05, 1);
    // ������ �������� ���� VOLUME
    Reg05.bVOLUME = Value;
    // ����� ������� 0x05
    rda5807_write(I2Cx, 0x05, (uint16_t *) &Reg05, 1);
  }
    
  // ������ ������� 0x02
  rda5807_read(I2Cx, 0x02, (uint16_t *) &Reg02, 1);
  // ������ �������� ���� VOLUME
  Reg02.bDMUTE = Mute ? 0 : 1;
  // ����� ������� 0x02
  rda5807_write(I2Cx, 0x02, (uint16_t *) &Reg02, 1);
}
//==============================================================================


//==============================================================================
// ��������� ��������/��������� BassBoost
//==============================================================================
void rda5807_SetBassBoost(I2C_TypeDef* I2Cx, uint8_t Value)
{
  tReg02h Reg02;

  // ������ ������� 0x02
  rda5807_read(I2Cx, 0x02, (uint16_t *) &Reg02, 1);
  // ������ �������� ���� BASS
  Reg02.bBASS = (Value) ? 1 : 0;
  // ����� ������� 0x02
  rda5807_write(I2Cx, 0x02, (uint16_t *) &Reg02, 1);
}
//==============================================================================


//==============================================================================
// ��������� ������������� ������� ������� Freq100kHz � �������� ������������� rda5807 �� ��� �������.
// ��������� �������� ����� ��������� ����� ������������������ �� ��������� ���� STR � �������� 0x0A (�������� rda5807_Get_SeekTuneReadyFlag)
//==============================================================================
void rda5807_SetFreq_In100Khz(I2C_TypeDef* I2Cx, uint16_t Freq100kHz)
{
  tReg03h Reg03;
  
  // �������� �������� ��������� ��� ��������� 87�108 MHz (US/Europe)
  if (Freq100kHz < 870)
    Freq100kHz = 870;
  if (Freq100kHz > 1080)
    Freq100kHz = 1080;
  
  // �������� ������ ��������� (87 ���)
  Freq100kHz -= 870;

  // ������ ������� 0x03
  rda5807_read(I2Cx, 0x03, (uint16_t *) &Reg03, 1);
  // ������ �������� ���� CHAN
  Reg03.bCHAN = Freq100kHz;
  // ���������� ���� ������ ������������� �� �����
  Reg03.bTUNE = 1;
  // ����� ������� 0x03
  rda5807_write(I2Cx, 0x03, (uint16_t *) &Reg03, 1);
}
//==============================================================================


//==============================================================================
// ������� ������ ������� �������, �� ������� �������� rda5807
//==============================================================================
uint16_t rda5807_GetFreq_In100Khz(I2C_TypeDef* I2Cx)
{
  tReg0Ah Reg0A;
  
  // ������ ������� 0x0A
  rda5807_read(I2Cx, 0x0A, (uint16_t *) &Reg0A, 1);

  uint16_t Freq100kHz = Reg0A.bREADCHAN;
  
  if (Freq100kHz == 319)
    return 0;
  
  // ���������� ������ ��������� (87 ���)
  Freq100kHz += 870;
  
  return Freq100kHz;
}
//==============================================================================


//==============================================================================
// ��������� �������� ����� ������������ �����/����
//==============================================================================
void rda5807_StartSeek(I2C_TypeDef* I2Cx, uint8_t Up)
{
  tReg02h Reg02;

  // ������ ������� 0x02
  rda5807_read(I2Cx, 0x02, (uint16_t *) &Reg02, 1);
  
  Reg02.bSKMODE = 1;          // 07 Seek Mode (0 = wrap at the upper or lower band limit and continue seeking; 1 = stop seeking at the upper or lower band limit)
  Reg02.bSEEK = 1;            // 08 Seek (0 = Disable stop seek; 1 = Enable)   
  Reg02.bSEEKUP = Up ? 1 : 0; // 09 Seek Up (0 = Seek down; 1 = Seek up)

  // ����� ������� 0x02
  rda5807_write(I2Cx, 0x02, (uint16_t *) &Reg02, 1);
}
//==============================================================================


//==============================================================================
// ������� ���������� ��������� ���� STR (SeekTuneReadyFlag)
// SeekTuneReadyFlag=1 ���� ��� ������� ��������� �� ������� ��� ����� ������������.
//==============================================================================
uint8_t rda5807_Get_SeekTuneReadyFlag(I2C_TypeDef* I2Cx)
{
  tReg0Ah Reg0A;
  
  // ������ ������� 0x0A
  rda5807_read(I2Cx, 0x0A, (uint16_t *) &Reg0A, 1);

  return Reg0A.bSTC;
}
//==============================================================================
