#ifndef _RDA5807_H
#define _RDA5807_H

#include "stm32f0xx.h"
#include "i2c.h"

#define RDA5807_SeqAccess_Addr  0x10u
#define RDA5807_RandAccess_Addr 0x11u

#define RDA5807_TO              1000    // Длительность ожидания таймаута операций на i2c при работе с pcf8574


// Возможные значения поля bANT_TYPE в регистре 0x05
#define ANT_TYPE_Off            0       // Выкл
#define ANT_TYPE_Headphones     1       // Выход на наушники
#define ANT_TYPE_External       2       // Отдельный выход
#define ANT_TYPE_Both           3       // Оба

typedef struct {
    uint16_t bENABLE            :1;   // 00 Power Up Enable (0 = Disabled; 1 = Enabled)
    uint16_t bSOFT_RESET        :1;   // 01 Soft reset (0 = not reset; 1 = reset)
    uint16_t bNEW_METHOD        :1;   // 02 New Demodulate Method Enable, can improve the receive sensitivity about 1dB.
    uint16_t bRDS_EN            :1;   // 03 RDS/RBDS enable (1 = rds/rbds enable)
    uint16_t bCLK_MODE          :3;   // 4-6 Частота внешнего резонатора
    uint16_t bSKMODE            :1;   // 07 Seek Mode (0 = wrap at the upper or lower band limit and continue seeking;
                                      //               1 = stop seeking at the upper or lower band limit)
    uint16_t bSEEK              :1;   // 08 Seek (0 = Disable stop seek; 1 = Enable)
    uint16_t bSEEKUP            :1;   // 09 Seek Up (0 = Seek down; 1 = Seek up)
    uint16_t bRCLK_DirectInput  :1;   // 10 RCLK DIRECT INPUT MODE (1 = RCLK clock use the directly input mode)
    uint16_t bRCLK_NonCalibMode :1;   // 11 RCLK NON-CALIBRATE MODE
    uint16_t bBASS              :1;   // 12 Bass Boost (0 = Disabled; 1 = Bass boost enabled)
    uint16_t bMONO              :1;   // 13 Mono Select (0 = Stereo; 1 = Force mono)
    uint16_t bDMUTE             :1;   // 14 Mute Disable (0 = Mute; 1 = Normal operation)
    uint16_t bDHIZ              :1;   // 15 Audio Output High-Z Disable (0 = High impedance; 1 = Normal operation)
} tReg02h;      // 

typedef struct {
    uint16_t bSPACE             :2;    // 0-1 Channel Spacing.
    // 00 = 100 kHz
    // 01 = 200 kHz
    // 10 = 50kHz
    // 11 = 25KHz
    uint16_t bBAND              :2;    // 2-3 Band Select
    // 00 = 87–108 MHz (US/Europe)
    // 01 = 76–91 MHz (Japan)
    // 10 = 76–108 MHz (world wide)
    // 11 = 65 –76 MHz (East Europe) or 50-65MHz
    uint16_t bTUNE              :1;    // 04 Tune (0 = Disable; 1 = Enable)
    uint16_t bDIRECT_MODE       :1;    // 05 Directly Control Mode, Only used when test.
    uint16_t bCHAN              :10;   // 6-15 Channel Select
    // BAND = 0          Frequency = Channel Spacing (kHz) x CHAN + 87.0 MHz
    // BAND = 1 or 2     Frequency = Channel Spacing (kHz) x CHAN + 76.0 MHz
    // BAND = 3          Frequency = Channel Spacing (kHz) x CHAN + 65.0 MHz
} tReg03h;      // 

typedef struct {
    uint16_t bRSVD1             :8;    // 0-7 Reserved
    uint16_t bAFCD              :1;    // 08 AFC disable (0 = afc work; 1 = afc disabled)
    uint16_t bSOFTMUTE_EN       :1;    // 09 1 = softmute enable
    uint16_t bRSVD2             :1;    // 10 Reserved
    uint16_t bDE                :1;    // 11 De-emphasis (0 = 75 µs; 1 = 50 µs)
    uint16_t bRSVD3             :4;    // 12-15 Reserved
} tReg04h;      // 

typedef struct {
    uint16_t bVOLUME            :4;    // 0-3 Volume - DAC Gain Control Bits (0000=min; 1111=max)
    uint16_t bANT_GAIN          :2;    // 4-5 Коэффициент усиления антены (предположительно)
    uint16_t bANT_TYPE          :2;    // 6-7 Тип антены (см. константы ANT_TYPE)
    uint16_t bSEEKTH            :4;    // 8-11 Seek SNR threshold value
    uint16_t bRSVD3             :3;    // 12-14 Reserved
    uint16_t bINT_MODE          :1;    // 15 INT MODE (0 = generate 5ms interrupt;
                                       //              1 = interrupt last until read reg0CH action occurs)
} tReg05h;      // 

typedef struct {
    uint16_t bRSVD1             :13;   // 0-12 Resvered
    uint16_t bOPEN_MODE         :2;    // 13-14 Open reserved register mode (11 = open behind registers writing
                                       //        function others: only open behind registers reading function
    uint16_t bRSVD2             :1;    // 15 Reserved
} tReg06h;      // 

typedef struct {
    uint16_t bFREQ_MODE
            :1;     // 00 If 1, then freq setting changed (Freq = 76000(or 87000) kHz + freq_direct (08H) kHz)
    uint16_t bSOFTBLEND_EN      :1;     // 01 Softblend enable
    uint16_t bSEEK_TH_OLD       :6;     // 2-7 Seek threshold for old seek mode, Valid when Seek_Mode=001
    uint16_t bRSVD1             :1;     // 08 Resvered
    uint16_t b65M_50M_MODE      :1;     // 09 Valid when band[1:0] = 2’b11 (0x03H_bit<3:2>)
    // 1 = 65~76 MHz;
    // 0 = 50~76 MHz.
    uint16_t bTH_SOFRBLEND      :5;     // 10-14 Threshold for noise soft blend setting, unit 2dB
    uint16_t bRSVD2             :1;     // 15 Reserved
} tReg07h;      // 

typedef struct {
    uint16_t bREADCHAN  :10;    // 0-9 Read Channel.
    // BAND = 0         Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 87.0 MHz
    // BAND = 1 or 2    Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 76.0 MHz
    // BAND = 3         Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 65.0 MHz
    uint16_t bST        :1;     // 10 Stereo Indicator (0 = Mono; 1 = Stereo)
    uint16_t bBLK_E     :1;     // 11 When RDS enable (1 = Block E has been found; 0 = no Block E has been found)
    uint16_t bRDSS      :1;     // 12 RDS Synchronization (0 = RDS decoder not synchronized(default);
                                //                         1 = RDS decoder synchronized)
    uint16_t bSF        :1;     // 13 Seek Fail (0 = Seek successful; 1 = Seek failure)
    uint16_t bSTC       :1;     // 14 Seek/Tune Complete (0 = Not complete; 1 = Complete)
    uint16_t bRDSR      :1;     // 15 RDS ready (0 = No RDS/RBDS group ready(default); 1 = New RDS/RBDS group ready)
} tReg0Ah;      // 

typedef struct {
    uint16_t bBLERB     :2;     // 0-1 Block Errors Level of RDS_DATA_1, and is always read as Errors Level
                                //     of RDS BLOCK B (in RDS mode ) or E (in RBDS mode when ABCD_E flag is 1)
    uint16_t bBLERA     :2;     // 2-3 Block Errors Level of RDS_DATA_0, and is always read as Errors Level
                                //     of RDS BLOCK A (in RDS mode) or BLOCK E (in RBDS mode when ABCD_E flag is 1)
    uint16_t bABCD_E    :1;     // 04 (1 = the block id of register 0cH,0dH,0eH,0fH is E; 0 = the block id
                                //          of register 0cH, 0dH, 0eH,0fH is A, B, C, D)
    uint16_t bRSVD1     :2;     // 5-6 Resvered
    uint16_t bFM_READY  :1;     // 07 1 = ready; 0 = not ready
    uint16_t bFM_TRUE   :1;     // 08 1 = the current channel is a station; 0 = the current channel is not a station
    uint16_t bRSSI      :7;     // 9-15 RSSI (000000 = min; 111111 = max) RSSI scale is logarithmic
} tReg0Bh;      // 



/*
//Структура регистрового файла rda5807
typedef union 
{
  struct 
  {
    tReg02h     Reg02;
    tReg03h     Reg03;
    tReg04h     Reg04;
    tReg05h     Reg05;
    tReg06h     Reg06;
    tReg07h     Reg07;
    uint16_t    Reg08;        // Регистр используется для прямой установки частоты (не через каналы)
    uint16_t    Reg09;
    tReg0Ah     Reg0A;
    tReg0Bh     Reg0B;
    uint16_t    RDSA;
    uint16_t    RDSB;
    uint16_t    RDSC;
    uint16_t    RDSD;
  } Regs;
  uint16_t Words[14];
  uint8_t Bytes[28];
} tRDA5807_RegFile;
*/

// Процедура меняет местами байты попарно в буфере pBuff
void rda5807_bytes_change(uint8_t *pBuff, uint8_t Count);

// Процедура читает из rda5807 группу регистров (кол-во RegNum) начиная с 0x0A
// Используется I2C-адрес RDA5807_SeqAccess_Addr
void rda5807_read_regfile(I2C_TypeDef *I2Cx, uint16_t *pBuff, uint8_t RegNum);

// Процедура пишет в rda5807 группу регистров (кол-во RegNum) начиная с 0x02
// Используется I2C-адрес RDA5807_SeqAccess_Addr
void rda5807_write_regfile(I2C_TypeDef *I2cHandle, uint16_t *pBuff, uint8_t RegNum);

// Процедура читает из rda5807 группу регистров (кол-во RegNum) начиная с RegAddr
// Используется I2C-адрес RDA5807_RandAccess_Addr (для режима совместимости с rda5800)
void rda5807_read(I2C_TypeDef *I2cHandle, uint8_t RegAddr, uint16_t *pBuff, uint8_t RegNum);

// Процедура пишет в rda5807 группу регистров (кол-во RegNum) начиная с RegAddr
// Используется I2C-адрес RDA5807_RandAccess_Addr (для режима совместимости с rda5800)
void rda5807_write(I2C_TypeDef *I2Cx, uint8_t RegAddr, uint16_t *pBuff, uint8_t RegNum);

// Процедура инициализации обмена с rda5807
void rda5807_init(I2C_TypeDef *I2Cx);

// Процедура делает программный сброс rda5807
void rda5807_SoftReset(I2C_TypeDef *I2Cx);

// Процедура производит начальную настройку rda5807
void rda5807_SetupDefault(I2C_TypeDef *I2Cx);

// Процедура устанавливает уровень громкости (0..16) выхода rda5807. При Value=0 включает MUTE
void rda5807_SetVolume(I2C_TypeDef *I2Cx, uint8_t Value);

// Процедура включает/выключает BassBoost
void rda5807_SetBassBoost(I2C_TypeDef *I2Cx, uint8_t Value);

// Процедура устанавливает текущую частоту Freq100kHz и стартует перенастройку rda5807 на эту частоту.
// Окончание процесса можно установки можно проконтроллировать по обнулению бита STR в регистре 0x0A (функцией rda5807_Get_SeekTuneReadyFlag)
void rda5807_SetFreq_In100Khz(I2C_TypeDef *I2Cx, uint16_t Freq100kHz);

// Функция читает текущую частоту, на которую настроен rda5807
uint16_t rda5807_GetFreq_In100Khz(I2C_TypeDef *I2Cx);

// Процедура стартует поиск радиостанции вверх/вниз
void rda5807_StartSeek(I2C_TypeDef *I2Cx, uint8_t Up);

// Функция возвращает состояние бита STR (SeekTuneReadyFlag)
// SeekTuneReadyFlag=1 пока идёт процесс настройки на частоту или поиск радиостанции.
uint8_t rda5807_Get_SeekTuneReadyFlag(I2C_TypeDef *I2Cx);

#endif
