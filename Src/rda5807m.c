#include "rda5807m.h"
#include "main.h"


/**
  * Инициализация rda5807
  */
void rda5807_init() {
    uint16_t ID;

    i2c_init();

    // check register 0x0 for chip ID
    uint8_t err = I2C_Mem_Read(RDA5807_RandAccess_Addr << 1u, 0x0, (uint8_t *) &ID, 2);
    if (err != 0 || ID != 0x0458) {
        Error_Handler();
    }

    rda5807_SoftReset();
    rda5807_SetupDefault();

    uint32_t start = stick;
    while (stick - start < 2);

    rda5807_SetVolume(1);
    rda5807_StartSeek(1);
}

/**
  * Процедура меняет местами байты попарно в буфере pBuff
  */
void rda5807_bytes_change(uint8_t *pBuff, uint8_t Count) {
    while (Count > 1) {   // Если осталась хотя бы пара байт
        uint8_t Temp = *(pBuff + 1);
        *(pBuff + 1) = *pBuff;
        *pBuff = Temp;
        pBuff += 2;
        Count -= 2;
    }
}

/**
  * Процедура читает из rda5807 группу регистров (кол-во RegNum) начиная с 0x0A
  * Используется I2C-адрес RDA5807_SeqAccess_Addr
  */
void rda5807_read_regfile(uint16_t *pBuff, uint8_t RegNum) {
    if (I2C_Master_Receive(RDA5807_SeqAccess_Addr << 1u, (uint8_t *) pBuff, RegNum << 1u)) {
        Error_Handler();
    }
    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);
}

/**
  * Процедура пишет в rda5807 группу регистров (кол-во RegNum) начиная с 0x02
  * Используется I2C-адрес RDA5807_SeqAccess_Addr
  */
void rda5807_write_regfile(uint16_t *pBuff, uint8_t RegNum) {
    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);

    if (I2C_Master_Transmit(RDA5807_SeqAccess_Addr << 1u, (uint8_t *) pBuff, RegNum << 1u)) {
        Error_Handler();
    }

    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);
}

/**
  * Процедура читает из rda5807 группу регистров (кол-во RegNum) начиная с RegAddr
  * Используется I2C-адрес RDA5807_RandAccess_Addr (для режима совместимости с rda5800)
  */
void rda5807_read(uint8_t RegAddr, uint16_t *pBuff, uint8_t RegNum) {
    if (I2C_Mem_Read(RDA5807_RandAccess_Addr << 1u, RegAddr, (uint8_t *) pBuff, RegNum << 1u)) {
        Error_Handler();
    }

    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);
}

/**
  * Процедура пишет в rda5807 группу регистров (кол-во RegNum) начиная с RegAddr
  * Используется I2C-адрес RDA5807_RandAccess_Addr (для режима совместимости с rda5800)
  */
void rda5807_write(uint8_t RegAddr, uint16_t *pBuff, uint8_t RegNum) {
    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);

    if (I2C_Mem_Write(RDA5807_RandAccess_Addr << 1u, RegAddr, (uint8_t *) pBuff, RegNum << 1u)) {
        return; // Error_Handler();
    }

    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);
}

/**
  * Процедура делает программный сброс rda5807
  */
void rda5807_SoftReset() {
    tReg02h Reg02;

    Reg02.bENABLE = 1;
    Reg02.bSOFT_RESET = 1;
    rda5807_write(0x02, (uint16_t *) &Reg02, 1);

    Reg02.bENABLE = 1;
    Reg02.bSOFT_RESET = 0;
    rda5807_write(0x02, (uint16_t *) &Reg02, 1);
}

/**
  * Процедура производит начальную настройку rda5807
  */
void rda5807_SetupDefault() {
    // Набор регистров rda5807 для записи настроек (кроме 0x08, 0x09)
    struct {
        tReg02h Reg02;
        tReg03h Reg03;
        tReg04h Reg04;
        tReg05h Reg05;
        tReg06h Reg06;
        tReg07h Reg07;
    } Buff;

    // Регистр 0x02
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
    // Регистр 0x03
    Buff.Reg03.bSPACE = 0;   // Шаг настройки = 100 КГц
    Buff.Reg03.bBAND = 0;    // Диапазон 87–108 MHz (US/Europe)
    Buff.Reg03.bTUNE = 1;
    Buff.Reg03.bDIRECT_MODE = 0;
    Buff.Reg03.bCHAN = 0;
    // Регистр 0x04
    Buff.Reg04.bRSVD1 = 0;
    Buff.Reg04.bAFCD = 0;
    Buff.Reg04.bSOFTMUTE_EN = 1;
    Buff.Reg04.bRSVD2 = 0;
    Buff.Reg04.bDE = 0;
    Buff.Reg04.bRSVD3 = 0;
    // Регистр 0x05
    Buff.Reg05.bVOLUME = 0;
    Buff.Reg05.bANT_GAIN = 0;
    Buff.Reg05.bANT_TYPE = ANT_TYPE_Both;//ANT_TYPE_External;//ANT_TYPE_Headphones;//ANT_TYPE_Both;
    Buff.Reg05.bSEEKTH = 8;
    Buff.Reg05.bRSVD3 = 0;
    Buff.Reg05.bINT_MODE = 1;
    // Регистр 0x06
    Buff.Reg06.bRSVD1 = 0;
    Buff.Reg06.bOPEN_MODE = 0;
    Buff.Reg06.bRSVD2 = 0;
    // Регистр 0x07
    Buff.Reg07.bFREQ_MODE = 0;
    Buff.Reg07.bSOFTBLEND_EN = 1;
    Buff.Reg07.bSEEK_TH_OLD = 0;
    Buff.Reg07.bRSVD1 = 0;
    Buff.Reg07.b65M_50M_MODE = 1;
    Buff.Reg07.bTH_SOFRBLEND = 16;
    Buff.Reg07.bRSVD2 = 0;

    // Пишем регистры функцией записи регистрового файла
    rda5807_write_regfile((uint16_t *) &(Buff.Reg02), 7);
}

/**
  * Процедура устанавливает уровень громкости (0..16) выхода rda5807. При Value=0 включает MUTE
  */
void rda5807_SetVolume(uint8_t Value) {
    tReg02h Reg02;
    tReg05h Reg05;
    uint8_t Mute = Value ? 0 : 1;

    if (Value > 16)
        Value = 16;

    Value--;    // Значение для поля Volume на 1 меньше, чем входной параметр функции

    if (!Mute) {
        // Читаем регистр 0x05
        rda5807_read(0x05, (uint16_t *) &Reg05, 1);
        // Меняем значение поля VOLUME
        Reg05.bVOLUME = Value;
        // Пишем регистр 0x05
        rda5807_write(0x05, (uint16_t *) &Reg05, 1);
    }

    // Читаем регистр 0x02
    rda5807_read(0x02, (uint16_t *) &Reg02, 1);
    // Меняем значение поля VOLUME
    Reg02.bDMUTE = Mute ? 0 : 1;
    // Пишем регистр 0x02
    rda5807_write(0x02, (uint16_t *) &Reg02, 1);
}

/**
  * Процедура включает/выключает BassBoost
  */
void rda5807_SetBassBoost(uint8_t Value) {
    tReg02h Reg02;

    // Читаем регистр 0x02
    rda5807_read(0x02, (uint16_t *) &Reg02, 1);
    // Меняем значение поля BASS
    Reg02.bBASS = (Value) ? 1 : 0;
    // Пишем регистр 0x02
    rda5807_write(0x02, (uint16_t *) &Reg02, 1);
}

/**
  * Процедура устанавливает текущую частоту Freq100kHz и стартует перенастройку rda5807 на эту частоту.
  * Окончание процесса можно установки можно проконтроллировать по обнулению бита STR в регистре 0x0A (функцией rda5807_Get_SeekTuneReadyFlag)
  */
void rda5807_SetFreq_In100Khz(uint16_t Freq100kHz) {
    tReg03h Reg03;

    // Проверка входного параметра для диапазона 87–108 MHz (US/Europe)
    if (Freq100kHz < 870)
        Freq100kHz = 870;
    if (Freq100kHz > 1080)
        Freq100kHz = 1080;

    // Вычитаем начало диапазона (87 МГц)
    Freq100kHz -= 870;

    // Читаем регистр 0x03
    rda5807_read(0x03, (uint16_t *) &Reg03, 1);
    // Меняем значение поля CHAN
    Reg03.bCHAN = Freq100kHz;
    // Выставляем флаг начала перенастройки на канал
    Reg03.bTUNE = 1;
    // Пишем регистр 0x03
    rda5807_write(0x03, (uint16_t *) &Reg03, 1);
}

/**
  * Функция читает текущую частоту, на которую настроен rda5807
  */
uint16_t rda5807_GetFreq_In100Khz() {
    tReg0Ah Reg0A;

    // Читаем регистр 0x0A
    rda5807_read(0x0A, (uint16_t *) &Reg0A, 1);

    uint16_t Freq100kHz = Reg0A.bREADCHAN;

    if (Freq100kHz == 319)
        return 0;

    // Прибавляем начало диапазона (87 МГц)
    Freq100kHz += 870;

    return Freq100kHz;
}

/**
  * Процедура стартует поиск радиостанции вверх/вниз
  */
void rda5807_StartSeek(uint8_t Up) {
    tReg02h Reg02;

    // Читаем регистр 0x02
    rda5807_read(0x02, (uint16_t *) &Reg02, 1);

    Reg02.bSKMODE = 1;          // 07 Seek Mode (0 = wrap at the upper or lower band limit and continue seeking; 1 = stop seeking at the upper or lower band limit)
    Reg02.bSEEK = 1;            // 08 Seek (0 = Disable stop seek; 1 = Enable)
    Reg02.bSEEKUP = (uint16_t) (Up ? 1 : 0); // 09 Seek Up (0 = Seek down; 1 = Seek up)

    // Пишем регистр 0x02
    rda5807_write(0x02, (uint16_t *) &Reg02, 1);
}

/**
  * Функция возвращает состояние бита STR (SeekTuneReadyFlag)
  * SeekTuneReadyFlag=1 пока идёт процесс настройки на частоту или поиск радиостанции.
  */
uint8_t rda5807_Get_SeekTuneReadyFlag() {
    tReg0Ah Reg0A;

    // Читаем регистр 0x0A
    rda5807_read(0x0A, (uint16_t *) &Reg0A, 1);

    return (uint8_t) Reg0A.bSTC;
}

tReg0Ah rda5807_tReg0Ah() {
    tReg0Ah Reg0A;

    // Читаем регистр 0x0A
    rda5807_read(0x0A, (uint16_t *) &Reg0A, 1);

    return Reg0A;
}

tReg0Bh rda5807_tReg0Bh() {
    tReg0Bh Reg0B;

    // Читаем регистр 0x0B
    rda5807_read(0x0B, (uint16_t *) &Reg0B, 1);

    return Reg0B;
}

/**
  * Функция читает RSSI (Receive signal strength indicator)
  */
uint16_t rda5807_GetRSSI() {
    tReg0Bh Reg0B;

    // Читаем регистр 0x0B
    rda5807_read(0x0B, (uint16_t *) &Reg0B, 1);

    return Reg0B.bRSSI;
}

char rdsStation[128] = "";
char rdsRadioText[128] = "";
char segRDS[8];
char segRDS1[64];
uint16_t RDS[8];
//uint8_t rds_regs[32];
//Regs regs;

void addChar(char *str, char ch) {
    for(int i = 0; i<127; i++) {
        if(str[i]==0) {
            str[i] = ch;
            str[i+1] = '\0';
            break;
        }
    }
}

void rda5807_GetRDS() {
    char grp, ver;

    tReg0Ah regA = rda5807_tReg0Ah();
    if (regA.bRDSR == 0) {
        return;
    }

    // Читаем регистры 0x0c 0x0d 0x0e 0x0f
    rda5807_read(0x0C, (uint16_t *) &RDS, 4);
//    rda5807_read(0x02, (uint16_t *) &regs, 14);
//    rda5807_read_regfile((uint16_t *) &rds_regs, 6);

    int i;
    grp = (char) ((RDS[1] >> 12) & 0xf);
    if (RDS[1] & 0x0800) {
        ver = 1;
    } else {
        switch (grp) {
            case 0:
                i = (RDS[1] & 3) << 1;
                segRDS[i] = (char) (RDS[3] >> 8);
                segRDS[i + 1] = (char) (RDS[3] & 0xFF);

                rdsStation[0] = '\0';
                for (i = 0; i < 8; i++) {
                    if (segRDS[i] > 31 && segRDS[i] < 127) {
                        addChar(rdsStation, segRDS[i]);
                    }
                    else {
                        addChar(rdsStation, ' ');
                    }
                }
                rdsStation[9] = '\0';
                break;
            case 2:
                i = (RDS[1] & 15) << 2;
                segRDS1[i] = (char) (RDS[2] >> 8);
                segRDS1[i + 1] = (char) (RDS[2] & 0xFF);
                segRDS1[i + 2] = (char) (RDS[3] >> 8);
                segRDS1[i + 3] = (char) (RDS[3] & 0xFF);

                rdsRadioText[0] = '\0';
                for (i = 0; i < 42; i++) {
                    addChar(rdsRadioText, segRDS[i]);
                }
                break;
            default:;
        }
    }
}
