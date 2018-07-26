#include "rda5807m.h"
#include "main.h"

uint8_t buf10[64];
uint8_t buf11[64];


/**
  * Инициализация rda5807
  */
void rda5807_init() {
    uint8_t buf[2] = {0, 0};

    i2cm_init();
    uint8_t err = I2C_Mem_Read(RDA5807_RandAccess_Addr << 1u, 0, buf, 2);

    //HAL_StatusTypeDef err = HAL_I2C_Mem_Read(I2C1, RDA5807_RandAccess_Addr << 1u, 0, I2C_MEMADD_SIZE_8BIT, buf, 1);
    prints("\n\r -- init err: ");
    printi(err);
    prints(" | id: ");
    printi(buf[0]);
    printi(buf[1]);
    prints("\n\r");

    if (err != 0 || buf[0] != 0x58 || buf[1] != 4) {
        Error_Handler();
    }
/*    err = HAL_I2C_Mem_Read(I2C1, RDA5807_RandAccess_Addr << 1u, 0, I2C_MEMADD_SIZE_8BIT, buf, 1);
    printf("\n\r -- err: %x | 0: %x 1: %x  --\n\r", err, buf[0], buf[1]);

    if (err != HAL_OK || buf[0] != 0x58 || buf[1] != 0) {
        Error_Handler();
    }

    for (i = 0; i < 64; i++) {
        buf10[i] = 0;
        buf11[i] = 0;
    }

    err = HAL_I2C_Mem_Read(I2C1, RDA5807_RandAccess_Addr << 1u, 0, I2C_MEMADD_SIZE_8BIT, buf11, 64);
    if (err != HAL_OK) {
        Error_Handler();
    }
    err = HAL_I2C_Master_Receive(I2C1, RDA5807_SeqAccess_Addr << 1u, buf10, 64);
    if (err != HAL_OK) {
        Error_Handler();
    }

    int i;
    for (i = 0; i < 64; i++) { printf(" %02x", buf10[i]); }
    printf("\n\r");
    for (i = 0; i < 64; i++) { printf(" %02x", buf11[i]); }
    printf("\n\r");
*/
    rda5807_SoftReset();
    rda5807_SetupDefault();

    uint32_t ii = 10000000;
    while (ii--);
//    printf("start freq: %u \n\r", rda5807_GetFreq_In100Khz());
    rda5807_StartSeek(1);

/*    while (rda5807_Get_SeekTuneReadyFlag())
        printf(".");
    printf(" ---\n\r");
    printf("tuned freq: %u \n\r", rda5807_GetFreq_In100Khz());//*/
//    prints("\n\rtuned freq: ");
//    printi(rda5807_GetFreq_In100Khz());
}

void rda5807_bus_init() {
    i2cm_init();
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
    /*/ Выдаём START на шину
    if (i2cm_Start(RDA5807_SeqAccess_Addr, 1)) {
        Error_Handler();
    }

    // Читаем
    i2cm_ReadBuffAndStop((uint8_t *) pBuff, RegNum << 1u); //*/

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
//    int8_t err;

    /*/ Выдаём START на шину
    err = i2cm_Start(I2Cx, RDA5807_SeqAccess_Addr, 0, RDA5807_TO);
    if (err) {
        Error_Handler();
    } //*/

    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);

//    err = i2cm_WriteBuff(I2Cx, (uint8_t *) pBuff, RegNum << 1, RDA5807_TO);
//    i2cm_Stop(I2Cx, RDA5807_TO);

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
    /*/ Выдаём START на шину
    if (i2cm_Start(I2Cx, RDA5807_RandAccess_Addr, 0, RDA5807_TO)) {
        Error_Handler();
    }
    if (i2cm_WriteBuff(I2Cx, &RegAddr, 1, RDA5807_TO)) {
        Error_Handler();
    }

    // Выдаём START на шину
    if (i2cm_Start(I2Cx, RDA5807_RandAccess_Addr, 1, RDA5807_TO)) {
        Error_Handler();
    }
    // Читаем
    i2cm_ReadBuffAndStop(I2Cx, (uint8_t *) pBuff, RegNum << 1, RDA5807_TO); //*/

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
    //int8_t err;

    /*/ Выдаём START на шину
    err = i2cm_Start(I2Cx, RDA5807_RandAccess_Addr, 0, RDA5807_TO);
    if (err) {
        Error_Handler();
    }

    if (i2cm_WriteBuff(I2Cx, &RegAddr, 1, RDA5807_TO)) {
        Error_Handler();
    } //*/

    rda5807_bytes_change((uint8_t *) pBuff, RegNum << 1u);

    //err = i2cm_WriteBuff(I2Cx, (uint8_t *) pBuff, RegNum << 1, RDA5807_TO);
    //i2cm_Stop(I2Cx, RDA5807_TO);

    if (I2C_Mem_Write(RDA5807_RandAccess_Addr << 1u, RegAddr, (uint8_t *) pBuff, RegNum << 1u)) {
        Error_Handler();
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
    Reg02.bSEEKUP = Up ? 1 : 0; // 09 Seek Up (0 = Seek down; 1 = Seek up)

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

    return Reg0A.bSTC;
}
