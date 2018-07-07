#include "main.h"
#include "stm32f0xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

void i2c_scan() {
    char uart2Data[24] = "Connected to UART Two\r\n";
    /*
     * Output to uart2
     * use screen or putty or whatever terminal software
     * 8N1 115200
     */
    HAL_UART_Transmit(&huart2, (uint8_t *) &uart2Data, sizeof(uart2Data), 0xFFFF);
    printf("\r\n");

    printf("Scanning I2C bus:\r\n");
    HAL_StatusTypeDef result;
    uint8_t i;
    for (i = 1; i < 128; i++) {
        /*
         * the HAL wants a left aligned i2c address
         * &hi2c1 is the handle
         * (uint16_t)(i<<1) is the i2c address left aligned
         * retries 2
         * timeout 2
         */
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 2, 2);
        if (result != HAL_OK) { // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
            printf("."); // No ACK received at that address
        }
        if (result == HAL_OK) {
            printf("\n\r0x%X", i); // Received an ACK at that address
        }
    }
}