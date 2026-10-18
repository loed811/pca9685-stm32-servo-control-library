/*
 * Minimal stand-in for a CubeMX-generated main.h, used only to compile-check
 * pca9685_i2c_hal.c and the examples in CI without the STM32 HAL sources.
 * The declarations match the real STM32 HAL signatures. Not for hardware.
 */
#ifndef STM32_STUB_MAIN_H
#define STM32_STUB_MAIN_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef struct {
    void *Instance;
} I2C_HandleTypeDef;

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                          uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                         uint8_t *pData, uint16_t Size, uint32_t Timeout);
void HAL_Delay(uint32_t Delay);
void Error_Handler(void);

#endif /* STM32_STUB_MAIN_H */
