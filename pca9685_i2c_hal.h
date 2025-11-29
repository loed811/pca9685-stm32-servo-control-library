#ifndef MAIN_PCA9685_I2C_HAL
#define MAIN_PCA9685_I2C_HAL

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define PCA9685_ERR     -1
#define PCA9685_OK      0x00

/**
 * @brief User implementation for I2C initialization
 */
int16_t pca9685_i2c_hal_init();

/**
 * @brief User implementation for I2C read
 */
int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count);

/**
 * @brief User implementation for I2C write
 */
int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count);

/**
 * @brief User implementation for milliseconds delay
 */
void pca9685_i2c_hal_ms_delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_PCA9685_I2C_HAL */
