#ifndef MAIN_PCA9685_I2C_HAL
#define MAIN_PCA9685_I2C_HAL

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/**
 * @brief Error code definition for failed operations.
 */
#define PCA9685_ERR     -1

/**
 * @brief Success code definition for successful operations.
 */
#define PCA9685_OK      0x00

/**
 * @brief User-implemented initialization for the I2C hardware.
 * * This function should perform any specific hardware setup required
 * before I2C communication can occur (e.g., checking handles, GPIO setup).
 * * @return PCA9685_OK on success, or PCA9685_ERR on failure.
 */
int16_t pca9685_i2c_hal_init();

/**
 * @brief User-implemented I2C read function.
 * * This function is responsible for reading data from a specific register
 * on the PCA9685 device.
 * * @param address The 7-bit I2C device address.
 * @param reg Pointer to the register address to read from (1 byte).
 * @param data Pointer to the buffer where the read data will be stored.
 * @param count The number of bytes to read.
 * @return PCA9685_OK on success, or PCA9685_ERR on failure.
 */
int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count);

/**
 * @brief User-implemented I2C write function.
 * * This function sends data to the PCA9685 device. The data buffer
 * typically starts with the target register address followed by the payload.
 * * @param address The 7-bit I2C device address.
 * @param data Pointer to the buffer containing [Register Address, Data...].
 * @param count The total number of bytes to write.
 * @return PCA9685_OK on success, or PCA9685_ERR on failure.
 */
int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count);

/**
 * @brief User-implemented blocking delay function.
 * * Used for hardware stabilization delays (e.g., oscillator startup).
 * * @param ms The number of milliseconds to wait.
 */
void pca9685_i2c_hal_ms_delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_PCA9685_I2C_HAL */
