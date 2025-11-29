PCA9685 I2C Driver

A platform-independent C driver for the NXP PCA9685 16-channel, 12-bit PWM Fm+ I2C-bus LED controller.

This library is designed to be easily portable to any microcontroller architecture (STM32, AVR, ESP32, PIC, etc.) by implementing a simple Hardware Abstraction Layer (HAL).

Features

Platform Agnostic: logic is separated from hardware implementation.

Full Control: Access to all 16 PWM channels.

Precision: Set PWM duty cycle and phase delay with 12-bit resolution (0-4096 steps).

Frequency Control: Adjustable PWM frequency (typically 24Hz to 1526Hz).

Power Management: Support for Sleep Mode and Wake-up/Restart sequences.

Address Management: Support for I2C Sub-Addresses and All Call Addresses.

Output Configuration: Configurable output logic (Inverted/Non-inverted) and drive type (Open-drain/Totem-pole).

File Structure

pca9685_i2c.c / .h: The core driver logic and API definitions.

pca9685_i2c_hal.h: The header defining the low-level functions you must implement for your specific hardware.

pca9685_i2c_hal.c: An implementation example (e.g., for STM32 using HAL).

Getting Started

1. Porting to your Platform (HAL)

To use this library, you must implement the four functions defined in pca9685_i2c_hal.h.

Required Functions:

int16_t pca9685_i2c_hal_init(): Initialize your I2C hardware.

int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count): Read bytes from the device.

int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count): Write bytes to the device.

void pca9685_i2c_hal_ms_delay(uint32_t ms): Millisecond delay function.

Note: An example implementation for STM32 (using CubeMX HAL) is provided in the pca9685_i2c_hal.c file.

2. Initialization

Declare a pca9685_dev_t structure.

Call pca9685_i2c_register to set up the address struct.

Initialize the hardware interface.

Reset the device and set the PWM frequency.

#include "pca9685_i2c.h"

pca9685_dev_t dev;

void setup() {
    // 1. Initialize Hardware (I2C)
    if (pca9685_i2c_hal_init() != PCA9685_OK) {
        // Handle error
    }

    // 2. Register Device Addresses
    // I2C Address: 0x40 (default)
    // AllCall: 0x70, Sub1: 0x71, Sub2: 0x72, Sub3: 0x74
    pca9685_i2c_register(&dev, I2C_ADDRESS_PCA9685, 
                               I2C_ALL_CALL_ADDRESS_PCA9685,
                               I2C_SUB_ADDRESS_1_PCA9685,
                               I2C_SUB_ADDRESS_2_PCA9685,
                               I2C_SUB_ADDRESS_3_PCA9685);

    // 3. Reset and Wake up
    pca9685_i2c_reset(); // Software Reset
    pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP); // Go to sleep to set prescaler

    // 4. Set Frequency to 50Hz (Standard Servo frequency)
    pca9685_i2c_write_pre_scale(dev, 50.0, 25000000.0); 
    
    // 5. Wake up and Restart
    pca9685_i2c_sleep_mode(dev, PCA9685_MODE_NORMAL);
    pca9685_i2c_restart(dev);
}


3. Controlling Outputs

Set a specific channel (e.g., Channel 0) to 50% Duty Cycle:

// Channel 0, 50% Duty Cycle, 0% Phase delay
pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 0.0f);


Turn a channel fully ON or OFF:

// Turn Channel 1 ON completely (no PWM)
pca9685_i2c_led_set(dev, 1, PCA9685_LED_ON);

// Turn Channel 1 OFF
pca9685_i2c_led_set(dev, 1, PCA9685_LED_OFF);


STM32 Implementation Details

If you are using the provided pca9685_i2c_hal.c for STM32:

Generate your project using STM32CubeMX.

Enable I2C1 (or change the handle in the HAL file).

Ensure main.h is included in the HAL file to access I2C_HandleTypeDef.

The driver assumes extern I2C_HandleTypeDef hi2c1; exists.

License

MIT License

Copyright (c) 2025 loed811

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
