PCA9685 Generic C Driver

A platform-independent, lightweight C driver for the NXP PCA9685 16-channel, 12-bit PWM I2C controller.

This library is designed for portability. It separates the core logic from the hardware implementation, allowing you to run it on STM32, AVR, ESP32, PIC, nRF, or any other architecture by simply defining four low-level HAL functions.

🌟 Features

🧩 Platform Agnostic: Pure C logic separated from hardware calls.

🎛 Full Control: Individual control over all 16 PWM channels.

🎯 High Precision: 12-bit resolution for Duty Cycle and Phase Delay (0-4096 steps).

⚡ Frequency Control: Adjustable PWM frequency (approx. 24Hz to 1526Hz).

💤 Power Management: Built-in support for Sleep Mode and Wake-up sequences.

🔗 Advanced Addressing: Support for I2C Sub-Addresses and All Call Addresses.

⚙️ Hardware Config: Support for Inverted logic and Open-drain/Totem-pole output driver selection.

📂 File Structure

File

Description

pca9685_i2c.c

Core Source: Contains the logic and API implementation.

pca9685_i2c.h

Core Header: API definitions, structs, and enums.

pca9685_i2c_hal.h

HAL Header: Defines the 4 low-level functions you must implement.

pca9685_i2c_hal.c

HAL Source: An example implementation (e.g., for STM32 HAL).

🚀 Integration Guide

To use this driver, you need to "bridge" it to your microcontroller's hardware by implementing the Hardware Abstraction Layer (HAL).

1. Implement the HAL

Define the following four functions in your project (declared in pca9685_i2c_hal.h):

// 1. Initialize your I2C peripheral
int16_t pca9685_i2c_hal_init(void);

// 2. Read bytes from I2C
int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count);

// 3. Write bytes to I2C
int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count);

// 4. Millisecond delay
void pca9685_i2c_hal_ms_delay(uint32_t ms);


Note: A complete implementation for STM32 (CubeMX) is already provided in pca9685_i2c_hal.c.

2. Initialization Sequence

Here is a standard startup routine:

#include "pca9685_i2c.h"

pca9685_dev_t servo_controller;

void PCA9685_Setup(void) {
    // 1. Initialize the low-level I2C hardware
    if (pca9685_i2c_hal_init() != PCA9685_OK) {
        // Handle Hardware Error
        return;
    }

    // 2. Register the device addresses
    // (Main Address: 0x40, plus default AllCall/Sub addresses)
    pca9685_i2c_register(&servo_controller, 
                         I2C_ADDRESS_PCA9685, 
                         I2C_ALL_CALL_ADDRESS_PCA9685,
                         I2C_SUB_ADDRESS_1_PCA9685,
                         I2C_SUB_ADDRESS_2_PCA9685,
                         I2C_SUB_ADDRESS_3_PCA9685);

    // 3. Reset the device (Software Reset)
    pca9685_i2c_reset();

    // 4. Set PWM Frequency (e.g., 50Hz for Servos)
    // Device must be in SLEEP mode to set prescaler
    pca9685_i2c_sleep_mode(servo_controller, PCA9685_MODE_SLEEP);
    pca9685_i2c_write_pre_scale(servo_controller, 50.0, 25000000.0); // 50Hz, 25MHz Internal Clock
    
    // 5. Wake up and Restart
    pca9685_i2c_sleep_mode(servo_controller, PCA9685_MODE_NORMAL);
    pca9685_i2c_restart(servo_controller);
}


3. Usage Examples

Controlling a Servo (PWM)

Set Channel 0 to a specific duty cycle.

// Set Channel 0: 50% Duty Cycle, 0% Phase Delay
// Range is 0.0f to 100.0f
pca9685_i2c_led_pwm_set(servo_controller, 0, 50.0f, 0.0f);


Digital Control (ON/OFF)

Turn a channel fully ON (logic high) or OFF (logic low) bypassing the PWM counter.

// Turn Channel 1 completely ON
pca9685_i2c_led_set(servo_controller, 1, PCA9685_LED_ON);

// Turn Channel 1 completely OFF
pca9685_i2c_led_set(servo_controller, 1, PCA9685_LED_OFF);


🛠 STM32 Specifics

This repository includes a ready-to-use HAL for STM32.

CubeMX: Generate your project with I2C enabled.

Include: Ensure main.h is included in pca9685_i2c_hal.c.

Handle: The driver assumes the global handle is named hi2c1.

If your handle is different (e.g., hi2c2), update the extern declaration in pca9685_i2c_hal.c.

📄 License

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
