PCA9685 I2C Driver

A lightweight, platform-agnostic C driver for the NXP PCA9685 16-channel, 12-bit PWM I2C bus controller.

Designed for modularity, this library separates the core logic from the hardware layer, making it easy to use with STM32 (out of the box) or port to any other architecture (AVR, ESP32, PIC, etc.) by simply replacing the HAL file.

📋 Features

Full Control: Access to Mode 1 & 2 registers, Sub-Addresses, and All-Call functionality.

Precision PWM: Helper functions to set Duty Cycle and Phase Delay in percentages (0.0 - 100.0%).

Frequency Control: Easy configuration of the PRE_SCALE register (e.g., setting 50Hz for servos).

Power Management: Sleep, Wake, and Restart sequence implementation.

Hardware Abstraction Layer (HAL): Clean separation between driver logic and I2C hardware calls.

STM32 Ready: Includes a default HAL implementation for STM32 using the HAL Library (hi2c1).

📂 File Structure

File

Description

pca9685_i2c.c

Core driver implementation. Platform independent.

pca9685_i2c.h

Main header file. Includes struct definitions and API prototypes.

pca9685_i2c_hal.c

Hardware Abstraction Layer. Currently implements STM32 I2C.

pca9685_i2c_hal.h

HAL Header. Defines the interface required by the core driver.

🚀 Getting Started (STM32)

This driver comes pre-configured for STM32 using the standard HAL library.

Add Files: Copy the .c and .h files into your project source/include directories.

Setup I2C: Ensure your I2C peripheral is initialized in main.c (usually done automatically by CubeMX).

Check Handle: The driver assumes your I2C handle is named hi2c1. If yours is different (e.g., hi2c2), update the extern declaration in pca9685_i2c_hal.c.

Example Usage

Here is a quick example of initializing the driver, setting the frequency to 50Hz (standard for servos), and moving a servo on Channel 0.

#include "pca9685_i2c.h"

// Device handle
pca9685_dev_t pca9685;

void setup_pca9685(void) {
    // 1. Initialize the HAL (if needed)
    pca9685_i2c_hal_init();

    // 2. Register device with default addresses
    // (I2C Address: 0x40, AllCall: 0x70, Sub1: 0x71, Sub2: 0x72, Sub3: 0x74)
    pca9685_i2c_register(&pca9685, I2C_ADDRESS_PCA9685, 
                                   I2C_ALL_CALL_ADDRESS_PCA9685,
                                   I2C_SUB_ADDRESS_1_PCA9685,
                                   I2C_SUB_ADDRESS_2_PCA9685,
                                   I2C_SUB_ADDRESS_3_PCA9685);

    // 3. Reset the device to default state
    pca9685_i2c_reset();

    // 4. Set Output Frequency to 50Hz
    // Note: You must sleep the oscillator to change the prescaler
    pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_SLEEP);
    pca9685_i2c_write_pre_scale(pca9685, 50.0, 25000000); // 25MHz internal clock
    pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_NORMAL);

    // 5. Wake up and restart
    pca9685_i2c_restart(pca9685);
    
    // 6. Configure Output Structure (Totem Pole, Not Inverted)
    pca9685_output_t output_conf = {
        .outdrv = PCA9685_OUTPUT_TOTEM_POLE,
        .outne  = PCA9685_OUTPUT_LOW,
        .och    = PCA9685_CH_ONSTOP,
        .invrt  = PCA9685_OUTPUT_NOTINVERT
    };
    pca9685_i2c_output_init(pca9685, output_conf);
}

void set_servo_position(void) {
    // Set Channel 0 to ~7.5% duty cycle (1.5ms pulse for neutral servo position)
    pca9685_i2c_led_pwm_set(pca9685, 0, 7.5, 0.0);
}


🔌 Porting to Other Platforms

To use this driver on Arduino, ESP32, or other microcontrollers, you only need to modify pca9685_i2c_hal.c.

Implement the following 4 functions inside pca9685_i2c_hal.c using your platform's I2C API:

pca9685_i2c_hal_init(): Initialize I2C hardware.

pca9685_i2c_hal_read(...): Read count bytes from a specific register.

pca9685_i2c_hal_write(...): Write count bytes (the first byte is usually the register address).

pca9685_i2c_hal_ms_delay(...): Millisecond delay function.

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
