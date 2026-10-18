# PCA9685 PWM & Servo Driver for STM32

[![CI](https://github.com/loed811/pca9685-stm32-servo-control-library/actions/workflows/ci.yml/badge.svg)](https://github.com/loed811/pca9685-stm32-servo-control-library/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![C99](https://img.shields.io/badge/C-99-informational)
![Platform](https://img.shields.io/badge/platform-STM32%20%7C%20portable-lightgrey)

A lightweight, platform-agnostic C driver for the NXP **PCA9685**, a 16-channel, 12-bit PWM controller on I2C, typically used to drive hobby servos and LEDs.

The driver logic is separated from the hardware layer: it works with the STM32 HAL out of the box and can be ported to any MCU by implementing four functions. It is tested without hardware against a register-level simulator of the chip.

## Features

- **Full register control**: MODE1/MODE2, sleep and restart, internal or external clock, auto-increment, All Call address and Sub-Addresses.
- **PWM in percent**: duty cycle and phase delay as `0.0 - 100.0 %`, with correct wrap-around and true 0 % / 100 % outputs.
- **Servo helper**: set a pulse width in microseconds, e.g. `1500 us` for the neutral position.
- **Frequency control**: PRE_SCALE calculation with range checking (24 - 1526 Hz).
- **Portable HAL**: 4 functions to implement per platform. A reference STM32 HAL port is included.
- **Tested**: 34 unit tests run against a simulated PCA9685 in CI with GCC (ASan/UBSan) and Clang.
- **Small**: about 1.5 KB of flash, no dynamic memory, no libm. Builds warning-free with `-Wconversion` for Cortex-M0+/M3/M4/M7.

## Repository layout

```
include/            Public headers (driver API and HAL interface)
src/                Platform-independent driver
port/stm32/         HAL implementation for STM32 (STM32Cube HAL)
examples/
  servo_sweep/      Board bring-up, angle -> pulse mapping, servo sweep
tests/
  sim/              Register-level PCA9685 simulator (implements the HAL)
  test_pca9685.c    Unity test suite
```

## Hardware

Typical wiring of a PCA9685 breakout board to an STM32 Nucleo board (e.g. Nucleo-F401RE, I2C1):

| PCA9685 | STM32                | Notes                                                 |
|---------|----------------------|-------------------------------------------------------|
| VCC     | 3.3 V                | Logic supply                                          |
| GND     | GND                  | Common ground with the servo power supply             |
| SDA     | PB9 (I2C1_SDA, D14)  | Breakout boards usually have I2C pull-ups             |
| SCL     | PB8 (I2C1_SCL, D15)  |                                                       |
| OE      | GND or a GPIO        | Active-low output enable                              |
| V+      | External 5 - 6 V     | Servo power. Do not power servos from the Nucleo board |

## Quick start (STM32CubeIDE)

1. Enable **I2C1** in CubeMX (Standard or Fast mode). CubeMX generates `MX_I2C1_Init()` and the `hi2c1` handle.
2. Copy into your project:
   - `include/pca9685_i2c.h`, `include/pca9685_i2c_hal.h`
   - `src/pca9685_i2c.c`
   - `port/stm32/pca9685_i2c_hal.c`
   - optionally `examples/servo_sweep/servo_sweep.{c,h}`
3. Using another I2C peripheral? Add `PCA9685_I2C_HANDLE=hi2c2` to the preprocessor symbols. No need to edit the port file.
4. Call the example from `main.c`:

```c
#include "servo_sweep.h"

/* USER CODE BEGIN 2 */
pca9685_dev_t pca9685;
if (servo_board_init(&pca9685) != PCA9685_OK)
{
    Error_Handler();
}
/* USER CODE END 2 */

while (1)
{
    /* 0 -> 180 -> 0 degrees on channel 0, 5 degree steps, 20 ms per step */
    servo_sweep(pca9685, 0, 5.0f, 20);
}
```

`servo_board_init()` performs the full bring-up sequence:

```c
pca9685_i2c_reset();                                    /* Power-up state, oscillator asleep */
pca9685_i2c_register(&dev, I2C_ADDRESS_PCA9685, ...);   /* Program All Call / Sub-Addresses */
pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_ON);    /* Required for 4-byte PWM writes */
pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP);        /* PRE_SCALE is only writable in sleep */
pca9685_i2c_write_pre_scale(dev, 50.0, PCA9685_INTERNAL_OSC_HZ);
pca9685_i2c_output_init(dev, output);                   /* Totem-pole outputs */
pca9685_i2c_restart(dev);                               /* Wake the oscillator */
```

After that, drive the outputs:

```c
pca9685_i2c_servo_pulse_set(dev, 0, 1500.0f, 50.0f);   /* Servo on channel 0 to neutral */
pca9685_i2c_led_pwm_set(dev, 4, 25.0f, 50.0f);         /* 25 % duty, shifted by half a period */
pca9685_i2c_led_set(dev, 15, PCA9685_LED_ON);          /* Constant high */
```

Every function returns `PCA9685_OK` (0) or `PCA9685_ERR` (-1).

## How the PWM works

- **Frequency.** `f = osc / (4096 * (PRE_SCALE + 1))`. For servos, 50 Hz gives `PRE_SCALE = 121`, an actual 50.03 Hz.
- **Resolution.** At 50 Hz, one of the 4096 steps is `20 ms / 4096 = 4.88 us`. A standard servo pulse of 1.0 - 2.0 ms spans 205 - 410 steps, so a 180° servo moves about 0.9° per step.
- **ON/OFF counts.** Each channel goes high when the 12-bit counter reaches its ON value and low at its OFF value. The phase delay moves the ON point, and the OFF point wraps around the end of the period. Staggering channels this way spreads out the current spikes when many servos or LEDs switch at once.
- **0 % and 100 %** use the dedicated full OFF / full ON bits, because the datasheet forbids equal ON and OFF values.
- **Oscillator tolerance.** The internal oscillator is nominally 25 MHz but varies between chips. If timing matters, measure the output frequency with a logic analyzer and correct the clock: `osc = 25 MHz * f_measured / f_expected`.

## API overview

| Group        | Functions |
|--------------|-----------|
| Setup        | `pca9685_i2c_register`, `pca9685_i2c_reset`, `pca9685_i2c_restart`, `pca9685_i2c_sleep_mode`, `pca9685_i2c_autoincrement`, `pca9685_i2c_clock`, `pca9685_i2c_output_init` |
| Frequency    | `pca9685_i2c_write_pre_scale`, `pca9685_i2c_read_pre_scale` |
| Outputs      | `pca9685_i2c_led_pwm_set`, `pca9685_i2c_all_led_pwm_set`, `pca9685_i2c_led_set`, `pca9685_i2c_all_led_set`, `pca9685_i2c_servo_pulse_set` |
| Addressing   | `pca9685_i2c_write_allcall_addr`, `pca9685_i2c_read_allcall_addr`, `pca9685_i2c_write_sub_addr`, `pca9685_i2c_read_sub_addr`, `pca9685_i2c_sub_addr_resp`, `pca9685_i2c_allcall_address_resp` |
| Registers    | `pca9685_i2c_read_mode_1`, `pca9685_i2c_read_mode_2` |

All functions are documented with Doxygen comments in [`include/pca9685_i2c.h`](include/pca9685_i2c.h).

## Porting to other platforms

Implement the four functions from [`include/pca9685_i2c_hal.h`](include/pca9685_i2c_hal.h) for your MCU (ESP32, AVR, RP2040, Linux `i2c-dev`, ...). Use [`port/stm32/pca9685_i2c_hal.c`](port/stm32/pca9685_i2c_hal.c) as a reference.

| Function | Responsibility |
|----------|----------------|
| `int16_t pca9685_i2c_hal_init(void)` | Check or initialize the I2C peripheral |
| `int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)` | Write the register address `*reg`, then read `count` bytes |
| `int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)` | Write `count` bytes; `data[0]` is the register address |
| `void pca9685_i2c_hal_ms_delay(uint32_t ms)` | Blocking delay in milliseconds |

`address` is the 7-bit I2C address; shift it if your platform expects an 8-bit one.

## Testing without hardware

The tests run the unmodified driver against [`tests/sim`](tests/sim), a register-level model of the PCA9685 that implements the HAL. It reproduces the datasheet behaviour the driver depends on:

- register auto-increment and its rollover
- PRE_SCALE writes being ignored while the oscillator runs
- the RESTART flag and the sticky EXTCLK bit
- ALL_LED broadcast, Software Reset via General Call
- All Call / Sub-Address responses
- injected I2C bus failures

```bash
cmake -S . -B build -DPCA9685_SANITIZE=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Requirements: CMake 3.21+ and a C99 compiler. The [Unity](https://github.com/ThrowTheSwitch/Unity) test framework is fetched automatically.

CI runs on every push. It runs the test suite with GCC (AddressSanitizer + UndefinedBehaviorSanitizer) and Clang, cross-compiles the driver for Cortex-M0+/M3/M4/M7 with `-Werror` and reports code size, and runs `cppcheck`.

## References

- [NXP PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)

## License

[MIT](LICENSE) © 2025 loed811
