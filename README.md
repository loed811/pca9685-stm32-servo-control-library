# PCA9685 I2C Driver for STM32 (servo control)

🇷🇺 [Документация на русском — README.ru.md](README.ru.md)

A lightweight, platform-agnostic C driver for the NXP **PCA9685** 16-channel, 12-bit PWM I2C controller. It is mainly used to drive up to 16 hobby servos (or LEDs) from an STM32 over two I2C wires.

The core logic (`pca9685_i2c.c/.h`) is separate from the hardware layer (`pca9685_i2c_hal.c/.h`). An STM32 HAL implementation comes with the library. To port it to another MCU (AVR, ESP32, PIC, …), you replace only the HAL file.

## Contents

- [Features](#features)
- [Files](#files)
- [Wiring](#wiring)
- [STM32CubeMX / CubeIDE setup](#stm32cubemx--cubeide-setup)
- [Minimal usage example](#minimal-usage-example)
- [Servo pulse ↔ duty cycle](#servo-pulse--duty-cycle)
- [API reference](#api-reference)
- [Examples](#examples)
- [Troubleshooting](#troubleshooting)
- [Porting to other platforms](#porting-to-other-platforms)
- [License](#license)

## Features

- **Full control**: access to the MODE1 and MODE2 registers, sub-addresses and All-Call.
- **Precision PWM**: helper functions take the duty cycle and phase delay as percentages (0.0 – 100.0 %).
- **Frequency control**: easy PRE_SCALE configuration (e.g. 50 Hz for servos).
- **Power management**: sleep, wake and restart sequences.
- **Hardware abstraction layer**: driver logic is kept apart from the I2C hardware calls.
- **STM32 ready**: includes a default HAL implementation for STM32 using the HAL library (`hi2c1`).

## Files

| File | Purpose |
|------|---------|
| `pca9685_i2c.h` / `pca9685_i2c.c` | Driver: types, register map, all device functions |
| `pca9685_i2c_hal.h` | Interface the driver needs from the platform (init/read/write/delay) |
| `pca9685_i2c_hal.c` | STM32 HAL implementation of that interface (uses `hi2c1`, 100 ms timeout) |
| `examples/` | Copy-paste snippets for a CubeMX `main.c` |

## Wiring

Typical setup: an STM32 board (e.g. Nucleo / Blue Pill) and a common PCA9685 breakout board (16 servo headers, V+ screw terminal).

```
 STM32                    PCA9685 board                 Servo supply
 ─────                    ─────────────                 ────────────
 3V3  ─────────────────── VCC   (logic, 2.3–5.5 V)
 GND  ─────────────────── GND ──────────────────────────── GND (–)
 I2C SCL (e.g. PB6/PB8) ─ SCL
 I2C SDA (e.g. PB7/PB9) ─ SDA
 GPIO (optional) ──────── OE    (active LOW)
                          V+    ─────────────────────────── +5…6 V (+)
                          PWM0…15 / V+ / GND ── servo signal / red / brown
```

- **SCL / SDA**: connect them to the pins of the I2C peripheral you enabled in CubeMX. Common STM32F1/F4 choices are I2C1 on PB6 = SCL and PB7 = SDA, or the remap PB8 = SCL and PB9 = SDA. On Nucleo-64 boards the Arduino header D15/D14 is PB8/PB9.
- **Pull-up resistors**: I2C needs pull-ups on SDA and SCL. Most PCA9685 breakouts already have 10 kΩ pull-ups to VCC. If yours has none, add 2.2–4.7 kΩ to 3.3 V. The STM32 internal pull-ups (~40 kΩ) are too weak to rely on.
- **VCC vs V+**:
  - **VCC** powers the PCA9685 logic. Power it from the STM32's **3.3 V**, so the I2C pull-ups also go to 3.3 V. If you feed VCC with 5 V, the bus is pulled up to 5 V: check that your SDA/SCL pins are 5 V tolerant (FT in the datasheet).
  - **V+** only feeds the servo power rail on the board's headers. Use a **separate** 5–6 V supply that can deliver the servos' stall current (roughly 1 A or more per standard servo under load). **Do not** power servos from the ST-Link/Nucleo 5 V pin or from USB.
  - Put a large electrolytic capacitor (470–1000 µF) across V+ and GND close to the board.
- **Common GND**: the STM32, the PCA9685 and the servo power supply must share ground. Without it, the PWM signal has no reference and the servos twitch or don't move.
- **OE (Output Enable)** is active **LOW**. Breakouts pull it to GND, so the outputs are enabled when OE is left unconnected. If you wire it to a GPIO, start with that GPIO **high** (outputs off), configure the chip, then drive it **low**. This avoids random servo jumps at power-up. While OE is high, the outputs follow the `outne` setting (see `pca9685_i2c_output_init`).
- **Address jumpers A0–A5**: open = 0x40 (default). Each soldered jumper adds 1, 2, 4, 8, 16 or 32, which gives addresses 0x40–0x7F. Avoid 0x70, which is the default All-Call address.

## STM32CubeMX / CubeIDE setup

1. **Pinout & Configuration → Connectivity → I2C1** (or another I2C) → Mode: **I2C**.
2. **Parameter Settings**:
   - *I2C Speed Mode*: **Standard Mode (100 kHz)** is the safest choice. **Fast Mode (400 kHz)** works with short wires and proper pull-ups. The PCA9685 supports up to 1 MHz (Fm+).
   - *Primary Address Length*: 7-bit (the address of the STM32 itself does not matter in master mode).
3. **GPIO Settings**: check that SCL/SDA landed on the pins you wired, as *Alternate Function Open Drain*. Setting them to *Pull-up* is harmless but does not replace external resistors.
4. (Optional) Configure one GPIO as **Output, initial level High** for OE.
5. Generate the code. CubeMX creates `I2C_HandleTypeDef hi2c1;` and calls `MX_I2C1_Init()` in `main()`.
6. Copy `pca9685_i2c.c` and `pca9685_i2c_hal.c` to `Core/Src`, and `pca9685_i2c.h` and `pca9685_i2c_hal.h` to `Core/Inc`.
7. **Different I2C handle?** If you use I2C2/I2C3, edit `pca9685_i2c_hal.c` and replace `hi2c1` (the `extern` declaration and its uses) with your handle, e.g. `hi2c2`.
8. The driver uses `round()` from `<math.h>`. If the linker reports `undefined reference to 'round'`, add `m` to *Project → Properties → C/C++ Build → Settings → MCU GCC Linker → Libraries*.
9. Call the driver **after** `MX_I2C1_Init()`.

## Minimal usage example

Here the device is initialized, the output frequency is set to 50 Hz (standard for servos), and the servo on channel 0 moves to its centre position.

```c
#include "pca9685_i2c.h"

// Device handle
pca9685_dev_t pca9685;

void setup_pca9685(void) {
    // 1. Check that the I2C handle is initialised (MX_I2C1_Init() already called)
    pca9685_i2c_hal_init();

    // 2. Software reset to power-up defaults (do this BEFORE register(),
    //    otherwise the reset discards the address configuration)
    pca9685_i2c_reset();

    // 3. Register device with default addresses
    //    (I2C: 0x40, All-Call: 0x70, Sub1: 0x71, Sub2: 0x72, Sub3: 0x74)
    pca9685_i2c_register(&pca9685, I2C_ADDRESS_PCA9685,
                                   I2C_ALL_CALL_ADDRESS_PCA9685,
                                   I2C_SUB_ADDRESS_1_PCA9685,
                                   I2C_SUB_ADDRESS_2_PCA9685,
                                   I2C_SUB_ADDRESS_3_PCA9685);

    // 4. Enable register auto-increment: REQUIRED for pca9685_i2c_led_pwm_set(),
    //    which writes the 4 LEDn_ON/OFF registers in one transfer
    pca9685_i2c_autoincrement(pca9685, PCA9685_AUTOINCR_ON);

    // 5. Set output frequency to 50 Hz.
    //    The oscillator must be asleep while PRE_SCALE is written.
    pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_SLEEP);
    pca9685_i2c_write_pre_scale(pca9685, 50.0, 25000000); // 25 MHz internal clock
    pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_NORMAL);

    // 6. Restart PWM
    pca9685_i2c_restart(pca9685);

    // 7. Output structure: totem pole, not inverted
    pca9685_output_t output_conf = {
        .outdrv = PCA9685_OUTPUT_TOTEM_POLE,
        .outne  = PCA9685_OUTPUT_LOW,
        .och    = PCA9685_CH_ONSTOP,
        .invrt  = PCA9685_OUTPUT_NOTINVERT
    };
    pca9685_i2c_output_init(pca9685, output_conf);
}

void set_servo_position(void) {
    // Channel 0: 7.5 % duty at 50 Hz = 1.5 ms pulse = servo neutral position
    pca9685_i2c_led_pwm_set(pca9685, 0, 7.5, 0.0);
}
```

Note: `pca9685_i2c_register()` takes a **pointer** (`&pca9685`). All other functions take the device structure **by value** (`pca9685`).

## Servo pulse ↔ duty cycle

At 50 Hz the period is 20 ms, so `duty % = pulse_ms / 20 ms × 100`:

| Pulse | Duty cycle | Typical meaning |
|-------|-----------|-----------------|
| 0.5 ms | 2.5 % | extended min (some servos) |
| 1.0 ms | 5.0 % | 0° / full one way |
| 1.5 ms | 7.5 % | centre (90°) |
| 2.0 ms | 10.0 % | 180° / full other way |
| 2.5 ms | 12.5 % | extended max (some servos) |

One PWM step is 20 ms / 4096 ≈ 4.9 µs. Check your servo's datasheet for its exact range: pushing a servo past its mechanical end stops makes it buzz and heat up.

## API reference

All functions are declared in `pca9685_i2c.h` (driver) and `pca9685_i2c_hal.h` (platform layer). Unless stated otherwise, they return `int16_t`: `PCA9685_OK` (0) on success, otherwise non-zero (`PCA9685_ERR` = −1; functions that do several transfers can return the sum of errors, so test with `!= PCA9685_OK`).

### Device structure and constants

```c
typedef struct {
    uint8_t i2c_addr;      // main 7-bit address (default 0x40)
    uint8_t allcall_addr;  // All-Call address  (default 0x70)
    uint8_t sub_addr_1;    // default 0x71
    uint8_t sub_addr_2;    // default 0x72
    uint8_t sub_addr_3;    // default 0x74
} pca9685_dev_t;
```

| Macro | Value | Meaning |
|-------|-------|---------|
| `I2C_ADDRESS_PCA9685` | 0x40 | Default device address |
| `I2C_GEN_CALL_ADDRESS_PCA9685` | 0x00 | I2C General Call (used by reset) |
| `I2C_ALL_CALL_ADDRESS_PCA9685` | 0x70 | Default All-Call address |
| `I2C_SUB_ADDRESS_1/2/3_PCA9685` | 0x71 / 0x72 / 0x74 | Default sub-addresses |
| `PWM_OUTPUT_COUNTER_MAX` | 4096 | 12-bit PWM resolution |
| `PCA9685_OK` / `PCA9685_ERR` | 0 / −1 | Return codes |

### Initialisation and power

| Function | Description |
|----------|-------------|
| `void pca9685_i2c_register(pca9685_dev_t *dev, uint8_t i2c_addr, uint8_t allcall_addr, uint8_t sub_addr_1, uint8_t sub_addr_2, uint8_t sub_addr_3)` | Fills `*dev` and immediately writes the All-Call address and sub-addresses 1–3 to the chip. Returns nothing (errors are not reported). |
| `int16_t pca9685_i2c_reset(void)` | Sends the software-reset byte `SWRST` (0x06) to the General Call address 0x00, then waits 1 ms. This resets **every** PCA9685 on the bus to power-up defaults: sleep on, auto-increment off, All-Call on, default addresses. |
| `int16_t pca9685_i2c_sleep_mode(pca9685_dev_t dev, pca9685_sleep_mode_t mode)` | `PCA9685_MODE_SLEEP` turns the oscillator off (PWM stops, registers still accessible). `PCA9685_MODE_NORMAL` turns it on. Waits 1 ms afterwards. |
| `int16_t pca9685_i2c_restart(pca9685_dev_t dev)` | Restart sequence: rewrites MODE1, waits for the oscillator, then writes MODE1 again so a pending RESTART bit resumes the previous PWM. Call it after waking with `pca9685_i2c_sleep_mode(..., PCA9685_MODE_NORMAL)`. |
| `int16_t pca9685_i2c_clock(pca9685_dev_t dev, pca9685_auto_extclk_t clk)` | `PCA9685_CLK_INTERNAL` (25 MHz, default) or `PCA9685_CLK_EXTERNAL` (EXTCLK pin). Per the datasheet, EXTCLK can only be set while in sleep, and only a power cycle or reset clears it. |
| `int16_t pca9685_i2c_autoincrement(pca9685_dev_t dev, pca9685_auto_incr_t setting)` | `PCA9685_AUTOINCR_ON` / `_OFF`. **Must be ON** before using `pca9685_i2c_led_pwm_set`, `pca9685_i2c_led_set`, `pca9685_i2c_all_led_set` and `pca9685_i2c_all_led_pwm_set`, which write 4 registers in one transfer. |
| `int16_t pca9685_i2c_output_init(pca9685_dev_t dev, pca9685_output_t setting)` | Configures MODE2 from a `pca9685_output_t` (see below). |
| `int16_t pca9685_i2c_read_mode_1(pca9685_dev_t dev, uint8_t *mode)` | Reads the raw MODE1 register. |
| `int16_t pca9685_i2c_read_mode_2(pca9685_dev_t dev, uint8_t *mode)` | Reads the raw MODE2 register. |

`pca9685_output_t` fields:

| Field | Values | Meaning |
|-------|--------|---------|
| `outdrv` | `PCA9685_OUTPUT_TOTEM_POLE` (use for servos) / `PCA9685_OUTPUT_OPEN_DRAIN` | Output stage type |
| `outne` | `PCA9685_OUTPUT_LOW` / `PCA9685_OUTPUT_HIGH` / `PCA9685_OUTPUT_HIGH_IMPEDANCE` | Output state while OE pin is high. `PCA9685_OUTPUT_LOW` is the safe choice for servos. |
| `och` | `PCA9685_CH_ONSTOP` / `PCA9685_CH_ONACK` | Outputs update on I2C STOP or on each ACK |
| `invrt` | `PCA9685_OUTPUT_NOTINVERT` / `PCA9685_OUTPUT_INVERT` | Invert output logic (keep NOTINVERT for servos) |

### Frequency

| Function | Description |
|----------|-------------|
| `int16_t pca9685_i2c_write_pre_scale(pca9685_dev_t dev, double frequency, double osc_clk_hz)` | Writes `PRE_SCALE = round(osc_clk_hz / (4096 × frequency)) − 1`. **The device must be in SLEEP mode.** Valid range is roughly 24 Hz to 1526 Hz with the 25 MHz internal clock. For 50 Hz at 25 MHz, the prescale value is 121. |
| `int16_t pca9685_i2c_read_pre_scale(pca9685_dev_t dev, double *frequency, double osc_clk_hz)` | Reads PRE_SCALE and computes the resulting frequency `osc_clk_hz / (4096 × (PRE_SCALE + 1))`. |

### PWM outputs (channels 0–15)

| Function | Description |
|----------|-------------|
| `int16_t pca9685_i2c_led_pwm_set(pca9685_dev_t dev, uint8_t led_no, float d_cycle, float delay)` | Sets channel `led_no` (0–15) to a `d_cycle` % high time, starting `delay` % into the period. Both values are percentages, 0.0–100.0. Use `delay = 0` for servos, or stagger it per channel to spread current peaks. Keep `d_cycle + delay ≤ 100`. Needs auto-increment ON. |
| `int16_t pca9685_i2c_all_led_pwm_set(pca9685_dev_t dev, float d_cycle, float delay)` | Same, for all 16 channels at once (ALL_LED registers). In the current implementation it returns `PCA9685_ERR` when `delay` is 0, so pass a small non-zero delay, or call `pca9685_i2c_led_pwm_set` for each channel. |
| `int16_t pca9685_i2c_led_set(pca9685_dev_t dev, uint8_t led_no, pca9685_led_state_t state)` | Sets a channel fully ON or fully OFF without PWM (`PCA9685_LED_ON` / `PCA9685_LED_OFF`). Not useful for servos: a constant-high signal is not a valid servo pulse. |
| `int16_t pca9685_i2c_all_led_set(pca9685_dev_t dev, pca9685_led_state_t state)` | Same, for all channels. `PCA9685_LED_OFF` is a quick way to make every output stop sending pulses. |

### Addressing (All-Call and sub-addresses)

These are useful when several PCA9685 chips share a bus and must be updated at the same time.

| Function | Description |
|----------|-------------|
| `int16_t pca9685_i2c_write_allcall_addr(pca9685_dev_t dev, uint8_t allcall_addr)` | Programs the 7-bit All-Call address. |
| `int16_t pca9685_i2c_read_allcall_addr(pca9685_dev_t dev, uint8_t *allcall_addr)` | Reads it back (7-bit). |
| `int16_t pca9685_i2c_allcall_address_resp(pca9685_dev_t dev, pca9685_addr_resp_t resp)` | `PCA9685_ADDR_RESPOND` / `PCA9685_ADDR_NORESPOND`: whether the chip answers on its All-Call address. |
| `int16_t pca9685_i2c_write_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t sub_addr)` | Programs sub-address `PCA9685_SUB_ADDR_1/2/3`. |
| `int16_t pca9685_i2c_read_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t *sub_addr)` | Reads a sub-address back (7-bit). |
| `int16_t pca9685_i2c_sub_addr_resp(pca9685_dev_t dev, pca9685_subaddr_no_t sub_addr, pca9685_addr_resp_t resp)` | Enables or disables the response to a sub-address. All sub-addresses are disabled after reset. |

To send a command to every chip that answers on the All-Call address, build a second handle whose `i2c_addr` is that address. For example, use `pca9685_dev_t all = pca9685; all.i2c_addr = I2C_ALL_CALL_ADDRESS_PCA9685;` and pass `all` to the write functions. Reads from a shared address are meaningless.

### Platform layer (`pca9685_i2c_hal.h`)

| Function | Description |
|----------|-------------|
| `int16_t pca9685_i2c_hal_init(void)` | STM32 version: returns `PCA9685_ERR` if `hi2c1.Instance` is NULL (I2C not initialised). It does not configure the hardware; CubeMX's `MX_I2C1_Init()` does that. |
| `int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)` | Writes the 1-byte register address, then reads `count` bytes. `address` is 7-bit; the STM32 implementation shifts it left by 1. |
| `int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)` | Writes `count` bytes; `data[0]` is the register address. |
| `void pca9685_i2c_hal_ms_delay(uint32_t ms)` | Blocking delay (`HAL_Delay` on STM32). |

## Examples

| Example | What it shows |
|---------|---------------|
| [`examples/single_servo_sweep/main.c`](examples/single_servo_sweep/main.c) | Full init sequence, microseconds → duty-cycle helper, one servo sweeping between 1.0 and 2.0 ms |
| [`examples/multi_servo/main.c`](examples/multi_servo/main.c) | Four servos, an angle → pulse helper, staggered phase delays, reading back the PWM frequency |

The examples are fragments of a CubeMX-generated `main.c`. Copy each `USER CODE BEGIN … END` block into the section of the same name in your project.

## Troubleshooting

**Every call returns `PCA9685_ERR` / nothing moves**
- Check the address. 0x40 is the 7-bit default; soldered A0–A5 jumpers change it. The driver takes the **7-bit** address and the STM32 HAL layer shifts it, so don't pass 0x80. To scan the bus, call `HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 10)` for `addr` = 0x03 to 0x77 and print the addresses that answer `HAL_OK`.
- Check that `MX_I2C1_Init()` runs before the driver and that the handle in `pca9685_i2c_hal.c` matches yours (`hi2c1`, `hi2c2`, …).
- SDA and SCL may be swapped, or the pull-ups may be missing. Without pull-ups the bus lines stay low and HAL reports `HAL_BUSY` or timeouts.
- VCC must be powered. V+ alone does not power the chip's logic.
- If the STM32 HAL I2C locks up in `HAL_BUSY` after a glitch, power-cycle the board. Some STM32F1 parts also need the documented I2C "busy flag" errata workaround.

**I2C works, but the servos don't move**
- Auto-increment is not enabled. After `pca9685_i2c_reset()` it is OFF, and `pca9685_i2c_led_pwm_set()` then writes all 4 bytes into the same register. Call `pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_ON)`.
- The chip is still in sleep: it starts in sleep after reset. Call `pca9685_i2c_sleep_mode(dev, PCA9685_MODE_NORMAL)` and `pca9685_i2c_restart(dev)`.
- The OE pin is pulled high, or the GPIO wired to it is never driven low.
- There is no V+ supply, or the supply has no common GND with the STM32.

**Servos jitter, twitch or reset the MCU**
- The power supply is too weak or the wires are too thin. Servos draw current peaks of 1 A or more when they start moving. Use a dedicated 5–6 V supply on V+ and add 470–1000 µF across V+/GND.
- If the STM32 resets when the servos move, they are being powered from the same 5 V as the MCU. Separate the supplies and keep only GND common.
- The pulse may be outside the servo's range. Stay within roughly 1.0–2.0 ms (5–10 % at 50 Hz) unless the servo's datasheet allows more.
- The frequency may be off: the internal oscillator is only approximately 25 MHz (a few percent tolerance). Measure the PWM period with a scope or logic analyser. If it is off, pass the corrected oscillator value as `osc_clk_hz` (e.g. `26000000` if you measured 52 Hz instead of 50 Hz).
- Long or unshielded signal wires running next to motor wires can pick up noise. Keep them short, or twist each signal wire with GND.
- Set `outdrv = PCA9685_OUTPUT_TOTEM_POLE`. Open-drain outputs without pull-ups give no clean signal.

**Servo jumps at power-up**
- The outputs are active before the firmware has configured them. Wire OE to a GPIO that starts high, then drive it low once the PWM is set, or set the first position before waking the chip.

**`undefined reference to 'round'`**
- Link the math library (`-lm`, or add `m` in the CubeIDE linker libraries).

## Porting to other platforms

To use this driver on Arduino, ESP32 or other microcontrollers, you only need to modify `pca9685_i2c_hal.c`.

Implement these 4 functions in `pca9685_i2c_hal.c` with your platform's I2C API:

- `pca9685_i2c_hal_init()`: initialise the I2C hardware.
- `pca9685_i2c_hal_read(address, reg, data, count)`: write the register address `*reg`, then read `count` bytes from the 7-bit `address`.
- `pca9685_i2c_hal_write(address, data, count)`: write `count` bytes to `address`; the first byte is the register address.
- `pca9685_i2c_hal_ms_delay(ms)`: delay for `ms` milliseconds.

## License

MIT License — Copyright (c) 2025 loed811. See [LICENSE](LICENSE).
