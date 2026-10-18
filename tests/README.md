# Host-side unit tests

Run the driver's logic on a Linux/macOS host. No STM32 board and no PCA9685 chip are needed.

```sh
make -C tests            # build and run everything
make -C tests STRICT=1   # also fail on known driver bugs
make -C tests CC=clang
```

| Binary | Code under test | What is faked |
| --- | --- | --- |
| `test_pca9685_i2c` | `pca9685_i2c.c` | `pca9685_i2c_hal_*` (`fake_pca9685_hal.c`: 256-byte register file plus a log of every I2C transaction) |
| `test_pca9685_i2c_hal_stm32` | `pca9685_i2c_hal.c`, plus a full-stack test | `HAL_I2C_Master_Transmit/Receive`, `HAL_Delay` (`stubs/main.h`) |

Covered: PRE_SCALE from frequency (and back), duty/delay % → ON/OFF ticks, channel → register mapping,
range clamping, MODE1/MODE2 bit manipulation, address registers, SWRST, error propagation,
STM32 address shift and timeout.

`KNOWN_BUG(...)` checks describe the correct behaviour that the driver does **not** have yet.
They print `XFAIL` and do not fail the build. Once a bug is fixed they print `XPASS`; at that point,
turn the check into a plain `CHECK`.
