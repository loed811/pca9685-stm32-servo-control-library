#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

/**
 * @brief Returns the LEDn_ON_L register address of a channel.
 * @param led_no LED channel number (0-15).
 * @return Register address.
 */
static uint8_t pca9685_led_reg(uint8_t led_no)
{
    return (uint8_t)(LED_OFFSET_ADR + (led_no * 4));
}

/**
 * @brief Converts a percentage to 12-bit counter steps (0-4096).
 * Out-of-range values (and NaN) are clamped to 0-100 %.
 * @param percent Value in percent.
 * @return Number of counter steps.
 */
static uint16_t pca9685_percent_to_counts(float percent)
{
    if(!(percent > 0.0f))
        return 0;
    if(percent >= 100.0f)
        return PWM_OUTPUT_COUNTER_MAX;
    /* Round to nearest step without pulling in libm */
    return (uint16_t)(percent * (PWM_OUTPUT_COUNTER_MAX / 100.0f) + 0.5f);
}

/**
 * @brief Writes the four ON/OFF registers starting at 'reg' in one transfer.
 * Bit 12 of 'on'/'off' maps to the full ON/OFF bit (bit 4 of the _H register).
 * @param dev The device structure.
 * @param reg LEDn_ON_L or ALL_LED_ON_L register address.
 * @param on Value for the ON registers.
 * @param off Value for the OFF registers.
 * @return Error code.
 */
static int16_t pca9685_i2c_write_led(pca9685_dev_t dev, uint8_t reg, uint16_t on, uint16_t off)
{
    uint8_t data[5];
    data[0] = reg;
    data[1] = (uint8_t)(on & 0xFF);   /* LEDn_ON_L */
    data[2] = (uint8_t)(on >> 8);     /* LEDn_ON_H */
    data[3] = (uint8_t)(off & 0xFF);  /* LEDn_OFF_L */
    data[4] = (uint8_t)(off >> 8);    /* LEDn_OFF_H */

    return pca9685_i2c_hal_write(dev.i2c_addr, data, 5);
}

/**
 * @brief Writes a PWM duty cycle and phase delay to the four ON/OFF registers at 'reg'.
 * The output goes high when the counter reaches ON and low when it reaches OFF,
 * so OFF may wrap around past the end of the 4096-step period.
 * @param dev The device structure.
 * @param reg LEDn_ON_L or ALL_LED_ON_L register address.
 * @param d_cycle Duty cycle in percent.
 * @param delay Phase delay in percent.
 * @return Error code.
 */
static int16_t pca9685_i2c_write_pwm(pca9685_dev_t dev, uint8_t reg, float d_cycle, float delay)
{
    uint16_t high_tm = pca9685_percent_to_counts(d_cycle);
    uint16_t delay_tm = pca9685_percent_to_counts(delay) % PWM_OUTPUT_COUNTER_MAX;

    /* ON and OFF must never hold the same value, use the full OFF / full ON bits instead */
    if(high_tm == 0)
        return pca9685_i2c_write_led(dev, reg, 0, PCA9685_LED_FULL);
    if(high_tm >= PWM_OUTPUT_COUNTER_MAX)
        return pca9685_i2c_write_led(dev, reg, PCA9685_LED_FULL, 0);

    uint16_t led_off_tm = (uint16_t)((delay_tm + high_tm) % PWM_OUTPUT_COUNTER_MAX);
    return pca9685_i2c_write_led(dev, reg, delay_tm, led_off_tm);
}

/**
 * @brief Read-modify-write of the MODE1 register.
 * The RESTART bit is always written as 0: writing back a set RESTART bit
 * would restart the PWM channels, while writing 0 has no effect.
 * @param dev The device structure.
 * @param mask Bits to modify.
 * @param value New value of the masked bits.
 * @return Error code.
 */
static int16_t pca9685_i2c_update_mode_1(pca9685_dev_t dev, uint8_t mask, uint8_t value)
{
    uint8_t mode;

    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = REG_MODE_1;
    data[1] = (uint8_t)(((mode & ~mask) | (value & mask)) & ~PCA9685_MODE1_RESTART);

    return pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
}

/**
 * @brief Reads the content of the MODE1 register.
 * @param dev The device structure containing the I2C address.
 * @param mode Pointer to store the read value.
 * @return Error code (PCA9685_OK or PCA9685_ERR).
 */
int16_t pca9685_i2c_read_mode_1(pca9685_dev_t dev, uint8_t *mode)
{
    uint8_t reg = REG_MODE_1;
    /* Read 1 byte from REG_MODE_1 */
    int16_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, mode, 1);
    return err;
}

/**
 * @brief Reads the content of the MODE2 register.
 * @param dev The device structure.
 * @param mode Pointer to store the read value.
 * @return Error code.
 */
int16_t pca9685_i2c_read_mode_2(pca9685_dev_t dev, uint8_t *mode)
{
    uint8_t reg = REG_MODE_2;
    /* Read 1 byte from REG_MODE_2 */
    int16_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, mode, 1);
    return err;
}

/**
 * @brief Configures the EXTCLK bit in the MODE1 register.
 * Used to switch between the internal 25MHz clock and an external clock source.
 * Note: The device must be in sleep mode, and EXTCLK can only be cleared by a reset.
 * @param dev The device structure.
 * @param clk Clock setting (Auto or External).
 * @return Error code.
 */
int16_t pca9685_i2c_clock(pca9685_dev_t dev, pca9685_auto_extclk_t clk)
{
    return pca9685_i2c_update_mode_1(dev, PCA9685_MODE1_EXTCLK,
                                     clk == PCA9685_CLK_EXTERNAL ? PCA9685_MODE1_EXTCLK : 0);
}

/**
 * @brief Configures the AI (Auto-Increment) bit in the MODE1 register.
 * When enabled, the control register is automatically incremented after a read or write.
 * @param dev The device structure.
 * @param setting Enable or Disable auto-increment.
 * @return Error code.
 */
int16_t pca9685_i2c_autoincrement(pca9685_dev_t dev, pca9685_auto_incr_t setting)
{
    return pca9685_i2c_update_mode_1(dev, PCA9685_MODE1_AI,
                                     setting == PCA9685_AUTOINCR_ON ? PCA9685_MODE1_AI : 0);
}

/**
 * @brief Performs the restart sequence required to wake the device from sleep
 * and resume PWM generation.
 * @param dev The device structure.
 * @return Error code.
 */
int16_t pca9685_i2c_restart(pca9685_dev_t dev)
{
    uint8_t mode;

    /* 1. Read MODE1. RESTART (bit 7) set means PWM was running before sleep. */
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    uint8_t restart_pending = mode & PCA9685_MODE1_RESTART;

    uint8_t data[2];
    data[0] = REG_MODE_1;
    /* 2. Clear the SLEEP bit to start the oscillator (writing 0 to RESTART has no effect) */
    data[1] = (uint8_t)(mode & ~(PCA9685_MODE1_SLEEP | PCA9685_MODE1_RESTART));
    if(pca9685_i2c_hal_write(dev.i2c_addr, data, 2) != PCA9685_OK)
        return PCA9685_ERR;

    /* Wait for oscillator to stabilize (typically 500us) */
    pca9685_i2c_hal_ms_delay(STAB_TIME);

    if(!restart_pending)
        return PCA9685_OK;

    /* 3. Write logic 1 to bit 7 to restart PWM channels */
    data[1] |= PCA9685_MODE1_RESTART;
    return pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
}

/**
 * @brief Puts the device into or takes it out of Sleep mode (Low power mode).
 * Note: Oscillator is off in sleep mode.
 * @param dev The device structure.
 * @param sleep_mode Enable or Disable sleep.
 * @return Error code.
 */
int16_t pca9685_i2c_sleep_mode(pca9685_dev_t dev, pca9685_sleep_mode_t sleep_mode)
{
    int16_t err = pca9685_i2c_update_mode_1(dev, PCA9685_MODE1_SLEEP,
                                            sleep_mode == PCA9685_MODE_SLEEP ? PCA9685_MODE1_SLEEP : 0);
    if(err != PCA9685_OK)
        return err;

    /* Allow time for stabilization if waking up */
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    return PCA9685_OK;
}

/**
 * @brief Sends a Software Reset (SWRST) command via I2C General Call.
 * This resets the PCA9685 to its power-up state.
 * @return Error code.
 */
int16_t pca9685_i2c_reset(void)
{
    uint8_t data = SWRST;

    /* Write SWRST byte to the I2C General Call Address (0x00) */
    int16_t err = pca9685_i2c_hal_write(I2C_GEN_CALL_ADDRESS_PCA9685, &data, 1);
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    return err;
}

/**
 * @brief Configures the output driver structure in MODE2 register.
 * Controls Invert, Output Change (OCH), Drive type (Open-drain/Totem-pole), and Output Enable (OUTNE).
 * @param dev The device structure.
 * @param output Structure containing output configuration flags.
 * @return Error code.
 */
int16_t pca9685_i2c_output_init(pca9685_dev_t dev, pca9685_output_t output)
{
    uint8_t reg = REG_MODE_2;
    uint8_t data[2];
    uint8_t mode;

    int16_t err = pca9685_i2c_read_mode_2(dev, &mode);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    /* Clear lower 5 bits (0xE0) and apply new configuration */
    /* Bit 4: INVRT, Bit 3: OCH, Bit 2: OUTDRV, Bit 1-0: OUTNE */
    data[1] = (uint8_t)((mode & 0xE0) |
                        ((output.invrt & 0x01) << 4) |
                        ((output.och & 0x01) << 3) |
                        ((output.outdrv & 0x01) << 2) |
                        (output.outne & 0x03));

    return pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
}

/**
 * @brief Sets a specific LED channel to fully ON or fully OFF (no PWM).
 * @param dev The device structure.
 * @param led_no The LED channel number (0-15).
 * @param state State to set (PCA9685_LED_ON or PCA9685_LED_OFF).
 * @return Error code.
 */
int16_t pca9685_i2c_led_set(pca9685_dev_t dev, uint8_t led_no, pca9685_led_state_t state)
{
    if(led_no >= PCA9685_LED_COUNT)
        return PCA9685_ERR;

    /* Bit 4 in LEDn_ON_H sets the LED fully ON, bit 4 in LEDn_OFF_H fully OFF */
    if(state == PCA9685_LED_ON)
        return pca9685_i2c_write_led(dev, pca9685_led_reg(led_no), PCA9685_LED_FULL, 0);
    return pca9685_i2c_write_led(dev, pca9685_led_reg(led_no), 0, PCA9685_LED_FULL);
}

/**
 * @brief Sets ALL LED channels to fully ON or fully OFF simultaneously.
 * @param dev The device structure.
 * @param state State to set (PCA9685_LED_ON or PCA9685_LED_OFF).
 * @return Error code.
 */
int16_t pca9685_i2c_all_led_set(pca9685_dev_t dev, pca9685_led_state_t state)
{
    if(state == PCA9685_LED_ON)
        return pca9685_i2c_write_led(dev, REG_ALL_LED, PCA9685_LED_FULL, 0);
    return pca9685_i2c_write_led(dev, REG_ALL_LED, 0, PCA9685_LED_FULL);
}

/**
 * @brief Sets the PWM duty cycle and delay for a specific LED channel.
 * @param dev The device structure.
 * @param led_no LED channel (0-15).
 * @param d_cycle Duty cycle in percent (0.0 to 100.0).
 * @param delay Phase delay in percent (0.0 to 100.0).
 * @return Error code.
 */
int16_t pca9685_i2c_led_pwm_set(pca9685_dev_t dev, uint8_t led_no, float d_cycle, float delay)
{
    if(led_no >= PCA9685_LED_COUNT)
        return PCA9685_ERR;

    return pca9685_i2c_write_pwm(dev, pca9685_led_reg(led_no), d_cycle, delay);
}

/**
 * @brief Sets the PWM duty cycle and delay for ALL LED channels.
 * @param dev The device structure.
 * @param d_cycle Duty cycle in percent.
 * @param delay Phase delay in percent.
 * @return Error code.
 */
int16_t pca9685_i2c_all_led_pwm_set(pca9685_dev_t dev, float d_cycle, float delay)
{
    return pca9685_i2c_write_pwm(dev, REG_ALL_LED, d_cycle, delay);
}

/**
 * @brief Sets the PWM frequency by writing to the PRE_SCALE register.
 * Note: SLEEP bit in MODE1 must be set to 1 before calling this (oscillator must be off).
 * @param dev The device structure.
 * @param frequency Target frequency in Hz.
 * @param osc_clk_hz Oscillator clock frequency (typically 25000000 for internal).
 * @return Error code.
 */
int16_t pca9685_i2c_write_pre_scale(pca9685_dev_t dev, double frequency, double osc_clk_hz)
{
    if(!(frequency > 0.0) || !(osc_clk_hz > 0.0))
        return PCA9685_ERR;

    /* Formula: prescale = round(osc_clock / (4096 * update_rate)) - 1 */
    double prescale = osc_clk_hz / (PWM_OUTPUT_COUNTER_MAX * frequency) - 1.0 + 0.5;
    if(prescale < PCA9685_PRE_SCALE_MIN || prescale >= PCA9685_PRE_SCALE_MAX + 1)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = REG_PRE_SCALE;
    data[1] = (uint8_t)prescale;
    return pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
}

/**
 * @brief Reads the PRE_SCALE register and calculates the current frequency.
 * @param dev The device structure.
 * @param frequency Pointer to store the calculated frequency.
 * @param osc_clk_hz Oscillator clock frequency.
 * @return Error code.
 */
int16_t pca9685_i2c_read_pre_scale(pca9685_dev_t dev, double *frequency, double osc_clk_hz)
{
    uint8_t reg = REG_PRE_SCALE;
    uint8_t data;
    int16_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1);
    if(err != PCA9685_OK)
        return err;

    /* Reverse calculation of the formula */
    *frequency = (osc_clk_hz) / (PWM_OUTPUT_COUNTER_MAX * ((data) + 1));
    return PCA9685_OK;
}

/**
 * @brief Writes the I2C All Call address.
 * Devices responding to this address can be controlled simultaneously.
 * @param dev The device structure.
 * @param allcall_addr The 7-bit All Call address.
 * @return Error code.
 */
int16_t pca9685_i2c_write_allcall_addr(pca9685_dev_t dev, uint8_t allcall_addr)
{
    uint8_t reg = REG_ALLCALLADR;
    uint8_t data[2];
    data[0] = reg;
    /* Shift left by 1 for I2C write format */
    data[1] = (uint8_t)(allcall_addr << 1);

    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
}

/**
 * @brief Reads the programmed I2C All Call address.
 * @param dev The device structure.
 * @param allcall_addr Pointer to store the address.
 * @return Error code.
 */
int16_t pca9685_i2c_read_allcall_addr(pca9685_dev_t dev, uint8_t *allcall_addr)
{
    uint8_t reg = REG_ALLCALLADR;
    uint8_t data;
    int16_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1);
    if(err != PCA9685_OK)
        return err;

    *allcall_addr = data >> 1;
    return PCA9685_OK;
}

/**
 * @brief Writes one of the three I2C Sub-Addresses.
 * @param dev The device structure.
 * @param addr_no Which sub-address to write (1, 2, or 3).
 * @param sub_addr The 7-bit sub-address.
 * @return Error code.
 */
int16_t pca9685_i2c_write_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t sub_addr)
{
    if(addr_no < PCA9685_SUB_ADDR_1 || addr_no > PCA9685_SUB_ADDR_3)
        return PCA9685_ERR;

    uint8_t reg = (uint8_t)(addr_no + SUBADR_OFFSET_ADR);
    uint8_t data[2];
    data[0] = reg;
    data[1] = (uint8_t)(sub_addr << 1);

    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
}

/**
 * @brief Reads one of the three I2C Sub-Addresses.
 * @param dev The device structure.
 * @param addr_no Which sub-address to read (1, 2, or 3).
 * @param sub_addr Pointer to store the address.
 * @return Error code.
 */
int16_t pca9685_i2c_read_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t *sub_addr)
{
    if(addr_no < PCA9685_SUB_ADDR_1 || addr_no > PCA9685_SUB_ADDR_3)
        return PCA9685_ERR;

    uint8_t reg = (uint8_t)(addr_no + SUBADR_OFFSET_ADR);
    uint8_t data;
    int16_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1);
    if(err != PCA9685_OK)
        return err;

    *sub_addr = data >> 1;
    return PCA9685_OK;
}

/**
 * @brief Enables or disables the device response to a specific Sub-Address in MODE1.
 * @param dev The device structure.
 * @param sub_addr The sub-address number (1, 2, or 3).
 * @param resp Enable (1) or Disable (0) response.
 * @return Error code.
 */
int16_t pca9685_i2c_sub_addr_resp(pca9685_dev_t dev, pca9685_subaddr_no_t sub_addr, pca9685_addr_resp_t resp)
{
    if(sub_addr < PCA9685_SUB_ADDR_1 || sub_addr > PCA9685_SUB_ADDR_3)
        return PCA9685_ERR;

    /* MODE1 bits: Bit 3 = SUB1, Bit 2 = SUB2, Bit 1 = SUB3 */
    uint8_t bit = (uint8_t)(1 << (4 - sub_addr));
    return pca9685_i2c_update_mode_1(dev, bit, resp == PCA9685_ADDR_RESPOND ? bit : 0);
}

/**
 * @brief Enables or disables the device response to the All Call Address in MODE1.
 * @param dev The device structure.
 * @param resp Enable (1) or Disable (0) response.
 * @return Error code.
 */
int16_t pca9685_i2c_allcall_address_resp(pca9685_dev_t dev, pca9685_addr_resp_t resp)
{
    /* Modify bit 0 (ALLCALL) of MODE1 */
    return pca9685_i2c_update_mode_1(dev, PCA9685_MODE1_ALLCALL,
                                     resp == PCA9685_ADDR_RESPOND ? PCA9685_MODE1_ALLCALL : 0);
}

/**
 * @brief Sets a channel's pulse width in microseconds.
 * @param dev The device structure.
 * @param led_no Channel (0-15).
 * @param pulse_us Pulse width in microseconds.
 * @param pwm_freq_hz PWM frequency in Hz.
 * @return Error code.
 */
int16_t pca9685_i2c_servo_pulse_set(pca9685_dev_t dev, uint8_t led_no, float pulse_us, float pwm_freq_hz)
{
    if(!(pwm_freq_hz > 0.0f) || !(pulse_us >= 0.0f))
        return PCA9685_ERR;

    /* duty [%] = pulse / period * 100 = pulse_us * f / 10^6 * 100 */
    float d_cycle = pulse_us * pwm_freq_hz / 10000.0f;
    return pca9685_i2c_led_pwm_set(dev, led_no, d_cycle, 0.0f);
}

/**
 * @brief Initializes the PCA9685 device structure and programs the addresses to the chip.
 * @param dev Pointer to the device structure.
 * @param _i2c_addr The hardware I2C address of the chip.
 * @param _allcall_addr The All Call address to be used.
 * @param _sub_addr_1 Sub-address 1.
 * @param _sub_addr_2 Sub-address 2.
 * @param _sub_addr_3 Sub-address 3.
 * @return Error code.
 */
int16_t pca9685_i2c_register(pca9685_dev_t *dev,
                             uint8_t _i2c_addr,
                             uint8_t _allcall_addr,
                             uint8_t _sub_addr_1,
                             uint8_t _sub_addr_2,
                             uint8_t _sub_addr_3){

    dev->i2c_addr = _i2c_addr;
    dev->allcall_addr = _allcall_addr;
    dev->sub_addr_1 = _sub_addr_1;
    dev->sub_addr_2 = _sub_addr_2;
    dev->sub_addr_3 = _sub_addr_3;

    /* Program the configuration to the device */
    if(pca9685_i2c_write_allcall_addr(*dev, dev->allcall_addr) != PCA9685_OK ||
       pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_1, dev->sub_addr_1) != PCA9685_OK ||
       pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_2, dev->sub_addr_2) != PCA9685_OK ||
       pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_3, dev->sub_addr_3) != PCA9685_OK)
        return PCA9685_ERR;

    return PCA9685_OK;
}
