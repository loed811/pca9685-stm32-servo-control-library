#include "math.h"

#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

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
 * @param dev The device structure.
 * @param clk Clock setting (Auto or External).
 * @return Error code.
 */
int16_t pca9685_i2c_clock(pca9685_dev_t dev, pca9685_auto_extclk_t clk)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;

    /* Read current mode configuration */
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = reg;
    /* Mask out bit 6 (EXTCLK) then set it based on the 'clk' parameter */
    /* 0xBF = 1011 1111 (Clears bit 6) */
    data[1] = (mode & 0xBF) | (clk << 6);

    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
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
    uint8_t reg = REG_MODE_1;
    uint8_t mode;

    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = reg;
    /* Mask out bit 5 (AI) then set it based on 'setting' */
    /* 0xDF = 1101 1111 (Clears bit 5) */
    data[1] = (mode & 0xDF) | (setting << 5);

    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
}

/**
 * @brief Performs the restart sequence required to wake the device from sleep
 * and resume PWM generation.
 * @param dev The device structure.
 * @return Error code.
 */
int16_t pca9685_i2c_restart(pca9685_dev_t dev)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;

    /* Read MODE1. Check if RESTART bit (bit 7) is set, implying a restart is pending/possible. */
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK && !(mode & (1 << 7)))
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = reg;
    /* Clear the SLEEP bit (bit 4) to wake up the device */
    data[1] = mode | (mode & ~(1 << 4));
    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);

    /* Wait for oscillator to stabilize (typically 500us) */
    pca9685_i2c_hal_ms_delay(STAB_TIME);

    /* Check MODE1 again */
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    /* Write logic 1 to bit 7 to restart PWM channels */
    data[1] = mode | (mode & (1 << 7));
    err += pca9685_i2c_hal_write(dev.i2c_addr, data, 2);

    return err;
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
    uint8_t reg = REG_MODE_1;
    uint8_t data[2];
    uint8_t mode;

    int16_t err = pca9685_i2c_read_mode_1(dev, &mode);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    /* Mask bit 4 (SLEEP) and set new state */
    /* 0xEF = 1110 1111 */
    data[1] = (mode & 0xEF) | (sleep_mode << 4);

    err += pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    /* Allow time for stabilization if waking up */
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    return err;
}

/**
 * @brief Sends a Software Reset (SWRST) command via I2C General Call.
 * This resets the PCA9685 to its power-up state.
 * @return Error code.
 */
int16_t pca9685_i2c_reset()
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
    data[1] = (mode & 0xE0) | (output.invrt << 4) | (output.och << 3) | (output.outdrv << 2) | (output.outne << 1);

    err += pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
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
    /* Calculate register address offset for the specific LED */
    uint8_t reg = (led_no * 4) + LED_OFFSET_ADR;
    uint8_t data[5];
    data[0] = reg;

    /* Bit 4 in LEDn_ON_H sets the LED fully ON */
    data[2] = 1 << 4;

    /* If requesting OFF, set Bit 4 in LEDn_OFF_H instead */
    if (state == PCA9685_LED_OFF)
        data[4] = 1 << 4;

    /* Note: data[1] (ON_L) and data[3] (OFF_L) are effectively 0 here, which is fine for full ON/OFF */
    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 5);
    return err;
}

/**
 * @brief Sets ALL LED channels to fully ON or fully OFF simultaneously.
 * @param dev The device structure.
 * @param state State to set (PCA9685_LED_ON or PCA9685_LED_OFF).
 * @return Error code.
 */
int16_t pca9685_i2c_all_led_set(pca9685_dev_t dev, pca9685_led_state_t state)
{
    uint8_t reg = REG_ALL_LED;
    uint8_t data[5];
    data[0] = reg;
    /* Set Full ON bit in ALL_LED_ON_H */
    data[2] = 1 << 4;

    /* If requesting OFF, set Full OFF bit in ALL_LED_OFF_H */
    if (state == PCA9685_LED_OFF)
        data[4] = 1 << 4;

    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 5);
    return err;
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
    uint8_t reg = (led_no * 4) + LED_OFFSET_ADR;
    uint8_t data[5];

    /* Convert percentage to 12-bit counter value (0-4096) */
    uint16_t delay_tm = round(delay * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_on_tm = round(d_cycle * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_off_tm = delay_tm + led_on_tm;

    if(delay_tm == 0)
        delay_tm = 1;

    data[0] = reg;
    /* LED_ON_L */
    data[1] = (delay_tm - 1) & 0xFF;
    /* LED_ON_H */
    data[2] = (delay_tm - 1) >> 8;

    /* LED_OFF calculation. Handles wrapping if OFF time > 4096 */
    /* LED_OFF_L */
    data[3] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
    /* LED_OFF_H */
    data[4] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 5);
    return err;
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
    uint8_t reg = REG_ALL_LED;
    uint8_t data[5];
    uint16_t delay_tm = round(delay * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_on_tm = round(d_cycle * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_off_tm = delay_tm + led_on_tm;

    /* The LEDn_ON and LEDn_OFF count registers should never be programmed with the same values */
    if (led_on_tm == led_off_tm)
        return PCA9685_ERR;

    data[0] = reg;
    /* ALL_LED_ON_L */
    data[1] = (delay_tm - 1) & 0xFF;
    /* ALL_LED_ON_H */
    data[2] = (delay_tm - 1) >> 8;
    /* ALL_LED_OFF_L (with wrap-around logic) */
    data[3] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
    /* ALL_LED_OFF_H (with wrap-around logic) */
    data[4] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 5);
    return err;
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
    uint8_t reg = REG_PRE_SCALE;
    uint8_t data[2];
    data[0] = reg;
    /* Formula: prescale = round(osc_clock / (4096 * update_rate)) - 1 */
    data[1] = round(osc_clk_hz / (PWM_OUTPUT_COUNTER_MAX * frequency)) - 1;
    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
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
    /* Reverse calculation of the formula */
    *frequency = (osc_clk_hz) / (PWM_OUTPUT_COUNTER_MAX * ((data) + 1));
    return err;
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
    data[1] = allcall_addr << 1;

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
    *allcall_addr = data >> 1;

    return err;
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
    uint8_t reg = addr_no + SUBADR_OFFSET_ADR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = sub_addr << 1;

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
    uint8_t reg = addr_no + SUBADR_OFFSET_ADR;
    uint8_t data;
    int16_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1);
    *sub_addr = data >> 1;

    return err;
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
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    /* Adjustment logic: Map enum values to correct MODE1 register bit positions */
    /* MODE1 bits: Bit 3 = SUB1, Bit 2 = SUB2, Bit 1 = SUB3 */
    if(sub_addr == 1)
        sub_addr = 3;
    else if(sub_addr == 3)
        sub_addr = 1;

    uint8_t data[2];
    data[0] = reg;
    /* Modify the specific bit corresponding to the sub-address */
    data[1] = (mode & ~(1 << sub_addr)) | (resp << sub_addr);
    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
}

/**
 * @brief Enables or disables the device response to the All Call Address in MODE1.
 * @param dev The device structure.
 * @param resp Enable (1) or Disable (0) response.
 * @return Error code.
 */
int16_t pca9685_i2c_allcall_address_resp(pca9685_dev_t dev, pca9685_addr_resp_t resp)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    /* Modify bit 0 (ALLCALL) of MODE1 */
    data[1] = (mode & ~(1 << 0)) | resp;
    int16_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, 2);
    return err;
}

/**
 * @brief Initializes the PCA9685 device structure and programs the addresses to the chip.
 * @param dev Pointer to the device structure.
 * @param _i2c_addr The hardware I2C address of the chip.
 * @param _allcall_addr The All Call address to be used.
 * @param _sub_addr_1 Sub-address 1.
 * @param _sub_addr_2 Sub-address 2.
 * @param _sub_addr_3 Sub-address 3.
 */
void pca9685_i2c_register(pca9685_dev_t *dev,
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
    pca9685_i2c_write_allcall_addr(*dev, dev->allcall_addr);
    pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_1, dev->sub_addr_1);
    pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_2, dev->sub_addr_2);
    pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_3, dev->sub_addr_3);
}
