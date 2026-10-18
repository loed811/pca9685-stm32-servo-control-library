#include <stddef.h>

#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

#define PCA9685_COUNTER_MASK    0x0FFF  /* 12-bit counter values 0..4095 */
#define PCA9685_ADDR_7BIT_MAX   0x7F

/**
 * @brief Writes a buffer and maps any platform error to PCA9685_ERR.
 */
static int16_t write_buf(pca9685_dev_t dev, uint8_t *data, uint16_t count)
{
    return pca9685_i2c_hal_write(dev.i2c_addr, data, count) == PCA9685_OK
           ? PCA9685_OK : PCA9685_ERR;
}

/**
 * @brief Reads one register and maps any platform error to PCA9685_ERR.
 * @p value is only written when the read succeeds.
 */
static int16_t read_reg(pca9685_dev_t dev, uint8_t reg, uint8_t *value)
{
    uint8_t data;

    if (value == NULL)
        return PCA9685_ERR;
    if (pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1) != PCA9685_OK)
        return PCA9685_ERR;

    *value = data;
    return PCA9685_OK;
}

/**
 * @brief Read-modify-write of MODE1: clears @p clear_mask, then sets @p set_mask.
 *
 * The RESTART bit (bit 7) is always written as 0 here. Writing 0 to it has no
 * effect, while writing 1 restarts the PWM channels and clears the bit. A bit
 * read back as 1 must therefore not be copied back, or the pending restart
 * fires at the wrong moment (e.g. before the oscillator is stable) and is lost.
 * Only pca9685_i2c_restart() writes it as 1.
 */
static int16_t mode1_update(pca9685_dev_t dev, uint8_t clear_mask, uint8_t set_mask)
{
    uint8_t mode;

    if (read_reg(dev, REG_MODE_1, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = REG_MODE_1;
    data[1] = (uint8_t)(((mode & (uint8_t)~clear_mask) | set_mask) & (uint8_t)~PCA9685_MODE1_RESTART);
    return write_buf(dev, data, 2);
}

/**
 * @brief Writes the 4 ON/OFF registers starting at @p reg in one transfer.
 * @p on and @p off are 13-bit values: 12-bit count plus PCA9685_LED_FULL.
 */
static int16_t write_led_regs(pca9685_dev_t dev, uint8_t reg, uint16_t on, uint16_t off)
{
    uint8_t data[5];

    data[0] = reg;
    data[1] = (uint8_t)(on & 0xFF);     /* LEDn_ON_L  */
    data[2] = (uint8_t)(on >> 8);       /* LEDn_ON_H  */
    data[3] = (uint8_t)(off & 0xFF);    /* LEDn_OFF_L */
    data[4] = (uint8_t)(off >> 8);      /* LEDn_OFF_H */
    return write_buf(dev, data, 5);
}

/**
 * @brief Converts a percentage of the PWM period to counter ticks (0..4096).
 * Negative values and NaN give 0; 100 % and above give 4096.
 */
static uint16_t percent_to_ticks(float percent)
{
    /* Written as !(x > 0) so that NaN also takes this branch. */
    if (!(percent > 0.0f))
        return 0;
    if (percent >= 100.0f)
        return PWM_OUTPUT_COUNTER_MAX;

    return (uint16_t)(percent * (PWM_OUTPUT_COUNTER_MAX / 100.0f) + 0.5f);
}

/**
 * @brief Computes the ON/OFF register values for a duty cycle and phase delay.
 *
 * The output goes high when the counter reaches ON and low when it reaches
 * OFF, so the high time is (OFF - ON) mod 4096 counts. ON and OFF must never
 * be equal (datasheet), so 0 % and 100 % use the full OFF / full ON bits.
 */
static void pwm_to_counts(float d_cycle, float delay, uint16_t *on, uint16_t *off)
{
    uint16_t high = percent_to_ticks(d_cycle);

    if (high == 0) {
        *on = 0;
        *off = PCA9685_LED_FULL;    /* full OFF */
        return;
    }
    if (high >= PWM_OUTPUT_COUNTER_MAX) {
        *on = PCA9685_LED_FULL;     /* full ON */
        *off = 0;
        return;
    }

    /* A 100 % delay is the same moment as 0 %. */
    uint16_t start = (uint16_t)(percent_to_ticks(delay) & PCA9685_COUNTER_MASK);

    *on = start;
    /* OFF < ON means the pulse wraps into the next period; the chip supports it. */
    *off = (uint16_t)((start + high) & PCA9685_COUNTER_MASK);
}

/* -------------------------------------------------------------------------- */
/* Mode registers                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Reads the content of the MODE1 register.
 * @param dev The device structure containing the I2C address.
 * @param mode Pointer to store the read value (unchanged on failure).
 * @return Error code (PCA9685_OK or PCA9685_ERR).
 */
int16_t pca9685_i2c_read_mode_1(pca9685_dev_t dev, uint8_t *mode)
{
    return read_reg(dev, REG_MODE_1, mode);
}

/**
 * @brief Reads the content of the MODE2 register.
 * @param dev The device structure.
 * @param mode Pointer to store the read value (unchanged on failure).
 * @return Error code.
 */
int16_t pca9685_i2c_read_mode_2(pca9685_dev_t dev, uint8_t *mode)
{
    return read_reg(dev, REG_MODE_2, mode);
}

/**
 * @brief Configures the EXTCLK bit in the MODE1 register.
 *
 * Switching to the external clock follows the datasheet sequence: set SLEEP
 * first, then write SLEEP and EXTCLK together. The device stays asleep.
 * EXTCLK can only be cleared by a power cycle or a software reset, so
 * selecting the internal clock afterwards has no effect on the chip.
 * @param dev The device structure.
 * @param clk Clock setting (Internal or External).
 * @return Error code.
 */
int16_t pca9685_i2c_clock(pca9685_dev_t dev, pca9685_auto_extclk_t clk)
{
    if (clk == PCA9685_CLK_EXTERNAL) {
        if (mode1_update(dev, 0, PCA9685_MODE1_SLEEP) != PCA9685_OK)
            return PCA9685_ERR;
        return mode1_update(dev, 0, PCA9685_MODE1_SLEEP | PCA9685_MODE1_EXTCLK);
    }
    if (clk == PCA9685_CLK_INTERNAL)
        return mode1_update(dev, PCA9685_MODE1_EXTCLK, 0);

    return PCA9685_ERR;
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
    if (setting == PCA9685_AUTOINCR_ON)
        return mode1_update(dev, 0, PCA9685_MODE1_AI);
    if (setting == PCA9685_AUTOINCR_OFF)
        return mode1_update(dev, PCA9685_MODE1_AI, 0);

    return PCA9685_ERR;
}

/**
 * @brief Performs the restart sequence required to wake the device from sleep
 * and resume PWM generation (datasheet section 7.3.1.1).
 * @param dev The device structure.
 * @return Error code.
 */
int16_t pca9685_i2c_restart(pca9685_dev_t dev)
{
    uint8_t mode;

    if (read_reg(dev, REG_MODE_1, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = REG_MODE_1;

    /* 1. Clear SLEEP. RESTART is written as 0 so a pending restart is kept. */
    data[1] = (uint8_t)(mode & (uint8_t)~(PCA9685_MODE1_SLEEP | PCA9685_MODE1_RESTART));
    if (write_buf(dev, data, 2) != PCA9685_OK)
        return PCA9685_ERR;

    /* 2. Wait for the oscillator to stabilise (500 us max). */
    pca9685_i2c_hal_ms_delay(STAB_TIME);

    /* 3. Write 1 to RESTART: PWM channels resume and the bit clears.
     *    If no restart was pending this write has no effect. */
    data[1] = (uint8_t)((mode & (uint8_t)~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART);
    return write_buf(dev, data, 2);
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
    if (sleep_mode == PCA9685_MODE_SLEEP)
        return mode1_update(dev, 0, PCA9685_MODE1_SLEEP);

    if (sleep_mode != PCA9685_MODE_NORMAL)
        return PCA9685_ERR;

    if (mode1_update(dev, PCA9685_MODE1_SLEEP, 0) != PCA9685_OK)
        return PCA9685_ERR;

    /* Allow the oscillator to stabilise after waking up. */
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    return PCA9685_OK;
}

/**
 * @brief Sends a Software Reset (SWRST) command via I2C General Call.
 * This resets every PCA9685 on the bus to its power-up state.
 * @return Error code.
 */
int16_t pca9685_i2c_reset(void)
{
    uint8_t data = SWRST;

    /* Write SWRST byte to the I2C General Call Address (0x00) */
    int16_t err = pca9685_i2c_hal_write(I2C_GEN_CALL_ADDRESS_PCA9685, &data, 1);
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    return err == PCA9685_OK ? PCA9685_OK : PCA9685_ERR;
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
    uint8_t mode;

    if (read_reg(dev, REG_MODE_2, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = REG_MODE_2;
    /* Keep the reserved upper bits, replace bits 4..0. */
    data[1] = (uint8_t)(mode & 0xE0);
    if (output.invrt == PCA9685_OUTPUT_INVERT)
        data[1] |= PCA9685_MODE2_INVRT;
    if (output.och == PCA9685_CH_ONACK)
        data[1] |= PCA9685_MODE2_OCH;
    if (output.outdrv == PCA9685_OUTPUT_TOTEM_POLE)
        data[1] |= PCA9685_MODE2_OUTDRV;
    data[1] |= (uint8_t)((uint8_t)output.outne & PCA9685_MODE2_OUTNE_MASK);

    return write_buf(dev, data, 2);
}

/* -------------------------------------------------------------------------- */
/* Outputs                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Sets a specific LED channel to fully ON or fully OFF (no PWM).
 * @param dev The device structure.
 * @param led_no The LED channel number (0-15).
 * @param state State to set (PCA9685_LED_ON or PCA9685_LED_OFF).
 * @return Error code.
 */
int16_t pca9685_i2c_led_set(pca9685_dev_t dev, uint8_t led_no, pca9685_led_state_t state)
{
    if (led_no > 15)
        return PCA9685_ERR;

    uint8_t reg = (uint8_t)(led_no * 4 + LED_OFFSET_ADR);

    if (state == PCA9685_LED_ON)
        return write_led_regs(dev, reg, PCA9685_LED_FULL, 0);
    if (state == PCA9685_LED_OFF)
        return write_led_regs(dev, reg, 0, PCA9685_LED_FULL);

    return PCA9685_ERR;
}

/**
 * @brief Sets ALL LED channels to fully ON or fully OFF simultaneously.
 * @param dev The device structure.
 * @param state State to set (PCA9685_LED_ON or PCA9685_LED_OFF).
 * @return Error code.
 */
int16_t pca9685_i2c_all_led_set(pca9685_dev_t dev, pca9685_led_state_t state)
{
    if (state == PCA9685_LED_ON)
        return write_led_regs(dev, REG_ALL_LED, PCA9685_LED_FULL, 0);
    if (state == PCA9685_LED_OFF)
        return write_led_regs(dev, REG_ALL_LED, 0, PCA9685_LED_FULL);

    return PCA9685_ERR;
}

/**
 * @brief Sets the PWM duty cycle and delay for a specific LED channel.
 * @param dev The device structure.
 * @param led_no LED channel (0-15).
 * @param d_cycle Duty cycle in percent (clamped to 0.0 .. 100.0).
 * @param delay Phase delay in percent (clamped to 0.0 .. 100.0).
 * @return Error code.
 */
int16_t pca9685_i2c_led_pwm_set(pca9685_dev_t dev, uint8_t led_no, float d_cycle, float delay)
{
    uint16_t on, off;

    if (led_no > 15)
        return PCA9685_ERR;

    pwm_to_counts(d_cycle, delay, &on, &off);
    return write_led_regs(dev, (uint8_t)(led_no * 4 + LED_OFFSET_ADR), on, off);
}

/**
 * @brief Sets the PWM duty cycle and delay for ALL LED channels.
 * @param dev The device structure.
 * @param d_cycle Duty cycle in percent (clamped to 0.0 .. 100.0).
 * @param delay Phase delay in percent (clamped to 0.0 .. 100.0).
 * @return Error code.
 */
int16_t pca9685_i2c_all_led_pwm_set(pca9685_dev_t dev, float d_cycle, float delay)
{
    uint16_t on, off;

    pwm_to_counts(d_cycle, delay, &on, &off);
    return write_led_regs(dev, REG_ALL_LED, on, off);
}

/* -------------------------------------------------------------------------- */
/* Frequency                                                                  */
/* -------------------------------------------------------------------------- */

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
    /* Written as !(x > 0) so that NaN is rejected too. */
    if (!(frequency > 0.0) || !(osc_clk_hz > 0.0))
        return PCA9685_ERR;

    /* Formula: prescale = round(osc_clock / (4096 * update_rate)) - 1 */
    double pre_scale = osc_clk_hz / ((double)PWM_OUTPUT_COUNTER_MAX * frequency) - 1.0;

    uint8_t data[2];
    data[0] = REG_PRE_SCALE;
    /* PRE_SCALE hardware range is 3..255. Clamp before converting to an integer. */
    if (pre_scale < 3.0)
        data[1] = 3;
    else if (pre_scale > 255.0)
        data[1] = 255;
    else
        data[1] = (uint8_t)(pre_scale + 0.5);

    return write_buf(dev, data, 2);
}

/**
 * @brief Reads the PRE_SCALE register and calculates the current frequency.
 * @param dev The device structure.
 * @param frequency Pointer to store the calculated frequency (unchanged on failure).
 * @param osc_clk_hz Oscillator clock frequency.
 * @return Error code.
 */
int16_t pca9685_i2c_read_pre_scale(pca9685_dev_t dev, double *frequency, double osc_clk_hz)
{
    uint8_t data;

    if (frequency == NULL)
        return PCA9685_ERR;
    if (read_reg(dev, REG_PRE_SCALE, &data) != PCA9685_OK)
        return PCA9685_ERR;

    /* Reverse calculation of the formula */
    *frequency = osc_clk_hz / ((double)PWM_OUTPUT_COUNTER_MAX * ((double)data + 1.0));
    return PCA9685_OK;
}

/* -------------------------------------------------------------------------- */
/* Addressing                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Writes the I2C All Call address.
 * Devices responding to this address can be controlled simultaneously.
 * @param dev The device structure.
 * @param allcall_addr The 7-bit All Call address.
 * @return Error code.
 */
int16_t pca9685_i2c_write_allcall_addr(pca9685_dev_t dev, uint8_t allcall_addr)
{
    if (allcall_addr > PCA9685_ADDR_7BIT_MAX)
        return PCA9685_ERR;

    uint8_t data[2];
    data[0] = REG_ALLCALLADR;
    /* The register holds the address in bits 7..1. */
    data[1] = (uint8_t)(allcall_addr << 1);
    return write_buf(dev, data, 2);
}

/**
 * @brief Reads the programmed I2C All Call address.
 * @param dev The device structure.
 * @param allcall_addr Pointer to store the address (unchanged on failure).
 * @return Error code.
 */
int16_t pca9685_i2c_read_allcall_addr(pca9685_dev_t dev, uint8_t *allcall_addr)
{
    uint8_t data;

    if (allcall_addr == NULL)
        return PCA9685_ERR;
    if (read_reg(dev, REG_ALLCALLADR, &data) != PCA9685_OK)
        return PCA9685_ERR;

    *allcall_addr = (uint8_t)(data >> 1);
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
    if (addr_no < PCA9685_SUB_ADDR_1 || addr_no > PCA9685_SUB_ADDR_3)
        return PCA9685_ERR;
    if (sub_addr > PCA9685_ADDR_7BIT_MAX)
        return PCA9685_ERR;

    uint8_t data[2];
    /* SUBADR1..3 are registers 0x02..0x04 */
    data[0] = (uint8_t)(addr_no + SUBADR_OFFSET_ADR);
    data[1] = (uint8_t)(sub_addr << 1);
    return write_buf(dev, data, 2);
}

/**
 * @brief Reads one of the three I2C Sub-Addresses.
 * @param dev The device structure.
 * @param addr_no Which sub-address to read (1, 2, or 3).
 * @param sub_addr Pointer to store the address (unchanged on failure).
 * @return Error code.
 */
int16_t pca9685_i2c_read_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t *sub_addr)
{
    uint8_t data;

    if (addr_no < PCA9685_SUB_ADDR_1 || addr_no > PCA9685_SUB_ADDR_3)
        return PCA9685_ERR;
    if (sub_addr == NULL)
        return PCA9685_ERR;
    if (read_reg(dev, (uint8_t)(addr_no + SUBADR_OFFSET_ADR), &data) != PCA9685_OK)
        return PCA9685_ERR;

    *sub_addr = (uint8_t)(data >> 1);
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
    uint8_t bit;

    /* MODE1 bits: Bit 3 = SUB1, Bit 2 = SUB2, Bit 1 = SUB3 */
    switch (sub_addr) {
    case PCA9685_SUB_ADDR_1: bit = PCA9685_MODE1_SUB1; break;
    case PCA9685_SUB_ADDR_2: bit = PCA9685_MODE1_SUB2; break;
    case PCA9685_SUB_ADDR_3: bit = PCA9685_MODE1_SUB3; break;
    default: return PCA9685_ERR;
    }

    if (resp == PCA9685_ADDR_RESPOND)
        return mode1_update(dev, 0, bit);
    if (resp == PCA9685_ADDR_NORESPOND)
        return mode1_update(dev, bit, 0);

    return PCA9685_ERR;
}

/**
 * @brief Enables or disables the device response to the All Call Address in MODE1.
 * @param dev The device structure.
 * @param resp Enable (1) or Disable (0) response.
 * @return Error code.
 */
int16_t pca9685_i2c_allcall_address_resp(pca9685_dev_t dev, pca9685_addr_resp_t resp)
{
    if (resp == PCA9685_ADDR_RESPOND)
        return mode1_update(dev, 0, PCA9685_MODE1_ALLCALL);
    if (resp == PCA9685_ADDR_NORESPOND)
        return mode1_update(dev, PCA9685_MODE1_ALLCALL, 0);

    return PCA9685_ERR;
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
                             uint8_t _sub_addr_3)
{
    int16_t err = PCA9685_OK;

    if (dev == NULL)
        return PCA9685_ERR;

    dev->i2c_addr = _i2c_addr;
    dev->allcall_addr = _allcall_addr;
    dev->sub_addr_1 = _sub_addr_1;
    dev->sub_addr_2 = _sub_addr_2;
    dev->sub_addr_3 = _sub_addr_3;

    if (_i2c_addr > PCA9685_ADDR_7BIT_MAX)
        return PCA9685_ERR;

    /* Program the configuration to the device. Keep going after a failure so
     * one bad value does not leave the other registers unprogrammed. */
    if (pca9685_i2c_write_allcall_addr(*dev, dev->allcall_addr) != PCA9685_OK)
        err = PCA9685_ERR;
    if (pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_1, dev->sub_addr_1) != PCA9685_OK)
        err = PCA9685_ERR;
    if (pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_2, dev->sub_addr_2) != PCA9685_OK)
        err = PCA9685_ERR;
    if (pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_3, dev->sub_addr_3) != PCA9685_OK)
        err = PCA9685_ERR;

    return err;
}
