#include "pca9685_sim.h"

#include <string.h>

#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

#define REG_SUBADR_1        0x02
#define REG_SUBADR_2        0x03
#define REG_SUBADR_3        0x04
#define REG_LED_LAST        0x45 /* LED15_OFF_H */
#define REG_ALL_LED_LAST    0xFD /* ALL_LED_OFF_H */
#define MODE2_WRITABLE      0x1F /* Bits 7-5 are reserved */

pca9685_sim_t pca9685_sim;

/* Register values after power-up or Software Reset (datasheet table 4) */
static void sim_load_defaults(void)
{
    uint8_t *regs = pca9685_sim.regs;

    memset(regs, 0, sizeof(pca9685_sim.regs));
    regs[REG_MODE_1] = PCA9685_MODE1_SLEEP | PCA9685_MODE1_ALLCALL;
    regs[REG_MODE_2] = 0x04;
    regs[REG_SUBADR_1] = I2C_SUB_ADDRESS_1_PCA9685 << 1;
    regs[REG_SUBADR_2] = I2C_SUB_ADDRESS_2_PCA9685 << 1;
    regs[REG_SUBADR_3] = I2C_SUB_ADDRESS_3_PCA9685 << 1;
    regs[REG_ALLCALLADR] = I2C_ALL_CALL_ADDRESS_PCA9685 << 1;
    for (uint8_t led = 0; led < PCA9685_LED_COUNT; led++)
        regs[LED_OFFSET_ADR + led * 4 + 3] = 0x10; /* LEDn_OFF_H: full OFF */
    regs[REG_PRE_SCALE] = 0x1E; /* 200 Hz */
}

void pca9685_sim_power_on(uint8_t address)
{
    memset(&pca9685_sim, 0, sizeof(pca9685_sim));
    pca9685_sim.address = address;
    pca9685_sim.fail_after = -1;
    sim_load_defaults();
}

void pca9685_sim_fail_after(int32_t n)
{
    pca9685_sim.fail_after = n;
}

/* Consumes one transaction from the failure budget, returns 0 if the bus fails */
static int sim_bus_ok(void)
{
    if (pca9685_sim.fail_after < 0)
        return 1;
    if (pca9685_sim.fail_after == 0)
        return 0;
    pca9685_sim.fail_after--;
    return 1;
}

/* Returns non-zero if the device acknowledges the 7-bit address */
static int sim_acks(uint8_t address)
{
    const uint8_t *regs = pca9685_sim.regs;
    uint8_t mode1 = regs[REG_MODE_1];

    if (address == pca9685_sim.address)
        return 1;
    if ((mode1 & PCA9685_MODE1_ALLCALL) && address == (regs[REG_ALLCALLADR] >> 1))
        return 1;
    if ((mode1 & PCA9685_MODE1_SUB1) && address == (regs[REG_SUBADR_1] >> 1))
        return 1;
    if ((mode1 & PCA9685_MODE1_SUB2) && address == (regs[REG_SUBADR_2] >> 1))
        return 1;
    if ((mode1 & PCA9685_MODE1_SUB3) && address == (regs[REG_SUBADR_3] >> 1))
        return 1;
    return 0;
}

/* Control register value after a byte is transferred */
static uint8_t sim_next_reg(uint8_t reg)
{
    if (!(pca9685_sim.regs[REG_MODE_1] & PCA9685_MODE1_AI))
        return reg;
    if (reg == REG_LED_LAST)
        return REG_MODE_1;
    return (uint8_t)(reg + 1);
}

static void sim_write_mode_1(uint8_t value)
{
    uint8_t old = pca9685_sim.regs[REG_MODE_1];
    uint8_t restart = old & PCA9685_MODE1_RESTART;

    /* Writing 1 clears RESTART and restarts the PWM channels, writing 0 has no effect */
    if (value & PCA9685_MODE1_RESTART) {
        if (restart)
            pca9685_sim.restart_count++;
        restart = 0;
    }

    /* Going to sleep while running flags the PWM channels for restart */
    if (!(old & PCA9685_MODE1_SLEEP) && (value & PCA9685_MODE1_SLEEP))
        restart = PCA9685_MODE1_RESTART;

    /* EXTCLK is sticky: only a reset clears it */
    pca9685_sim.regs[REG_MODE_1] = (uint8_t)((value & ~PCA9685_MODE1_RESTART) |
                                             (old & PCA9685_MODE1_EXTCLK) | restart);
}

static void sim_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t *regs = pca9685_sim.regs;

    if (reg == REG_MODE_1) {
        sim_write_mode_1(value);
    } else if (reg == REG_MODE_2) {
        regs[REG_MODE_2] = value & MODE2_WRITABLE;
    } else if (reg <= REG_LED_LAST) {
        regs[reg] = value;
    } else if (reg >= REG_ALL_LED && reg <= REG_ALL_LED_LAST) {
        for (uint8_t led = 0; led < PCA9685_LED_COUNT; led++)
            regs[LED_OFFSET_ADR + led * 4 + (reg - REG_ALL_LED)] = value;
    } else if (reg == REG_PRE_SCALE) {
        /* PRE_SCALE can only be written while the oscillator is off */
        if (regs[REG_MODE_1] & PCA9685_MODE1_SLEEP)
            regs[REG_PRE_SCALE] = value < PCA9685_PRE_SCALE_MIN ? PCA9685_PRE_SCALE_MIN : value;
    }
    /* Reserved registers and TestMode are ignored */
}

static uint8_t sim_read_reg(uint8_t reg)
{
    if (reg <= REG_LED_LAST || reg == REG_PRE_SCALE)
        return pca9685_sim.regs[reg];
    /* ALL_LED registers read back as zero */
    return 0;
}

int16_t pca9685_i2c_hal_init(void)
{
    return PCA9685_OK;
}

/* Signature is fixed by pca9685_i2c_hal.h */
// cppcheck-suppress constParameterPointer
int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)
{
    pca9685_sim.read_count++;
    if (!sim_bus_ok() || !sim_acks(address))
        return PCA9685_ERR;

    uint8_t r = *reg;
    for (uint16_t i = 0; i < count; i++) {
        data[i] = sim_read_reg(r);
        r = sim_next_reg(r);
    }
    return PCA9685_OK;
}

// cppcheck-suppress constParameterPointer
int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)
{
    pca9685_sim.write_count++;
    if (!sim_bus_ok())
        return PCA9685_ERR;

    if (address == I2C_GEN_CALL_ADDRESS_PCA9685) {
        if (count == 1 && data[0] == SWRST)
            sim_load_defaults();
        return PCA9685_OK;
    }

    if (!sim_acks(address) || count == 0)
        return PCA9685_ERR;

    /* First byte selects the control register, the rest is payload */
    uint8_t reg = data[0];
    for (uint16_t i = 1; i < count; i++) {
        sim_write_reg(reg, data[i]);
        reg = sim_next_reg(reg);
    }
    return PCA9685_OK;
}

void pca9685_i2c_hal_ms_delay(uint32_t ms)
{
    pca9685_sim.delay_ms += ms;
}

uint16_t pca9685_sim_led_on(uint8_t led_no)
{
    const uint8_t *regs = &pca9685_sim.regs[LED_OFFSET_ADR + led_no * 4];
    return (uint16_t)(regs[0] | ((regs[1] & 0x1F) << 8));
}

uint16_t pca9685_sim_led_off(uint8_t led_no)
{
    const uint8_t *regs = &pca9685_sim.regs[LED_OFFSET_ADR + led_no * 4];
    return (uint16_t)(regs[2] | ((regs[3] & 0x1F) << 8));
}

uint16_t pca9685_sim_high_counts(uint8_t led_no)
{
    uint16_t on = pca9685_sim_led_on(led_no);
    uint16_t off = pca9685_sim_led_off(led_no);

    if (off & PCA9685_LED_FULL)
        return 0;
    if (on & PCA9685_LED_FULL)
        return PWM_OUTPUT_COUNTER_MAX;
    return (uint16_t)((off - on) & (PWM_OUTPUT_COUNTER_MAX - 1));
}
