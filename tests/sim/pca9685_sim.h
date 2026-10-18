#ifndef PCA9685_SIM_H
#define PCA9685_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Register-level model of a PCA9685 attached to a virtual I2C bus.
 *
 * The simulator implements the HAL functions from pca9685_i2c_hal.h, so the
 * unmodified driver can be tested on a PC. It models the datasheet behaviour
 * the driver depends on:
 *  - register auto-increment (MODE1.AI), including the 0x45 -> 0x00 rollover,
 *  - PRE_SCALE writes being ignored unless MODE1.SLEEP is set,
 *  - the MODE1.RESTART flag (set when sleeping, cleared by writing 1),
 *  - the sticky EXTCLK bit,
 *  - ALL_LED registers broadcasting to every channel,
 *  - Software Reset through the General Call address,
 *  - responding to the All Call and Sub-Addresses only when enabled in MODE1.
 */
typedef struct {
    uint8_t regs[256];          /**< Register file */
    uint8_t address;            /**< 7-bit hardware address of the device */
    uint32_t write_count;       /**< Number of HAL write transactions */
    uint32_t read_count;        /**< Number of HAL read transactions */
    uint32_t delay_ms;          /**< Total time spent in pca9685_i2c_hal_ms_delay() */
    uint32_t restart_count;     /**< Number of PWM restarts (RESTART bit cleared by a write of 1) */
    int32_t fail_after;         /**< Transactions left before the bus fails, -1 = never */
} pca9685_sim_t;

/** @brief Global simulator state, inspected by the tests. */
extern pca9685_sim_t pca9685_sim;

/**
 * @brief Powers up the simulated device with datasheet default register values.
 * @param address 7-bit hardware address of the device.
 */
void pca9685_sim_power_on(uint8_t address);

/**
 * @brief Makes every I2C transaction fail after the next 'n' successful ones.
 * @param n Number of transactions that still succeed.
 */
void pca9685_sim_fail_after(int32_t n);

/** @brief LEDn_ON value of a channel, including the full ON bit (0x1000). */
uint16_t pca9685_sim_led_on(uint8_t led_no);

/** @brief LEDn_OFF value of a channel, including the full OFF bit (0x1000). */
uint16_t pca9685_sim_led_off(uint8_t led_no);

/**
 * @brief Number of counter steps (0-4096) per period the output is high.
 * Full OFF takes precedence over full ON, as in the hardware.
 */
uint16_t pca9685_sim_high_counts(uint8_t led_no);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_SIM_H */
