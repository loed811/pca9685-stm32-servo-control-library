#ifndef SERVO_SWEEP_H
#define SERVO_SWEEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pca9685_i2c.h"

#define SERVO_PWM_FREQ_HZ       50.0f   /**< Standard analog servo frame rate (20 ms period) */
#define SERVO_MIN_PULSE_US      1000.0f /**< Pulse width at 0 degrees */
#define SERVO_MAX_PULSE_US      2000.0f /**< Pulse width at SERVO_MAX_ANGLE_DEG */
#define SERVO_MAX_ANGLE_DEG     180.0f  /**< Mechanical range of the servo */

/**
 * @brief Brings the PCA9685 up for driving servos at 50 Hz.
 * Reset -> register addresses -> auto-increment -> PRE_SCALE (in sleep) -> MODE2 -> wake.
 * @param dev Device structure to fill.
 * @return PCA9685_OK on success, PCA9685_ERR on the first failing step.
 */
int16_t servo_board_init(pca9685_dev_t *dev);

/**
 * @brief Maps an angle to a pulse width between SERVO_MIN_PULSE_US and SERVO_MAX_PULSE_US.
 * @param angle_deg Angle in degrees, clamped to 0..SERVO_MAX_ANGLE_DEG.
 * @return Pulse width in microseconds.
 */
float servo_angle_to_pulse_us(float angle_deg);

/**
 * @brief Moves the servo on a channel to an angle.
 * @param dev The device structure.
 * @param channel PCA9685 channel (0-15).
 * @param angle_deg Target angle in degrees.
 * @return PCA9685_OK on success.
 */
int16_t servo_set_angle(pca9685_dev_t dev, uint8_t channel, float angle_deg);

/**
 * @brief Sweeps a servo from 0 to SERVO_MAX_ANGLE_DEG and back once.
 * @param dev The device structure.
 * @param channel PCA9685 channel (0-15).
 * @param step_deg Angle increment per step (> 0).
 * @param step_delay_ms Delay between steps.
 * @return PCA9685_OK on success.
 */
int16_t servo_sweep(pca9685_dev_t dev, uint8_t channel, float step_deg, uint32_t step_delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_SWEEP_H */
