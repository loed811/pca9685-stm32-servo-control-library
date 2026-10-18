#include "servo_sweep.h"
#include "pca9685_i2c_hal.h"

int16_t servo_board_init(pca9685_dev_t *dev)
{
    pca9685_output_t output = {
        .outdrv = PCA9685_OUTPUT_TOTEM_POLE, /* Push-pull outputs drive servo signal lines directly */
        .outne  = PCA9685_OUTPUT_LOW,
        .och    = PCA9685_CH_ONSTOP,
        .invrt  = PCA9685_OUTPUT_NOTINVERT,
    };

    if (pca9685_i2c_hal_init() != PCA9685_OK)
        return PCA9685_ERR;

    /* Start from the power-up state: SLEEP set, all outputs off */
    if (pca9685_i2c_reset() != PCA9685_OK)
        return PCA9685_ERR;

    if (pca9685_i2c_register(dev, I2C_ADDRESS_PCA9685,
                             I2C_ALL_CALL_ADDRESS_PCA9685,
                             I2C_SUB_ADDRESS_1_PCA9685,
                             I2C_SUB_ADDRESS_2_PCA9685,
                             I2C_SUB_ADDRESS_3_PCA9685) != PCA9685_OK)
        return PCA9685_ERR;

    /* PWM updates write the 4 ON/OFF registers of a channel in one transfer */
    if (pca9685_i2c_autoincrement(*dev, PCA9685_AUTOINCR_ON) != PCA9685_OK)
        return PCA9685_ERR;

    /* PRE_SCALE is only writable while the oscillator is off */
    if (pca9685_i2c_sleep_mode(*dev, PCA9685_MODE_SLEEP) != PCA9685_OK)
        return PCA9685_ERR;

    if (pca9685_i2c_write_pre_scale(*dev, SERVO_PWM_FREQ_HZ, PCA9685_INTERNAL_OSC_HZ) != PCA9685_OK)
        return PCA9685_ERR;

    if (pca9685_i2c_output_init(*dev, output) != PCA9685_OK)
        return PCA9685_ERR;

    /* Wake the oscillator */
    return pca9685_i2c_restart(*dev);
}

float servo_angle_to_pulse_us(float angle_deg)
{
    if (!(angle_deg > 0.0f))
        angle_deg = 0.0f;
    if (angle_deg > SERVO_MAX_ANGLE_DEG)
        angle_deg = SERVO_MAX_ANGLE_DEG;

    return SERVO_MIN_PULSE_US + (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle_deg / SERVO_MAX_ANGLE_DEG;
}

int16_t servo_set_angle(pca9685_dev_t dev, uint8_t channel, float angle_deg)
{
    return pca9685_i2c_servo_pulse_set(dev, channel, servo_angle_to_pulse_us(angle_deg), SERVO_PWM_FREQ_HZ);
}

int16_t servo_sweep(pca9685_dev_t dev, uint8_t channel, float step_deg, uint32_t step_delay_ms)
{
    if (!(step_deg > 0.0f))
        return PCA9685_ERR;

    /* Count steps with an integer so accumulated float error cannot add or drop a step */
    uint32_t steps = (uint32_t)(SERVO_MAX_ANGLE_DEG / step_deg);

    for (uint32_t i = 0; i <= steps; i++) {
        if (servo_set_angle(dev, channel, (float)i * step_deg) != PCA9685_OK)
            return PCA9685_ERR;
        pca9685_i2c_hal_ms_delay(step_delay_ms);
    }

    for (uint32_t i = steps + 1; i-- > 0;) {
        if (servo_set_angle(dev, channel, (float)i * step_deg) != PCA9685_OK)
            return PCA9685_ERR;
        pca9685_i2c_hal_ms_delay(step_delay_ms);
    }

    return PCA9685_OK;
}
