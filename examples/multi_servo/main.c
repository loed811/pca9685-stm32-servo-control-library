/*
 * Example: drive several servos on different PCA9685 channels.
 *
 * Channels 0..3 are moved to individual angles; the phase delay of each
 * channel is staggered so the pulses do not all start at the same moment
 * (spreads the current peaks on the servo supply).
 *
 * This is NOT a complete CubeMX main.c. Copy the marked blocks into the
 * matching USER CODE sections of your generated main.c. It assumes I2C1
 * (hi2c1) is enabled and the PCA9685 is at the default address 0x40.
 */

#include "main.h"          /* already at the top of a CubeMX main.c */

/* USER CODE BEGIN Includes */
#include "pca9685_i2c.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static pca9685_dev_t pca9685;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
#define SERVO_COUNT       4
#define SERVO_FREQ_HZ     50.0
#define SERVO_PERIOD_US   (1000000.0f / SERVO_FREQ_HZ)
#define SERVO_MIN_US      1000.0f   /* pulse width at 0 degrees   */
#define SERVO_MAX_US      2000.0f   /* pulse width at 180 degrees */
#define PCA9685_OSC_HZ    25000000.0

/* PCA9685 channel used by each servo. */
static const uint8_t servo_channel[SERVO_COUNT] = { 0, 1, 2, 3 };

/* Phase delay of each channel in percent of the period (5% = 1 ms).
 * Keep delay + duty cycle below 100%. */
static const float servo_delay[SERVO_COUNT] = { 0.0f, 5.0f, 10.0f, 15.0f };

static int16_t pca9685_servo_init(void)
{
    int16_t err = PCA9685_OK;

    if (pca9685_i2c_hal_init() != PCA9685_OK)
        return PCA9685_ERR;

    err |= pca9685_i2c_reset();
    pca9685_i2c_register(&pca9685,
                         I2C_ADDRESS_PCA9685,
                         I2C_ALL_CALL_ADDRESS_PCA9685,
                         I2C_SUB_ADDRESS_1_PCA9685,
                         I2C_SUB_ADDRESS_2_PCA9685,
                         I2C_SUB_ADDRESS_3_PCA9685);
    err |= pca9685_i2c_autoincrement(pca9685, PCA9685_AUTOINCR_ON);

    err |= pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_SLEEP);
    err |= pca9685_i2c_write_pre_scale(pca9685, SERVO_FREQ_HZ, PCA9685_OSC_HZ);
    err |= pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_NORMAL);
    err |= pca9685_i2c_restart(pca9685);

    pca9685_output_t out = {
        .outdrv = PCA9685_OUTPUT_TOTEM_POLE,
        .outne  = PCA9685_OUTPUT_LOW,
        .och    = PCA9685_CH_ONSTOP,
        .invrt  = PCA9685_OUTPUT_NOTINVERT,
    };
    err |= pca9685_i2c_output_init(pca9685, out);

    return err == PCA9685_OK ? PCA9685_OK : PCA9685_ERR;
}

/* Move servo 'idx' to 'angle_deg' (0..180). */
static int16_t servo_set_angle(uint8_t idx, float angle_deg)
{
    if (idx >= SERVO_COUNT)
        return PCA9685_ERR;
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    float pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle_deg / 180.0f;
    float duty     = pulse_us * 100.0f / SERVO_PERIOD_US;

    return pca9685_i2c_led_pwm_set(pca9685, servo_channel[idx], duty, servo_delay[idx]);
}
/* USER CODE END 0 */

int main(void)
{
    /* ... HAL_Init(), SystemClock_Config(), MX_GPIO_Init(), MX_I2C1_Init() ... */

    /* USER CODE BEGIN 2 */
    if (pca9685_servo_init() != PCA9685_OK)
    {
        Error_Handler();
    }

    /* Optional sanity check: read back the programmed PWM frequency. */
    double freq_hz = 0.0;
    pca9685_i2c_read_pre_scale(pca9685, &freq_hz, PCA9685_OSC_HZ);  /* ~50.0 Hz (prescale 121) */
    (void)freq_hz;
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    static const float pose_a[SERVO_COUNT] = {   0.0f,  45.0f,  90.0f, 135.0f };
    static const float pose_b[SERVO_COUNT] = { 180.0f, 135.0f,  90.0f,  45.0f };

    while (1)
    {
        for (uint8_t i = 0; i < SERVO_COUNT; i++)
            servo_set_angle(i, pose_a[i]);
        HAL_Delay(1000);

        for (uint8_t i = 0; i < SERVO_COUNT; i++)
            servo_set_angle(i, pose_b[i]);
        HAL_Delay(1000);
        /* USER CODE END WHILE */
    }
}
