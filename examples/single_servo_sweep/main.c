/*
 * Example: sweep one servo on PCA9685 channel 0 back and forth.
 *
 * This is NOT a complete CubeMX main.c. Copy the marked blocks into the
 * matching USER CODE sections of your generated main.c. It assumes:
 *   - I2C1 is enabled in CubeMX (handle name hi2c1, see pca9685_i2c_hal.c),
 *   - the PCA9685 is at the default address 0x40 (A0..A5 open),
 *   - a standard hobby servo (1.0 ms .. 2.0 ms pulse at 50 Hz) on channel 0.
 */

#include "main.h"          /* already at the top of a CubeMX main.c */

/* USER CODE BEGIN Includes */
#include "pca9685_i2c.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static pca9685_dev_t pca9685;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
#define SERVO_CHANNEL     0
#define SERVO_FREQ_HZ     50.0
#define SERVO_PERIOD_US   (1000000.0f / SERVO_FREQ_HZ)  /* 20000 us at 50 Hz */
#define SERVO_MIN_US      1000.0f   /* adjust to your servo, e.g. 500..2500 */
#define SERVO_MAX_US      2000.0f
#define PCA9685_OSC_HZ    25000000.0 /* nominal internal oscillator */

/* Convert a pulse width in microseconds to the duty-cycle percentage
 * expected by pca9685_i2c_led_pwm_set(). */
static float servo_us_to_percent(float pulse_us)
{
    return pulse_us * 100.0f / SERVO_PERIOD_US;
}

static int16_t pca9685_servo_init(void)
{
    int16_t err = PCA9685_OK;

    if (pca9685_i2c_hal_init() != PCA9685_OK)
        return PCA9685_ERR;               /* hi2c1 not initialised */

    /* Software reset first: it restores power-up defaults, which would undo
     * any address configuration written by pca9685_i2c_register(). */
    err |= pca9685_i2c_reset();

    pca9685_i2c_register(&pca9685,
                         I2C_ADDRESS_PCA9685,
                         I2C_ALL_CALL_ADDRESS_PCA9685,
                         I2C_SUB_ADDRESS_1_PCA9685,
                         I2C_SUB_ADDRESS_2_PCA9685,
                         I2C_SUB_ADDRESS_3_PCA9685);

    /* Required: pca9685_i2c_led_pwm_set() writes 4 registers in one
     * transfer, which only works with register auto-increment enabled. */
    err |= pca9685_i2c_autoincrement(pca9685, PCA9685_AUTOINCR_ON);

    /* PRE_SCALE can only be written while the oscillator is off (SLEEP=1). */
    err |= pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_SLEEP);
    err |= pca9685_i2c_write_pre_scale(pca9685, SERVO_FREQ_HZ, PCA9685_OSC_HZ);
    err |= pca9685_i2c_sleep_mode(pca9685, PCA9685_MODE_NORMAL);
    err |= pca9685_i2c_restart(pca9685);

    /* Totem-pole outputs drive servo signal lines directly. */
    pca9685_output_t out = {
        .outdrv = PCA9685_OUTPUT_TOTEM_POLE,
        .outne  = PCA9685_OUTPUT_LOW,
        .och    = PCA9685_CH_ONSTOP,
        .invrt  = PCA9685_OUTPUT_NOTINVERT,
    };
    err |= pca9685_i2c_output_init(pca9685, out);

    return err == PCA9685_OK ? PCA9685_OK : PCA9685_ERR;
}
/* USER CODE END 0 */

int main(void)
{
    /* ... HAL_Init(), SystemClock_Config(), MX_GPIO_Init(), MX_I2C1_Init() ... */

    /* USER CODE BEGIN 2 */
    if (pca9685_servo_init() != PCA9685_OK)
    {
        Error_Handler();                  /* check wiring / address / power */
    }
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    float pulse_us = SERVO_MIN_US;
    float step_us  = 10.0f;

    while (1)
    {
        pca9685_i2c_led_pwm_set(pca9685, SERVO_CHANNEL,
                                servo_us_to_percent(pulse_us), 0.0f);

        pulse_us += step_us;
        if (pulse_us >= SERVO_MAX_US || pulse_us <= SERVO_MIN_US)
            step_us = -step_us;           /* reverse direction at the ends */

        HAL_Delay(20);                    /* one PWM period at 50 Hz */
        /* USER CODE END WHILE */
    }
}
