#include <math.h>

#include "unity.h"

#include "pca9685_i2c.h"
#include "pca9685_sim.h"
#include "servo_sweep.h"

static pca9685_dev_t dev;

/* Puts the simulated device into the running state with auto-increment enabled */
static void run_with_auto_increment(void)
{
    pca9685_sim.regs[REG_MODE_1] = PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
}

void setUp(void)
{
    pca9685_sim_power_on(I2C_ADDRESS_PCA9685);
    dev.i2c_addr = I2C_ADDRESS_PCA9685;
    dev.allcall_addr = I2C_ALL_CALL_ADDRESS_PCA9685;
    dev.sub_addr_1 = I2C_SUB_ADDRESS_1_PCA9685;
    dev.sub_addr_2 = I2C_SUB_ADDRESS_2_PCA9685;
    dev.sub_addr_3 = I2C_SUB_ADDRESS_3_PCA9685;
}

void tearDown(void)
{
}

/* -------------------------------------------------------------------------- */
/* Reset and addressing                                                       */
/* -------------------------------------------------------------------------- */

static void test_reset_restores_power_on_defaults(void)
{
    pca9685_sim.regs[REG_MODE_1] = PCA9685_MODE1_AI;
    pca9685_sim.regs[REG_MODE_2] = 0x10;
    pca9685_sim.regs[REG_PRE_SCALE] = 0x79;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_reset());

    TEST_ASSERT_EQUAL_HEX8(0x11, pca9685_sim.regs[REG_MODE_1]);
    TEST_ASSERT_EQUAL_HEX8(0x04, pca9685_sim.regs[REG_MODE_2]);
    TEST_ASSERT_EQUAL_HEX8(0x1E, pca9685_sim.regs[REG_PRE_SCALE]);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(STAB_TIME, pca9685_sim.delay_ms);
}

static void test_register_programs_addresses(void)
{
    pca9685_dev_t d;
    uint8_t addr;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_register(&d, 0x40, 0x60, 0x61, 0x62, 0x63));

    TEST_ASSERT_EQUAL_HEX8(0x40, d.i2c_addr);
    TEST_ASSERT_EQUAL_HEX8(0xC0, pca9685_sim.regs[REG_ALLCALLADR]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_read_allcall_addr(d, &addr));
    TEST_ASSERT_EQUAL_HEX8(0x60, addr);
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_read_sub_addr(d, PCA9685_SUB_ADDR_1, &addr));
    TEST_ASSERT_EQUAL_HEX8(0x61, addr);
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_read_sub_addr(d, PCA9685_SUB_ADDR_2, &addr));
    TEST_ASSERT_EQUAL_HEX8(0x62, addr);
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_read_sub_addr(d, PCA9685_SUB_ADDR_3, &addr));
    TEST_ASSERT_EQUAL_HEX8(0x63, addr);
}

static void test_sub_addr_resp_maps_to_mode1_bits(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_1, PCA9685_ADDR_RESPOND));
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_SUB1, pca9685_sim.regs[REG_MODE_1]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_2, PCA9685_ADDR_RESPOND));
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_SUB2, pca9685_sim.regs[REG_MODE_1]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_3, PCA9685_ADDR_RESPOND));
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_SUB3, pca9685_sim.regs[REG_MODE_1]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_2, PCA9685_ADDR_NORESPOND));
    TEST_ASSERT_BITS_LOW(PCA9685_MODE1_SUB2, pca9685_sim.regs[REG_MODE_1]);
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_SUB1 | PCA9685_MODE1_SUB3, pca9685_sim.regs[REG_MODE_1]);
}

static void test_device_answers_sub_address_only_when_enabled(void)
{
    pca9685_dev_t sub = dev;
    uint8_t mode;
    sub.i2c_addr = I2C_SUB_ADDRESS_2_PCA9685;

    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_read_mode_1(sub, &mode));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_2, PCA9685_ADDR_RESPOND));
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_read_mode_1(sub, &mode));
}

static void test_allcall_response_can_be_disabled(void)
{
    pca9685_dev_t allcall = dev;
    uint8_t mode;
    allcall.i2c_addr = I2C_ALL_CALL_ADDRESS_PCA9685;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_read_mode_1(allcall, &mode));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_allcall_address_resp(dev, PCA9685_ADDR_NORESPOND));
    TEST_ASSERT_BITS_LOW(PCA9685_MODE1_ALLCALL, pca9685_sim.regs[REG_MODE_1]);
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_read_mode_1(allcall, &mode));
}

static void test_sub_addr_functions_reject_invalid_index(void)
{
    uint8_t addr = 0xAA;

    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_write_sub_addr(dev, (pca9685_subaddr_no_t)0, 0x50));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_read_sub_addr(dev, (pca9685_subaddr_no_t)4, &addr));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_sub_addr_resp(dev, (pca9685_subaddr_no_t)4, PCA9685_ADDR_RESPOND));

    TEST_ASSERT_EQUAL_HEX8(0xAA, addr);
    TEST_ASSERT_EQUAL_UINT32(0, pca9685_sim.write_count);
    TEST_ASSERT_EQUAL_UINT32(0, pca9685_sim.read_count);
}

/* -------------------------------------------------------------------------- */
/* MODE1: sleep, restart, auto-increment, clock                               */
/* -------------------------------------------------------------------------- */

static void test_sleep_mode_sets_and_clears_sleep_bit(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sleep_mode(dev, PCA9685_MODE_NORMAL));
    TEST_ASSERT_BITS_LOW(PCA9685_MODE1_SLEEP, pca9685_sim.regs[REG_MODE_1]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP));
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_SLEEP, pca9685_sim.regs[REG_MODE_1]);
}

static void test_autoincrement_and_clock_bits(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_ON));
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_AI, pca9685_sim.regs[REG_MODE_1]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_OFF));
    TEST_ASSERT_BITS_LOW(PCA9685_MODE1_AI, pca9685_sim.regs[REG_MODE_1]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_clock(dev, PCA9685_CLK_EXTERNAL));
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_EXTCLK, pca9685_sim.regs[REG_MODE_1]);
}

/* Regression: read-modify-write used to write a set RESTART bit back, restarting PWM */
static void test_mode1_updates_keep_pending_restart(void)
{
    pca9685_sim.regs[REG_MODE_1] = PCA9685_MODE1_RESTART | PCA9685_MODE1_SLEEP | PCA9685_MODE1_ALLCALL;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_ON));
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_allcall_address_resp(dev, PCA9685_ADDR_RESPOND));

    TEST_ASSERT_EQUAL_UINT32(0, pca9685_sim.restart_count);
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_RESTART, pca9685_sim.regs[REG_MODE_1]);
}

/* Regression: restart() used to write MODE1 back unchanged, leaving the device asleep */
static void test_restart_resumes_pwm_after_sleep(void)
{
    run_with_auto_increment();
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP));
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_RESTART, pca9685_sim.regs[REG_MODE_1]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_restart(dev));

    TEST_ASSERT_BITS_LOW(PCA9685_MODE1_SLEEP | PCA9685_MODE1_RESTART, pca9685_sim.regs[REG_MODE_1]);
    TEST_ASSERT_BITS_HIGH(PCA9685_MODE1_AI, pca9685_sim.regs[REG_MODE_1]);
    TEST_ASSERT_EQUAL_UINT32(1, pca9685_sim.restart_count);
}

static void test_restart_wakes_device_without_pending_restart(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_restart(dev));

    TEST_ASSERT_BITS_LOW(PCA9685_MODE1_SLEEP, pca9685_sim.regs[REG_MODE_1]);
    TEST_ASSERT_EQUAL_UINT32(0, pca9685_sim.restart_count);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(STAB_TIME, pca9685_sim.delay_ms);
}

/* -------------------------------------------------------------------------- */
/* MODE2: output configuration                                                */
/* -------------------------------------------------------------------------- */

/* Regression: OUTNE used to be shifted into the OUTDRV bit */
static void test_output_init_encodes_mode2(void)
{
    pca9685_output_t output = {
        .outdrv = PCA9685_OUTPUT_OPEN_DRAIN,
        .outne = PCA9685_OUTPUT_HIGH_IMPEDANCE,
        .och = PCA9685_CH_ONACK,
        .invrt = PCA9685_OUTPUT_INVERT,
    };

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_output_init(dev, output));
    TEST_ASSERT_EQUAL_HEX8(0x1A, pca9685_sim.regs[REG_MODE_2]);

    output.outdrv = PCA9685_OUTPUT_TOTEM_POLE;
    output.outne = PCA9685_OUTPUT_HIGH;
    output.och = PCA9685_CH_ONSTOP;
    output.invrt = PCA9685_OUTPUT_NOTINVERT;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_output_init(dev, output));
    TEST_ASSERT_EQUAL_HEX8(0x05, pca9685_sim.regs[REG_MODE_2]);
}

/* -------------------------------------------------------------------------- */
/* PWM frequency                                                              */
/* -------------------------------------------------------------------------- */

static void test_prescale_for_50hz_servo(void)
{
    double frequency = 0.0;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_write_pre_scale(dev, 50.0, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_UINT8(121, pca9685_sim.regs[REG_PRE_SCALE]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_read_pre_scale(dev, &frequency, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 50.0f, (float)frequency);
}

/* Example from the datasheet: 200 Hz -> PRE_SCALE = 0x1E */
static void test_prescale_datasheet_example_200hz(void)
{
    pca9685_sim.regs[REG_PRE_SCALE] = 0x50;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_write_pre_scale(dev, 200.0, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_HEX8(0x1E, pca9685_sim.regs[REG_PRE_SCALE]);
}

static void test_prescale_range_limits(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_write_pre_scale(dev, 1526.0, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_UINT8(PCA9685_PRE_SCALE_MIN, pca9685_sim.regs[REG_PRE_SCALE]);

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_write_pre_scale(dev, 23.85, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_UINT8(PCA9685_PRE_SCALE_MAX, pca9685_sim.regs[REG_PRE_SCALE]);
}

static void test_prescale_rejects_unreachable_frequencies(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_write_pre_scale(dev, 1800.0, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_write_pre_scale(dev, 23.7, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_write_pre_scale(dev, 0.0, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_write_pre_scale(dev, 50.0, 0.0));

    TEST_ASSERT_EQUAL_UINT32(0, pca9685_sim.write_count);
}

/* The hardware ignores PRE_SCALE writes while the oscillator runs */
static void test_prescale_needs_sleep_mode(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sleep_mode(dev, PCA9685_MODE_NORMAL));
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_write_pre_scale(dev, 50.0, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_HEX8(0x1E, pca9685_sim.regs[REG_PRE_SCALE]);
}

/* -------------------------------------------------------------------------- */
/* PWM duty cycle                                                             */
/* -------------------------------------------------------------------------- */

static void test_pwm_duty_and_phase_delay(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 3, 25.0f, 10.0f));

    TEST_ASSERT_EQUAL_UINT16(410, pca9685_sim_led_on(3));
    TEST_ASSERT_EQUAL_UINT16(1434, pca9685_sim_led_off(3));
    TEST_ASSERT_EQUAL_UINT16(1024, pca9685_sim_high_counts(3));
}

/* Regression: an OFF time past the end of the period used to set the full OFF bit */
static void test_pwm_off_time_wraps_past_end_of_period(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 75.0f));

    TEST_ASSERT_EQUAL_UINT16(3072, pca9685_sim_led_on(0));
    TEST_ASSERT_EQUAL_UINT16(1024, pca9685_sim_led_off(0));
    TEST_ASSERT_EQUAL_UINT16(2048, pca9685_sim_high_counts(0));
}

static void test_pwm_zero_and_full_duty_use_full_bits(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 1, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_HEX16(PCA9685_LED_FULL, pca9685_sim_led_off(1));
    TEST_ASSERT_EQUAL_UINT16(0, pca9685_sim_high_counts(1));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 1, 100.0f, 0.0f));
    TEST_ASSERT_EQUAL_HEX16(PCA9685_LED_FULL, pca9685_sim_led_on(1));
    TEST_ASSERT_EQUAL_HEX16(0, pca9685_sim_led_off(1));
    TEST_ASSERT_EQUAL_UINT16(PWM_OUTPUT_COUNTER_MAX, pca9685_sim_high_counts(1));
}

static void test_pwm_clamps_out_of_range_values(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 2, 150.0f, 0.0f));
    TEST_ASSERT_EQUAL_UINT16(PWM_OUTPUT_COUNTER_MAX, pca9685_sim_high_counts(2));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 2, -20.0f, 0.0f));
    TEST_ASSERT_EQUAL_UINT16(0, pca9685_sim_high_counts(2));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 2, NAN, 0.0f));
    TEST_ASSERT_EQUAL_UINT16(0, pca9685_sim_high_counts(2));

    /* A 100 % delay is a whole period, the same as no delay */
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 2, 50.0f, 100.0f));
    TEST_ASSERT_EQUAL_UINT16(0, pca9685_sim_led_on(2));
    TEST_ASSERT_EQUAL_UINT16(2048, pca9685_sim_high_counts(2));
}

static void test_pwm_rejects_invalid_channel(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_led_pwm_set(dev, PCA9685_LED_COUNT, 50.0f, 0.0f));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_led_set(dev, PCA9685_LED_COUNT, PCA9685_LED_ON));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_servo_pulse_set(dev, 200, 1500.0f, 50.0f));

    TEST_ASSERT_EQUAL_UINT32(0, pca9685_sim.write_count);
}

/* Regression: the unused ON/OFF bytes were sent uninitialized */
static void test_led_set_full_on_and_off(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_set(dev, 5, PCA9685_LED_ON));
    TEST_ASSERT_EQUAL_HEX16(PCA9685_LED_FULL, pca9685_sim_led_on(5));
    TEST_ASSERT_EQUAL_HEX16(0, pca9685_sim_led_off(5));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_set(dev, 5, PCA9685_LED_OFF));
    TEST_ASSERT_EQUAL_HEX16(0, pca9685_sim_led_on(5));
    TEST_ASSERT_EQUAL_HEX16(PCA9685_LED_FULL, pca9685_sim_led_off(5));
}

/* Regression: a zero phase delay used to set the full ON bit on every channel */
static void test_all_led_pwm_updates_every_channel(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_all_led_pwm_set(dev, 7.5f, 0.0f));

    for (uint8_t led = 0; led < PCA9685_LED_COUNT; led++) {
        TEST_ASSERT_EQUAL_UINT16(0, pca9685_sim_led_on(led));
        TEST_ASSERT_EQUAL_UINT16(307, pca9685_sim_high_counts(led));
    }
}

static void test_all_led_set(void)
{
    run_with_auto_increment();

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_all_led_set(dev, PCA9685_LED_ON));
    for (uint8_t led = 0; led < PCA9685_LED_COUNT; led++)
        TEST_ASSERT_EQUAL_UINT16(PWM_OUTPUT_COUNTER_MAX, pca9685_sim_high_counts(led));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_all_led_set(dev, PCA9685_LED_OFF));
    for (uint8_t led = 0; led < PCA9685_LED_COUNT; led++)
        TEST_ASSERT_EQUAL_UINT16(0, pca9685_sim_high_counts(led));
}

/* Without auto-increment every byte of a PWM update lands in LEDn_ON_L */
static void test_pwm_requires_auto_increment(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_sleep_mode(dev, PCA9685_MODE_NORMAL));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_led_pwm_set(dev, 0, 25.0f, 10.0f));

    TEST_ASSERT_NOT_EQUAL(1024, pca9685_sim_high_counts(0));
}

/* -------------------------------------------------------------------------- */
/* Servo helper                                                               */
/* -------------------------------------------------------------------------- */

static void test_servo_pulse_width_to_counts(void)
{
    run_with_auto_increment();

    /* 20 ms period / 4096 steps = 4.88 us per step */
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_servo_pulse_set(dev, 0, 1500.0f, 50.0f));
    TEST_ASSERT_EQUAL_UINT16(307, pca9685_sim_high_counts(0));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_servo_pulse_set(dev, 0, 1000.0f, 50.0f));
    TEST_ASSERT_EQUAL_UINT16(205, pca9685_sim_high_counts(0));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, pca9685_i2c_servo_pulse_set(dev, 0, 2000.0f, 50.0f));
    TEST_ASSERT_EQUAL_UINT16(410, pca9685_sim_high_counts(0));
}

static void test_servo_pulse_rejects_invalid_arguments(void)
{
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_servo_pulse_set(dev, 0, 1500.0f, 0.0f));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_servo_pulse_set(dev, 0, -1.0f, 50.0f));
    TEST_ASSERT_EQUAL_UINT32(0, pca9685_sim.write_count);
}

/* -------------------------------------------------------------------------- */
/* Error handling                                                             */
/* -------------------------------------------------------------------------- */

static void test_bus_errors_are_reported(void)
{
    pca9685_output_t output = {
        PCA9685_OUTPUT_TOTEM_POLE, PCA9685_OUTPUT_LOW, PCA9685_CH_ONSTOP, PCA9685_OUTPUT_NOTINVERT
    };
    uint8_t mode;

    pca9685_sim_fail_after(0);

    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_read_mode_1(dev, &mode));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_restart(dev));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_output_init(dev, output));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 0.0f));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_write_pre_scale(dev, 50.0, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_reset());
}

static void test_failed_reads_leave_outputs_untouched(void)
{
    double frequency = -1.0;
    uint8_t addr = 0xAA;

    pca9685_sim_fail_after(0);

    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_read_pre_scale(dev, &frequency, PCA9685_INTERNAL_OSC_HZ));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_read_allcall_addr(dev, &addr));
    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_read_sub_addr(dev, PCA9685_SUB_ADDR_1, &addr));

    TEST_ASSERT_EQUAL_FLOAT(-1.0f, (float)frequency);
    TEST_ASSERT_EQUAL_HEX8(0xAA, addr);
}

static void test_register_stops_on_first_failure(void)
{
    pca9685_dev_t d;

    pca9685_sim_fail_after(1);

    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, pca9685_i2c_register(&d, 0x40, 0x70, 0x71, 0x72, 0x74));
    TEST_ASSERT_EQUAL_UINT32(2, pca9685_sim.write_count);
}

/* -------------------------------------------------------------------------- */
/* Servo sweep example                                                        */
/* -------------------------------------------------------------------------- */

static void test_example_board_init(void)
{
    pca9685_dev_t d;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_board_init(&d));

    TEST_ASSERT_EQUAL_HEX8(PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL, pca9685_sim.regs[REG_MODE_1]);
    TEST_ASSERT_EQUAL_HEX8(0x04, pca9685_sim.regs[REG_MODE_2]);
    TEST_ASSERT_EQUAL_UINT8(121, pca9685_sim.regs[REG_PRE_SCALE]);
}

static void test_example_servo_angles(void)
{
    pca9685_dev_t d;
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_board_init(&d));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_set_angle(d, 2, 0.0f));
    TEST_ASSERT_EQUAL_UINT16(205, pca9685_sim_high_counts(2));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_set_angle(d, 2, 90.0f));
    TEST_ASSERT_EQUAL_UINT16(307, pca9685_sim_high_counts(2));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_set_angle(d, 2, 180.0f));
    TEST_ASSERT_EQUAL_UINT16(410, pca9685_sim_high_counts(2));

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_set_angle(d, 2, 270.0f));
    TEST_ASSERT_EQUAL_UINT16(410, pca9685_sim_high_counts(2));
}

static void test_example_sweep_returns_to_start(void)
{
    pca9685_dev_t d;
    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_board_init(&d));
    uint32_t delay_before = pca9685_sim.delay_ms;

    TEST_ASSERT_EQUAL_INT16(PCA9685_OK, servo_sweep(d, 1, 10.0f, 15));

    /* 0..180 and back in 10 degree steps: 19 positions each way */
    TEST_ASSERT_EQUAL_UINT32(38 * 15, pca9685_sim.delay_ms - delay_before);
    TEST_ASSERT_EQUAL_UINT16(205, pca9685_sim_high_counts(1));

    TEST_ASSERT_EQUAL_INT16(PCA9685_ERR, servo_sweep(d, 1, 0.0f, 15));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_reset_restores_power_on_defaults);
    RUN_TEST(test_register_programs_addresses);
    RUN_TEST(test_sub_addr_resp_maps_to_mode1_bits);
    RUN_TEST(test_device_answers_sub_address_only_when_enabled);
    RUN_TEST(test_allcall_response_can_be_disabled);
    RUN_TEST(test_sub_addr_functions_reject_invalid_index);

    RUN_TEST(test_sleep_mode_sets_and_clears_sleep_bit);
    RUN_TEST(test_autoincrement_and_clock_bits);
    RUN_TEST(test_mode1_updates_keep_pending_restart);
    RUN_TEST(test_restart_resumes_pwm_after_sleep);
    RUN_TEST(test_restart_wakes_device_without_pending_restart);

    RUN_TEST(test_output_init_encodes_mode2);

    RUN_TEST(test_prescale_for_50hz_servo);
    RUN_TEST(test_prescale_datasheet_example_200hz);
    RUN_TEST(test_prescale_range_limits);
    RUN_TEST(test_prescale_rejects_unreachable_frequencies);
    RUN_TEST(test_prescale_needs_sleep_mode);

    RUN_TEST(test_pwm_duty_and_phase_delay);
    RUN_TEST(test_pwm_off_time_wraps_past_end_of_period);
    RUN_TEST(test_pwm_zero_and_full_duty_use_full_bits);
    RUN_TEST(test_pwm_clamps_out_of_range_values);
    RUN_TEST(test_pwm_rejects_invalid_channel);
    RUN_TEST(test_led_set_full_on_and_off);
    RUN_TEST(test_all_led_pwm_updates_every_channel);
    RUN_TEST(test_all_led_set);
    RUN_TEST(test_pwm_requires_auto_increment);

    RUN_TEST(test_servo_pulse_width_to_counts);
    RUN_TEST(test_servo_pulse_rejects_invalid_arguments);

    RUN_TEST(test_bus_errors_are_reported);
    RUN_TEST(test_failed_reads_leave_outputs_untouched);
    RUN_TEST(test_register_stops_on_first_failure);

    RUN_TEST(test_example_board_init);
    RUN_TEST(test_example_servo_angles);
    RUN_TEST(test_example_sweep_returns_to_start);

    return UNITY_END();
}
