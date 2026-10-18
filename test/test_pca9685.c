/*
 * Host unit tests for the PCA9685 driver, run against the mock platform
 * layer in mock_hal.c. Build and run with `make -C test`.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mock_hal.h"
#include "pca9685_i2c.h"

/* -------------------------------------------------------------------------- */
/* Minimal test harness                                                       */
/* -------------------------------------------------------------------------- */

static int checks;
static int failures;
static const char *current_test;

#define CHECK(cond) do {                                                    \
        checks++;                                                           \
        if (!(cond)) {                                                      \
            failures++;                                                     \
            printf("%s:%d: [%s] CHECK(%s) failed\n",                        \
                   __FILE__, __LINE__, current_test, #cond);                \
        }                                                                   \
    } while (0)

#define CHECK_EQ(actual, expected) do {                                     \
        long a_ = (long)(actual), e_ = (long)(expected);                    \
        checks++;                                                           \
        if (a_ != e_) {                                                     \
            failures++;                                                     \
            printf("%s:%d: [%s] %s == %ld (0x%lX), expected %ld (0x%lX)\n", \
                   __FILE__, __LINE__, current_test, #actual,               \
                   a_, (unsigned long)a_, e_, (unsigned long)e_);           \
        }                                                                   \
    } while (0)

/* Compare the last write transfer with the given bytes. */
#define CHECK_LAST_WRITE(...)                                               \
    check_last_write(__FILE__, __LINE__, (const uint8_t[]){ __VA_ARGS__ },  \
                     sizeof((const uint8_t[]){ __VA_ARGS__ }))

static void check_last_write(const char *file, int line,
                             const uint8_t *expected, size_t len)
{
    const mock_event_t *ev = mock_last_write();

    checks++;
    if (ev != NULL && ev->len == len && memcmp(ev->data, expected, len) == 0)
        return;

    failures++;
    printf("%s:%d: [%s] last write mismatch\n  expected:", file, line, current_test);
    for (size_t i = 0; i < len; i++)
        printf(" %02X", expected[i]);
    printf("\n  actual:  ");
    if (ev == NULL) {
        printf(" (no write)");
    } else {
        for (uint16_t i = 0; i < ev->len; i++)
            printf(" %02X", ev->data[i]);
    }
    printf("\n");
}

#define RUN(test) do { current_test = #test; mock_reset(); test(); } while (0)

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static pca9685_dev_t dev(void)
{
    pca9685_dev_t d = { 0x40, 0x70, 0x71, 0x72, 0x74 };
    return d;
}

/* Decode the ON/OFF values of the last 5-byte LED write. */
static void last_on_off(uint16_t *on, uint16_t *off)
{
    const mock_event_t *ev = mock_last_write();
    *on = (uint16_t)(ev->data[1] | (ev->data[2] << 8));
    *off = (uint16_t)(ev->data[3] | (ev->data[4] << 8));
}

static const pca9685_output_t servo_output = {
    .outdrv = PCA9685_OUTPUT_TOTEM_POLE,
    .outne  = PCA9685_OUTPUT_LOW,
    .och    = PCA9685_CH_ONSTOP,
    .invrt  = PCA9685_OUTPUT_NOTINVERT,
};

/* -------------------------------------------------------------------------- */
/* PWM encoding                                                               */
/* -------------------------------------------------------------------------- */

static void test_pwm_servo_center(void)
{
    /* 7.5 % of 4096 = 307.2 -> 307 = 0x133 counts, starting at 0 */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 7.5f, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x33, 0x01);
}

static void test_pwm_with_delay(void)
{
    /* delay 5 % = 204.8 -> 205 (0xCD); OFF = 205 + 307 = 512 (0x200) */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 7.5f, 5.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0xCD, 0x00, 0x00, 0x02);
}

static void test_pwm_high_time_independent_of_delay(void)
{
    /* Used to be one count shorter with delay = 0. */
    for (int d = 0; d < 100; d += 3) {
        uint16_t on, off;
        CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 7.5f, (float)d), PCA9685_OK);
        last_on_off(&on, &off);
        CHECK(on < 4096 && off < 4096);
        CHECK_EQ((off - on) & 0x0FFF, 307);
    }
}

static void test_pwm_zero_is_full_off(void)
{
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 0.0f, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x00, 0x10);

    /* Rounds to 0 counts: still full OFF, never ON == OFF. */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 0.01f, 30.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x00, 0x10);
}

static void test_pwm_hundred_is_full_on(void)
{
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 100.0f, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x10, 0x00, 0x00);

    /* Rounds to 4096 counts. */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 99.995f, 10.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x10, 0x00, 0x00);
}

static void test_pwm_out_of_range_is_clamped(void)
{
    /* -5 % used to give ~95 % duty (3890 counts). */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, -5.0f, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x00, 0x10);

    /* 120 % used to wrap to 20 %. */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 120.0f, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x10, 0x00, 0x00);

    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, NAN, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x00, 0x10);

    /* Bad delays fall back to a start at count 0. */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 7.5f, -10.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x33, 0x01);
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 7.5f, 150.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x33, 0x01);
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 7.5f, NAN), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x00, 0x00, 0x33, 0x01);
}

static void test_pwm_wraps_into_next_period(void)
{
    /* Datasheet example: delay 90 % (3686), duty 20 % (819) -> OFF = 409 */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 0, 20.0f, 90.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x06, 0x66, 0x0E, 0x99, 0x01);
}

static void test_pwm_channel_addressing(void)
{
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 15, 7.5f, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0x42, 0x00, 0x00, 0x33, 0x01);

    mock_reset();
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev(), 16, 7.5f, 0.0f), PCA9685_ERR);
    CHECK_EQ(mock.writes, 0);
}

static void test_all_led_pwm(void)
{
    mock_set_running(0x21);     /* AI on, so all 4 bytes land */

    /* The README claimed delay = 0 returned an error here; it does not. */
    CHECK_EQ(pca9685_i2c_all_led_pwm_set(dev(), 7.5f, 0.0f), PCA9685_OK);
    CHECK_LAST_WRITE(0xFA, 0x00, 0x00, 0x33, 0x01);
    CHECK_EQ(mock.regs[0x06 + 4 * 9 + 2], 0x33);    /* LED9_OFF_L */
    CHECK_EQ(mock.regs[0x06 + 4 * 9 + 3], 0x01);    /* LED9_OFF_H */
}

static void test_led_set(void)
{
    CHECK_EQ(pca9685_i2c_led_set(dev(), 3, PCA9685_LED_ON), PCA9685_OK);
    CHECK_LAST_WRITE(0x12, 0x00, 0x10, 0x00, 0x00);

    CHECK_EQ(pca9685_i2c_led_set(dev(), 3, PCA9685_LED_OFF), PCA9685_OK);
    CHECK_LAST_WRITE(0x12, 0x00, 0x00, 0x00, 0x10);

    CHECK_EQ(pca9685_i2c_all_led_set(dev(), PCA9685_LED_OFF), PCA9685_OK);
    CHECK_LAST_WRITE(0xFA, 0x00, 0x00, 0x00, 0x10);

    mock_reset();
    CHECK_EQ(pca9685_i2c_led_set(dev(), 16, PCA9685_LED_ON), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_led_set(dev(), 0, (pca9685_led_state_t)2), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_all_led_set(dev(), (pca9685_led_state_t)2), PCA9685_ERR);
    CHECK_EQ(mock.writes, 0);
}

/* -------------------------------------------------------------------------- */
/* Frequency                                                                  */
/* -------------------------------------------------------------------------- */

static void test_pre_scale_values(void)
{
    /* Power-on state is asleep, so PRE_SCALE writes are accepted. */
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 50.0, 25e6), PCA9685_OK);
    CHECK_EQ(mock.regs[0xFE], 121);
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 24.0, 25e6), PCA9685_OK);
    CHECK_EQ(mock.regs[0xFE], 253);
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 1526.0, 25e6), PCA9685_OK);
    CHECK_EQ(mock.regs[0xFE], 3);

    /* Clamped to the hardware range 3..255. */
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 5000.0, 25e6), PCA9685_OK);
    CHECK_LAST_WRITE(0xFE, 3);
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 10.0, 25e6), PCA9685_OK);
    CHECK_LAST_WRITE(0xFE, 255);
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 1e-9, 25e6), PCA9685_OK);
    CHECK_LAST_WRITE(0xFE, 255);
}

static void test_pre_scale_rejects_bad_input(void)
{
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 0.0, 25e6), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), -50.0, 25e6), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), NAN, 25e6), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev(), 50.0, 0.0), PCA9685_ERR);
    CHECK_EQ(mock.writes, 0);
}

static void test_read_pre_scale(void)
{
    double f = -1.0;

    mock.regs[0xFE] = 121;
    CHECK_EQ(pca9685_i2c_read_pre_scale(dev(), &f, 25e6), PCA9685_OK);
    CHECK(fabs(f - 25e6 / (4096.0 * 122.0)) < 1e-9);

    /* A failed read must not hand back a frequency. */
    f = -1.0;
    mock.fail_read_at = mock.reads + 1;
    CHECK_EQ(pca9685_i2c_read_pre_scale(dev(), &f, 25e6), PCA9685_ERR);
    CHECK(f == -1.0);

    CHECK_EQ(pca9685_i2c_read_pre_scale(dev(), NULL, 25e6), PCA9685_ERR);
}

/* -------------------------------------------------------------------------- */
/* MODE1: sleep, restart, clock, auto-increment                               */
/* -------------------------------------------------------------------------- */

static void test_readme_init_sequence(void)
{
    pca9685_dev_t d;

    CHECK_EQ(pca9685_i2c_reset(), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_register(&d, I2C_ADDRESS_PCA9685,
                                  I2C_ALL_CALL_ADDRESS_PCA9685,
                                  I2C_SUB_ADDRESS_1_PCA9685,
                                  I2C_SUB_ADDRESS_2_PCA9685,
                                  I2C_SUB_ADDRESS_3_PCA9685), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_autoincrement(d, PCA9685_AUTOINCR_ON), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_sleep_mode(d, PCA9685_MODE_SLEEP), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_write_pre_scale(d, 50.0, 25e6), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_sleep_mode(d, PCA9685_MODE_NORMAL), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_restart(d), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_output_init(d, servo_output), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_led_pwm_set(d, 0, 7.5f, 0.0f), PCA9685_OK);

    CHECK_EQ(mock.regs[0x00], 0x21);        /* awake, AI, ALLCALL */
    CHECK_EQ(mock.regs[0x01], 0x04);        /* totem pole */
    CHECK_EQ(mock.regs[0xFE], 121);         /* 50 Hz */
    CHECK_EQ(mock.regs[0x06], 0x00);        /* LED0_ON_L  */
    CHECK_EQ(mock.regs[0x07], 0x00);        /* LED0_ON_H  */
    CHECK_EQ(mock.regs[0x08], 0x33);        /* LED0_OFF_L */
    CHECK_EQ(mock.regs[0x09], 0x01);        /* LED0_OFF_H */
    CHECK_EQ(mock.ignored_prescale_writes, 0);
    CHECK_EQ(mock.bad_restarts, 0);
}

static void test_sleep_wake_keeps_pending_restart(void)
{
    /* Running with AI on; going to sleep leaves RESTART set. */
    mock_set_running(0x21);
    CHECK_EQ(pca9685_i2c_sleep_mode(dev(), PCA9685_MODE_SLEEP), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0xB1);

    /* Changing other MODE1 bits must not fire the restart. */
    CHECK_EQ(pca9685_i2c_autoincrement(dev(), PCA9685_AUTOINCR_ON), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_allcall_address_resp(dev(), PCA9685_ADDR_RESPOND), PCA9685_OK);

    /* Waking must not fire it either: the oscillator is not stable yet.
     * The old code copied RESTART=1 back here and restarted too early. */
    CHECK_EQ(pca9685_i2c_sleep_mode(dev(), PCA9685_MODE_NORMAL), PCA9685_OK);
    CHECK_EQ(mock.restarts, 0);
    CHECK_EQ(mock.regs[0x00], 0xA1);        /* awake, restart still pending */

    CHECK_EQ(pca9685_i2c_restart(dev()), PCA9685_OK);
    CHECK_EQ(mock.restarts, 1);
    CHECK_EQ(mock.bad_restarts, 0);
    CHECK_EQ(mock.regs[0x00], 0x21);
}

static void test_restart_from_sleep(void)
{
    mock_set_running(0x21);
    CHECK_EQ(pca9685_i2c_sleep_mode(dev(), PCA9685_MODE_SLEEP), PCA9685_OK);
    mock.n_events = 0;

    CHECK_EQ(pca9685_i2c_restart(dev()), PCA9685_OK);
    CHECK_EQ(mock.restarts, 1);
    CHECK_EQ(mock.bad_restarts, 0);
    CHECK_EQ(mock.regs[0x00], 0x21);

    /* read MODE1, wake, wait, restart */
    CHECK_EQ(mock.n_events, 4);
    CHECK_EQ(mock.events[0].type, MOCK_EV_READ);
    CHECK_EQ(mock.events[1].type, MOCK_EV_WRITE);
    CHECK_EQ(mock.events[1].data[1], 0x21);
    CHECK_EQ(mock.events[2].type, MOCK_EV_DELAY);
    CHECK(mock.events[2].ms >= 1);
    CHECK_EQ(mock.events[3].type, MOCK_EV_WRITE);
    CHECK_EQ(mock.events[3].data[1], 0xA1);
}

static void test_external_clock_sequence(void)
{
    mock_set_running(0x21);
    CHECK_EQ(pca9685_i2c_clock(dev(), PCA9685_CLK_EXTERNAL), PCA9685_OK);
    CHECK_EQ(mock.extclk_violations, 0);
    CHECK(mock.regs[0x00] & 0x40);          /* EXTCLK */
    CHECK(mock.regs[0x00] & 0x10);          /* still asleep */
    CHECK_EQ(mock.restarts, 0);

    CHECK_EQ(pca9685_i2c_clock(dev(), PCA9685_CLK_INTERNAL), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_clock(dev(), (pca9685_auto_extclk_t)2), PCA9685_ERR);
}

static void test_autoincrement(void)
{
    mock_set_running(0x01);
    CHECK_EQ(pca9685_i2c_autoincrement(dev(), PCA9685_AUTOINCR_ON), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0x21);
    CHECK_EQ(pca9685_i2c_autoincrement(dev(), PCA9685_AUTOINCR_OFF), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0x01);

    int writes = mock.writes;
    CHECK_EQ(pca9685_i2c_autoincrement(dev(), (pca9685_auto_incr_t)2), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_sleep_mode(dev(), (pca9685_sleep_mode_t)2), PCA9685_ERR);
    CHECK_EQ(mock.writes, writes);
}

/* -------------------------------------------------------------------------- */
/* MODE2 and addressing                                                       */
/* -------------------------------------------------------------------------- */

static void test_output_init(void)
{
    pca9685_output_t out = {
        .outdrv = PCA9685_OUTPUT_OPEN_DRAIN,
        .outne  = PCA9685_OUTPUT_HIGH_IMPEDANCE,
        .och    = PCA9685_CH_ONACK,
        .invrt  = PCA9685_OUTPUT_INVERT,
    };

    mock.regs[0x01] = 0xA4;     /* upper bits must survive */
    CHECK_EQ(pca9685_i2c_output_init(dev(), out), PCA9685_OK);
    CHECK_EQ(mock.regs[0x01], 0xBA);
    CHECK_EQ(pca9685_i2c_output_init(dev(), servo_output), PCA9685_OK);
    CHECK_EQ(mock.regs[0x01], 0xA4);
}

static void test_sub_addr_response_bits(void)
{
    mock_set_running(0x01);
    CHECK_EQ(pca9685_i2c_sub_addr_resp(dev(), PCA9685_SUB_ADDR_1, PCA9685_ADDR_RESPOND), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0x09);
    CHECK_EQ(pca9685_i2c_sub_addr_resp(dev(), PCA9685_SUB_ADDR_2, PCA9685_ADDR_RESPOND), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0x0D);
    CHECK_EQ(pca9685_i2c_sub_addr_resp(dev(), PCA9685_SUB_ADDR_3, PCA9685_ADDR_RESPOND), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0x0F);
    CHECK_EQ(pca9685_i2c_sub_addr_resp(dev(), PCA9685_SUB_ADDR_1, PCA9685_ADDR_NORESPOND), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0x07);

    CHECK_EQ(pca9685_i2c_allcall_address_resp(dev(), PCA9685_ADDR_NORESPOND), PCA9685_OK);
    CHECK_EQ(mock.regs[0x00], 0x06);

    int writes = mock.writes;
    CHECK_EQ(pca9685_i2c_sub_addr_resp(dev(), (pca9685_subaddr_no_t)0, PCA9685_ADDR_RESPOND), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_sub_addr_resp(dev(), (pca9685_subaddr_no_t)4, PCA9685_ADDR_RESPOND), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_allcall_address_resp(dev(), (pca9685_addr_resp_t)2), PCA9685_ERR);
    CHECK_EQ(mock.writes, writes);
}

static void test_register_programs_addresses(void)
{
    pca9685_dev_t d;

    mock.addr = 0x41;
    CHECK_EQ(pca9685_i2c_register(&d, 0x41, 0x60, 0x61, 0x62, 0x63), PCA9685_OK);
    CHECK_EQ(d.i2c_addr, 0x41);
    CHECK_EQ(d.sub_addr_3, 0x63);
    CHECK_EQ(mock.regs[0x05], 0xC0);
    CHECK_EQ(mock.regs[0x02], 0xC2);
    CHECK_EQ(mock.regs[0x03], 0xC4);
    CHECK_EQ(mock.regs[0x04], 0xC6);
}

static void test_register_reports_errors(void)
{
    pca9685_dev_t d;

    CHECK_EQ(pca9685_i2c_register(NULL, 0x40, 0x70, 0x71, 0x72, 0x74), PCA9685_ERR);

    /* Invalid All Call address: reported, the sub-addresses are still written. */
    CHECK_EQ(pca9685_i2c_register(&d, 0x40, 0x80, 0x61, 0x62, 0x63), PCA9685_ERR);
    CHECK_EQ(mock.regs[0x05], 0xE0);
    CHECK_EQ(mock.regs[0x02], 0xC2);

    /* Nothing answers on the bus. */
    mock_reset();
    mock.addr = 0x50;
    CHECK_EQ(pca9685_i2c_register(&d, 0x40, 0x70, 0x71, 0x72, 0x74), PCA9685_ERR);

    mock_reset();
    CHECK_EQ(pca9685_i2c_register(&d, 0x80, 0x70, 0x71, 0x72, 0x74), PCA9685_ERR);
    CHECK_EQ(mock.writes, 0);
}

static void test_read_addresses(void)
{
    uint8_t a = 0xAA;

    CHECK_EQ(pca9685_i2c_read_allcall_addr(dev(), &a), PCA9685_OK);
    CHECK_EQ(a, 0x70);
    CHECK_EQ(pca9685_i2c_read_sub_addr(dev(), PCA9685_SUB_ADDR_3, &a), PCA9685_OK);
    CHECK_EQ(a, 0x74);
    CHECK_EQ(pca9685_i2c_write_sub_addr(dev(), PCA9685_SUB_ADDR_2, 0x55), PCA9685_OK);
    CHECK_EQ(pca9685_i2c_read_sub_addr(dev(), PCA9685_SUB_ADDR_2, &a), PCA9685_OK);
    CHECK_EQ(a, 0x55);

    a = 0xAA;
    mock.fail_read_at = mock.reads + 1;
    CHECK_EQ(pca9685_i2c_read_allcall_addr(dev(), &a), PCA9685_ERR);
    CHECK_EQ(a, 0xAA);

    CHECK_EQ(pca9685_i2c_read_sub_addr(dev(), (pca9685_subaddr_no_t)0, &a), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_write_sub_addr(dev(), (pca9685_subaddr_no_t)4, 0x10), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_write_sub_addr(dev(), PCA9685_SUB_ADDR_1, 0x80), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_write_allcall_addr(dev(), 0x80), PCA9685_ERR);
}

/* -------------------------------------------------------------------------- */
/* Errors and reset                                                           */
/* -------------------------------------------------------------------------- */

static void test_errors_are_always_pca9685_err(void)
{
    uint8_t mode = 0xAA;

    /* Second write of the restart sequence fails. */
    mock_set_running(0x21);
    mock.fail_write_at = 2;
    CHECK_EQ(pca9685_i2c_restart(dev()), PCA9685_ERR);

    mock_reset();
    mock.fail_write_at = 1;
    CHECK_EQ(pca9685_i2c_restart(dev()), PCA9685_ERR);
    CHECK_EQ(mock_count(MOCK_EV_DELAY), 0);     /* gave up before waiting */

    mock_reset();
    mock.fail_read_at = 1;
    CHECK_EQ(pca9685_i2c_sleep_mode(dev(), PCA9685_MODE_NORMAL), PCA9685_ERR);
    CHECK_EQ(mock.writes, 0);

    mock_reset();
    mock.fail_read_at = 1;
    CHECK_EQ(pca9685_i2c_read_mode_1(dev(), &mode), PCA9685_ERR);
    CHECK_EQ(mode, 0xAA);

    mock_reset();
    mock.fail_write_at = 1;
    CHECK_EQ(pca9685_i2c_reset(), PCA9685_ERR);

    /* Wrong address: the chip does not acknowledge. */
    mock_reset();
    pca9685_dev_t other = dev();
    other.i2c_addr = 0x41;
    CHECK_EQ(pca9685_i2c_led_pwm_set(other, 0, 7.5f, 0.0f), PCA9685_ERR);
}

static void test_reset(void)
{
    mock_set_running(0x21);
    mock.regs[0xFE] = 121;

    CHECK_EQ(pca9685_i2c_reset(), PCA9685_OK);
    const mock_event_t *ev = mock_last_write();
    CHECK(ev != NULL);
    CHECK_EQ(ev->addr, 0x00);               /* General Call */
    CHECK_LAST_WRITE(0x06);
    CHECK_EQ(mock.regs[0x00], 0x11);        /* back to power-on state */
    CHECK_EQ(mock.regs[0xFE], 0x1E);
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    RUN(test_pwm_servo_center);
    RUN(test_pwm_with_delay);
    RUN(test_pwm_high_time_independent_of_delay);
    RUN(test_pwm_zero_is_full_off);
    RUN(test_pwm_hundred_is_full_on);
    RUN(test_pwm_out_of_range_is_clamped);
    RUN(test_pwm_wraps_into_next_period);
    RUN(test_pwm_channel_addressing);
    RUN(test_all_led_pwm);
    RUN(test_led_set);

    RUN(test_pre_scale_values);
    RUN(test_pre_scale_rejects_bad_input);
    RUN(test_read_pre_scale);

    RUN(test_readme_init_sequence);
    RUN(test_sleep_wake_keeps_pending_restart);
    RUN(test_restart_from_sleep);
    RUN(test_external_clock_sequence);
    RUN(test_autoincrement);

    RUN(test_output_init);
    RUN(test_sub_addr_response_bits);
    RUN(test_register_programs_addresses);
    RUN(test_register_reports_errors);
    RUN(test_read_addresses);

    RUN(test_errors_are_always_pca9685_err);
    RUN(test_reset);

    printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
