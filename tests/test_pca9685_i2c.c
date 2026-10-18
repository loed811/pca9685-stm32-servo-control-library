/*
 * Host-side unit tests for pca9685_i2c.c.
 *
 * The driver is linked against fake_pca9685_hal.c, which replaces the
 * pca9685_i2c_hal_* layer with an in-memory register file and a transaction
 * log, so every byte the driver would send over I2C can be checked.
 */
#include <string.h>

#include "fake_pca9685_hal.h"
#include "pca9685_i2c.h"
#include "test_framework.h"

TF_DEFINE_GLOBALS;

#define OSC_HZ 25000000.0

static pca9685_dev_t dev = { .i2c_addr = 0x41 };

static void test_setup(void)
{
    fake_reset();
}

/*
 * Fill the stack below the caller with 0xFF so that bytes the driver leaves
 * uninitialised in its local buffers show up deterministically in the log.
 */
static void __attribute__((noinline)) poison_stack(void)
{
    volatile uint8_t buf[4096];
    for (size_t i = 0; i < sizeof(buf); i++)
        buf[i] = 0xFF;
}

static uint16_t tx_on(const fake_tx_t *tx)  { return tx->bytes[1] | (tx->bytes[2] << 8); }
static uint16_t tx_off(const fake_tx_t *tx) { return tx->bytes[3] | (tx->bytes[4] << 8); }

/* High time in counter ticks for an ON/OFF pair (counter wraps at 4096). */
static int tx_width(const fake_tx_t *tx)
{
    return ((tx_off(tx) & 0x0FFF) - (tx_on(tx) & 0x0FFF) + 4096) % 4096;
}

/* Bits 7..5 of LEDn_ON_H / LEDn_OFF_H are reserved and must stay 0. */
static int tx_reserved_clear(const fake_tx_t *tx)
{
    return (tx->bytes[2] & 0xE0) == 0 && (tx->bytes[4] & 0xE0) == 0;
}

/* ------------------------------------------------------------------------ */
/* PRE_SCALE                                                                */
/* ------------------------------------------------------------------------ */

static void test_prescale_from_frequency(void)
{
    static const struct { double hz; uint8_t prescale; } cases[] = {
        {   50.0, 121 },    /* servo frequency */
        {   60.0, 101 },
        {  200.0,  30 },    /* datasheet example: 0x1E */
        { 1000.0,   5 },
        { 1526.0,   3 },    /* datasheet maximum frequency, minimum prescale */
        {   24.0, 253 },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        fake_reset();
        CHECK_EQ(pca9685_i2c_write_pre_scale(dev, cases[i].hz, OSC_HZ), PCA9685_OK);
        const fake_tx_t *tx = fake_last_write();
        CHECK(tx != NULL);
        CHECK_EQ(tx->addr, 0x41);
        CHECK_EQ(tx->count, 2);
        CHECK_EQ(tx->bytes[0], REG_PRE_SCALE);
        CHECK_EQ(tx->bytes[1], cases[i].prescale);
    }
}

static void test_prescale_external_clock(void)
{
    /* 50 Hz from a 16 MHz external clock: round(78.125) - 1 = 77 */
    pca9685_i2c_write_pre_scale(dev, 50.0, 16000000.0);
    CHECK_EQ(fake_last_write()->bytes[1], 77);
}

static void test_prescale_clamped_to_valid_range(void)
{
    /* Datasheet: PRE_SCALE is limited to 3..255. Out-of-range frequencies
     * must be clamped or rejected instead of truncated/wrapped. */
    int16_t err = pca9685_i2c_write_pre_scale(dev, 10.0, OSC_HZ);
    KNOWN_BUG("10 Hz (prescale 609) is not clamped to 255 / rejected",
              err == PCA9685_ERR || fake_last_write()->bytes[1] == 255);

    fake_reset();
    err = pca9685_i2c_write_pre_scale(dev, 5000.0, OSC_HZ);
    KNOWN_BUG("5000 Hz (prescale 0) is not clamped to 3 / rejected",
              err == PCA9685_ERR || fake_last_write()->bytes[1] == 3);
}

static void test_prescale_read_back_frequency(void)
{
    double hz = 0;
    fake.regs[REG_PRE_SCALE] = 121;
    CHECK_EQ(pca9685_i2c_read_pre_scale(dev, &hz, OSC_HZ), PCA9685_OK);
    CHECK_NEAR(hz, OSC_HZ / (4096.0 * 122), 1e-9);
    CHECK_NEAR(hz, 50.0, 0.1);
    CHECK_EQ(fake.log[0].reg, REG_PRE_SCALE);
    CHECK_EQ(fake.log[0].addr, 0x41);
}

static void test_prescale_round_trip(void)
{
    static const double freqs[] = { 50.0, 60.0, 100.0, 200.0, 1000.0 };
    for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
        double hz = 0;
        fake_reset();
        pca9685_i2c_write_pre_scale(dev, freqs[i], OSC_HZ);
        pca9685_i2c_read_pre_scale(dev, &hz, OSC_HZ);
        /* Quantisation error of one prescale step. */
        CHECK_NEAR(hz, freqs[i], freqs[i] * freqs[i] * 4096.0 / OSC_HZ);
    }
}

static void test_prescale_read_error(void)
{
    double hz = 0;
    fake.fail_reads = 1;
    CHECK_EQ(pca9685_i2c_read_pre_scale(dev, &hz, OSC_HZ), PCA9685_ERR);
}

/* ------------------------------------------------------------------------ */
/* PWM duty cycle -> ticks                                                  */
/* ------------------------------------------------------------------------ */

static void test_pwm_channel_register_mapping(void)
{
    for (uint8_t ch = 0; ch < 16; ch++) {
        fake_reset();
        CHECK_EQ(pca9685_i2c_led_pwm_set(dev, ch, 50.0f, 10.0f), PCA9685_OK);
        const fake_tx_t *tx = fake_last_write();
        CHECK_EQ(tx->addr, 0x41);
        CHECK_EQ(tx->count, 5);
        CHECK_EQ(tx->bytes[0], 0x06 + 4 * ch);  /* LEDn_ON_L */
        /* Register file: ON_L..OFF_H land in the four channel registers. */
        CHECK_EQ(fake.regs[0x06 + 4 * ch + 3], tx->bytes[4]);
    }
}

static void test_pwm_duty_with_delay(void)
{
    /* 50 % duty, 10 % delay: delay = 410 ticks, high time = 2048 ticks. */
    pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 10.0f);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx_on(tx), 409);
    CHECK_EQ(tx_off(tx), 2457);
    CHECK_EQ(tx_width(tx), 2048);
    CHECK(tx_reserved_clear(tx));

    /* 25 % duty, 50 % delay: 1024 ticks starting at tick 2048. */
    fake_reset();
    pca9685_i2c_led_pwm_set(dev, 5, 25.0f, 50.0f);
    tx = fake_last_write();
    CHECK_EQ(tx_on(tx), 2047);
    CHECK_EQ(tx_off(tx), 3071);
    CHECK_EQ(tx_width(tx), 1024);
}

static void test_pwm_duty_rounding(void)
{
    /* 7.5 % = 307.2 ticks -> 307 ; 12.5 % = 512 ticks exactly. */
    pca9685_i2c_led_pwm_set(dev, 0, 7.5f, 25.0f);
    CHECK_EQ(tx_width(fake_last_write()), 307);
    fake_reset();
    pca9685_i2c_led_pwm_set(dev, 0, 12.5f, 25.0f);
    CHECK_EQ(tx_width(fake_last_write()), 512);
}

static void test_pwm_zero_delay_width(void)
{
    /* Typical servo use: 1.5 ms pulse at 50 Hz = 7.5 % = 307 ticks, no delay.
     * With delay 0 the driver writes ON = 0 but still subtracts one from OFF,
     * so the pulse is one tick short. */
    pca9685_i2c_led_pwm_set(dev, 0, 7.5f, 0.0f);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx_on(tx), 0);
    KNOWN_BUG("delay=0: pulse is 1 tick shorter than requested (306 vs 307)",
              tx_width(tx) == 307);

    fake_reset();
    pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 0.0f);
    KNOWN_BUG("delay=0: 50 % duty gives 2047 ticks instead of 2048",
              tx_width(fake_last_write()) == 2048);
}

static void test_pwm_wraparound(void)
{
    /* 50 % duty with 75 % delay: ON at 3072, OFF must wrap to 1024. */
    pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 75.0f);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx_on(tx), 3071);
    KNOWN_BUG("OFF wrap-around (delay+duty > 100 %) computes 4096-off "
              "(negative) instead of off-4096",
              tx_width(tx) == 2048);
    KNOWN_BUG("OFF wrap-around sets reserved bits in LEDn_OFF_H",
              tx_reserved_clear(tx));
}

static void test_pwm_zero_duty(void)
{
    pca9685_i2c_led_pwm_set(dev, 0, 0.0f, 0.0f);
    const fake_tx_t *tx = fake_last_write();
    KNOWN_BUG("0 % duty writes 0xFFFF to LEDn_OFF (reserved bits set)",
              tx_reserved_clear(tx));
}

static void test_pwm_duty_clamping(void)
{
    int16_t err = pca9685_i2c_led_pwm_set(dev, 0, 150.0f, 0.0f);
    const fake_tx_t *tx = fake_last_write();
    KNOWN_BUG("duty > 100 % is neither clamped nor rejected",
              err == PCA9685_ERR || (tx_reserved_clear(tx) && tx_width(tx) >= 4095));

    fake_reset();
    err = pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 120.0f);
    tx = fake_last_write();
    KNOWN_BUG("delay > 100 % is neither clamped nor rejected",
              err == PCA9685_ERR || tx_reserved_clear(tx));
}

static void test_pwm_channel_bounds(void)
{
    int16_t err = pca9685_i2c_led_pwm_set(dev, 16, 50.0f, 10.0f);
    KNOWN_BUG("led_pwm_set(led_no=16) is not rejected (writes reserved reg 0x46)",
              err == PCA9685_ERR && fake.n_writes == 0);

    fake_reset();
    err = pca9685_i2c_led_pwm_set(dev, 61, 50.0f, 10.0f);
    KNOWN_BUG("led_pwm_set(led_no=61) is not rejected (aliases ALL_LED 0xFA)",
              err == PCA9685_ERR && fake.n_writes == 0);
}

static void test_all_led_pwm(void)
{
    CHECK_EQ(pca9685_i2c_all_led_pwm_set(dev, 50.0f, 10.0f), PCA9685_OK);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx->bytes[0], REG_ALL_LED);
    CHECK_EQ(tx->count, 5);
    CHECK_EQ(tx_on(tx), 409);
    CHECK_EQ(tx_off(tx), 2457);
}

static void test_all_led_pwm_zero_delay(void)
{
    /* led_on_tm == led_off_tm is true whenever delay == 0, so the most common
     * call (no phase delay) is rejected. */
    int16_t err = pca9685_i2c_all_led_pwm_set(dev, 7.5f, 0.0f);
    KNOWN_BUG("all_led_pwm_set(delay=0) returns PCA9685_ERR and writes nothing",
              err == PCA9685_OK && fake.n_writes == 1);
}

/* ------------------------------------------------------------------------ */
/* Full ON / OFF                                                            */
/* ------------------------------------------------------------------------ */

static void test_led_set_on(void)
{
    poison_stack();
    CHECK_EQ(pca9685_i2c_led_set(dev, 3, PCA9685_LED_ON), PCA9685_OK);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx->count, 5);
    CHECK_EQ(tx->bytes[0], 0x06 + 4 * 3);
    CHECK_EQ(tx->bytes[2] & 0x10, 0x10);        /* LEDn_ON_H full-ON bit */
    KNOWN_BUG("led_set(ON) sends uninitialised ON_L/OFF_L/OFF_H bytes "
              "(OFF_H bit 4 may turn the LED off)",
              tx->bytes[1] == 0 && tx->bytes[3] == 0 && tx->bytes[4] == 0);
}

static void test_led_set_off(void)
{
    poison_stack();
    CHECK_EQ(pca9685_i2c_led_set(dev, 15, PCA9685_LED_OFF), PCA9685_OK);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx->bytes[0], 0x42);
    CHECK_EQ(tx->bytes[4], 0x10);               /* full-OFF wins over full-ON */
    KNOWN_BUG("led_set(OFF) sends uninitialised ON_L/OFF_L bytes",
              tx->bytes[1] == 0 && tx->bytes[3] == 0);
}

static void test_led_set_channel_bounds(void)
{
    int16_t err = pca9685_i2c_led_set(dev, 16, PCA9685_LED_ON);
    KNOWN_BUG("led_set(led_no=16) is not rejected",
              err == PCA9685_ERR && fake.n_writes == 0);
}

static void test_all_led_set(void)
{
    poison_stack();
    CHECK_EQ(pca9685_i2c_all_led_set(dev, PCA9685_LED_OFF), PCA9685_OK);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx->bytes[0], REG_ALL_LED);
    CHECK_EQ(tx->bytes[4], 0x10);
    KNOWN_BUG("all_led_set sends uninitialised bytes",
              tx->bytes[1] == 0 && tx->bytes[3] == 0);
}

/* ------------------------------------------------------------------------ */
/* MODE1 / MODE2                                                            */
/* ------------------------------------------------------------------------ */

static void test_clock(void)
{
    fake.regs[REG_MODE_1] = 0x01;
    CHECK_EQ(pca9685_i2c_clock(dev, PCA9685_CLK_EXTERNAL), PCA9685_OK);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x41);
    CHECK_EQ(fake_last_write()->bytes[0], REG_MODE_1);

    fake.regs[REG_MODE_1] = 0xFF;
    pca9685_i2c_clock(dev, PCA9685_CLK_INTERNAL);
    CHECK_EQ(fake.regs[REG_MODE_1], 0xBF);
}

static void test_autoincrement(void)
{
    fake.regs[REG_MODE_1] = 0x11;
    CHECK_EQ(pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_ON), PCA9685_OK);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x31);
    pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_OFF);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x11);
}

static void test_sleep_mode(void)
{
    fake.regs[REG_MODE_1] = 0x01;
    CHECK_EQ(pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP), PCA9685_OK);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x11);
    pca9685_i2c_sleep_mode(dev, PCA9685_MODE_NORMAL);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x01);
    CHECK(fake.delay_total_ms >= 1);    /* oscillator needs 500 us */
}

static void test_read_modify_write_read_error(void)
{
    pca9685_output_t out = { 0 };
    fake.fail_reads = 1;
    CHECK_EQ(pca9685_i2c_clock(dev, PCA9685_CLK_EXTERNAL), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_autoincrement(dev, PCA9685_AUTOINCR_ON), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_output_init(dev, out), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_1, PCA9685_ADDR_RESPOND), PCA9685_ERR);
    CHECK_EQ(pca9685_i2c_allcall_address_resp(dev, PCA9685_ADDR_NORESPOND), PCA9685_ERR);
    CHECK_EQ(fake.n_writes, 0);
}

static void test_write_error_propagates(void)
{
    fake.fail_writes = 1;
    CHECK(pca9685_i2c_led_pwm_set(dev, 0, 50.0f, 10.0f) != PCA9685_OK);
    CHECK(pca9685_i2c_write_pre_scale(dev, 50.0, OSC_HZ) != PCA9685_OK);
    CHECK(pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP) != PCA9685_OK);
    CHECK(pca9685_i2c_reset() != PCA9685_OK);
}

static void test_restart(void)
{
    /* MODE1 after sleeping with PWM active: RESTART | SLEEP | ALLCALL. */
    fake.regs[REG_MODE_1] = 0x91;
    CHECK_EQ(pca9685_i2c_restart(dev), PCA9685_OK);
    CHECK_EQ(fake.n_writes, 2);
    const fake_tx_t *first = fake_write(0);
    const fake_tx_t *second = fake_write(1);
    CHECK_EQ(first->bytes[0], REG_MODE_1);
    KNOWN_BUG("restart: first write does not clear SLEEP (writes 0x91, "
              "expected 0x81)",
              (first->bytes[1] & 0x10) == 0);
    CHECK(fake.delay_total_ms >= 1);
    CHECK_EQ(second->bytes[1] & 0x80, 0x80);    /* write 1 to RESTART */
    KNOWN_BUG("restart: SLEEP still set after the sequence",
              (fake.regs[REG_MODE_1] & 0x10) == 0);
}

static void test_restart_read_error(void)
{
    /* The fake leaves 0xFF in 'mode' (RESTART bit set) on a failed read. */
    fake.fail_reads = 1;
    CHECK_EQ(pca9685_i2c_restart(dev), PCA9685_ERR);
    KNOWN_BUG("restart: a failed MODE1 read still writes MODE1 from an "
              "invalid value ('&&' should be '||')",
              fake.n_writes == 0);
}

static void test_output_init(void)
{
    pca9685_output_t out = {
        .outdrv = PCA9685_OUTPUT_TOTEM_POLE,
        .outne = PCA9685_OUTPUT_LOW,
        .och = PCA9685_CH_ONSTOP,
        .invrt = PCA9685_OUTPUT_NOTINVERT,
    };
    fake.regs[REG_MODE_2] = 0xE0;   /* reserved bits must be preserved */
    CHECK_EQ(pca9685_i2c_output_init(dev, out), PCA9685_OK);
    CHECK_EQ(fake_last_write()->bytes[0], REG_MODE_2);
    CHECK_EQ(fake.regs[REG_MODE_2], 0xE4);

    out.invrt = PCA9685_OUTPUT_INVERT;
    out.och = PCA9685_CH_ONACK;
    pca9685_i2c_output_init(dev, out);
    CHECK_EQ(fake.regs[REG_MODE_2], 0xFC);
}

static void test_output_init_outne(void)
{
    pca9685_output_t out = {
        .outdrv = PCA9685_OUTPUT_OPEN_DRAIN,
        .outne = PCA9685_OUTPUT_HIGH,
    };
    pca9685_i2c_output_init(dev, out);
    KNOWN_BUG("output_init: OUTNE shifted by 1 (bits 1..0 expected); "
              "OUTPUT_HIGH writes 0x02 instead of 0x01",
              fake.regs[REG_MODE_2] == 0x01);

    out.outne = PCA9685_OUTPUT_HIGH_IMPEDANCE;
    pca9685_i2c_output_init(dev, out);
    KNOWN_BUG("output_init: OUTPUT_HIGH_IMPEDANCE sets OUTDRV (0x04) "
              "instead of OUTNE=10 (0x02)",
              fake.regs[REG_MODE_2] == 0x02);
}

/* ------------------------------------------------------------------------ */
/* Addresses, reset, register                                               */
/* ------------------------------------------------------------------------ */

static void test_reset(void)
{
    CHECK_EQ(pca9685_i2c_reset(), PCA9685_OK);
    const fake_tx_t *tx = fake_last_write();
    CHECK_EQ(tx->addr, I2C_GEN_CALL_ADDRESS_PCA9685);
    CHECK_EQ(tx->count, 1);
    CHECK_EQ(tx->bytes[0], SWRST);
    CHECK_EQ(fake.delay_calls, 1);
}

static void test_allcall_addr(void)
{
    uint8_t addr = 0;
    CHECK_EQ(pca9685_i2c_write_allcall_addr(dev, 0x70), PCA9685_OK);
    CHECK_EQ(fake.regs[REG_ALLCALLADR], 0xE0);
    CHECK_EQ(pca9685_i2c_read_allcall_addr(dev, &addr), PCA9685_OK);
    CHECK_EQ(addr, 0x70);
}

static void test_sub_addr(void)
{
    static const uint8_t vals[] = { 0x71, 0x72, 0x74 };
    for (int n = 1; n <= 3; n++) {
        uint8_t addr = 0;
        CHECK_EQ(pca9685_i2c_write_sub_addr(dev, (pca9685_subaddr_no_t)n, vals[n - 1]), PCA9685_OK);
        CHECK_EQ(fake_last_write()->bytes[0], 0x01 + n);   /* SUBADR1..3 = 0x02..0x04 */
        CHECK_EQ(fake.regs[0x01 + n], vals[n - 1] << 1);
        CHECK_EQ(pca9685_i2c_read_sub_addr(dev, (pca9685_subaddr_no_t)n, &addr), PCA9685_OK);
        CHECK_EQ(addr, vals[n - 1]);
    }
}

static void test_sub_addr_resp(void)
{
    /* MODE1: SUB1 = bit 3, SUB2 = bit 2, SUB3 = bit 1. */
    fake.regs[REG_MODE_1] = 0x01;
    pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_1, PCA9685_ADDR_RESPOND);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x09);
    pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_2, PCA9685_ADDR_RESPOND);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x0D);
    pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_3, PCA9685_ADDR_RESPOND);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x0F);
    pca9685_i2c_sub_addr_resp(dev, PCA9685_SUB_ADDR_1, PCA9685_ADDR_NORESPOND);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x07);
}

static void test_allcall_resp(void)
{
    fake.regs[REG_MODE_1] = 0x21;
    pca9685_i2c_allcall_address_resp(dev, PCA9685_ADDR_NORESPOND);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x20);
    pca9685_i2c_allcall_address_resp(dev, PCA9685_ADDR_RESPOND);
    CHECK_EQ(fake.regs[REG_MODE_1], 0x21);
}

static void test_register(void)
{
    pca9685_dev_t d;
    memset(&d, 0, sizeof(d));
    pca9685_i2c_register(&d, 0x42, 0x70, 0x71, 0x72, 0x74);
    CHECK_EQ(d.i2c_addr, 0x42);
    CHECK_EQ(d.allcall_addr, 0x70);
    CHECK_EQ(d.sub_addr_3, 0x74);
    CHECK_EQ(fake.n_writes, 4);
    for (int i = 0; i < 4; i++)
        CHECK_EQ(fake_write(i)->addr, 0x42);
    CHECK_EQ(fake_write(0)->bytes[0], REG_ALLCALLADR);
    CHECK_EQ(fake.regs[REG_ALLCALLADR], 0xE0);
    CHECK_EQ(fake.regs[0x02], 0xE2);
    CHECK_EQ(fake.regs[0x03], 0xE4);
    CHECK_EQ(fake.regs[0x04], 0xE8);
}

int main(void)
{
    printf("== pca9685_i2c.c (fake pca9685_i2c_hal layer) ==\n");

    RUN_TEST(test_prescale_from_frequency);
    RUN_TEST(test_prescale_external_clock);
    RUN_TEST(test_prescale_clamped_to_valid_range);
    RUN_TEST(test_prescale_read_back_frequency);
    RUN_TEST(test_prescale_round_trip);
    RUN_TEST(test_prescale_read_error);

    RUN_TEST(test_pwm_channel_register_mapping);
    RUN_TEST(test_pwm_duty_with_delay);
    RUN_TEST(test_pwm_duty_rounding);
    RUN_TEST(test_pwm_zero_delay_width);
    RUN_TEST(test_pwm_wraparound);
    RUN_TEST(test_pwm_zero_duty);
    RUN_TEST(test_pwm_duty_clamping);
    RUN_TEST(test_pwm_channel_bounds);
    RUN_TEST(test_all_led_pwm);
    RUN_TEST(test_all_led_pwm_zero_delay);

    RUN_TEST(test_led_set_on);
    RUN_TEST(test_led_set_off);
    RUN_TEST(test_led_set_channel_bounds);
    RUN_TEST(test_all_led_set);

    RUN_TEST(test_clock);
    RUN_TEST(test_autoincrement);
    RUN_TEST(test_sleep_mode);
    RUN_TEST(test_read_modify_write_read_error);
    RUN_TEST(test_write_error_propagates);
    RUN_TEST(test_restart);
    RUN_TEST(test_restart_read_error);
    RUN_TEST(test_output_init);
    RUN_TEST(test_output_init_outne);

    RUN_TEST(test_reset);
    RUN_TEST(test_allcall_addr);
    RUN_TEST(test_sub_addr);
    RUN_TEST(test_sub_addr_resp);
    RUN_TEST(test_allcall_resp);
    RUN_TEST(test_register);

    TF_SUMMARY_AND_EXIT("test_pca9685_i2c");
}
