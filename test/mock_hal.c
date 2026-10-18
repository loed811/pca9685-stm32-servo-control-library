#include <string.h>

#include "mock_hal.h"
#include "pca9685_i2c_hal.h"

#define M1_RESTART  0x80
#define M1_EXTCLK   0x40
#define M1_AI       0x20
#define M1_SLEEP    0x10
#define M1_SUB1     0x08
#define M1_SUB2     0x04
#define M1_SUB3     0x02
#define M1_ALLCALL  0x01

mock_t mock;

static void power_on(void)
{
    memset(mock.regs, 0, sizeof mock.regs);
    mock.regs[0x00] = M1_SLEEP | M1_ALLCALL;    /* MODE1 */
    mock.regs[0x01] = 0x04;                     /* MODE2: OUTDRV */
    mock.regs[0x02] = 0xE2;                     /* SUBADR1 = 0x71 */
    mock.regs[0x03] = 0xE4;                     /* SUBADR2 = 0x72 */
    mock.regs[0x04] = 0xE8;                     /* SUBADR3 = 0x74 */
    mock.regs[0x05] = 0xE0;                     /* ALLCALLADR = 0x70 */
    for (int ch = 0; ch < 16; ch++)
        mock.regs[0x06 + 4 * ch + 3] = 0x10;    /* LEDn_OFF_H: full OFF */
    mock.regs[0xFE] = 0x1E;                     /* PRE_SCALE: 200 Hz */
    mock.osc_stable = 0;
}

void mock_reset(void)
{
    memset(&mock, 0, sizeof mock);
    mock.addr = 0x40;
    power_on();
}

void mock_set_running(uint8_t mode1)
{
    mock.regs[0x00] = (uint8_t)(mode1 & ~M1_SLEEP);
    mock.osc_stable = 1;
}

static void log_event(mock_event_type_t type, uint8_t addr, const uint8_t *data,
                      uint16_t len, uint32_t ms)
{
    if (mock.n_events >= MOCK_MAX_EVENTS)
        return;

    mock_event_t *ev = &mock.events[mock.n_events++];
    ev->type = type;
    ev->addr = addr;
    ev->len = len > MOCK_MAX_BYTES ? MOCK_MAX_BYTES : len;
    if (data != NULL)
        memcpy(ev->data, data, ev->len);
    ev->ms = ms;
}

const mock_event_t *mock_last_write(void)
{
    for (int i = mock.n_events - 1; i >= 0; i--)
        if (mock.events[i].type == MOCK_EV_WRITE)
            return &mock.events[i];
    return NULL;
}

int mock_count(mock_event_type_t type)
{
    int n = 0;
    for (int i = 0; i < mock.n_events; i++)
        if (mock.events[i].type == type)
            n++;
    return n;
}

/* Does the chip acknowledge this 7-bit address? */
static int answers(uint8_t addr)
{
    uint8_t m1 = mock.regs[0x00];

    if (addr == mock.addr)
        return 1;
    if ((m1 & M1_ALLCALL) && addr == (mock.regs[0x05] >> 1))
        return 1;
    if ((m1 & M1_SUB1) && addr == (mock.regs[0x02] >> 1))
        return 1;
    if ((m1 & M1_SUB2) && addr == (mock.regs[0x03] >> 1))
        return 1;
    if ((m1 & M1_SUB3) && addr == (mock.regs[0x04] >> 1))
        return 1;
    return 0;
}

static void write_mode1(uint8_t v)
{
    uint8_t old = mock.regs[0x00];
    uint8_t nv = (uint8_t)(v & ~M1_RESTART);

    /* EXTCLK is sticky, and may only be set while SLEEP is already 1. */
    if (old & M1_EXTCLK) {
        nv |= M1_EXTCLK;
    } else if ((v & M1_EXTCLK) && !(old & M1_SLEEP)) {
        mock.extclk_violations++;
        nv &= (uint8_t)~M1_EXTCLK;
    }

    /* Sleep transitions. Going to sleep while running leaves a restart
     * pending; waking up needs a delay before the oscillator is stable. */
    if (!(old & M1_SLEEP) && (nv & M1_SLEEP)) {
        old |= M1_RESTART;
        mock.osc_stable = 0;
    }
    if ((old & M1_SLEEP) && !(nv & M1_SLEEP))
        mock.osc_stable = 0;

    /* RESTART: writing 0 has no effect, writing 1 restarts and clears it. */
    if (v & M1_RESTART) {
        if (old & M1_RESTART) {
            mock.restarts++;
            if ((nv & M1_SLEEP) || !mock.osc_stable)
                mock.bad_restarts++;
        }
    } else {
        nv |= (uint8_t)(old & M1_RESTART);
    }

    mock.regs[0x00] = nv;
}

static void write_reg(uint8_t reg, uint8_t v)
{
    if (reg == 0x00) {
        write_mode1(v);
    } else if (reg == 0xFE) {
        /* PRE_SCALE is write-protected while the oscillator runs. */
        if (!(mock.regs[0x00] & M1_SLEEP))
            mock.ignored_prescale_writes++;
        else
            mock.regs[0xFE] = v < 3 ? 3 : v;
    } else if (reg >= 0xFA && reg <= 0xFD) {
        /* ALL_LED registers load every channel. */
        for (int ch = 0; ch < 16; ch++)
            mock.regs[0x06 + 4 * ch + (reg - 0xFA)] = v;
    } else {
        mock.regs[reg] = v;
    }
}

int16_t pca9685_i2c_hal_init(void)
{
    return PCA9685_OK;
}

int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)
{
    mock.writes++;
    log_event(MOCK_EV_WRITE, address, data, count, 0);

    if (mock.writes == mock.fail_write_at)
        return PCA9685_ERR;

    /* General call: SWRST resets the chip. */
    if (address == 0x00) {
        if (count == 1 && data[0] == 0x06)
            power_on();
        return PCA9685_OK;
    }

    if (!answers(address) || count == 0)
        return PCA9685_ERR;

    uint8_t reg = data[0];
    for (uint16_t i = 1; i < count; i++) {
        write_reg(reg, data[i]);
        if (mock.regs[0x00] & M1_AI)
            reg++;
    }
    return PCA9685_OK;
}

int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)
{
    mock.reads++;
    log_event(MOCK_EV_READ, address, reg, 1, 0);

    if (mock.reads == mock.fail_read_at || !answers(address))
        return PCA9685_ERR;

    uint8_t r = *reg;
    for (uint16_t i = 0; i < count; i++) {
        data[i] = mock.regs[r];
        if (mock.regs[0x00] & M1_AI)
            r++;
    }
    return PCA9685_OK;
}

void pca9685_i2c_hal_ms_delay(uint32_t ms)
{
    log_event(MOCK_EV_DELAY, 0, NULL, 0, ms);

    /* 1 ms is more than the 500 us the oscillator needs. */
    if (ms >= 1 && !(mock.regs[0x00] & M1_SLEEP))
        mock.osc_stable = 1;
}
