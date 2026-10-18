/*
 * Mock platform layer for host unit tests.
 *
 * Implements pca9685_i2c_hal.h with a small model of one PCA9685: a register
 * file, register auto-increment, the MODE1 RESTART/SLEEP/EXTCLK rules from
 * the datasheet, PRE_SCALE write protection and the SWRST general call.
 * Every transfer and delay is logged so tests can check the exact bytes.
 */
#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <stdint.h>

#define MOCK_MAX_EVENTS     64
#define MOCK_MAX_BYTES      80

typedef enum {
    MOCK_EV_WRITE,
    MOCK_EV_READ,
    MOCK_EV_DELAY,
} mock_event_type_t;

typedef struct {
    mock_event_type_t type;
    uint8_t  addr;                  /* 7-bit address (write/read) */
    uint8_t  data[MOCK_MAX_BYTES];  /* bytes written, or the register read */
    uint16_t len;
    uint32_t ms;                    /* delay length */
} mock_event_t;

typedef struct {
    uint8_t addr;                   /* 7-bit address the chip answers on */
    uint8_t regs[256];

    int osc_stable;                 /* oscillator running and settled */
    int restarts;                   /* RESTART written as 1 while it was pending */
    int bad_restarts;               /* ...while asleep or before the oscillator settled */
    int extclk_violations;          /* EXTCLK written without SLEEP set beforehand */
    int ignored_prescale_writes;    /* PRE_SCALE written while awake */

    int fail_write_at;              /* make the Nth write fail (1-based), 0 = never */
    int fail_read_at;               /* make the Nth read fail (1-based), 0 = never */
    int writes;
    int reads;

    mock_event_t events[MOCK_MAX_EVENTS];
    int n_events;
} mock_t;

extern mock_t mock;

/* Power-on state, address 0x40, empty log. */
void mock_reset(void);

/* Device awake and running with the given MODE1 value. */
void mock_set_running(uint8_t mode1);

/* Last write transfer, or NULL if there was none. */
const mock_event_t *mock_last_write(void);

/* Number of logged events of one type. */
int mock_count(mock_event_type_t type);

#endif /* MOCK_HAL_H */
