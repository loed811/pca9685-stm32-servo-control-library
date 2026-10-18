/*
 * Fake implementation of the pca9685_i2c_hal_* layer.
 *
 * Models the PCA9685 as a 256-byte register file: writes store bytes starting
 * at the register given in data[0] (sequential, as with auto-increment on),
 * reads return bytes from the register file. Every transaction is logged so
 * tests can assert the exact bytes the driver put on the bus.
 */
#ifndef FAKE_PCA9685_HAL_H
#define FAKE_PCA9685_HAL_H

#include <stdint.h>

#define FAKE_MAX_TX     64
#define FAKE_MAX_BYTES  16

typedef struct {
    int is_read;
    uint8_t addr;
    uint8_t reg;        /* register for reads, data[0] for writes */
    uint16_t count;     /* bytes written (incl. register) or read */
    uint8_t bytes[FAKE_MAX_BYTES];
} fake_tx_t;

typedef struct {
    uint8_t regs[256];
    fake_tx_t log[FAKE_MAX_TX];
    int n_tx;
    int n_writes;
    int n_reads;
    uint32_t delay_total_ms;
    int delay_calls;
    int init_calls;
    int fail_reads;     /* non-zero: every read returns PCA9685_ERR and fills
                         * the destination with fail_fill, so code that uses
                         * the value after a failed read behaves
                         * deterministically */
    uint8_t fail_fill;  /* set to 0xFF by fake_reset() */
    int fail_writes;    /* non-zero: every write returns PCA9685_ERR */
} fake_hal_t;

extern fake_hal_t fake;

void fake_reset(void);

/* Returns the n-th write transaction (0-based), or NULL. */
const fake_tx_t *fake_write(int n);

/* Returns the most recent write transaction, or NULL. */
const fake_tx_t *fake_last_write(void);

#endif /* FAKE_PCA9685_HAL_H */
