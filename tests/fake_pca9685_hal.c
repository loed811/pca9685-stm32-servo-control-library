#include <string.h>

#include "fake_pca9685_hal.h"
#include "pca9685_i2c_hal.h"

fake_hal_t fake;

void fake_reset(void)
{
    memset(&fake, 0, sizeof(fake));
    fake.fail_fill = 0xFF;
}

static fake_tx_t *log_tx(void)
{
    if (fake.n_tx >= FAKE_MAX_TX)
        return NULL;
    return &fake.log[fake.n_tx++];
}

const fake_tx_t *fake_write(int n)
{
    for (int i = 0; i < fake.n_tx; i++)
        if (!fake.log[i].is_read && n-- == 0)
            return &fake.log[i];
    return NULL;
}

const fake_tx_t *fake_last_write(void)
{
    for (int i = fake.n_tx - 1; i >= 0; i--)
        if (!fake.log[i].is_read)
            return &fake.log[i];
    return NULL;
}

int16_t pca9685_i2c_hal_init()
{
    fake.init_calls++;
    return PCA9685_OK;
}

int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)
{
    fake_tx_t *tx = log_tx();
    fake.n_reads++;
    if (tx) {
        tx->is_read = 1;
        tx->addr = address;
        tx->reg = *reg;
        tx->count = count;
    }
    if (fake.fail_reads) {
        memset(data, fake.fail_fill, count);
        return PCA9685_ERR;
    }
    for (uint16_t i = 0; i < count; i++)
        data[i] = fake.regs[(uint8_t)(*reg + i)];
    return PCA9685_OK;
}

int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)
{
    fake_tx_t *tx = log_tx();
    fake.n_writes++;
    if (tx) {
        tx->is_read = 0;
        tx->addr = address;
        tx->reg = data[0];
        tx->count = count;
        memcpy(tx->bytes, data, count < FAKE_MAX_BYTES ? count : FAKE_MAX_BYTES);
    }
    if (fake.fail_writes)
        return PCA9685_ERR;
    /* General call (SWRST) is not a register write. */
    if (address != 0x00)
        for (uint16_t i = 1; i < count; i++)
            fake.regs[(uint8_t)(data[0] + i - 1)] = data[i];
    return PCA9685_OK;
}

void pca9685_i2c_hal_ms_delay(uint32_t ms)
{
    fake.delay_calls++;
    fake.delay_total_ms += ms;
}
