/*
 * Host-side unit tests for pca9685_i2c_hal.c (the STM32 HAL glue).
 *
 * stubs/main.h provides the STM32 types; HAL_I2C_Master_Transmit/Receive and
 * HAL_Delay are faked here and record every call. The last test links the
 * full stack (pca9685_i2c.c -> pca9685_i2c_hal.c -> fake STM32 HAL).
 */
#include <string.h>

#include "main.h"
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"
#include "test_framework.h"

TF_DEFINE_GLOBALS;

I2C_HandleTypeDef hi2c1;

#define MAX_CALLS 16

typedef struct {
    int is_rx;
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr;
    uint16_t size;
    uint32_t timeout;
    uint8_t bytes[16];
} hal_call_t;

static struct {
    hal_call_t calls[MAX_CALLS];
    int n_calls;
    HAL_StatusTypeDef tx_status;
    HAL_StatusTypeDef rx_status;
    uint8_t rx_data[16];
    uint32_t delay_ms;
    int delay_calls;
} hal;

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                          uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    if (hal.n_calls < MAX_CALLS) {
        hal_call_t *c = &hal.calls[hal.n_calls++];
        c->is_rx = 0;
        c->hi2c = hi2c;
        c->dev_addr = DevAddress;
        c->size = Size;
        c->timeout = Timeout;
        memcpy(c->bytes, pData, Size < sizeof(c->bytes) ? Size : sizeof(c->bytes));
    }
    return hal.tx_status;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                         uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    if (hal.n_calls < MAX_CALLS) {
        hal_call_t *c = &hal.calls[hal.n_calls++];
        c->is_rx = 1;
        c->hi2c = hi2c;
        c->dev_addr = DevAddress;
        c->size = Size;
        c->timeout = Timeout;
    }
    if (hal.rx_status == HAL_OK)
        memcpy(pData, hal.rx_data, Size < sizeof(hal.rx_data) ? Size : sizeof(hal.rx_data));
    return hal.rx_status;
}

void HAL_Delay(uint32_t Delay)
{
    hal.delay_calls++;
    hal.delay_ms += Delay;
}

static void test_setup(void)
{
    memset(&hal, 0, sizeof(hal));
    hal.tx_status = HAL_OK;
    hal.rx_status = HAL_OK;
    hi2c1.Instance = (void *)0x40005400;    /* I2C1 base on most STM32 */
}

static void test_init(void)
{
    CHECK_EQ(pca9685_i2c_hal_init(), PCA9685_OK);
    hi2c1.Instance = NULL;
    CHECK_EQ(pca9685_i2c_hal_init(), PCA9685_ERR);
}

static void test_write(void)
{
    uint8_t buf[3] = { 0xFE, 0x79, 0xAA };
    CHECK_EQ(pca9685_i2c_hal_write(0x40, buf, 3), PCA9685_OK);
    CHECK_EQ(hal.n_calls, 1);
    CHECK(hal.calls[0].hi2c == &hi2c1);
    CHECK_EQ(hal.calls[0].dev_addr, 0x80);  /* 7-bit address shifted left */
    CHECK_EQ(hal.calls[0].size, 3);
    CHECK_EQ(hal.calls[0].timeout, 100);
    CHECK_EQ(memcmp(hal.calls[0].bytes, buf, 3), 0);
}

static void test_write_address_shift(void)
{
    uint8_t buf[1] = { 0x06 };
    pca9685_i2c_hal_write(0x7F, buf, 1);
    CHECK_EQ(hal.calls[0].dev_addr, 0xFE);
    pca9685_i2c_hal_write(0x00, buf, 1);    /* general call */
    CHECK_EQ(hal.calls[1].dev_addr, 0x00);
}

static void test_write_errors(void)
{
    static const HAL_StatusTypeDef errs[] = { HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
    uint8_t buf[2] = { 0, 0 };
    for (size_t i = 0; i < 3; i++) {
        hal.tx_status = errs[i];
        CHECK_EQ(pca9685_i2c_hal_write(0x40, buf, 2), PCA9685_ERR);
    }
}

static void test_read(void)
{
    uint8_t reg = 0xFE, out[2] = { 0, 0 };
    hal.rx_data[0] = 0x79;
    hal.rx_data[1] = 0x55;
    CHECK_EQ(pca9685_i2c_hal_read(0x41, &reg, out, 2), PCA9685_OK);
    CHECK_EQ(hal.n_calls, 2);
    /* 1. register pointer write */
    CHECK_EQ(hal.calls[0].is_rx, 0);
    CHECK_EQ(hal.calls[0].dev_addr, 0x82);
    CHECK_EQ(hal.calls[0].size, 1);
    CHECK_EQ(hal.calls[0].bytes[0], 0xFE);
    /* 2. data read */
    CHECK_EQ(hal.calls[1].is_rx, 1);
    CHECK_EQ(hal.calls[1].dev_addr, 0x82);
    CHECK_EQ(hal.calls[1].size, 2);
    CHECK_EQ(out[0], 0x79);
    CHECK_EQ(out[1], 0x55);
}

static void test_read_errors(void)
{
    uint8_t reg = 0x00, out = 0;
    hal.tx_status = HAL_ERROR;
    CHECK_EQ(pca9685_i2c_hal_read(0x40, &reg, &out, 1), PCA9685_ERR);
    CHECK_EQ(hal.n_calls, 1);               /* no receive after failed tx */

    test_setup();
    hal.rx_status = HAL_TIMEOUT;
    CHECK_EQ(pca9685_i2c_hal_read(0x40, &reg, &out, 1), PCA9685_ERR);
}

static void test_delay(void)
{
    pca9685_i2c_hal_ms_delay(20);
    CHECK_EQ(hal.delay_calls, 1);
    CHECK_EQ(hal.delay_ms, 20);
}

static void test_full_stack_servo_setup(void)
{
    pca9685_dev_t dev = { .i2c_addr = I2C_ADDRESS_PCA9685 };

    /* 50 Hz -> PRE_SCALE = 121 */
    CHECK_EQ(pca9685_i2c_write_pre_scale(dev, 50.0, 25000000.0), PCA9685_OK);
    CHECK_EQ(hal.calls[0].dev_addr, 0x80);
    CHECK_EQ(hal.calls[0].size, 2);
    CHECK_EQ(hal.calls[0].bytes[0], 0xFE);
    CHECK_EQ(hal.calls[0].bytes[1], 121);

    /* Channel 2, 10 % duty, 10 % delay -> LED2_ON_L = 0x0E, ON = 409, OFF = 819 */
    CHECK_EQ(pca9685_i2c_led_pwm_set(dev, 2, 10.0f, 10.0f), PCA9685_OK);
    CHECK_EQ(hal.calls[1].size, 5);
    CHECK_EQ(hal.calls[1].bytes[0], 0x0E);
    CHECK_EQ(hal.calls[1].bytes[1] | (hal.calls[1].bytes[2] << 8), 409);
    CHECK_EQ(hal.calls[1].bytes[3] | (hal.calls[1].bytes[4] << 8), 819);

    /* Sleep: read MODE1 (0x01) then write 0x11 */
    hal.rx_data[0] = 0x01;
    CHECK_EQ(pca9685_i2c_sleep_mode(dev, PCA9685_MODE_SLEEP), PCA9685_OK);
    CHECK_EQ(hal.calls[2].bytes[0], REG_MODE_1);
    CHECK_EQ(hal.calls[3].is_rx, 1);
    CHECK_EQ(hal.calls[4].bytes[0], REG_MODE_1);
    CHECK_EQ(hal.calls[4].bytes[1], 0x11);
    CHECK(hal.delay_ms >= 1);
}

int main(void)
{
    printf("== pca9685_i2c_hal.c (fake STM32 HAL) ==\n");

    RUN_TEST(test_init);
    RUN_TEST(test_write);
    RUN_TEST(test_write_address_shift);
    RUN_TEST(test_write_errors);
    RUN_TEST(test_read);
    RUN_TEST(test_read_errors);
    RUN_TEST(test_delay);
    RUN_TEST(test_full_stack_servo_setup);

    TF_SUMMARY_AND_EXIT("test_pca9685_i2c_hal_stm32");
}
