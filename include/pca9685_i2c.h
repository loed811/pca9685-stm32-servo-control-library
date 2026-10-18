#ifndef MAIN_PCA9685_I2C
#define MAIN_PCA9685_I2C

#ifdef __cplusplus
extern "C" {
#endif

#include "pca9685_i2c_hal.h"

/**
 * @brief PCA9685 Device Structure.
 * Holds the I2C address configuration for a specific device instance.
 */
typedef struct{
    uint8_t i2c_addr;       /**< Main I2C address of the device (default 0x40) */
    uint8_t allcall_addr;   /**< I2C All Call address (default 0x70) */
    uint8_t sub_addr_1;     /**< I2C Sub-Address 1 */
    uint8_t sub_addr_2;     /**< I2C Sub-Address 2 */
    uint8_t sub_addr_3;     /**< I2C Sub-Address 3 */
} pca9685_dev_t;

/**
 * @brief Clock Source Configuration (MODE1 Register - Bit 6).
 */
typedef enum{
    PCA9685_CLK_INTERNAL = 0x00, /**< Use internal 25MHz clock (default) */
    PCA9685_CLK_EXTERNAL  = 0x01, /**< Use external clock pin (PRE_SCALE must be set by user) */
} pca9685_auto_extclk_t;

/**
 * @brief Auto-Increment Enable (MODE1 Register - Bit 5).
 */
typedef enum{
    PCA9685_AUTOINCR_OFF = 0x00, /**< Auto-increment disabled */
    PCA9685_AUTOINCR_ON  = 0x01, /**< Auto-increment enabled (recommended for multi-byte writes) */
} pca9685_auto_incr_t;

/**
 * @brief Sleep Mode Configuration (MODE1 Register - Bit 4).
 */
typedef enum{
    PCA9685_MODE_NORMAL = 0x00, /**< Normal mode (Oscillator on) */
    PCA9685_MODE_SLEEP  = 0x01, /**< Low power mode (Oscillator off, registers accessible) */
} pca9685_sleep_mode_t;

/**
 * @brief I2C Address Response Enable (MODE1 Register - Bits 3, 2, 1, 0).
 */
typedef enum{
    PCA9685_ADDR_NORESPOND = 0x00, /**< Device does not respond to this address */
    PCA9685_ADDR_RESPOND = 0x01,   /**< Device responds to this address */
} pca9685_addr_resp_t;

/**
 * @brief Sub-Address Selection Identifier.
 */
typedef enum{
    PCA9685_SUB_ADDR_1 = 0x01, /**< Select Sub-Address 1 */
    PCA9685_SUB_ADDR_2 = 0x02, /**< Select Sub-Address 2 */
    PCA9685_SUB_ADDR_3 = 0x03, /**< Select Sub-Address 3 */
} pca9685_subaddr_no_t;

/**
 * @brief LED Full On/Off State.
 * Used for setting channels completely on or off without PWM.
 */
typedef enum{
    PCA9685_LED_OFF = 0x00, /**< LED driver is fully OFF */
    PCA9685_LED_ON  = 0x01, /**< LED driver is fully ON */
} pca9685_led_state_t;

/**
 * @brief Output Logic Inversion (MODE2 Register - Bit 4).
 */
typedef enum{
    PCA9685_OUTPUT_NOTINVERT = 0x00, /**< Output logic state is not inverted */
    PCA9685_OUTPUT_INVERT = 0x01,    /**< Output logic state is inverted */
} pca9685_output_invert_t;

/**
 * @brief Output Change Timing (MODE2 Register - Bit 3).
 */
typedef enum{
    PCA9685_CH_ONSTOP = 0x00, /**< Outputs change on I2C STOP condition */
    PCA9685_CH_ONACK = 0x01,  /**< Outputs change on I2C ACK */
} pca9685_output_change_t;

/**
 * @brief Output State when Disabled (OE = 1) (MODE2 Register - Bits 1-0).
 * Defines what happens when the Output Enable pin is high.
 */
typedef enum{
    PCA9685_OUTPUT_LOW = 0x00,            /**< Output is set to logical 0 */
    PCA9685_OUTPUT_HIGH = 0x01,           /**< Output is set to logical 1 (only when OUTDRV=1) */
    PCA9685_OUTPUT_HIGH_IMPEDANCE = 0x02, /**< Output is high-impedance */
} pca9685_output_not_enable_t;

/**
 * @brief Output Driver Configuration (MODE2 Register - Bit 2).
 */
typedef enum{
    PCA9685_OUTPUT_OPEN_DRAIN = 0x00, /**< Open-drain structure */
    PCA9685_OUTPUT_TOTEM_POLE = 0x01, /**< Totem-pole structure (push-pull) */
} pca9685_output_drive_t;

/**
 * @brief Composite Output Configuration Structure.
 * Aggregates all MODE2 register settings.
 */
typedef struct{
    pca9685_output_drive_t outdrv;      /**< Drive type (Open-drain/Totem-pole) */
    pca9685_output_not_enable_t outne;  /**< Behavior when OE pin is High */
    pca9685_output_change_t och;        /**< When outputs change (Stop/Ack) */
    pca9685_output_invert_t invrt;      /**< Invert output logic */
} pca9685_output_t;

/**
 * @brief PCA9685 I2C slave addresses
 */
#define I2C_ADDRESS_PCA9685             0x40 /**< Default base I2C Address */

/**
 * @brief PCA9685 default addresses
 */
#define I2C_GEN_CALL_ADDRESS_PCA9685    0x00 /**< I2C General Call Address */
#define I2C_ALL_CALL_ADDRESS_PCA9685    0x70 /**< Default All Call Address */
#define I2C_SUB_ADDRESS_1_PCA9685       0x71 /**< Default Sub Address 1 */
#define I2C_SUB_ADDRESS_2_PCA9685       0x72 /**< Default Sub Address 2 */
#define I2C_SUB_ADDRESS_3_PCA9685       0x74 /**< Default Sub Address 3 */

/**
 * @brief PCA9685 Register Map
 */
#define REG_RESET                       0x00 /**< Reset Register (Use with SWRST) */
#define REG_MODE_1                      0x00 /**< Mode Register 1 */
#define REG_MODE_2                      0x01 /**< Mode Register 2 */
#define REG_ALLCALLADR                  0x05 /**< All Call I2C Address Register */
#define REG_ALL_LED                     0xFA /**< Load all the LEDn_ON registers */
#define REG_PRE_SCALE                   0xFE /**< Prescaler for PWM output frequency */
#define REG_TEST_MODE                   0xFF /**< Test Mode Register */

/**
 * @brief PCA9685 software reset command
 */
#define SWRST                           0x06 /**< Software Reset Byte */

/**
 * @brief Other PCA9685 constants
 */
#define LED_OFFSET_ADR                  0x06 /**< Start address of LED0_ON_L */
#define SUBADR_OFFSET_ADR               0x01 /**< Offset for Sub-Address registers relative to register 0x02 */
#define STAB_TIME                       1      /**< Oscillator stabilization wait time (ms) */
#define PWM_OUTPUT_COUNTER_MAX          0x1000 /**< 12-bit counter max (4096 steps) */

/* -------------------------------------------------------------------------- */
/* Function Prototypes                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Reads the content of the MODE1 register.
 * @param dev The device structure.
 * @param mode Pointer to store the result.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_read_mode_1(pca9685_dev_t dev, uint8_t *mode);

/**
 * @brief Reads the content of the MODE2 register.
 * @param dev The device structure.
 * @param mode Pointer to store the result.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_read_mode_2(pca9685_dev_t dev, uint8_t *mode);

/**
 * @brief Configures the clock source (Internal vs External).
 * @param dev The device structure.
 * @param clk The clock source selection.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_clock(pca9685_dev_t dev, pca9685_auto_extclk_t clk);

/**
 * @brief Configures the register Auto-Increment feature.
 * @param dev The device structure.
 * @param setting Enable or Disable auto-increment.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_autoincrement(pca9685_dev_t dev, pca9685_auto_incr_t setting);

/**
 * @brief Performs a Restart sequence to wake the device and resume PWM.
 * @param dev The device structure.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_restart(pca9685_dev_t dev);

/**
 * @brief Controls the Sleep Mode of the device.
 * @param dev The device structure.
 * @param sleep_mode Enable (Sleep) or Disable (Normal Mode).
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_sleep_mode(pca9685_dev_t dev, pca9685_sleep_mode_t sleep_mode);

/**
 * @brief Resets the device via I2C Software Reset (SWRST).
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_reset();

/**
 * @brief Initializes the Output configuration (MODE2 register).
 * @param dev The device structure.
 * @param setting The output configuration struct.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_output_init(pca9685_dev_t dev, pca9685_output_t setting);

/**
 * @brief Sets a specific LED channel to full ON or full OFF.
 * @param dev The device structure.
 * @param led_no The LED channel number (0-15).
 * @param state State (ON/OFF).
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_led_set(pca9685_dev_t dev, uint8_t led_no, pca9685_led_state_t state);

/**
 * @brief Sets ALL LED channels to full ON or full OFF.
 * @param dev The device structure.
 * @param state State (ON/OFF).
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_all_led_set(pca9685_dev_t dev, pca9685_led_state_t state);

/**
 * @brief Sets PWM parameters for a specific LED channel.
 * @param dev The device structure.
 * @param led_no The LED channel number (0-15).
 * @param d_cycle Duty cycle percentage (0-100).
 * @param delay Phase delay percentage (0-100).
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_led_pwm_set(pca9685_dev_t dev, uint8_t led_no, float d_cycle, float delay);

/**
 * @brief Sets PWM parameters for ALL LED channels.
 * @param dev The device structure.
 * @param d_cycle Duty cycle percentage (0-100).
 * @param delay Phase delay percentage (0-100).
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_all_led_pwm_set(pca9685_dev_t dev, float d_cycle, float delay);

/**
 * @brief Calculates and writes the Pre-scale value for a target frequency.
 * @note Device must be in SLEEP mode before calling this.
 * @param dev The device structure.
 * @param frequency Target frequency in Hz (e.g., 50.0).
 * @param osc_clk_hz Oscillator frequency (internal is ~25MHz).
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_write_pre_scale(pca9685_dev_t dev, double frequency, double osc_clk_hz);

/**
 * @brief Reads the Pre-scale register and calculates the actual frequency.
 * @param dev The device structure.
 * @param frequency Pointer to store the calculated frequency.
 * @param osc_clk_hz Oscillator frequency.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_read_pre_scale(pca9685_dev_t dev, double *frequency, double osc_clk_hz);

/**
 * @brief Configures the All Call I2C address.
 * @param dev The device structure.
 * @param allcall_addr The new 7-bit All Call address.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_write_allcall_addr(pca9685_dev_t dev, uint8_t allcall_addr);

/**
 * @brief Reads the current All Call I2C address.
 * @param dev The device structure.
 * @param allcall_addr Pointer to store the address.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_read_allcall_addr(pca9685_dev_t dev, uint8_t *allcall_addr);

/**
 * @brief Writes a specific Sub-Address (1, 2, or 3).
 * @param dev The device structure.
 * @param addr_no Sub-Address identifier (1-3).
 * @param sub_addr The new 7-bit address.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_write_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t sub_addr);

/**
 * @brief Reads a specific Sub-Address.
 * @param dev The device structure.
 * @param addr_no Sub-Address identifier (1-3).
 * @param sub_addr Pointer to store the address.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_read_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t *sub_addr);

/**
 * @brief Enables/Disables response to a Sub-Address.
 * @param dev The device structure.
 * @param sub_addr Sub-Address identifier (1-3).
 * @param resp Enable or Disable response.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_sub_addr_resp(pca9685_dev_t dev, pca9685_subaddr_no_t sub_addr, pca9685_addr_resp_t resp);

/**
 * @brief Enables/Disables response to the All Call Address.
 * @param dev The device structure.
 * @param resp Enable or Disable response.
 * @return PCA9685_OK on success.
 */
int16_t pca9685_i2c_allcall_address_resp(pca9685_dev_t dev, pca9685_addr_resp_t resp);

/**
 * @brief Registers and initializes the device structure with address settings.
 * @note This function writes the address configuration to the device immediately.
 * @param dev Pointer to the device structure.
 * @param _i2c_addr The hardware I2C address.
 * @param _allcall_addr The All Call address to set.
 * @param _sub_addr_1 Sub-Address 1 to set.
 * @param _sub_addr_2 Sub-Address 2 to set.
 * @param _sub_addr_3 Sub-Address 3 to set.
 */
void pca9685_i2c_register(pca9685_dev_t *dev,
                          uint8_t _i2c_addr,
                          uint8_t _allcall_addr,
                          uint8_t _sub_addr_1,
                          uint8_t _sub_addr_2,
                          uint8_t _sub_addr_3);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_PCA9685_I2C */
