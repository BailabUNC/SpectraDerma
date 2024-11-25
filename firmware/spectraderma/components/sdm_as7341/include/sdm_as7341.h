#ifndef AS7341_H
#define AS7341_H

#include "driver/i2c.h"
#include "esp_err.h"

#define AS7341_I2CADDR_DEFAULT  0x39        ///< AS7341 default i2c address
#define AS7341_CHIP_ID          0x09        ///< AS7341 default device id from WHOAMI

#define AS7341_WHOAMI           0x92        ///< Chip ID register

#define AS7341_ASTATUS          0x60        ///< AS7341_ASTATUS (unused)
#define AS7341_CH0_DATA_L_      0x61        ///< AS7341_CH0_DATA_L (unused)
#define AS7341_CH0_DATA_H_      0x62        ///< AS7341_CH0_DATA_H (unused)
#define AS7341_ITIME_L          0x63        ///< AS7341_ITIME_L (unused)
#define AS7341_ITIME_M          0x64        ///< AS7341_ITIME_M (unused)
#define AS7341_ITIME_H          0x65        ///< AS7341_ITIME_H (unused)
#define AS7341_CONFIG           0x70        ///< Enables LED control and sets light sensing mode
#define AS7341_STAT             0x71        ///< AS7341_STAT (unused)
#define AS7341_EDGE             0x72        ///< AS7341_EDGE (unused)
#define AS7341_GPIO             0x73        ///< Connects photo diode to GPIO or INT pins
#define AS7341_LED              0x74        ///< LED Register; Enables and sets current limit
#define AS7341_ENABLE           0x80        ///< Main enable register. Controls SMUX, Flicker Detection, Spectral
                                            ///< Measurements and Power
#define AS7341_ATIME            0x81        ///< Sets ADC integration step count
#define AS7341_WTIME            0x83        ///< AS7341_WTIME (unused)
#define AS7341_SP_LOW_TH_L      0x84        ///< Spectral measurement Low Threshold low byte
#define AS7341_SP_LOW_TH_H      0x85        ///< Spectral measurement Low Threshold high byte
#define AS7341_SP_HIGH_TH_L     0x86        ///< Spectral measurement High Threshold low byte
#define AS7341_SP_HIGH_TH_H     0x87        ///< Spectral measurement High Threshold low byte
#define AS7341_AUXID            0x90        ///< AS7341_AUXID (unused)
#define AS7341_REVID            0x91        ///< AS7341_REVID (unused)
#define AS7341_ID               0x92        ///< AS7341_ID (unused)
#define AS7341_STATUS           0x93        ///< Interrupt status registers. Indicates the occourance of an interrupt
#define AS7341_ASTATUS_         0x94        ///< AS7341_ASTATUS, same as 0x60 (unused)
#define AS7341_CH0_DATA_L       0x95        ///< ADC Channel Data
#define AS7341_CH0_DATA_H       0x96        ///< ADC Channel Data
#define AS7341_CH1_DATA_L       0x97        ///< ADC Channel Data
#define AS7341_CH1_DATA_H       0x98        ///< ADC Channel Data
#define AS7341_CH2_DATA_L       0x99        ///< ADC Channel Data
#define AS7341_CH2_DATA_H       0x9A        ///< ADC Channel Data
#define AS7341_CH3_DATA_L       0x9B        ///< ADC Channel Data
#define AS7341_CH3_DATA_H       0x9C        ///< ADC Channel Data
#define AS7341_CH4_DATA_L       0x9D        ///< ADC Channel Data
#define AS7341_CH4_DATA_H       0x9E        ///< ADC Channel Data
#define AS7341_CH5_DATA_L       0x9F        ///< ADC Channel Data
#define AS7341_CH5_DATA_H       0xA0        ///< ADC Channel Data
#define AS7341_STATUS2          0xA3        ///< Measurement status flags; saturation, validity
#define AS7341_STATUS3          0xA4        ///< Spectral interrupt source, high or low threshold
#define AS7341_STATUS5          0xA6        ///< AS7341_STATUS5 (unused)
#define AS7341_STATUS6          0xA7        ///< AS7341_STATUS6 (unused)
#define AS7341_CFG0             0xA9        ///< Sets Low power mode, Register bank, and Trigger lengthening
#define AS7341_CFG1             0xAA        ///< Controls ADC Gain
#define AS7341_CFG3             0xAC        ///< AS7341_CFG3 (unused)
#define AS7341_CFG6             0xAF        ///< Used to configure Smux
#define AS7341_CFG8             0xB1        ///< AS7341_CFG8 (unused)
#define AS7341_CFG9             0xB2        ///< Enables flicker detection and smux command completion system
                                            ///< interrupts
#define AS7341_CFG10            0xB3        ///< AS7341_CFG10 (unused)
#define AS7341_CFG12            0xB5        ///< Spectral threshold channel for interrupts, persistence and auto-gain
#define AS7341_PERS             0xBD        ///< Number of measurement cycles outside thresholds to trigger an
                                            ///< interupt
#define AS7341_GPIO2            0xBE        ///< GPIO Settings and status: polarity, direction, sets output, reads
                                            ///< input
#define AS7341_ASTEP_L          0xCA        ///< Integration step size ow byte
#define AS7341_ASTEP_H          0xCB        ///< Integration step size high byte
#define AS7341_AGC_GAIN_MAX     0xCF        ///< AS7341_AGC_GAIN_MAX (unused)
#define AS7341_AZ_CONFIG        0xD6        ///< AS7341_AZ_CONFIG (unused)
#define AS7341_FD_TIME1         0xD8        ///< Flicker detection integration time low byte
#define AS7341_FD_TIME2         0xDA        ///< Flicker detection gain and high nibble
#define AS7341_FD_CFG0          0xD7        ///< AS7341_FD_CFG0 (unused)
#define AS7341_FD_STATUS        0xDB        ///< Flicker detection status; measurement valid, saturation, flicker
                                            ///< type
#define AS7341_INTENAB          0xF9        ///< Enables individual interrupt types
#define AS7341_CONTROL          0xFA        ///< Auto-zero, fifo clear, clear SAI active
#define AS7341_FIFO_MAP         0xFC        ///< AS7341_FIFO_MAP (unused)
#define AS7341_FIFO_LVL         0xFD        ///< AS7341_FIFO_LVL (unused)
#define AS7341_FDATA_L          0xFE        ///< AS7341_FDATA_L (unused)
#define AS7341_FDATA_H          0xFF        ///< AS7341_FDATA_H (unused)

/*
    Integration time = (ATIME + 1) * (ASTEP + 1) * 2.78 us
    ADC scale = (ATIME + 1) * (ASTEP + 1)
    Wait time = (WTIME + 1) * 2.78 ms
*/
#define SDM_AS7341_ATIME        16
#define SDM_AS7341_ASTEP        16
#define SDM_AS7341_WTIME        0
#define SDM_AS7341_GAIN         AS7341_GAIN_256X

#define AS7341_SPECTRAL_INT_HIGH_MSK    0b00100000  ///< bitmask to check for a high threshold interrupt
#define AS7341_SPECTRAL_INT_LOW_MSK     0b00010000  ///< bitmask to check for a low threshold interrupt

/**
 * @brief Register bank access
 *
 */
typedef enum {
    AS7341_REG_BANK_H,          ///< Access to register 0x80 and above
    AS7341_REG_BANK_L,          ///< Access to register 0x60 to 0x74
} sdm_as7341_reg_bank_t;

/**
 * @brief Ambient light sensing integration mode
 *
 */
typedef enum {
    AS7341_INT_MODE_SPM,        ///< spectral measurement, no sync
    AS7341_INT_MODE_SYNS,       ///< spectral measurement, start sync
    AS7341_INT_MODE_RESERVED,   ///< (reserved)
    AS7341_INT_MODE_SYND,       ///< spectral measurement, start/stop sync
} sdm_as7341_int_mode_t;

/**
 * @brief Modes of channel mapping
 *
 */
typedef enum {
    AS7341_CH_F1F4_CLEAR_NIR,   ///< Map the values of the registers of 6 channels to F1, F2, F3, F4, clear, NIR
    AS7341_CH_F5F8_CLEAR_NIR,   ///< Map the values of the registers of 6 channels to F5, F6, F7, F8, clear, NIR
} sdm_channel_mapping_mode_t;


/**
 * @brief Data structure for the 6 channels under channel mapping mode AS7341_CH_F1F4_CLEAR_NIR 
 *
 */
typedef struct {
    uint16_t ADF1;
    uint16_t ADF2;
    uint16_t ADF3;
    uint16_t ADF4;
    uint16_t ADCLEAR;
    uint16_t ADNIR;
} sdm_mode_one_data_t;

/**
 * @brief Data structure for the 6 channels under channel mapping mode AS7341_CH_F5F8_CLEAR_NIR 
 *
 */
typedef struct {
    uint16_t ADF5;
    uint16_t ADF6;
    uint16_t ADF7;
    uint16_t ADF8;
    uint16_t ADCLEAR;
    uint16_t ADNIR;
} sdm_mode_two_data_t;

/**
 * @brief Allowable gain multipliers for `setGain`
 *
 */
typedef enum {
    AS7341_GAIN_0_5X,
    AS7341_GAIN_1X,
    AS7341_GAIN_2X,
    AS7341_GAIN_4X,
    AS7341_GAIN_8X,
    AS7341_GAIN_16X,
    AS7341_GAIN_32X,
    AS7341_GAIN_64X,
    AS7341_GAIN_128X,
    AS7341_GAIN_256X,
    AS7341_GAIN_512X,
} sdm_as7341_gain_t;

/**
 * @brief Available SMUX configuration commands
 *
 */
typedef enum {
    AS7341_SMUX_CMD_ROM_RESET,    ///< ROM code initialization of SMUX
    AS7341_SMUX_CMD_READ,         ///< Read SMUX configuration to RAM from SMUX chain
    AS7341_SMUX_CMD_WRITE,        ///< Write SMUX configuration from RAM to SMUX chain
} sdm_as7341_smux_cmd_t;

/**
 * @brief ADC Channel specifiers for configuration
 *
 */
typedef enum {
    AS7341_ADC_CHANNEL_0,
    AS7341_ADC_CHANNEL_1,
    AS7341_ADC_CHANNEL_2,
    AS7341_ADC_CHANNEL_3,
    AS7341_ADC_CHANNEL_4,
    AS7341_ADC_CHANNEL_5,
} sdm_as7341_adc_channel_t;

/**
 * @brief Spectral Channel specifiers for configuration and reading
 *
 */
typedef enum {
    AS7341_CHANNEL_415nm_F1,
    AS7341_CHANNEL_445nm_F2,
    AS7341_CHANNEL_480nm_F3,
    AS7341_CHANNEL_515nm_F4,
    AS7341_CHANNEL_CLEAR_0,
    AS7341_CHANNEL_NIR_0,
    AS7341_CHANNEL_555nm_F5,
    AS7341_CHANNEL_590nm_F6,
    AS7341_CHANNEL_630nm_F7,
    AS7341_CHANNEL_680nm_F8,
    AS7341_CHANNEL_CLEAR,
    AS7341_CHANNEL_NIR,
} sdm_as7341_color_channel_t;

/**
 * @brief GPIO input/output mode
 *
 */
typedef enum {
    AS7341_GPIO_INPUT,
    AS7341_GPIO_OUTPUT,
} sdm_as7341_gpio_mode_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
} sdm_as7341_t;

esp_err_t sdm_as7341_init(sdm_as7341_t *dev, i2c_port_t port, uint8_t addr);
esp_err_t sdm_as7341_write_reg(sdm_as7341_t *dev, uint8_t reg, uint8_t data_wr);
esp_err_t sdm_as7341_read_reg(sdm_as7341_t *dev, uint8_t reg, uint8_t *data_wr);

esp_err_t sdm_as7341_enable(sdm_as7341_t *dev, bool enable);
esp_err_t sdm_as7341_enable_spectral_measurement(sdm_as7341_t *dev, bool enable);
esp_err_t sdm_as7341_enable_wait(sdm_as7341_t *dev, bool enable);
esp_err_t sdm_as7341_enable_smux(sdm_as7341_t *dev, bool enable);

esp_err_t sdm_as7341_start_measure(sdm_as7341_t *dev, sdm_channel_mapping_mode_t mode);

esp_err_t sdm_as7341_setup_f1f4_clear_nir(sdm_as7341_t *dev);
esp_err_t sdm_as7341_setup_f5f8_clear_nir(sdm_as7341_t *dev);

esp_err_t sdm_as7341_set_integration_mode(sdm_as7341_t *dev, sdm_as7341_int_mode_t mode);
esp_err_t sdm_as7341_set_integration_time(sdm_as7341_t *dev, uint8_t atime, uint16_t astep);
esp_err_t sdm_as7341_set_gain(sdm_as7341_t *dev, sdm_as7341_gain_t gain);
esp_err_t sdm_as7341_set_wait_time(sdm_as7341_t *dev, uint8_t wtime);
esp_err_t sdm_as7341_set_gpio_state(sdm_as7341_t *dev, sdm_as7341_gpio_mode_t gpio_mode);

esp_err_t sdm_as7341_read_channel_data(sdm_as7341_t *dev, uint16_t *data);

#endif  // AS7341_H