#include <stdio.h>
#include "sdm_as7341.h"

#define LOG_LOCAL_LEVEL            ESP_LOG_DEBUG

#include "esp_log.h"

static const char *TAG = "SDM_AS7341";

#define ACK_CHECK_EN                0x1              /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS               0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                     0x0              /*!< I2C ack value */
#define NACK_VAL                    0x1              /*!< I2C nack value */

// Initialize the AS7341 sensor
esp_err_t sdm_as7341_init(sdm_as7341_t *dev, i2c_port_t port, uint8_t addr)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_port = port;
    dev->i2c_addr = addr;

    uint8_t whoami;
    esp_err_t ret = sdm_as7341_read_reg(dev, AS7341_WHOAMI, &whoami);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read WHOAMI register!");
        return ESP_FAIL;
    }
    else if ((whoami & 0xFC) != (AS7341_CHIP_ID << 2))
    {
        ESP_LOGE(TAG, "Wrong WHOAMI response: 0x%02X", whoami);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "AS7341 found: WHOAMI: 0x%02X", whoami);

    // set to syns mode 
    sdm_as7341_set_integration_mode(dev, AS7341_INT_MODE_SYNS);
    sdm_as7341_set_gpio_state(dev, AS7341_GPIO_INPUT);

    
    sdm_as7341_set_integration_time(dev, SDM_AS7341_ATIME, SDM_AS7341_ASTEP);
    sdm_as7341_set_gain(dev, SDM_AS7341_GAIN);

    
    sdm_as7341_set_wait_time(dev, SDM_AS7341_WTIME);

    // enable device
    sdm_as7341_enable(dev, true);
    vTaskDelay(15);  // auto zero

    return ESP_OK;
}

esp_err_t sdm_as7341_write_reg(sdm_as7341_t *dev, uint8_t reg, uint8_t data_wr)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_wr, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        dev->i2c_port,
        cmd,
        pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing to register 0x%02X", reg);
    }

    return ret;
}

esp_err_t sdm_as7341_read_reg(sdm_as7341_t *dev, uint8_t reg, uint8_t *data_rd)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    // Read data
    i2c_master_start(cmd);  // repeat start for read
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);  // Read one byte
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading from register 0x%02X", reg);
    }

    return ret;
}

esp_err_t sdm_as7341_enable(sdm_as7341_t *dev, bool enable)
{
    esp_err_t ret;
    uint8_t enable_reg;

    // Read the current value of AS7341_ENABLE register
    sdm_as7341_read_reg(dev, AS7341_ENABLE, &enable_reg);

    // Set or clear the enable bit (bit 0)
    if (enable)
    {
        enable_reg |= (1 << 0);
    }
    else
    {
        enable_reg &= ~(1 << 0);
    }

    // Write the modified register
    ret = sdm_as7341_write_reg(dev, AS7341_ENABLE, enable_reg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to %s device", enable ? "enable" : "disable");
    }

    return ret;
}

esp_err_t sdm_as7341_enable_spectral_measurement(sdm_as7341_t *dev, bool enable)
{
    esp_err_t ret;
    uint8_t enable_reg;

    // Read the current value of AS7341_ENABLE register
    sdm_as7341_read_reg(dev, AS7341_ENABLE, &enable_reg);

    // Set or clear the spectral measurement enable bit (bit 1)
    if (enable)
    {
        enable_reg |= (1 << 1);
    }
    else
    {
        enable_reg &= ~(1 << 1);
    }

    // Write the modified register
    ret = sdm_as7341_write_reg(dev, AS7341_ENABLE, enable_reg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to %s spectral measurement", enable ? "enable" : "disable");
    }

    return ret;
}

esp_err_t sdm_as7341_enable_wait(sdm_as7341_t *dev, bool enable)
{
    esp_err_t ret;
    uint8_t enable_reg;

    // Read the current value of AS7341_ENABLE register
    sdm_as7341_read_reg(dev, AS7341_ENABLE, &enable_reg);

    // Set or clear the wait enable bit (bit 3)
    if (enable)
    {
        enable_reg |= (1 << 3);
    }
    else
    {
        enable_reg &= ~(1 << 3);
    }

    // Write the modified register
    ret = sdm_as7341_write_reg(dev, AS7341_ENABLE, enable_reg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to %s wiat", enable ? "enable" : "disable");
    }

    return ret;
}

esp_err_t sdm_as7341_enable_smux(sdm_as7341_t *dev, bool enable)
{
    esp_err_t ret;
    uint8_t enable_reg;

    // Read the current value of AS7341_ENABLE register
    sdm_as7341_read_reg(dev, AS7341_ENABLE, &enable_reg);

    // Set or clear the smux enable bit (bit 4)
    if (enable)
    {
        enable_reg |= (1 << 4);
    }
    else
    {
        enable_reg &= ~(1 << 4);
    }

    // Write the modified register
    ret = sdm_as7341_write_reg(dev, AS7341_ENABLE, enable_reg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to %s smux", enable ? "enable" : "disable");
    }

    return ret;
}

esp_err_t sdm_as7341_set_reg_bank(sdm_as7341_t *dev, sdm_as7341_reg_bank_t bank)
{
    esp_err_t ret;
    uint8_t cfg0_reg;

    // Read the current value of AS7341_CFG0 register
    sdm_as7341_read_reg(dev, AS7341_CFG0, &cfg0_reg);

    // Set or clear reg back bit (bit 4)
    switch (bank)
    {
        case AS7341_REG_BANK_L:
            cfg0_reg |= (1 << 4);
            break;

        case AS7341_REG_BANK_H:
            cfg0_reg &= ~(1 << 4);
            break;

        default:
            ESP_LOGE(TAG, "Wrong register bank selected.");
            return ESP_ERR_INVALID_ARG;
    }

    ret = sdm_as7341_write_reg(dev, AS7341_CFG0, cfg0_reg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write to cfg0.");
    }

    return ret;
}

esp_err_t sdm_as7341_set_integration_mode(sdm_as7341_t *dev, sdm_as7341_int_mode_t mode)
{
    esp_err_t ret;
    uint8_t config_reg;

    // Read the current value of AS7341_CONFIG register
    sdm_as7341_read_reg(dev, AS7341_CONFIG, &config_reg);

    // Clear the current mode bits (bits 1:0)
    config_reg &= ~(1 << 0);
    config_reg &= ~(1 << 1);

    // Set the appropriate mode bits
    switch (mode)
    {
        case AS7341_INT_MODE_SPM:
            break;
        case AS7341_INT_MODE_SYNS:
            config_reg |= (1 << 0);  // set bit 0
            break;
        case AS7341_INT_MODE_RESERVED:
            config_reg |= (1 << 1);  // set bit 1
            break;
        case AS7341_INT_MODE_SYND:
            config_reg |= (1 << 0);  // set bit 0
            config_reg |= (1 << 1);  // set bit 1
            break;
        default:
            ESP_LOGE(TAG, "Invalid integration mode specified.");
            return ESP_ERR_INVALID_ARG;
    }

    ret = sdm_as7341_write_reg(dev, AS7341_CONFIG, config_reg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set AS7341 mode");
    }

    return ret;
}

esp_err_t sdm_as7341_set_integration_time(sdm_as7341_t *dev, uint8_t atime, uint16_t astep)
{
    esp_err_t ret;

    // Set ATIME (0x81)
    ret = sdm_as7341_write_reg(dev, AS7341_ATIME, atime);
    if (ret != ESP_OK) return ret;

    // Set ASTEP_L (0xCA)
    ret = sdm_as7341_write_reg(dev, AS7341_ASTEP_L, astep & 0xFF);
    if (ret != ESP_OK) return ret;

    // Set ASTEP_H (0xCB)
    ret = sdm_as7341_write_reg(dev, AS7341_ASTEP_H, (astep >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t sdm_as7341_set_gain(sdm_as7341_t *dev, sdm_as7341_gain_t gain)
{
    // Gain is set in CFG1 register (0xAA)
    // The gain value is in the lower 4 bits of CFG1
    return sdm_as7341_write_reg(dev, AS7341_CFG1, gain & 0x0F);
}


esp_err_t sdm_as7341_set_wait_time(sdm_as7341_t *dev, uint8_t wtime)
{
    esp_err_t ret;

    // Set WTIME (0x83)
    ret = sdm_as7341_write_reg(dev, AS7341_WTIME, wtime);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t sdm_as7341_set_gpio_state(sdm_as7341_t *dev, sdm_as7341_gpio_mode_t gpio_mode)
{
    esp_err_t ret;
    uint8_t gpio2_reg;

    gpio2_reg = sdm_as7341_read_reg(dev, AS7341_GPIO2, &gpio2_reg);
    gpio2_reg &= ~(0x0E);  // clear bit 3:1

    // Set gpio mode
    switch (gpio_mode)
    {
        case AS7341_GPIO_INPUT:
            gpio2_reg |= (1 << 2);  // set bit 2
            break;
        
        case AS7341_GPIO_OUTPUT:
            gpio2_reg |= (1 << 1);  // set bit 1
            break;

        default:
            ESP_LOGE(TAG, "Attempted to set a non-valid GPIO mode.");
            return ESP_ERR_INVALID_ARG;
    }
    ret = sdm_as7341_write_reg(dev, AS7341_GPIO2, gpio2_reg);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t sdm_as7341_start_measure(sdm_as7341_t *dev, sdm_channel_mapping_mode_t mode)
{
    esp_err_t ret;

    // Disable SMUX to apply configuration
    ret = sdm_as7341_enable_smux(dev, false);
    if (ret != ESP_OK) return ret;

    // Load SMUX configuration
    ret = sdm_as7341_write_reg(dev, AS7341_CFG6, 0x10);
    if (ret != ESP_OK) return ret;

    // Configure SMUX
    if (mode == AS7341_CH_F1F4_CLEAR_NIR) ret = sdm_as7341_setup_f1f4_clear_nir(dev);
    else if (mode == AS7341_CH_F5F8_CLEAR_NIR) ret = sdm_as7341_setup_f5f8_clear_nir(dev);
    else return ESP_ERR_INVALID_ARG;
    if (ret != ESP_OK) return ret;

    // Enable spectral measurement
    ret = sdm_as7341_enable_spectral_measurement(dev, true);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t sdm_as7341_setup_f1f4_clear_nir(sdm_as7341_t *dev)
{
    esp_err_t ret;

    // SMUX configuration for F1-F4, Clear, NIR
    // These values are based on the AS7341 datasheet
    uint8_t smux_conf[] = {
        0x00, 0x00,
        0x01, 0x30,
        0x02, 0x01,
        0x03, 0x00,
        0x04, 0x00,
        0x05, 0x42,
        0x06, 0x00,
        0x07, 0x00,
        0x08, 0x50,
        0x09, 0x00,
        0x0A, 0x00,
        0x0B, 0x00,
        0x0C, 0x00,
    };

    // Write SMUX configuration
    for (int i = 0; i < sizeof(smux_conf) / 2; i++)
    {
        uint8_t reg = smux_conf[i * 2];
        uint8_t value = smux_conf[i * 2 + 1];
        ret = sdm_as7341_write_reg(dev, reg, value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write SMUX reg 0x%02X", reg);
            return ret;
        }
    }

    // Load SMUX configuration
    ret = sdm_as7341_write_reg(dev, AS7341_CFG6, 0x01);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load SMUX configuration");
    }

    return ret;
}

esp_err_t sdm_as7341_setup_f5f8_clear_nir(sdm_as7341_t *dev)
{
    esp_err_t ret;

    // SMUX configuration for F5-F8, Clear, NIR
    uint8_t smux_conf[] = {
        0x00, 0x00,
        0x01, 0x00,
        0x02, 0x00,
        0x03, 0x00,
        0x04, 0x00,
        0x05, 0x00,
        0x06, 0x04,
        0x07, 0x30,
        0x08, 0x00,
        0x09, 0x24,
        0x0A, 0x00,
        0x0B, 0x00,
        0x0C, 0x00,
    };

    // Write SMUX configuration
    for (int i = 0; i < sizeof(smux_conf) / 2; i++)
    {
        uint8_t reg = smux_conf[i * 2];
        uint8_t value = smux_conf[i * 2 + 1];
        ret = sdm_as7341_write_reg(dev, reg, value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write SMUX reg 0x%02X", reg);
            return ret;
        }
    }

    // Load SMUX configuration
    ret = sdm_as7341_write_reg(dev, AS7341_CFG6, 0x01);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load SMUX configuration");
    }

    return ret;
}

esp_err_t sdm_as7341_read_channel_data(sdm_as7341_t *dev, uint16_t *data)
{
    esp_err_t ret;

    // Check if data is ready
    uint8_t status2;
    sdm_as7341_read_reg(dev, AS7341_STATUS2, &status2);

    if (!(status2 & 0x40))
    {  // Bit 6 indicates measurement complete
        ESP_LOGW(TAG, "Measurement not completed, STATUS2: 0x%02X", status2);
        return ESP_ERR_INVALID_STATE;
    }

    // Read 6 channels
    for (int i = 0; i < 6; i++)
    {
        uint8_t data_low = 0, data_high = 0;
        ret = sdm_as7341_read_reg(dev, AS7341_CH0_DATA_L + i * 2, &data_low);
        if (ret != ESP_OK) return ret;
        ret = sdm_as7341_read_reg(dev, AS7341_CH0_DATA_H + i * 2, &data_high);
        if (ret != ESP_OK) return ret;

        data[i] = ((uint16_t)data_high << 8) | data_low;
    }

    return ESP_OK;
}

// esp_err_t sdm_as7341_read_all_channels(sdm_as7341_t *dev, uint16_t *data)
// {
//     esp_err_t ret;

//     // First measurement: F1-F4, Clear, NIR
//     ret = sdm_as7341_start_measure(dev, AS7341_CH_F1F4_CLEAR_NIR);
//     if (ret != ESP_OK) return ret;

//     // Wait for measurement to complete
//     vTaskDelay(pdMS_TO_TICKS(15));  // Adjust based on integration time

//     // Read data
//     ret = sdm_as7341_read_channel_data(dev, data);  // data[0..5]
//     if (ret != ESP_OK) return ret;

//     // Disable spectral measurement
//     ret = sdm_as7341_enable_spectral_measurement(dev, false);
//     if (ret != ESP_OK) return ret;

//     // Second measurement: F5-F8, Clear, NIR
//     ret = sdm_as7341_start_measure(dev, AS7341_CH_F5F8_CLEAR_NIR);
//     if (ret != ESP_OK) return ret;

//     // Wait for measurement to complete
//     vTaskDelay(pdMS_TO_TICKS(15));  // Adjust based on integration time

//     // Read data
//     ret = sdm_as7341_read_channel_data(dev, &data[6]);  // data[6..11]
//     if (ret != ESP_OK) return ret;

//     // Disable spectral measurement
//     ret = sdm_as7341_enable_spectral_measurement(dev, false);
//     if (ret != ESP_OK) return ret;

//     return ESP_OK;
// }