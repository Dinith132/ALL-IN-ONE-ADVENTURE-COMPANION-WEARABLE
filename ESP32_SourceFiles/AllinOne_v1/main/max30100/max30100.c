#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "max30100.h"

static const char *TAG = "MAX30100";

/**
 * @brief Initialize I2C for MAX30100
 */
esp_err_t max30100_i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

/**
 * @brief Write to MAX30100 register
 */
esp_err_t max30100_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_PORT, MAX30100_ADDRESS, data, sizeof(data), pdMS_TO_TICKS(100));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to register 0x%02X", reg);
    }
    return ret;
}

/**
 * @brief Read from MAX30100 register
 */
esp_err_t max30100_read_register(uint8_t reg, uint8_t *data, size_t len) {
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_PORT, MAX30100_ADDRESS, &reg, 1, data, len, pdMS_TO_TICKS(100));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X", reg);
    }
    return ret;
}

/**
 * @brief Initialize MAX30100 sensor
 */
esp_err_t max30100_init(void) {
    esp_err_t ret = max30100_write_register(0x06, 0x03); // Example: Set mode register
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MAX30100");
        return ret;
    }
    
    ESP_LOGI(TAG, "MAX30100 Initialized Successfully");
    return ESP_OK;
}
