#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

// MAX30100 Register Addresses
#define MAX30100_I2C_ADDR          0x57
#define MAX30100_REG_INT_STATUS    0x00
#define MAX30100_REG_INT_ENABLE    0x01
#define MAX30100_REG_FIFO_WR_PTR   0x02
#define MAX30100_REG_FIFO_RD_PTR   0x03
#define MAX30100_REG_FIFO_DATA     0x05
#define MAX30100_REG_MODE_CONFIG   0x06
#define MAX30100_REG_SPO2_CONFIG   0x07
#define MAX30100_REG_LED_CONFIG    0x09
#define MAX30100_REG_TEMP_INT      0x16

// I2C Configuration
#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

static const char *TAG = "MAX30100";

// Function Prototypes
static esp_err_t i2c_master_init(void);
static esp_err_t max30100_write_reg(uint8_t reg_addr, uint8_t data);
static esp_err_t max30100_read_reg(uint8_t reg_addr, uint8_t *data);
static esp_err_t max30100_read_fifo(uint8_t *data, size_t len);
static void max30100_init(void);
static void max30100_task(void *pvParameters);

// Global variables for storing sensor data
static float heart_rate = 0;
static float spo2 = 0;

// I2C Master Initialization
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_NUM_0, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Write to MAX30100 register
static esp_err_t max30100_write_reg(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read from MAX30100 register
static esp_err_t max30100_read_reg(uint8_t reg_addr, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read FIFO data
static esp_err_t max30100_read_fifo(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MAX30100_REG_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Initialize MAX30100
static void max30100_init(void)
{
    // Reset the sensor
    max30100_write_reg(MAX30100_REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure mode (HR + SpO2)
    max30100_write_reg(MAX30100_REG_MODE_CONFIG, 0x03);

    // SpO2 configuration
    // Sample rate: 100Hz, LED pulse width: 1600us, ADC range: 16384
    max30100_write_reg(MAX30100_REG_SPO2_CONFIG, 0x27);

    // LED current: Red=7.6mA, IR=7.6mA
    max30100_write_reg(MAX30100_REG_LED_CONFIG, 0x77);
}

// Main sensor reading task
static void max30100_task(void *pvParameters)
{
    uint8_t fifo_data[4];
    uint16_t ir_data, red_data;
    
    while (1) {
        // Read FIFO data (2 bytes IR + 2 bytes Red)
        if (max30100_read_fifo(fifo_data, 4) == ESP_OK) {
            ir_data = (fifo_data[0] << 8) | fifo_data[1];
            red_data = (fifo_data[2] << 8) | fifo_data[3];

            // Simple heart rate calculation (this is a basic implementation)
            // In a real application, you would need more sophisticated algorithms
            heart_rate = (float)ir_data / 1000.0;
            spo2 = (float)red_data / (float)ir_data * 100.0;

            // Limit values to realistic ranges
            if (heart_rate < 50) heart_rate = 50;
            if (heart_rate > 180) heart_rate = 180;
            if (spo2 > 100) spo2 = 100;
            if (spo2 < 80) spo2 = 80;

            ESP_LOGI(TAG, "Heart Rate: %.1f bpm, SpO2: %.1f%%", heart_rate, spo2);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz sampling rate
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing MAX30100...");

    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    
    // Initialize MAX30100 sensor
    max30100_init();
    
    // Create task for sensor reading
    xTaskCreate(max30100_task, "max30100_task", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "MAX30100 initialization complete");
}
