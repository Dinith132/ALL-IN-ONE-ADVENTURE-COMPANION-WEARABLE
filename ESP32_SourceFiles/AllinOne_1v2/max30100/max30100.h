#ifndef MAX30100_H
#define MAX30100_H

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
/*#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_FREQ_HZ         400000*/
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

static const char *TAG = "MAX30100";

// Function Prototypes
//esp_err_t i2c_master_init(void);
static esp_err_t max30100_write_reg(uint8_t reg_addr, uint8_t data);
static esp_err_t max30100_read_reg(uint8_t reg_addr, uint8_t *data);
esp_err_t max30100_read_fifo(uint8_t *data, size_t len);
void max30100_init(void);


#endif // MAX30100_H
