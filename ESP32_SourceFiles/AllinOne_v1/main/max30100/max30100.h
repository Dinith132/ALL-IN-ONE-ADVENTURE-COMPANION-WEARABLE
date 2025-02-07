#ifndef MAX30100_H
#define MAX30100_H

#include "esp_err.h"
#include <stddef.h>

#define MAX30100_ADDRESS  0x57
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT I2C_NUM_0

esp_err_t max30100_i2c_init(void);
esp_err_t max30100_write_register(uint8_t reg, uint8_t value);
esp_err_t max30100_read_register(uint8_t reg, uint8_t *data, size_t len);
esp_err_t max30100_init(void);

#endif // MAX30100_H
