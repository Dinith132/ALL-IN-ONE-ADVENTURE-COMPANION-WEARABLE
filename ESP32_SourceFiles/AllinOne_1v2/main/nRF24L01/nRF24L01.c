/*
 * nRF24L01.c
 *
 *  Created on: Feb 12, 2025
 *      Author: panka
 */

#include "nRF24L01.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "nrf24";

// Static variables to hold SPI device handle and CE pin number.
static spi_device_handle_t s_spi = NULL;
static int s_ce_pin = -1;

// Helper functions to control the CE pin.
static inline void nrf24_ce_high(void) {
    gpio_set_level(s_ce_pin, 1);
}

static inline void nrf24_ce_low(void) {
    gpio_set_level(s_ce_pin, 0);
}

// Write to a register (includes command and data bytes)
static esp_err_t nrf24_write_register(uint8_t reg, const uint8_t *data, size_t len) {
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = NULL,
    };

    uint8_t *tx_buf = malloc(len + 1);
    if (tx_buf == NULL) return ESP_ERR_NO_MEM;

    tx_buf[0] = NRF24_CMD_W_REGISTER | (reg & 0x1F);
    memcpy(tx_buf + 1, data, len);
    t.tx_buffer = tx_buf;

    esp_err_t ret = spi_device_transmit(s_spi, &t);
    free(tx_buf);
    return ret;
}

// Read from a register.
static esp_err_t nrf24_read_register(uint8_t reg, uint8_t *data, size_t len) {
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };

    uint8_t *tx_buf = calloc(len + 1, 1);
    uint8_t *rx_buf = calloc(len + 1, 1);
    if (tx_buf == NULL || rx_buf == NULL) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }
    tx_buf[0] = NRF24_CMD_R_REGISTER | (reg & 0x1F);
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    esp_err_t ret = spi_device_transmit(s_spi, &t);
    if (ret == ESP_OK) {
        memcpy(data, rx_buf + 1, len);
    }
    free(tx_buf);
    free(rx_buf);
    return ret;
}

// Write payload for transmission.
static esp_err_t nrf24_write_payload(const uint8_t *data, size_t len) {
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = NULL,
    };

    uint8_t *tx_buf = malloc(len + 1);
    if (tx_buf == NULL) return ESP_ERR_NO_MEM;
    tx_buf[0] = NRF24_CMD_W_TX_PAYLOAD;
    memcpy(tx_buf + 1, data, len);
    t.tx_buffer = tx_buf;

    esp_err_t ret = spi_device_transmit(s_spi, &t);
    free(tx_buf);
    return ret;
}

// Read payload received.
static esp_err_t nrf24_read_payload(uint8_t *data, size_t len) {
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };

    uint8_t *tx_buf = calloc(len + 1, 1);
    uint8_t *rx_buf = calloc(len + 1, 1);
    if (tx_buf == NULL || rx_buf == NULL) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }
    tx_buf[0] = NRF24_CMD_R_RX_PAYLOAD;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    esp_err_t ret = spi_device_transmit(s_spi, &t);
    if (ret == ESP_OK) {
        memcpy(data, rx_buf + 1, len);
    }
    free(tx_buf);
    free(rx_buf);
    return ret;
}

// Public API implementations

void nrf24_init(spi_device_handle_t spi, int ce_pin) {
    s_spi = spi;
    s_ce_pin = ce_pin;
    gpio_reset_pin(ce_pin);
    gpio_set_direction(ce_pin, GPIO_MODE_OUTPUT);
    nrf24_ce_low();
    // Allow the module to power up.
    vTaskDelay(pdMS_TO_TICKS(100));
}

bool nrf24_is_present(void) {
    uint8_t testAddr[5] = {'N','R','F','2','4'};
    if (nrf24_write_register(NRF24_REG_TX_ADDR, testAddr, 5) != ESP_OK) {
        return false;
    }
    uint8_t readAddr[5] = {0};
    if (nrf24_read_register(NRF24_REG_TX_ADDR, readAddr, 5) != ESP_OK) {
        return false;
    }
    return (memcmp(testAddr, readAddr, 5) == 0);
}

void nrf24_setPALevel(uint8_t level) {
    uint8_t rf_setup;
    nrf24_read_register(NRF24_REG_RF_SETUP, &rf_setup, 1);
    // Clear PA bits (bits 2:1)
    rf_setup &= ~(0x06);
    switch (level) {
        case NRF24_PA_MIN:
            rf_setup |= 0x00;
            break;
        case NRF24_PA_LOW:
            rf_setup |= 0x02;
            break;
        case NRF24_PA_HIGH:
            rf_setup |= 0x04;
            break;
        case NRF24_PA_MAX:
            rf_setup |= 0x06;
            break;
        default:
            break;
    }
    nrf24_write_register(NRF24_REG_RF_SETUP, &rf_setup, 1);
}

void nrf24_setRetries(uint8_t delay, uint8_t count) {
    uint8_t retr = ((delay & 0x0F) << 4) | (count & 0x0F);
    nrf24_write_register(NRF24_REG_SETUP_RETR, &retr, 1);
}

void nrf24_setDataRate(uint8_t data_rate) {
    uint8_t rf_setup;
    nrf24_read_register(NRF24_REG_RF_SETUP, &rf_setup, 1);
    // Clear data rate bits (bit 5 and bit 3)
    rf_setup &= ~((1 << 5) | (1 << 3));
    if (data_rate == NRF24_250KBPS) {
        rf_setup |= (1 << 5);
    } else if (data_rate == NRF24_2MBPS) {
        rf_setup |= (1 << 3);
    }
    nrf24_write_register(NRF24_REG_RF_SETUP, &rf_setup, 1);
}

void nrf24_setChannel(uint8_t channel) {
    nrf24_write_register(NRF24_REG_RF_CH, &channel, 1);
}

void nrf24_open_writing_pipe(const uint8_t *address) {
    nrf24_write_register(NRF24_REG_TX_ADDR, address, 5);
    nrf24_write_register(NRF24_REG_RX_ADDR_P0, address, 5);
}

void nrf24_open_reading_pipe(uint8_t pipe, const uint8_t *address) {
    if (pipe > 5) return;
    if (pipe == 0) {
        nrf24_write_register(NRF24_REG_RX_ADDR_P0, address, 5);
    } else if (pipe == 1) {
        nrf24_write_register(NRF24_REG_RX_ADDR_P0 + pipe, address, 5);
    } else {
        // For pipes 2-5, only the least significant byte is written.
        nrf24_write_register(NRF24_REG_RX_ADDR_P0 + pipe, address, 1);
    }
    uint8_t payload_size = 32;
    uint8_t reg = NRF24_REG_RX_PW_P0 + pipe;
    nrf24_write_register(reg, &payload_size, 1);
}

void nrf24_start_listening(void) {
    uint8_t config;
    nrf24_read_register(NRF24_REG_CONFIG, &config, 1);
    // Set PRIM_RX (bit 0) and PWR_UP (bit 1)
    config |= (1 << 0) | (1 << 1);
    nrf24_write_register(NRF24_REG_CONFIG, &config, 1);
    nrf24_ce_high();
    vTaskDelay(pdMS_TO_TICKS(2));
}

void nrf24_stop_listening(void) {
    nrf24_ce_low();
    uint8_t config;
    nrf24_read_register(NRF24_REG_CONFIG, &config, 1);
    config &= ~(1 << 0); // Clear PRIM_RX
    nrf24_write_register(NRF24_REG_CONFIG, &config, 1);
}

bool nrf24_write(const uint8_t *buf, uint8_t len) {
    nrf24_stop_listening();
    if (nrf24_write_payload(buf, len) != ESP_OK) {
        return false;
    }
    // In a production driver, you would check IRQ flags to confirm TX success.
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

bool nrf24_available(void) {
    uint8_t fifo_status;
    nrf24_read_register(NRF24_REG_FIFO_STATUS, &fifo_status, 1);
    // If bit 0 (RX_EMPTY) is clear, data is available.
    return !(fifo_status & 0x01);
}

void nrf24_read(uint8_t *buf, uint8_t len) {
    nrf24_read_payload(buf, len);
}
