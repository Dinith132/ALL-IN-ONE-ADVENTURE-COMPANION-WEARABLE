/*
 * nRF24L01.h
 *
 *  Created on: Feb 12, 2025
 *      Author: panka
 */

#ifndef MAIN_NRF24L01_NRF24L01_H_
#define MAIN_NRF24L01_NRF24L01_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// nRF24L01+ registers
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA       0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW    0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH       0x05
#define NRF24_REG_RF_SETUP    0x06
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_RX_ADDR_P0  0x0A
#define NRF24_REG_TX_ADDR     0x10
#define NRF24_REG_RX_PW_P0    0x11
#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD       0x1C
#define NRF24_REG_FEATURE     0x1D

// nRF24L01+ commands
#define NRF24_CMD_R_REGISTER  0x00
#define NRF24_CMD_W_REGISTER  0x20
#define NRF24_CMD_R_RX_PAYLOAD 0x61
#define NRF24_CMD_W_TX_PAYLOAD 0xA0
#define NRF24_CMD_FLUSH_TX    0xE1
#define NRF24_CMD_FLUSH_RX    0xE2
#define NRF24_CMD_NOP         0xFF

// PA levels
#define NRF24_PA_MIN  0
#define NRF24_PA_LOW  1
#define NRF24_PA_HIGH 2
#define NRF24_PA_MAX  3

// Data rates
#define NRF24_250KBPS 0
#define NRF24_1MBPS   1
#define NRF24_2MBPS   2

/**
 * @brief Initialize the nRF24L01 module.
 *
 * @param spi   SPI device handle (created by spi_bus_add_device)
 * @param ce_pin  GPIO pin number used for CE control.
 */
void nrf24_init(spi_device_handle_t spi, int ce_pin);

/**
 * @brief Check if the nRF24L01 module is present.
 *
 * @return true if present, false otherwise.
 */
bool nrf24_is_present(void);

/**
 * @brief Set the power amplifier level.
 *
 * @param level One of NRF24_PA_MIN, NRF24_PA_LOW, NRF24_PA_HIGH, NRF24_PA_MAX.
 */
void nrf24_setPALevel(uint8_t level);

/**
 * @brief Configure automatic retransmission.
 *
 * @param delay  Delay (in 250us increments)
 * @param count  Number of retries.
 */
void nrf24_setRetries(uint8_t delay, uint8_t count);

/**
 * @brief Set the data rate.
 *
 * @param data_rate One of NRF24_250KBPS, NRF24_1MBPS, NRF24_2MBPS.
 */
void nrf24_setDataRate(uint8_t data_rate);

/**
 * @brief Set the RF channel.
 *
 * @param channel Channel number (0-125)
 */
void nrf24_setChannel(uint8_t channel);

/**
 * @brief Open a writing pipe with the given 5-byte address.
 *
 * @param address Pointer to a 5-byte address.
 */
void nrf24_open_writing_pipe(const uint8_t *address);

/**
 * @brief Open a reading pipe.
 *
 * @param pipe     Pipe number (0 for full address, 1-5 for reduced addressing).
 * @param address  Pointer to the address (5 bytes for pipe 0/1; for others, only LSB is used).
 */
void nrf24_open_reading_pipe(uint8_t pipe, const uint8_t *address);

/**
 * @brief Put the nRF24L01 into listening (RX) mode.
 */
void nrf24_start_listening(void);

/**
 * @brief Take the nRF24L01 out of listening mode.
 */
void nrf24_stop_listening(void);

/**
 * @brief Write data (transmit payload).
 *
 * @param buf Pointer to the data buffer.
 * @param len Length of the data.
 * @return true if transmission was (assumed) successful.
 */
bool nrf24_write(const uint8_t *buf, uint8_t len);

/**
 * @brief Check if data is available in the RX FIFO.
 *
 * @return true if data available, false otherwise.
 */
bool nrf24_available(void);

/**
 * @brief Read received data into a buffer.
 *
 * @param buf Pointer to the buffer.
 * @param len Length of data to read.
 */
void nrf24_read(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // NRF24_H
