#include "neo6m.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GPS";
static char nmeaLine[BUF_SIZE] = {0};
static GPS_Coordinates lastCoords = {0.0, 0.0}; // Stores the latest valid coordinates

/**
 * @brief Initializes the GPS module (Neo-6M) with UART configuration.
 */
void gps_init(void) {
    // Configure UART parameters for GPS
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
}

/**
 * @brief Task to read and process GPS data from UART.
 */
void gps_start(void) {
    uint8_t data[BUF_SIZE];

    
        int len = uart_read_bytes(GPS_UART_PORT, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char inChar = data[i];

                // Accumulate NMEA sentence
                if (inChar != '\n' && inChar != '\r') {
                    strncat(nmeaLine, &inChar, 1);
                }

                // When newline is received, process the sentence
                if (inChar == '\n') {
                    ESP_LOGI(TAG, "GPS Data: %s", nmeaLine);
                    parse_gpgll(nmeaLine);
                    memset(nmeaLine, 0, sizeof(nmeaLine)); // Clear buffer
                }
            }
        }
        
}

/**
 * @brief Parses a GPGLL sentence and updates last known coordinates.
 */
void parse_gpgll(const char *nmea) {
    char latitude[15], longitude[15], lat_dir, lon_dir;
    char utc_time[10], status, mode;

    if (sscanf(nmea, "$GPGLL,%[^,],%c,%[^,],%c,%[^,],%c,%c*", latitude, &lat_dir, longitude, &lon_dir, utc_time, &status, &mode) == 7) {
        if (status == 'A') {  // 'A' means valid data
            double lat_raw = atof(latitude);
            double lon_raw = atof(longitude);

            // Convert NMEA format (DDMM.MMMM) to decimal degrees (DD.DDDDDD)
            int lat_deg = (int)(lat_raw / 100);
            double lat_min = lat_raw - (lat_deg * 100);
            double lat_dd = lat_deg + (lat_min / 60.0);

            int lon_deg = (int)(lon_raw / 100);
            double lon_min = lon_raw - (lon_deg * 100);
            double lon_dd = lon_deg + (lon_min / 60.0);

            // Adjust for S/W hemispheres
            if (lat_dir == 'S') lat_dd *= -1;
            if (lon_dir == 'W') lon_dd *= -1;

            // Update last known coordinates
            lastCoords.latitude = lat_dd;
            lastCoords.longitude = lon_dd;

            ESP_LOGI(TAG, "Latitude: %.6f %c", lat_dd, lat_dir);
            ESP_LOGI(TAG, "Longitude: %.6f %c", lon_dd, lon_dir);
        } else {
            ESP_LOGI(TAG, "No valid GPS fix");
        }
    } else {
        ESP_LOGI(TAG, "Invalid GPGLL format");
    }
}

/**
 * @brief Returns the last known GPS coordinates.
 */
GPS_Coordinates get_gps_coordinates(void) {
	gps_start();
    return lastCoords;
}
