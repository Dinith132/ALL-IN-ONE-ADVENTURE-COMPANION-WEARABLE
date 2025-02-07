#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#define GPS_BAUD 9600   // Baud rate for the GPS module
#define RXD2 16         // RX pin for Serial2 (adjust as needed)
#define TXD2 17         // TX pin for Serial2 (adjust as needed)
#define BUF_SIZE (1024) // Buffer size for reading from UART

// UART port number for the GPS module
#define UART_NUM_UART2 2

// Create a tag for logging
static const char *TAG = "GPS";

void app_main() {
    // Configure UART parameters for GPS
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_UART2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_UART2, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_UART2, BUF_SIZE, BUF_SIZE, 0, NULL, 0));

    // Buffer for reading data
    uint8_t data[BUF_SIZE];

    // Variable to hold the NMEA sentence
    char nmeaLine[BUF_SIZE] = {0};

    while (1) {
        // Read data from the GPS module
        int len = uart_read_bytes(UART_NUM_UART2, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char inChar = data[i];

                // Accumulate NMEA sentence
                if (inChar != '\n' && inChar != '\r') {
                    strncat(nmeaLine, &inChar, 1); // Append character to string
                }

                // When newline is received, process the sentence
                if (inChar == '\n') {
                    // Print the NMEA sentence
                    ESP_LOGI(TAG, "GPS Data: %s", nmeaLine);

                    // Optionally, parse the sentence here if needed
                    // For example, check if it starts with "$GPRMC", "$GPGGA", etc.

                    // Clear the sentence buffer for the next line
                    memset(nmeaLine, 0, sizeof(nmeaLine)); // Clear the buffer
                }
            }
        }

        // Optionally add a small delay
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
