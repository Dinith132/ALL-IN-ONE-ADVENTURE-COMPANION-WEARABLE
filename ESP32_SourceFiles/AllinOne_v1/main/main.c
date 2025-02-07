#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>  // For PRIu32
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // Include for mutex
#include "nvs_flash.h"
#include "neo6m/neo6m.h"
#include "max30100/max30100.h"

#define SPP_SERVER_NAME "SPP_SERVER"
#define SPP_TAG "SPP_BIDIRECTIONAL"

static uint32_t spp_handle = 0;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static double lan = 0.0, lot = 0.0;

static SemaphoreHandle_t coordinates_mutex = NULL; // Mutex to protect coordinates

/* GPS Module Task */
void neo6m_task(void *arg) {
    gps_init();  // Ensure proper initialization.

    while (1) {
        // Update coordinates in a thread-safe way
        if (xSemaphoreTake(coordinates_mutex, portMAX_DELAY)) {
            GPS_Coordinates coordinate = get_gps_coordinates();
            lan = coordinate.latitude;
            lot = coordinate.longitude;
            xSemaphoreGive(coordinates_mutex); // Release mutex after updating coordinates
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

/**
 * Sends data via Bluetooth SPP
 */
static inline void spp_send_message(const char *message) {
    if (spp_handle) {  
        esp_spp_write(spp_handle, strlen(message), (uint8_t *)message);
        ESP_LOGI(SPP_TAG, "Sent: %s", message);
    } else {
        ESP_LOGW(SPP_TAG, "SPP handle invalid, cannot send data.");
    }
}

/**
 * Task to send data periodically
 */
void send_task(void *arg) {
    while (1) {
        // Send coordinates or an error message if not available
        if (xSemaphoreTake(coordinates_mutex, portMAX_DELAY)) {
            if (true) { // Assuming GPS data is valid for now
                char msg[50];  // Buffer to hold formatted message
                snprintf(msg, sizeof(msg), "Lat: %.6f, Lon: %.6f\n", lan, lot);
                spp_send_message(msg);
            } else {
                spp_send_message("Invalid GPS response\n");
            }
            xSemaphoreGive(coordinates_mutex); // Release mutex after sending data
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
    }
}

/**
 * Read MAX30100 sensor data
 */
void read_max30100(void *arg) {
    uint8_t sensor_data;
    while (1) {
        if (max30100_read_register(0x05, &sensor_data, 1) == ESP_OK) {
            ESP_LOGI("MAX30100", "Sensor Data: 0x%02X", sensor_data);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

/**
 * SPP Callback Function
 */
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_spp_vfs_register();
            break;

        case ESP_SPP_START_EVT:
            esp_bt_gap_set_device_name(SPP_SERVER_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            spp_handle = param->srv_open.handle;
            ESP_LOGI(SPP_TAG, "SPP Connected, handle: %" PRIu32, spp_handle);
            spp_send_message("ESP32 connected! Ready to send & receive data.\n");
            xTaskCreatePinnedToCore(send_task, "send_task", 3072, NULL, 5, NULL, 1);
            break;

        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(SPP_TAG, "SPP Disconnected, handle: %" PRIu32, param->close.handle);
            spp_handle = 0;
            break;

        case ESP_SPP_DATA_IND_EVT:
            if (param->data_ind.len > 0) {
                char received_data[256] = {0};
                size_t copy_len = param->data_ind.len < sizeof(received_data) ? param->data_ind.len : sizeof(received_data) - 1;
                memcpy(received_data, param->data_ind.data, copy_len);
                received_data[copy_len] = '\0'; // Ensure null termination

                ESP_LOGI(SPP_TAG, "Received %d bytes: %s", param->data_ind.len, received_data);
                ESP_LOG_BUFFER_HEX(SPP_TAG, param->data_ind.data, param->data_ind.len);
            }
            break;

        case ESP_SPP_VFS_REGISTER_EVT:
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
            break;

        default:
            break;
    }
}

/**
 * GAP Callback Function
 */
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_AUTH_CMPL_EVT) {
        ESP_LOGI(SPP_TAG, "Authentication complete, status: %d", param->auth_cmpl.stat);
    } else if (event == ESP_BT_GAP_PIN_REQ_EVT) {
        esp_bt_pin_code_t pin_code = { '1', '2', '3', '4' };
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
    }
}

/**
 * Main Application Entry Point
 */
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create the mutex for coordinates synchronization
    coordinates_mutex = xSemaphoreCreateMutex();
    if (coordinates_mutex == NULL) {
        ESP_LOGE(SPP_TAG, "Failed to create mutex for coordinates.");
        return;  // Fail if mutex creation fails
    }

    // Start the GPS task (on Core 1)
    xTaskCreatePinnedToCore(neo6m_task, "neo6m_task", 4096, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(read_max30100, "read_max30100", 4096, NULL, 5, NULL, 1);


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));

    ESP_ERROR_CHECK(esp_spp_init(esp_spp_mode));

    ESP_LOGI(SPP_TAG, "Bluetooth SPP ready, waiting for connections...");
}
