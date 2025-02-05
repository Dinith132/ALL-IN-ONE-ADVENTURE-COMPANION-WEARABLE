#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
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
#include "nvs_flash.h"


#define SPP_SERVER_NAME "SPP_SERVER"
#define SPP_TAG "SPP_BIDIRECTIONAL"

static uint32_t spp_handle = 0;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char local_device_name[] = "ESP32_SPP_BIDIRECTIONAL";

/**
 * Converts a Bluetooth address to a string
 */
static char *bda2str(const uint8_t *bda, char *str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }
    snprintf(str, size, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

/**
 * Sends data via Bluetooth SPP
 */
void spp_send_message(const char *message) {
    if (spp_handle != 0) {  // Ensure a connection is established
        esp_spp_write(spp_handle, strlen(message), (uint8_t *)message);
        ESP_LOGI(SPP_TAG, "Sent message: %s", message);
    } else {
        ESP_LOGW(SPP_TAG, "SPP handle is not valid, cannot send message.");
    }
}

/**
 * Task to send data periodically
 */
void send_task(void *arg) {
    while (1) {
        if (spp_handle != 0) {
            spp_send_message("Hello from ESP32!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));  // Send every 5 seconds
    }
}

/**
 * Callback for SPP events (handles sending & receiving data)
 */
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    char bda_str[18] = {0};

    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_vfs_register();
            break;

        case ESP_SPP_START_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
            esp_bt_gap_set_device_name(local_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT, handle:%" PRIu32 ", remote_bda:[%s]",
                     param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
            spp_handle = param->srv_open.handle;

            // Send initial message upon connection
            spp_send_message("ESP32 connected! Ready to send & receive data.\n");

            // Start the task that sends messages periodically
            xTaskCreate(send_task, "send_task", 4096, NULL, 5, NULL);
            break;

        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT handle:%" PRIu32, param->close.handle);
            spp_handle = 0;
            break;

        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(SPP_TAG, "Received %d bytes", param->data_ind.len);
            char received_data[256] = {0};
            if (param->data_ind.len < sizeof(received_data)) {
                memcpy(received_data, param->data_ind.data, param->data_ind.len);
                ESP_LOGI(SPP_TAG, "Received message: %s", received_data);
            } else {
                ESP_LOGW(SPP_TAG, "Received data too large to store");
            }
            break;

        case ESP_SPP_VFS_REGISTER_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_VFS_REGISTER_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
            break;

        default:
            ESP_LOGI(SPP_TAG, "Unhandled SPP Event: %d", event);
            break;
    }
}

/**
 * Callback for GAP events
 */
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            ESP_LOGI(SPP_TAG, "Authentication complete, status: %d", param->auth_cmpl.stat);
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(SPP_TAG, "PIN requested, default PIN: 1234");
            esp_bt_pin_code_t pin_code = { '1', '2', '3', '4' };
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;

        default:
            ESP_LOGI(SPP_TAG, "Unhandled GAP Event: %d", event);
            break;
    }
}

/**
 * Main application entry point
 */
void app_main(void) {
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release BLE memory (not needed for SPP)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Configure and initialize the BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(SPP_TAG, "BT Controller initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(SPP_TAG, "BT Controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(SPP_TAG, "Bluedroid initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(SPP_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register callbacks
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));

    // Initialize SPP
    ESP_ERROR_CHECK(esp_spp_init(esp_spp_mode));

    ESP_LOGI(SPP_TAG, "Bluetooth SPP ready, waiting for connections...");
}
