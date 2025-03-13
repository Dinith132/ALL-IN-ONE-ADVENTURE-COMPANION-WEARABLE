#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <Math.h>
#include <esp_task_wdt.h>
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
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "neo6m/neo6m.h"
#include "max30100/max30100.h"
#include "nRF24L01/mirf.h"
#include "driver/gpio.h"
#include "driver/i2s.h" //Include I2S
#include "max4466/max4466.h"
#include "driver/spi_master.h"
#include "oled/oled.h"


#define SPP_SERVER_NAME "AllInOne1"
#define SPP_TAG "AllInOne1"
#define BUTTON_PIN GPIO_NUM_33
#define DEBOUNCE_TIME_MS 50
#define AUDIO_BUFFER_SIZE 32

//nrf24 configarations

static uint8_t rx_addr[5] = {'T','X','A','D','R'};
static uint8_t tx_addr[5] = {'R','X','A','D','R'};
NRF24_t dev;
static SemaphoreHandle_t nrf24_mutex = NULL;

static QueueHandle_t gpio_evt_queue = NULL;
static bool transmitting = false;
static uint32_t spp_handle = 0;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

//static const uint8_t address[][5] = {"NODE1", "NODE2"};
/*static float heart_rate = 0;
static float spo2 = 0;*/

//message mutex
static SemaphoreHandle_t message_mutex = NULL;
static SemaphoreHandle_t input_message_mutex1 = NULL;
static SemaphoreHandle_t output_message_mutex1 = NULL;
static uint8_t received[32] = {0};
static uint8_t MessageRecived[32] = {0};
static uint8_t MessageSend[32] = {0};




// Structure to hold GPS coordinates
typedef struct {
    double latitude;
    double longitude;
    bool valid; // Indicate if data is valid
} gps_data_t;

typedef struct {
    double heart_rate;
    double spo2;
} max30100_data_t;

typedef struct {
    uint8_t rx_buffer[32];
    char tx_buffer[32];
    bool valid; // Indicate if data is valid
} nrf24_data_t;

static nrf24_data_t nrf24_data;
static gps_data_t gps_data; // Global variable to store GPS data
static gps_data_t gps_data1;

// Mutex to protect GPS data
static SemaphoreHandle_t gps_data_mutex = NULL;

extern SemaphoreHandle_t max30100_data_mutex = NULL;

static SemaphoreHandle_t gps_data_mutex1 = NULL;

extern SemaphoreHandle_t max30100_data_mutex1 = NULL;

static max30100_data_t max30100_data;
static max30100_data_t max30100_data1;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)(intptr_t) arg;  // Proper casting
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (gpio_evt_queue != NULL) {  // Check if queue is initialized
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // Yield if necessary
}


void button_task(void *arg) {
    uint32_t io_num;
    uint32_t last_press = 0;

    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            uint32_t now = xTaskGetTickCount();
            if((now - last_press) > pdMS_TO_TICKS(DEBOUNCE_TIME_MS)) {
                int level = gpio_get_level(io_num);
                if(level == 0) { // Pressed
                    //nrf24_start_transmission();
                    transmitting = true;
                    ESP_LOGI("BUTTON", "Transmission STARTED");
                } else { // Released
                   // nrf24_stop_transmission();
                    transmitting = false;
                    ESP_LOGI("BUTTON", "Transmission STOPPED");
                }
                last_press = now;
            }
        }
    }
}

/* GPS Module Task */
void neo6m_task(void *arg) {
    

    while (1) {
        GPS_Coordinates coordinate = get_gps_coordinates();
        bool valid = (coordinate.latitude != 0.0 && coordinate.longitude != 0.0); // Simple validity check
        if (xSemaphoreTake(gps_data_mutex, portMAX_DELAY) == pdTRUE) {
            gps_data.latitude = coordinate.latitude;
            gps_data.longitude = coordinate.longitude;
            gps_data.valid = valid;
            xSemaphoreGive(gps_data_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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
        if (xSemaphoreTake(gps_data_mutex, portMAX_DELAY) == pdTRUE &&
            xSemaphoreTake(max30100_data_mutex, portMAX_DELAY) == pdTRUE &&
            xSemaphoreTake(gps_data_mutex1, portMAX_DELAY) == pdTRUE &&
            xSemaphoreTake(max30100_data_mutex1, portMAX_DELAY) == pdTRUE &&
            xSemaphoreTake(input_message_mutex1, portMAX_DELAY) == pdTRUE) {

            char msg[512];

            if (gps_data.valid) {
                snprintf(msg, sizeof(msg),
                    "{\n"
                    "  \"message\": {\n"
                    "    \"message_id\":2,\n"
                    "    \"text\": \"%s\"\n"
                    "  },\n"
                    "  \"users\": [\n"
                    "    {\n"
                    "      \"user_id\": 1,\n"
                    "      \"location\": {\n"
                    "        \"latitude\": %.6lf,\n"
                    "        \"longitude\": %.6lf\n"
                    "      },\n"
                    "      \"heart rate\": {\n"
                    "        \"heart rate\": %.4lf,\n"
                    "        \"blood oxygen\": %.4lf\n"
                    "      }\n"
                    "    }\n"
                    "  ]\n"
                    "}",
                    MessageRecived,
                    (double)gps_data.latitude, (double)gps_data.longitude,
                    (double)max30100_data.heart_rate, (double)max30100_data.spo2);
            } else {
                snprintf(msg, sizeof(msg),
                    "{\n"
                    "  \"message\": {\n"
                    "    \"message_id\":2,\n"
                    "    \"text\": \"%s\"\n"
                    "  },\n"
                    "  \"users\": [\n"
                    "    {\n"
                    "      \"user_id\": 1,\n"
                    "      \"location\": {\n"
                    "        \"latitude\": 9.310265,\n"
                    "        \"longitude\":80.40112\n"
                    "      },\n"
                    "      \"heart rate\": {\n"
                    "        \"heart rate\": %.4lf,\n"
                    "        \"blood oxygen\": %.4lf\n"
                    "      }\n"
                    "    }\n"
                    "  ]\n"
                    "}",
                    MessageRecived,
                    (double)max30100_data1.heart_rate, (double)max30100_data1.spo2);
            }

            spp_send_message(msg);

            // Release mutexes safely
            xSemaphoreGive(gps_data_mutex);
            xSemaphoreGive(input_message_mutex1);
            xSemaphoreGive(max30100_data_mutex);
            xSemaphoreGive(gps_data_mutex1);
            xSemaphoreGive(max30100_data_mutex1);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * Read MAX30100 sensor data
 */
static void read_max30100(void *pvParameters) {
    uint8_t fifo_data[4];
    uint16_t ir_data, red_data;
    TickType_t mutex_timeout = pdMS_TO_TICKS(50); // Mutex timeout (50ms)
    
    // 1. Stack usage monitoring
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Task stack space: %u bytes", uxHighWaterMark);

    while (1) {
        // 2. Read FIFO data from sensor
        esp_err_t ret = max30100_read_fifo(fifo_data, 4);
        if (/*ret != ESP_OK*/false) {
            ESP_LOGE(TAG, "FIFO read error: 0x%x", ret);
            vTaskDelay(pdMS_TO_TICKS(10)); // Short delay before retry
            continue;
        }

        // 3. Extract IR and Red values
        ir_data = (fifo_data[0] << 8) | fifo_data[1];
        red_data = (fifo_data[2] << 8) | fifo_data[3];

        // 4. Validate sensor data (prevent division by zero)
        if (/*ir_data == 0 || red_data == 0*/false) {
            ESP_LOGW(TAG, "Invalid sensor data (IR: %u, RED: %u)", ir_data, red_data);
            vTaskDelay(pdMS_TO_TICKS(10)); // Short delay before retry
            continue;
        }

        // 5. Acquire Mutex before modifying shared data
        if (xSemaphoreTake(max30100_data_mutex, mutex_timeout) == pdTRUE) {
            // 6. Calculate heart rate and SpO2 with bounds checking
            float ratio = (float) red_data / (float) ir_data;
            max30100_data.heart_rate = fmaxf(50.0f, fminf(180.0f, (ir_data / 1000.0f)));
            max30100_data.spo2 = fmaxf(80.0f, fminf(100.0f, ratio * 100.0f));

            // 7. Store values for logging (outside mutex)
            float log_hr = max30100_data.heart_rate;
            float log_spo2 = max30100_data.spo2;

            // 8. Release Mutex after updating shared data
            xSemaphoreGive(max30100_data_mutex);

            // 9. Rate-limited logging (every 10th cycle)
            static uint32_t log_count = 0;
            if (++log_count % 10 == 0) { 
                ESP_LOGI(TAG, "HR: %.1f bpm, SpO2: %.1f%%", log_hr, log_spo2);
            }
        } else {
            ESP_LOGW(TAG, "Mutex timeout! Skipping update.");
        }

        // 10. Precise 10Hz sampling delay
        vTaskDelay(pdMS_TO_TICKS(100));
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
            xTaskCreatePinnedToCore(send_task, "send_task", 4096, NULL, 5, NULL, 1);
            break;

        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(SPP_TAG, "Disconnected. Reinitializing BT stack...");
            
            // Clean shutdown sequence
            esp_bluedroid_disable();
            esp_bluedroid_deinit();
            esp_bt_controller_disable();
            
            // Allow time for cleanup
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // Reinitialize
            /*esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
            esp_bluedroid_init();
            esp_bluedroid_enable();
            esp_spp_init(ESP_SPP_MODE_CB);
            
            ESP_LOGI(SPP_TAG, "BT stack reinitialized");
            break;*/

		case ESP_SPP_DATA_IND_EVT: {
		    if (param->data_ind.len > 0) {
		        char *received_data = (char *)malloc(param->data_ind.len + 1); // Allocate memory
		        if (received_data != NULL && xSemaphoreTake(output_message_mutex1, pdMS_TO_TICKS(100)) == pdTRUE) {
		            memcpy(received_data, param->data_ind.data, param->data_ind.len);
		            received_data[param->data_ind.len] = '\0'; // Null terminate
		
		            // Safely copy into MessageSend, ensuring no overflow
		            strncpy((char *)MessageSend, received_data, sizeof(MessageSend) - 1);
		            MessageSend[sizeof(MessageSend) - 1] = '\0'; // Ensure null termination
		            
		            ESP_LOGI(SPP_TAG, "Received %d bytes: %s", param->data_ind.len, MessageSend);
		            ESP_LOG_BUFFER_HEX(SPP_TAG, param->data_ind.data, param->data_ind.len);
		            free(received_data); // Free the allocated memory
		            xSemaphoreGive(output_message_mutex1);
		        } else {
		            ESP_LOGE(SPP_TAG, "Failed to allocate memory for received data or acquire mutex!");
		        }
		    }
		    break;
		}


        case ESP_SPP_VFS_REGISTER_EVT:
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
            break;

        default:
            break;
    }
}


void init_bt_stack() {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    bt_cfg.bt_max_acl_conn = 3;
    
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    // Register callback AFTER enable
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));
    vTaskDelay(pdMS_TO_TICKS(100));
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

static uint8_t decode_message(char msg[32]) {
	  if (msg[1] == 'M') {
	    // Handle message updates
	    if (xSemaphoreTake(input_message_mutex1, pdMS_TO_TICKS(100)) == pdTRUE) {
	        size_t j = 0;
	        for (size_t i = 2; msg[i] != '\0' && j < sizeof(MessageRecived) - 1; i++) {
	            if (msg[i] != '}' || msg[i] != ',' ) {  // Skip '}'
	                MessageRecived[j++] = msg[i];
	            }
	        }
	        MessageRecived[j] = '\0';  // Null-terminate the string
	        xSemaphoreGive(input_message_mutex1);
	    } else {
	        ESP_LOGW("DECODE", "Failed to acquire input_message_mutex1");
	    }
	  }


    if (msg[1] == 'G') {
        // Handle GPS data
        if (xSemaphoreTake(gps_data_mutex1, pdMS_TO_TICKS(100)) == pdTRUE) {
            double lat, lon;
            sscanf(msg + 2, "%lf,%lf", &lat, &lon);
            gps_data1.latitude = (float)lat;
            gps_data1.longitude = (float)lon;
            xSemaphoreGive(gps_data_mutex1);
        } else {
            ESP_LOGW("DECODE", "Failed to acquire gps_data_mutex1");
        }
    }

    if (msg[1] == 'H') {
        // Handle heart rate data
        if (xSemaphoreTake(max30100_data_mutex1, pdMS_TO_TICKS(100)) == pdTRUE) {
            double hr, spo2;
            sscanf(msg + 2, "%lf,%lf", &hr, &spo2);
            max30100_data1.heart_rate = (float)hr;
            max30100_data1.spo2 = (float)spo2;
            xSemaphoreGive(max30100_data_mutex1);
        } else {
            ESP_LOGW("DECODE", "Failed to acquire max30100_data_mutex1");
        }
    }

    return 0;
}

void oled_task(void *pvParameters) {  // Accept a void* argument
    while (1) {  // Tasks should run in a loop
        if (xSemaphoreTake(max30100_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            display_heartrate(max30100_data.heart_rate, max30100_data.spo2);
            xSemaphoreGive(max30100_data_mutex);
        } else {
            ESP_LOGW("DECODE", "Failed to acquire max30100_data_mutex1");
        }

        if (xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            display_coordinates(gps_data1.latitude, gps_data1.longitude);
            xSemaphoreGive(gps_data_mutex);
        } else {
            ESP_LOGW("DECODE", "Failed to acquire gps_data_mutex1");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Task should yield to FreeRTOS
    }
}



/*NRF240l Task*/
void nrf_init(void) {
    // Initialize NRF24L01
    Nrf24_init(&dev);
    
    // Configure channel and payload size
    uint8_t channel = 90;
    uint8_t payload = 16; // Adjust based on your needs (max 32)
    Nrf24_config(&dev, channel, payload);
    
    // Set transmitter address
    esp_err_t ret = Nrf24_setTADDR(&dev, tx_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TX address");
        return;
    }
    
    // Set receiver address
    ret = Nrf24_setRADDR(&dev, rx_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set RX address");
        return;
    }
    
    // Print NRF24L01 configuration details
    Nrf24_printDetails(&dev);
}

void nrf24_task(void *pvParameter) {
    // Buffers for data to send
    uint8_t data[32] = {0};
    uint8_t heartrate[32] = {0};
    uint8_t received[32] = {0}; // Buffer for incoming data
    static const char *TAG = "NRF24";
    
    // Initialize watchdog for this task
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    while (1) {
        // Reset watchdog to prevent timeout
        esp_task_wdt_reset();
        
        // --- Prepare GPS Data ---
        if (xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Clear data buffer before filling it
            memset(data, 0, sizeof(data));
            
            if (gps_data.valid) {
                snprintf((char*)data, sizeof(data),
                         "{G,%.6f,%.6f}",
                         gps_data.latitude, gps_data.longitude);
            } else {
                // Use strncpy to copy default data
                strncpy((char*)data, "{G,12.35,98.72}", sizeof(data) - 1);
            }
            xSemaphoreGive(gps_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire gps_data_mutex");
        }
        
        // --- Prepare Heart Rate Data ---
        if (xSemaphoreTake(max30100_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Clear heartrate buffer before filling it
            memset(heartrate, 0, sizeof(heartrate));
            
            if (gps_data.valid) {
                snprintf((char*)heartrate, sizeof(heartrate),
                         "{H,%.2f,%.2f}",
                         max30100_data.heart_rate, max30100_data.spo2);
            } else {
                // Use strncpy to copy default data
                strncpy((char*)heartrate, "{H,78.55,98.77}", sizeof(heartrate) - 1);
            }
            xSemaphoreGive(max30100_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire max30100_data_mutex");
        }
        
        // Reset watchdog before SPI operations
        esp_task_wdt_reset();
        
        // --- Send GPS Data and Check for Received Data ---
        if (xSemaphoreTake(nrf24_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Send GPS data
            Nrf24_send(&dev, data);
            if (Nrf24_isSend(&dev, 200)) {
                ESP_LOGI(TAG, "Data sent: %s", data);
            } else {
                ESP_LOGW(TAG, "Send timeout for GPS data");
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // Check for received data
            if (xSemaphoreTake(message_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (Nrf24_dataReady(&dev)) {
                    Nrf24_getData(&dev, received);
                    ESP_LOGI(TAG, "Received: %s", received);
                    decode_message((char *)received);
                }
                xSemaphoreGive(message_mutex);
            }
            // Release SPI mutex after sending/receiving
            xSemaphoreGive(nrf24_mutex);
        } else {
            ESP_LOGW(TAG, "Could not get NRF24 mutex");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // --- Send Hardcoded Message ---
        // Here we convert the plain text in MessageSend into the format "{M,Message}"
        if (xSemaphoreTake(nrf24_mutex, pdMS_TO_TICKS(100)) == pdTRUE && 
            xSemaphoreTake(output_message_mutex1, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Create a temporary buffer for the formatted message
            char formattedMessage[32] = {0};
            // Format the message as "{M,MessageSend}"
            // Assuming MessageSend contains pure text like "Hello User"
            snprintf(formattedMessage, sizeof(formattedMessage), "{M,%.24s}", MessageSend);
            
            Nrf24_send(&dev, (uint8_t *)formattedMessage);
            if (Nrf24_isSend(&dev, 200)) {
                ESP_LOGI(TAG, "Message sent: %s", formattedMessage);
                memset(MessageSend, 0, sizeof(MessageSend));

            } else {
                ESP_LOGW(TAG, "Send timeout for message");
            }
            xSemaphoreGive(nrf24_mutex);
            xSemaphoreGive(output_message_mutex1);
        } else {
            ESP_LOGW(TAG, "Could not get NRF24 mutex for message");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // --- Send Heart Rate Data ---
        if (xSemaphoreTake(nrf24_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            Nrf24_send(&dev, heartrate);
            if (Nrf24_isSend(&dev, 200)) {
                ESP_LOGI(TAG, "Heart rate sent: %s", heartrate);
            } else {
                ESP_LOGW(TAG, "Send timeout for heart rate");
            }
            xSemaphoreGive(nrf24_mutex);
        } else {
            ESP_LOGW(TAG, "Could not get NRF24 mutex for heart rate");
        }
        
        // Reset watchdog before next delay
        esp_task_wdt_reset();
        
        // Delay before repeating the loop
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // This point should never be reached
    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
}




/**
 * Main Application Entry Point
 */
void app_main(void) {
			esp_task_wdt_config_t twdt_config = {
		    .timeout_ms = 10000,          // 5-second timeout
		    .trigger_panic = false,       // Reboot on timeout
		    .idle_core_mask = (1 << 0) | (1 << 1)  // Monitor both cores
		};
		ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));
		
	/*esp_task_wdt_config_t wdt_config = {
	    .timeout_ms = 5000,
	    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Monitor all cores
	    .trigger_panic = true                              // Reboot on timeout
	};
	ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));
	
	// Add current task to watchdog monitoring
	ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
*/

	

	
	
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create the mutex for GPS data synchronization
    gps_data_mutex = xSemaphoreCreateMutex();
    if (gps_data_mutex == NULL) {
        ESP_LOGE(SPP_TAG, "Failed to create mutex for GPS data.");
        return;
    }
    gps_data_mutex1 = xSemaphoreCreateMutex();
    if (gps_data_mutex1 == NULL) {
        ESP_LOGE(SPP_TAG, "Failed to create mutex for GPS data1.");
        return;
    }
    
    max30100_data_mutex = xSemaphoreCreateMutex();
	if(max30100_data_mutex == NULL) {
	    ESP_LOGE("MAIN", "Failed to create mutex!");
	    return;
	}
	max30100_data_mutex1 = xSemaphoreCreateMutex();
	if(max30100_data_mutex1 == NULL) {
	    ESP_LOGE("MAIN", "Failed to create mutex!");
	    return;
	}
	
	message_mutex = xSemaphoreCreateMutex();
	if (message_mutex == NULL) {
        ESP_LOGE("INIT", "Failed to create message mutex");
    }
    
    input_message_mutex1 = xSemaphoreCreateMutex();
	if (input_message_mutex1 == NULL) {
        ESP_LOGE("INIT", "Failed to create input message mutex");
    }
    output_message_mutex1 = xSemaphoreCreateMutex();
	if (output_message_mutex1 == NULL) {
        ESP_LOGE("INIT", "Failed to create output message mutex");
    }

    //neo6m configaration
    gps_init();

    // Button configuration
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*)BUTTON_PIN);
    
    //OLED initialize
	init_oled();
	

    // Initialize I2C
   // ESP_ERROR_CHECK(i2c_master_init());
    
    // Initialize MAX30100 sensor
    max30100_init();
    
    //nrf24 initialize
    //NRF24_t dev;
    nrf24_mutex = xSemaphoreCreateMutex();
    if (nrf24_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create NRF24 mutex");
        return;
    }
    nrf_init();
    
    vTaskDelay(pdMS_TO_TICKS(100));
    // Start the GPS task (on Core 1)
    xTaskCreatePinnedToCore(neo6m_task, "neo6m_task", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(read_max30100, "read_max30100", 4096, NULL, 8, NULL, 1);
   // xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
     //xTaskCreatePinnedToCore(readerTask, "Reader Task", 8192, NULL, 11, NULL, 0);

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

    // Create a separate task to handle nRF24 transmission
    xTaskCreatePinnedToCore(nrf24_task, "nrf24_task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(oled_task, "oled_task", 4096, NULL, 3, NULL, 1);
    
}


