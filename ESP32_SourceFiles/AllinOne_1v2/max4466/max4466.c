#include "max4466.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MAX4466";
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static dac_oneshot_handle_t dac_handle;

void audio_capture_init(void) {
    // ADC Unit Configuration
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Channel Configuration
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config));

    // ADC Calibration
    adc_cali_handle_t handle = NULL;
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &handle));
    adc_cali_handle = handle;

    ESP_LOGI(TAG, "Audio capture initialized");
}

uint16_t* capture_audio(void) {
    static uint16_t audio_buffer[I2S_DMA_BUF_LEN];
    uint64_t startMillis = esp_timer_get_time() / 1000;
    uint16_t peakToPeak = 0;
    uint16_t signalMax = 0;
    uint16_t signalMin = 4095;

    while ((esp_timer_get_time() / 1000) - startMillis < 50) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
        
        if (raw < 4095) {
            if (raw > signalMax) {
                signalMax = raw;
            } else if (raw < signalMin) {
                signalMin = raw;
            }
        }
    }

    peakToPeak = signalMax - signalMin;
    for (int i = 0; i < I2S_DMA_BUF_LEN; i++) {
        audio_buffer[i] = peakToPeak;
    }

    return audio_buffer;
}

void audio_playback_init(void) {
    dac_oneshot_config_t config = {
        .chan_id = DAC_CHAN_0,  // Maps to GPIO25
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&config, &dac_handle));
    ESP_LOGI(TAG, "Audio playback initialized on GPIO25");
}

void process_audio(uint16_t *audio_data, size_t length) {
    if (!audio_data || length == 0) return;

    for (size_t i = 0; i < length; i++) {
        uint8_t dac_value = audio_data[i] >> 4; // Scale down to 8 bits
        ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac_handle, dac_value));
        esp_rom_delay_us(62); // Approximate 16kHz sample rate delay
    }
}
