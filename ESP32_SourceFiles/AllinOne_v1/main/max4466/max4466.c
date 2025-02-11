#include "max4466.h"
#include <stdio.h>
#include <stdlib.h>

static QueueHandle_t i2s_queue;

void readerTask(void *param)
{
    uint16_t audio_buffer[I2S_DMA_BUF_LEN];
    size_t bytes_read;

    while (true)
    {
        i2s_read(I2S_NUM_0, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
        printf("Sample: %d\n", audio_buffer[0]); // Debug output

        // Optional: Send buffer over NRF24L01
    }
}

void audio_capture_init()
{
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);

    // Configure I2S for ADC input
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_LSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    // Install I2S driver
    i2s_driver_install(I2S_NUM_0, &i2s_config, 4, &i2s_queue);

    // Set ADC as the input source for I2S
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_7);

    // Enable ADC for I2S
    i2s_adc_enable(I2S_NUM_0);
}

void start_audio_reader_task()
{
    xTaskCreatePinnedToCore(readerTask, "Reader Task", 8192, NULL, 11, NULL, 0);
}
