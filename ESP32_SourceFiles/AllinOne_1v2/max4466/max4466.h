/*
 * max4466.h
 *
 *  Created on: Feb 7, 2025
 *      Author: panka
 */

#ifndef MAIN_MAX4466_MAX4466_H_
#define MAIN_MAX4466_MAX4466_H_

#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/dac_oneshot.h"

#define SAMPLE_RATE       16000
#define I2S_DMA_BUF_LEN   16
#define ADC_CHANNEL       ADC1_CHANNEL_7  // GPIO35 for MAX4466 audio input

#ifdef __cplusplus
extern "C" {
#endif

void audio_capture_init(void);
uint16_t* capture_audio(void);
void audio_playback_init(void);
void process_audio(uint16_t *audio_data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // MAIN_MAX4466_MAX4466_H_
