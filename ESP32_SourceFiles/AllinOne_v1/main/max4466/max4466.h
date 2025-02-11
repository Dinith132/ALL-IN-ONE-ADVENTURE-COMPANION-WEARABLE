/*
 * max4466.h
 *
 *  Created on: Feb 7, 2025
 *      Author: panka
 */

#ifndef MAIN_MAX4466_MAX4466_H_
#define MAIN_MAX4466_MAX4466_H_

#include "driver/i2s.h"
#include "driver/adc.h"   
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SAMPLE_RATE 40000
#define I2S_DMA_BUF_LEN 1024

// Function prototypes
void audio_capture_init();
void start_audio_reader_task();
void readerTask(void *param);

#endif // MAIN_MAX4466_MAX4466_H_
