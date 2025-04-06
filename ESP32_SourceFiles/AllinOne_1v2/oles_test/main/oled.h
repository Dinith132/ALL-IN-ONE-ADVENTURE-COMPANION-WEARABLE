/*
 * oled.h
 *
 *  Created on: Feb 8, 2025
 *      Author: panka
 */

#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "driver/i2c.h"

// OLED Configuration
#define OLED_I2C_ADDRESS    0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_I2C_NUM       I2C_NUM_0
#define OLED_SDA_PIN       GPIO_NUM_21
#define OLED_SCL_PIN       GPIO_NUM_22
#define OLED_RST_PIN       -1  // Set to GPIO if using reset pin

// Function prototypes
void oled_init(void);
void oled_clear(void);
void oled_display_text(const char *text, uint8_t page);
void oled_display_bitmap(const uint8_t *bitmap);

#endif
