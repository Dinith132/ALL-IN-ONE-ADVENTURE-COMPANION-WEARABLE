/*
 * oled.h
 *
 *  Created on: Mar 13, 2025
 *      Author: panka
 */

#ifndef MAIN_OLED_OLED_H_
#define MAIN_OLED_OLED_H_

#include <stdio.h>
#include "ssd1306.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void init_oled();
void display_heartrate(double heart_rate, double so2);
void display_coordinates(double lat, double lon);

#ifdef __cplusplus
}
#endif

#endif // OLED_DISPLAY_H

