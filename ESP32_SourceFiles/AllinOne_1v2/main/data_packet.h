/*
 * data_packet.h
 *
 *  Created on: Mar 14, 2025
 *      Author: panka
 */

#ifndef MAIN_DATA_PACKET_H_
#define MAIN_DATA_PACKET_H_

#include <stdint.h>
#include <string.h>


typedef struct {
    char type;  // 'G' for GPS, 'H' for MAX30100, 'M' for Message
    union {
        struct { 
            float latitude;   // GPS latitude
            float longitude;  // GPS longitude 
        };
        struct {
            int heartRate;    // Heart rate
            int spo2;         // SpO2 
        };
        char text[30];        // Message text 
    };
} DataPacket;

#endif /

