#ifndef NEO6M_H_
#define NEO6M_H_

#include <stdint.h>

#define GPS_BAUD 9600   // Baud rate for the GPS module
#define RXD2 16         // RX pin for Serial2 (adjust as needed)
#define TXD2 17         // TX pin for Serial2 (adjust as needed)
#define BUF_SIZE 1024   // Buffer size for reading from UART
#define GPS_UART_PORT UART_NUM_2

typedef struct {
    double latitude;
    double longitude;
} GPS_Coordinates;



void gps_init(void);
char* getData(void);
void gps_start(void);
void parse_gpgll(const char *nmea);
GPS_Coordinates get_gps_coordinates(void);

#endif /* NEO6M_H_ */


