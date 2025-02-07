# GPS Module Test (ESP32)

## Overview
This project tests the functionality of a GPS module connected to an ESP32 using UART communication. The program reads NMEA sentences from the GPS module and logs them using ESP-IDF.

## Hardware Requirements
- **ESP32 development board**
- **NEO-6M GPS module** (or any compatible GPS module)
- **UART connections**:
  - **RX (ESP32 GPIO 16) → TX (GPS module)**
  - **TX (ESP32 GPIO 17) → RX (GPS module)**

## Software Requirements
- **ESP-IDF v5.4** (or compatible version)
- **C programming language**

## How It Works
1. The ESP32 is configured to communicate with the GPS module over UART.
2. The program continuously reads GPS data and stores complete NMEA sentences.
3. Received GPS sentences are logged to the console.

## Installation and Compilation
1. Set up **ESP-IDF** and configure your development environment.
2. Clone this repository or copy the `main.c` file into your ESP-IDF project.
3. Compile and flash the code:
   ```sh
   idf.py build flash monitor
