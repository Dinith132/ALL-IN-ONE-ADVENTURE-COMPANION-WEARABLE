# All-in-One Adventure Companion Wearable

## Introduction
The **All-in-One Adventure Companion Wearable** is a state-of-the-art device designed to cater to the needs of adventurers, outdoor enthusiasts, and explorers. This innovative wearable combines advanced health monitoring, precise navigation, reliable communication, and rugged durability to ensure safety, connectivity, and functionality during remote expeditions.

Key features include:
- **Health Monitoring:** Real-time tracking of heart rate and oxygen levels with the MAX30102/MAX30100 sensor.
- **Navigation:** Accurate location tracking with the NEO-6M GPS module.
- **Communication:** Long-distance connectivity with the NRF24L01+PA+LNA module and integrated audio components for seamless communication.
- **Additional Features:** OLED display for easy readability, and LED indicators for enhanced usability.


---
 
## Circuit Design
The wearable features a compact and efficient circuit design to integrate all components seamlessly. 

### Key Components and Descriptions:
1. **NRF24L01+PA+LNA Module**: Enables long-distance communication up to 1100m.
2. **DC 3V 70mA 12000 RPM Vibration Motor**: Provides silent notifications.
3. **MAX30102/MAX30100 Heart Rate Oxygen Pulse Sensor**: Tracks heart rate and oxygen levels in real-time.
4. **Mini Ublox NEO-6M GPS Module with Antenna**: Offers precise location tracking for navigation.
5. **Electret Condenser Microphone**: Facilitates audio input for team communication.
6. **ESP32 WROOM-32U Bluetooth BLE and Wi-Fi Development Board**: Central processing unit with wireless connectivity.
7. **Permanent Magnet Speaker**: Provides audio output for communication.
8. **OLED Display (0.96” 128X64)**: Displays health, navigation, and communication data.

#### Circuit Diagram

![Circuit Diagram](https://github.com/Dinith132/ALL-IN-ONE-ADVENTURE-COMPANION-WEARABLE/blob/87509a178b6350792eec762ce333d4cd9c8d5785/design/Circuit%20image.png)
*Figure 1: Complete circuit diagram of the All-in-One Adventure Companion Wearable.*

![3D View of PCB Design](https://github.com/Dinith132/ALL-IN-ONE-ADVENTURE-COMPANION-WEARABLE/blob/87509a178b6350792eec762ce333d4cd9c8d5785/design/3D%20view.PNG)
*Figure 2: 3D rendered view of the PCB design.*

![Schematic Diagram](https://github.com/Dinith132/ALL-IN-ONE-ADVENTURE-COMPANION-WEARABLE/blob/d84f2aab8f3bdcc224ee544cc9a6a3f7b5ed19e5/design/Schematic%20Diagram.png)
*Figure 3: Schematic diagram with detailed labels for each component.*

---

## Technologies Used

### Hardware:
-MAX30102/MAX30100 Heart Rate Oxygen Pulse Sensor
-Mini Ublox NEO-6M GPS Module with Antenna
-MAX9814 Microphone Amplifier
-ESP32 38Pin WROOM-32U Bluetooth BLE and WIFI Development Board with U.FL Antenna Connector
-NRF24L01+PA+LNA Module Long Distance 1100M 
-OLED Display 0.96″ White 128X64 IIC I2C DC 3V-5V SPI 
-AMS1117-3.3V SMD Voltage Regulator 
-LM7805 L7805 Voltage Regulator IC 5V 1.2A 
-LM741 Op-Amp 
-Permanent Magnet Speaker
-LED, Button
-Capacitors, Resistors


### Software:
- **Altium Designer**: For circuit diagram design.
- **EasyEDA Online PCB design & circuit simulator**: For circuit diagram design and PCB design.
- **ESP-IDF**: For build and upload code to your ESP32

### Frameworks:
- ESP-IDF (For build and upload code to your ESP32)

---

## References
1. [MAX30102/MAX30100 Sensor Datasheet](https://www.datasheets.com)
2. [NEO-6M GPS Module Documentation](https://www.u-blox.com)
3. [NRF24L01+ Module Guide](https://www.nordicsemi.com)
4. [ESP32 Official Documentation](https://docs.espressif.com)
5. [Arduino Programming Guide](https://www.arduino.cc)

---

## Team & Mentors

### Team Members:
- **Weerasinghe D.K.D**: Software Development and Documentation 
- **Hettiarachchi H.P.M**: PCB Design and Circuit Design 
- **Perera W.A.C.D**: PCB Design and Circuit Design 
- **Wijedasa M.M.D.P**: Software Development and Documentation 

### Mentors:
- *Nishankar S.*-Lecturer on University of Jaffna 

---

## Contact
For further inquiries, contact us at dinithp132@gmail.com.

