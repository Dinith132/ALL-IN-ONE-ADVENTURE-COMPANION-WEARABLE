# All-in-One Adventure Companion Wearable

## Introduction
The **All-in-One Adventure Companion Wearable** is a state-of-the-art device designed to cater to the needs of adventurers, outdoor enthusiasts, and explorers. This innovative wearable combines advanced health monitoring, precise navigation, reliable communication, and rugged durability to ensure safety, connectivity, and functionality during remote expeditions.

Key features include:
- **Health Monitoring:** Real-time tracking of heart rate and oxygen levels with the MAX30102/MAX30100 sensor.
- **Navigation:** Accurate location tracking with the NEO-6M GPS module.
- **Communication:** Long-distance connectivity with the NRF24L01+PA+LNA module and integrated audio components for seamless communication.
- **Additional Features:** OLED display for easy readability, and LED indicators for enhanced usability.

Built to withstand harsh environments, this lightweight wearable is waterproof, shock-resistant, and durable—making it the perfect companion for every adventure.

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
![Circuit Diagram](https://drive.google.com/file/d/1dnClkqRDui9AMM6XrjJy2LuvMDOQx8pD/view?usp=drive_link)
*Include a detailed schematic with clear labels for each component.*

---

## Technologies Used

### Hardware:
- NRF24L01+PA+LNA Module
- MAX30102/MAX30100 Heart Rate Sensor
- NEO-6M GPS Module
- ESP32 WROOM-32U Development Board
- Electret Condenser Microphone
- Permanent Magnet Speaker
- OLED Display
- DC Vibration Motor

### Software:
- **Arduino IDE**: For programming and uploading code to the ESP32.
- **Fritzing**: For circuit diagram design.
- **Eagle**: For PCB design.
- **PlatformIO**: For advanced software development and debugging.

### Frameworks:
- ESP-IDF (Espressif IoT Development Framework)
- FreeRTOS for ESP32

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
- **Weerasinghe D.K.D**: Software Development and Documentation (2021/E/024)
- **Hettiarachchi H.P.M**: PCB Design and Circuit Design (2021/E/051)
- **Perera W.A.C.D**: PCB Design and Circuit Design (2021/E/175)
- **Wijedasa M.M.D.P**: Software Development and Documentation (2021/E/188)

### Mentors:
- *[Mentor Name]*: Guidance on hardware integration and circuit design.
- *[Mentor Name]*: Software development and debugging support.

---

## How to Contribute
Contributions are welcome! To contribute:
1. Fork this repository.
2. Create a new branch for your feature or bug fix.
3. Commit your changes with detailed messages.
4. Submit a pull request for review.

---

## License
This project is licensed under the MIT License. See the LICENSE file for details.

---

## Contact
For further inquiries, contact us at [your_email@example.com].

