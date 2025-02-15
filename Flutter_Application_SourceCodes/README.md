# Flutter Bluetooth Terminal

## Overview
Flutter Bluetooth Terminal is an enterprise-level Flutter application that allows users to scan for nearby ESP32 (NodeMCU) devices using Bluetooth Classic, display them in a list, and establish communication with a selected device. This application provides a user-friendly interface for sending and receiving data over Bluetooth.

## Features
- Scan and list all available ESP32 devices around the phone.
- Establish a Bluetooth Classic connection with a selected ESP32 device.
- Send and receive messages between the mobile device and ESP32.
- Simple and intuitive terminal-like interface for communication.

## Requirements
- Flutter SDK (latest stable version)
- Android Studio or VS Code with Flutter and Dart plugins
- A mobile device with Bluetooth capability (Android)
- ESP32 (NodeMCU) device with Bluetooth Classic enabled

## Installation
### 1. Clone the Repository
```bash
git clone https://github.com/Dinith132/Flutter-Bluetooth-Terminal-Application.git
cd flutter-bluetooth-terminal
```

### 2. Install Dependencies
```bash
flutter pub get
```

### 3. Connect a Physical Device or Emulator
Ensure your Android device is connected via USB with developer mode enabled.

### 4. Run the Application 
```bash
flutter run
```

## Usage
1. Open the app and grant Bluetooth permissions.
2. Tap the "Scan" button to discover nearby ESP32 devices.
3. Select a device from the list to establish a connection.
4. Use the terminal interface to send and receive messages.
5. Disconnect when communication is complete.

## Configuration
Ensure your ESP32 firmware is set up to handle Bluetooth Classic communication. Below is an example Arduino sketch to enable Bluetooth Serial on ESP32:
```cpp
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_BT_Device");
    Serial.println("Bluetooth Started. Waiting for connections...");
}

void loop() {
    if (SerialBT.available()) {
        char incomingChar = SerialBT.read();
        Serial.write(incomingChar);
    }
}
```

## Contributing
We welcome contributions from the community! To contribute:
1. Fork the repository.
2. Create a new branch.
3. Commit your changes.
4. Push the branch and create a Pull Request.

## Contact
For any issues or feature requests, please open an issue on GitHub or contact dinithp132@gmail.com .

