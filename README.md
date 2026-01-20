# Autonomous Car

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange.svg)](https://platformio.org/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

> Wireless ESP32-based autonomous car control system using ESP-NOW protocol

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Pin Configuration](#pin-configuration)
- [Communication Protocol](#communication-protocol)
- [Building the Project](#building-the-project)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 🎯 Overview

This project implements a wireless remote control system for an autonomous car using two ESP32-C3 microcontrollers. The system uses ESP-NOW for low-latency, peer-to-peer communication between a transmitter (controller) and receiver (car).

## ✨ Features

- **Low-latency wireless control** via ESP-NOW protocol
- **Dual-mode firmware** supporting both sender and receiver configurations
- **Analog steering control** using potentiometer input (0-4095 ADC range)
- **Bidirectional motor control** with forward/backward capability
- **Servo-based steering mechanism** (500-2400μs pulse range)
- **Real-time serial monitoring** with debug output
- **Hardware interrupt-safe** data reception

## 🔧 Hardware Requirements

### Transmitter (Controller)
- 1× ESP32-C3 DevKit M-1
- 1× Potentiometer (analog input)
- 2× Push buttons (Forward/Backward)
- Power supply (3.3V)

### Receiver (Car)
- 1× ESP32-C3 DevKit M-1
- 1× Servo motor (steering)
- 1× DC motor driver (H-Bridge)
- 1× DC motor
- Power supply (suitable for motors)

## 🏗 System Architecture

```
┌─────────────┐                    ┌─────────────┐
│  SENDER     │                    │  RECEIVER   │
│  (Remote)   │  ──── ESP-NOW ──>  │  (Car)      │
│             │                    │             │
│ • POT       │                    │ • Servo     │
│ • Buttons   │                    │ • Motor     │
└─────────────┘                    └─────────────┘
```

### Data Flow
1. Sender reads potentiometer (steering) and button states (throttle)
2. Data packaged into `packet_t` structure
3. Transmitted via ESP-NOW to receiver MAC address
4. Receiver processes packet and controls servo + motor driver

## 📦 Installation

### Prerequisites
- [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) (VS Code extension recommended)
- USB cable for ESP32 programming

### Clone Repository
```bash
git clone https://github.com/Dutzu05/AutonomousCar.git
cd AutonomousCar
```

## ⚙️ Configuration

### 1. Update Receiver MAC Address
Before building, update the receiver's MAC address in `src/app.cpp`:

```cpp
uint8_t receiverMacAddress[] = {0xDC, 0xB4, 0xD9, 0x8B, 0xD0, 0xFC};
```

**To find your receiver's MAC address:**
1. Flash the receiver firmware first
2. Open serial monitor (115200 baud)
3. Copy the displayed MAC address
4. Update the sender code with this address

### 2. Build Flags
The firmware mode is controlled via PlatformIO build flags:
- `SENDER` - Transmitter mode
- `RECEIVER` - Car mode

## 📍 Pin Configuration

### Sender (Controller)
| Component | Pin | Description |
|-----------|-----|-------------|
| Potentiometer | GPIO 1 | Analog steering input |
| Forward Button | GPIO 4 | Forward throttle control |
| Backward Button | GPIO 5 | Reverse throttle control |

### Receiver (Car)
| Component | Pin | Description |
|-----------|-----|-------------|
| Servo Signal | GPIO 8 | PWM output for steering servo |
| Motor CW | GPIO 5 | Clockwise rotation (forward) |
| Motor CCW | GPIO 4 | Counter-clockwise rotation (backward) |

## 📡 Communication Protocol

### Packet Structure
```cpp
typedef struct {
  int value;          // ADC value (0-4095) - steering position
  uint8_t forward;    // 1 = forward button pressed
  uint8_t backward;   // 1 = backward button pressed
} packet_t;
```

### ESP-NOW Configuration
- **Channel**: Auto (0)
- **Encryption**: Disabled
- **TX Power**: 78 (maximum)
- **Update Rate**: ~50Hz (20ms delay on sender)

## 🔨 Building the Project

### Build Sender
```bash
pio run -e sender
pio run -e sender --target upload
```

### Build Receiver
```bash
pio run -e receiver
pio run -e receiver --target upload
```

### Monitor Serial Output
```bash
pio device monitor -b 115200
```

## 🐛 Troubleshooting

### Common Issues

#### ESP-NOW Initialization Failed
**Symptom**: "ESP-NOW INIT FAILED" message on startup
**Solution**: 
- Ensure WiFi.mode(WIFI_STA) is set before esp_now_init()
- Check that WiFi is not connected to an access point

#### No Communication Between Devices
**Symptom**: Sender shows "SEND FAIL"
**Solution**:
- Verify receiver MAC address is correct in sender code
- Ensure both devices are powered on
- Check that both devices are within range (~100m line-of-sight)

#### Motor Not Responding
**Symptom**: Servo moves but motor doesn't run
**Solution**:
- Check motor driver connections (CW_PIN, CCW_PIN)
- Verify motor driver has adequate power supply
- Test motor driver independently

#### Erratic Servo Movement
**Symptom**: Servo jitters or moves unpredictably
**Solution**: 
- Adjust SERVO_MIN_US/SERVO_MAX_US values for your specific servo
- Add external power supply for servo (don't power from ESP32)
- Check for electrical noise interference

## 🚀 Usage

1. **Power on receiver** (car) first
2. **Note the MAC address** from serial output
3. **Update sender code** with receiver MAC address
4. **Flash sender** device
5. **Power on sender** (controller)
6. **Control the car**:
   - Turn potentiometer to steer
   - Press forward button to drive forward
   - Press backward button to reverse
   - Release both buttons to stop

## 🛠 Development

### Code Structure
```
AutonomousCar/
├── src/
│   └── app.cpp          # Main application code
├── platformio.ini       # PlatformIO configuration
└── README.md
```

### Dependencies
- **ESP32Servo** (madhephaestus) - Servo motor control library

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📧 Contact

Project Link: [https://github.com/Dutzu05/AutonomousCar](https://github.com/Dutzu05/AutonomousCar)

---

**Made with ❤️ using ESP32 and PlatformIO**