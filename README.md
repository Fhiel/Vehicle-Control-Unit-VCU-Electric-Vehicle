# 🚗 VCU_X19e: Advanced Vehicle Control Unit
![ESP32-C3](https://img.shields.io/badge/Hardware-ESP32--C3-blue?style=for-the-badge&logo=espressif)
![LED](https://img.shields.io/badge/LED-WS2812B%20%2F%20NeoPixel-green?style=for-the-badge&logo=lightbulb)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)
[![GitHub Stars](https://img.shields.io/github/stars/Fhiel/myS3XY-Lightshow?style=for-the-badge&color=gold&logo=github)](https://github.com/Fhiel/myS3XY-Lightshow/stargazers)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/Fhiel/Vehicle-Control-Unit-VCU-Electric-Vehicle)

An ESP32-based Vehicle Control Unit (VCU) designed for the electric conversion of a Bertone X1/9  to a Bertone X1/9e electric speed. It acts as the central nervous system, bridging high-voltage safety, thermal management, and modern user interaction.
<img width="1364" height="648" alt="image" src="https://github.com/user-attachments/assets/2021f764-d12e-40d3-9a4a-b36c5d644bc2" />
## 🏎️ The Project: Bertone X1/9e
The X1/9e is more than a conversion; it's a modernization of a 1983 classic. This VCU was developed to solve the "missing links" between industrial EV components (Tesla, Hyper9, Bender) and custom vehicle requirements.
<img width="1107" height="389" alt="image" src="https://github.com/user-attachments/assets/d33cf675-ff92-4ca0-88b4-179c124d1a64" />
## 🛠️ Key Components & Challenges
|Component|Function|VCU Enhancement|
|:--------|:------|:---|
|BMS|Tesla-based Battery Management|Type 2 auto-locking, Thermal PWM, CAN Relay Control (ALARM, BUZZER, PUMPS, FAN)|
|MCU| NetGain Hyper9 Motor Controller|CAN Proxy (0x244), Safety Interlock, Derating|
|IMD|Bender Insulation Monitoring|Active Self-Test Sequence, HV Relay Logic|
|Dashboard|Custom RS485 Instrument Cluster|High-speed telemetry bridge (115200 bps)|
|Webserver|Cockpit Enhancement|Wireless Interface for Information, Diagnostic and Manual Control|

## 🚀 Advanced Features
- **Dashboard** Supports modified Dashboard via RS485
- **Webserver:** Real-time Web-UI via WebSocket. Responsive "Command Center" for Desktop and mobile "Cockpit View" for Smartphone integration.
- **Safety-First Core:** Multi-core RTOS implementation with Mutex protection and Watchdog supervision.
- **Thermal Intelligence:** Independent PWM control for Battery (5kHz) and Inverter (1kHz) pumps with 5-minute intelligent afterrun.
- **Active Safety (IMD):** Automated insulation self-test on startup/charge.
- **Hardware interlock:** prevents HV-release on failure.
- **Automotive Locking:** Fully automated Type 2 connector management with physical feedback validation and theft protection.
- **OTA Updates:** Wireless firmware updates via ElegantOTA, enabling modifications from the driver's seat.mDNS
- **Integration:** Reachable via http://vcu-x19e.local – no IP searching required.

## 📂 Project Structure
```
src/
├── main.ino            # Initialization, RTOS Task scheduling
├── main.h              # Centralized TelemetryData struct & Config
├── can_process.cpp     # High-priority TWAI filtering & decoding
├── lock_control.cpp    # Type 2 State Machine & Auto-Lock logic
├── pump_control.cpp    # Thermal management & PWM ramping
├── self_test.cpp       # Active IMD sequence & BMS Interlock
├── web_server.cpp      # Async HTTP, WebSocket & OTA management
├── web_ui.h            # Responsive Dashboard (HTML/CSS)
├── rs485.cpp           # Optimized telemetry for Display Unit
├── CAN_Transmit.cpp    # Outgoing Proxy & Charger control
├── led_control.cpp     # Visual feedback (FastLED)
└── utils.cpp           # Safety-printf & Math helpers
```

## 🔌 Hardware Configuration (LilyGO T-CAN485)

|Function|GPIO|Logic|
|:--------|:------:|---:|
|CAN TX/RX|27 / 26|TWAI 500kbps|
|RS485 TX/RX|22 / 21|Telemetry 115200bps|
|Type 2 Lock/Unlock|25 / 5|H-Bridge ControlLock |
|Feedback|18|Pull-Up|
|Manual Unlock|12|Physical Interrupt|
|Pump PWM (INV/BAT)| 33 / 32| Independent Channels|
|Status LED|4|WS2812B|

## 🚦 Status Indicators (WS2812B)
    🟢 Solid Green: System Ready / Drive Mode.
    🔵 Pulsing Blue: Charging (Daily Limit active).
    🟡 Yellow: Initializing / Self-Test in progress.
    🔴 Solid Red: Safety Warning (Unlocked during charge).
    🚨 Strobe Red: CRITICAL ERROR (IMD Failure / Self-Test Fail).

## 💻 Build & Deployment
This project is built using PlatformIO.
Partitioning: Uses a custom partitions.csv (provided) to support dual OTA banks for safe wireless updates.
Secrets: Rename secrets_example.h to secrets.h and enter your WiFi credentials.Upload:Bashpio run --target upload

## ⚖️ License & Safety
This software is provided under the MIT License.

> [!Warning]
> This VCU controls high-voltage components. Use at your own risk. Always implement physical E-Stops and fused circuits.
