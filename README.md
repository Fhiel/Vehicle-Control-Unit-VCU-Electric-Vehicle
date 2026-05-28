# 🚗 VCU_X19e: Advanced Vehicle Control Unit
![ESP32-C3](https://img.shields.io/badge/Hardware-ESP32--C3-blue?style=for-the-badge&logo=espressif)
![LED](https://img.shields.io/badge/LED-WS2812B%20%2F%20NeoPixel-green?style=for-the-badge&logo=lightbulb)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)
[![GitHub Stars](https://img.shields.io/github/stars/Fhiel/myS3XY-Lightshow?style=for-the-badge&color=gold&logo=github)](https://github.com/Fhiel/myS3XY-Lightshow/stargazers)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/Fhiel/Vehicle-Control-Unit-VCU-Electric-Vehicle)

An ESP32-based Vehicle Control Unit (VCU) designed for the electric conversion of a Bertone X1/9  to a Bertone X1/9e electric speed. It acts as the central nervous system, bridging high-voltage safety, thermal management, and modern user interaction.
<img width="1237" height="499" alt="image" src="https://github.com/user-attachments/assets/42c3e269-73cc-41e2-9322-74d887b69114" />
<img width="1239" height="498" alt="image" src="https://github.com/user-attachments/assets/00364c01-95b9-474c-b780-6f107081d9cc" />


## 🏎️ The Project: Bertone X1/9e
The X1/9e is more than a conversion; it's a modernization of a 1983 classic. This VCU was developed to solve the "missing links" between industrial EV components (Tesla, Hyper9, Bender) and custom vehicle requirements.
<img width="1227" height="845" alt="image" src="https://github.com/user-attachments/assets/6d5e7ee4-95d2-4d49-bb49-6bfd49054502" />
Manual controls allow test of all outputs attached to the VCU.
<img width="1210" height="551" alt="image" src="https://github.com/user-attachments/assets/d61e0ead-09c6-49bc-ae36-c482ee53a0e3" />

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

## 🔌 Hardware Configuration (LilyGO T-C2CAN) internal
|Function|GPIO|Logic|
|:--------|:------:|---:|
|Internal CAN A|7 / 6|TWAI 500kbps|
|MCP 2515 CAN B|10 / 9 / 8|Telemetry 500kbps|

## 🔌 Hardware Configuration (LilyGO T-C2CAN) 26 Pin Interface
|Function|GPIO|Pin no.|Logic
|:------------------|:----------|:---|:-----------|
|TYPE2_LOCK_IN1_PIN      |GPIO_NUM_18 |CN1.23 |DRV8871 IN1 (Lock)
|TYPE2_LOCK_IN2_PIN      |GPIO_NUM_21 |CN1.20 |DRV8871 IN2 (Unlock)
|TYPE2_FEEDBACK_PIN      |GPIO_NUM_36 |CN1.11 |Lock position feedback (input)
|TYPE2_MANUAL_UNLOCK_PIN |GPIO_NUM_35 |CN1.5 |Manual unlock button (input)
|BAT_PUMP_RELAY_PIN      |GPIO_NUM_37 |CN1.9 |Battery pump power relay / MOSFET
|BAT_PUMP_PWM_PIN        |GPIO_NUM_38 |CN1.7 |Battery pump speed control (PWM)
|INV_PUMP_RELAY_PIN      |GPIO_NUM_39 |CN1.6 |Inverter pump power relay / MOSFET
|INV_PUMP_PWM_PIN        |GPIO_NUM_40 |CN1.12 |Inverter pump speed control (PWM)
|PIEZO_RELAY_PIN         |GPIO_NUM_5  |CN1.16 |Buzzer (Fixed: Aligned name with relay_control)
|FAN_RELAY_PIN           |GPIO_NUM_17 |CN1.22 |Cooling fan relay
|WS2812_DATA_PIN         |GPIO_NUM_16 |CN.13 |Addressable RGB LED (status LED)
|LED_CHECK_OIL_PIN       |GPIO_NUM_14 |CN1.14 |Instrument Cluster "Check Oil" LED (via relay)
|LED_BATTERY_PIN         |GPIO_NUM_15 |CN1.15 |Instrument Cluster "Battery" LED (via relay)
|RELAY_11_PIN            |GPIO_NUM_42 |CN1.8 |General Purpose Relay 11
|RELAY_12_PIN            |GPIO_NUM_41 |CN1.10 |General Purpose Relay 12
|RELAY_13_PIN            |GPIO_NUM_47 |CN1.19 |General Purpose Relay 13
|RELAY_14_PIN            |GPIO_NUM_4 |CN1.21 |General Purpose Relay 14
|INPUT_13_PIN            |GPIO_NUM_3 |CN1.26 |General Purpose Input 

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
