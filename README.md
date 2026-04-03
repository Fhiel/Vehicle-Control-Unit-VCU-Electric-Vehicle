VCU_X19e: Advanced Vehicle Control Unit
An ESP32-based Vehicle Control Unit (VCU) designed for the electric conversion of a Bertone X1/9  to a Bertone X1/9e electric speed. It acts as the central nervous system, bridging high-voltage safety, thermal management, and modern user interaction.

🏎️ The Project: X1/9e
The X1/9e is more than a conversion; it's a modernization of a 1983 classic. This VCU was developed to solve the "missing links" between industrial EV components (Tesla, Hyper9, Bender) and custom vehicle requirements.

🛠️ Key Components & Challenges
ComponentFunctionVCU EnhancementBMSTesla-based Battery ManagementType 2 auto-locking, Thermal PWM, SoC ProxyingMCUNetGain Hyper9 Motor ControllerCAN Proxy (0x244), Safety Interlock, DeratingIMDBender Insulation MonitoringActive Self-Test Sequence, HV Relay LogicDashboardCustom RS485 Instrument ClusterHigh-speed telemetry bridge (115200 bps)

🚀 Advanced Features
Tesla-Style Dashboard: Real-time Web-UI via WebSocket. Responsive "Command Center" for Desktop and mobile "Cockpit View" for S10e integration.Safety-First Core: Multi-core RTOS implementation with Mutex protection and Watchdog supervision.Thermal Intelligence: Independent PWM control for Battery (5kHz) and Inverter (1kHz) pumps with 5-minute intelligent afterrun.Active Safety (IMD): Automated insulation self-test on startup/charge. Hardware interlock prevents HV-release on failure.Automotive Locking: Fully automated Type 2 connector management with physical feedback validation and theft protection.OTA Updates: Wireless firmware updates via ElegantOTA, enabling modifications from the driver's seat.mDNS Integration: Reachable via http://vcu-x19e.local – no IP searching required.

📂 Project Structure
Plaintextsrc/
├── main.ino            # Initialization, RTOS Task scheduling
├── main.h              # Centralized TelemetryData struct & Config
├── can_process.cpp     # High-priority TWAI filtering & decoding
├── lock_control.cpp    # Type 2 State Machine & Auto-Lock logic
├── pump_control.cpp    # Thermal management & PWM ramping
├── self_test.cpp       # Active IMD sequence & BMS Interlock
├── web_server.cpp      # Async HTTP, WebSocket & OTA management
├── web_ui.h            # Responsive Tesla-style Dashboard (HTML/CSS)
├── rs485.cpp           # Optimized telemetry for Display Unit
├── CAN_Transmit.cpp    # Outgoing Proxy & Charger control
├── led_control.cpp     # Visual feedback (FastLED)
└── utils.cpp           # Safety-printf & Math helpers

🔌 Hardware Configuration (LilyGO T-CAN485)

FunctionGPIOLogicCAN TX/RX27 / 26TWAI 500kbpsRS485 TX/RX22 / 21Telemetry 115200bpsType 2 Lock/Unlock25 / 5H-Bridge ControlLock Feedback18Pull-UpManual Unlock12Physical InterruptPump PWM (INV/BAT)33 / 32Independent ChannelsStatus LED4WS2812B

🚦 Status Indicators (WS2812B)
    🟢 Solid Green: System Ready / Drive Mode.
    🔵 Pulsing Blue: Charging (Daily Limit active).
    🟡 Yellow: Initializing / Self-Test in progress.
    🔴 Solid Red: Safety Warning (Unlocked during charge).
    🚨 Strobe Red: CRITICAL ERROR (IMD Failure / Self-Test Fail).

💻 Build & Deployment
This project is built using PlatformIO.
Partitioning: Uses a custom partitions.csv (provided) to support dual OTA banks for safe wireless updates.
Secrets: Rename secrets_example.h to secrets.h and enter your WiFi credentials.Upload:Bashpio run --target upload

⚖️ License & SafetyThis software is provided under the MIT License.
WARNING: This VCU controls high-voltage components. Use at your own risk. Always implement physical E-Stops and fused circuits.