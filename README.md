# VCU_X19e

**ESP32-based Vehicle Control Unit (VCU)** for electric vehicles. Interfaces CAN (TWAI), RS485 telemetry, Type 2 lock, pumps, and IMD self-test. Modular, real-time, safety-focused.
## Why _X19e? 
The reason for this project was the conversion of my 1983 FIAT/BERTONE X1/9 to electric. **The birth of the X1/9e**.  
Several components of my build are missing features or required functions. The CANBus was the only common link between all of them.
So I've started to interpret the signals and data formats of Battery Management System (BMS), Motor Control Unit (MCU) and Insulation Monitoring Device (IMD) to be able to control several not supported elements.

## Electric Vehicle Components
| Component | Features | Challenge  | 
|------------|--------|-------------|
| **Battery Management System (BMS)** | Battery Condition, HV contactors, charging process |Charge port LED, Type2 lock, battery pump control | 
| **Motor Control Unit (MCU)**| Motor Management | MCU pump control, understand BMS information | 
| **Display Unit (DU)** | Instrument panel | RS485 instead of CANBus  |
| **Insulation Monitoring Device (IMD)**  | Insulation measurement | Init selftest |
---
## Features
- **CAN (TWAI)**: Filters & processes BMS, MCU, IMD messages (500 kbps)
- **BMS proxy**: collect and transmit CAN ID to MCU in the required structure, 100 ms 
- **RS485**: 14-byte telemetry to Display Unit @ 115200 baud, 200 ms
- **Pumps**: PWM control (BAT: 5 kHz, INV: 1 kHz)
- **Type 2 Lock**: Auto/manual lock/unlock with feedback
- **IMD Self-Test**: HV relay check, BMS relay release on fail
- **LED**: WS2812 status (Green=OK, Blue=Charging, Red=Error, Blink=Self-test fail)
- **Safety**: Watchdog, mutexes, timeouts, plausibility checks
---

## Hardware
| Component | Pin |
|---------|-----------|
| TWAI TX/RX | GPIO 27/26 |
| RS485 TX/RX/RE | GPIO 22/21/17 |
| BAT Pump | GPIO 32 |
| INV Pump | GPIO 33 |
| Lock/Unlock | GPIO 25/5 |
| Feedback/Manual | GPIO 18/12 |
| LED | GPIO 4 |
| 5V EN | GPIO 16 |

---

## Requirements
- **Board**: ESP32 (LilyGO T-485 recommended)
- **Libraries**: [FastLED](https://github.com/FastLED/FastLED)
- **Arduino IDE**: ESP32 core ≥2.0.14

---

## Setup

## Configuration

Edit **`main.h`** to customize:

```cpp
// Pins
#define TYPE2_LOCK_PIN      GPIO_NUM_25
#define TYPE2_UNLOCK_PIN    GPIO_NUM_5
#define TYPE2_FEEDBACK_PIN  GPIO_NUM_18

// Timing
#define LOCK_TIME_MS        2000
#define UNLOCK_TIME_MS      2000
#define CAN_TIMEOUT_MS      1000

// Debug
#define DEBUG               // Remove for production
```

## LED States
| Color | Meaning |
|----------|----------------------------------------|
| Green | BMS OK, ready |
| Blue | Charging |
| Red | CAN timeout / IMD fault | 
| Yellow | IMD Self-test running / BMS warning |
| Red Flash | Self-test FAILED (50 ms on / 1 s off) |

## Project Structure

```text
VCU_X19e.ino →  setup(), loop(), tasks
├── main.h               →  Global constants, structs, externs
├── can_process.cpp      →  CAN receive, decode, timeouts
├── lock_control.cpp     →  Type 2 lock state machine
├── pump_control.cpp     →  PWM pump control (BAT/INV)
├── rs485.cpp            →  14-byte telemetry to RP2040
├── proxy_bms.cpp        →  CAN 0x244 proxy message
├── self_test.cpp        →  IMD self-test & relay control
├── led_control.cpp      →  WS2812 status + error blink
└── utils.cpp            →  safe_printf, mapFloat
```

## Build & Flash
Arduino IDE
Board: ESP32 Dev Module
Upload



