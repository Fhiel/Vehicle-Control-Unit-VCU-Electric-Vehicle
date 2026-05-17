// main.h
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/twai.h>
#include <driver/gpio.h>
#include <cstdint>
#include <esp_err.h> 
#include "Freenove_WS2812_Lib_for_ESP32.h"

#include "utils.h"
#include "CAN_Transmit.h"
#include "relay_control.h"

// ====================== MCP2515 (autowp) ======================
#include <SPI.h>          
#include <mcp2515.h>      

using namespace std;

// === CONSTANTS ===
#define NUM_FILTERS 9
#define MAX_IDS 50
#define RS485_BAUDRATE 115200
#define TWAI_BAUDRATE 500000

extern const uint32_t FAST_TIMEOUT_MS;
extern const uint32_t SLOW_TIMEOUT_MS;
extern const uint32_t CURRENT_THRESHOLD;
extern const uint32_t AUTO_UNLOCK_TIMEOUT_MS;

// === CAN IDs ===
extern const uint32_t FILTERED_IDS[NUM_FILTERS];
extern const uint32_t FAST_IDS[];
extern const uint32_t SLOW_IDS[];

// === PLAUSIBILITY ===
#define MOTOR_RPM_MAX       20000
#define TEMP_MIN            -20.0f
#define MCU_TEMP_MAX        85.0f
#define MOTOR_TEMP_MAX      100.0f
#define BMS_TEMP_MAX        60.0f
#define BMS_SOC_MAX         100
#define BMS_CURRENT_MAX     1000.0f
#define IMD_ISO_R_MAX       50000
#define BMS_STATUS_BOOT      0
#define BMS_STATUS_READY     1
#define BMS_STATUS_DRIVE     2
#define BMS_STATUS_CHARGE    3
#define BMS_STATUS_PRECHARGE 4
#define BMS_STATUS_ERROR     5

// === ALARM FLAGS ===


/**
 * @struct TelemetryData
 * @brief Central storage for all vehicle and battery data.
 * Optimized for Cross-Core access (volatile) and UI feedback.
 */
typedef struct {
    // --- MCU / Motor Data (ID 0x239) ---
    volatile uint16_t motorRPM;
    volatile int8_t motor_temp;
    volatile int8_t mcu_temp;
    volatile uint8_t mcuFaultLevel;
    volatile uint32_t mcuFlags;
    volatile bool motorRPMValid;
    volatile bool motorTempValid;
    volatile bool mcuTempValid;
    volatile bool mcuFaultLevelValid;

    // --- BMS Core Data (ID 0x355, 0x356, 0x379) ---
    volatile float bmsSoC;
    volatile float bmsCurrent;
    volatile float bat_temp;
    volatile uint8_t bmsStatus;       // 0x04 = CHARGE (Plugged in)
    volatile bool bmsSoCValid;
    volatile bool bmsCurrentValid;
    volatile bool batTempValid;
    volatile bool bmsStatusValid;
    volatile bool is_charging;        // Internal logic flag
    
    // --- BMS Alarms & Warnings (ID 0x35A) ---
    volatile bool bmsHardwareFault;
    volatile bool bmsLowVoltageWarn;
    volatile bool bmsHighTempWarn;
    volatile bool bmsLowTempWarn;
    volatile bool bmsHighCurrentWarn;

    // --- IMD / Insulation (ID 0x37, 0x22) ---
    volatile uint16_t imdIsoR;
    volatile uint16_t hv1Voltage;
    volatile uint16_t imdStatus;
    volatile uint16_t vifcStatus;
    volatile uint8_t selfTestResult;
    volatile bool selfTestResultValid;
    volatile bool selfTestRunning;
    volatile bool selfTestRequested;
    volatile bool selfTestFailed;
    volatile bool imdRelayOpen;
    volatile bool imdIsoRValid;
    volatile bool hv1VoltageValid;
    volatile bool imdStatusValid;
    volatile bool vifcStatusValid;

    // --- VCU State & Control ---
    // --- VCU Internal State ---
    volatile bool fiveVEnabled;       // Main 5V rail status (powers transceivers, MCP2515, etc.)
    volatile bool canBStby;           // CAN B Transceiver Standby

    // Type 2 Lock
    volatile bool isLocked;           // Bolt is physically engaged
    volatile bool isLocking;          // Motor is currently running (Lock)
    volatile bool isUnLocking;        // Motor is currently running (Unlock)
    volatile bool shouldLock;         // Request from Web UI
    volatile bool lockIn1;            // DRV8871 IN1
    volatile bool lockIn2;            // DRV8871 IN2
    volatile bool lockFeedback;       // Lock Position Feedback sensor
    volatile bool manualUnlockBtn;    // Manual Unlock Button

    // Coolant Pumps & Fan
    volatile bool batPumpRelay;       // Battery pump relay state
    volatile uint8_t bat_pump_pwm;    // Battery pump speed (PWM value for dashboard feedback)
    volatile bool invPumpRelay;       // Inverter pump relay state
    volatile uint8_t inv_pump_pwm;    // Inverter pump speed (PWM value for dashboard feedback)
    volatile bool fanRelay;           // Cooling fan relay state

    // Signal Outputs and Alarm Conditions
    volatile uint16_t alarmRegister;   // Central Alarm Register (bitfield for various fault conditions)
    volatile bool ledCheckOil;         // Dashboard "Check Oil" LED state
    volatile bool ledBattery;          // Dashboard "Battery" LED state
    volatile bool isAlarm;             // TRUE = A critical fault condition is active (e.g., BMS hardware fault, self-test failure, etc.) - Used for dashboard red status tag and alarm logic
    volatile bool isPiezoOn;           // TRUE = The piezo buzzer should be active (driven by critical fault conditions or manual override)
    volatile uint8_t ws2812Status;     // 0 = Off, 1 = Green (Normal), 2 = Red (Fault)

    // Auxiliary (General Purpose)
    // Digital Inputs
    volatile bool auxinput13;              // INPUT_13 

    // Relays / Outputs
    volatile bool auxRelay11;           // RELAY_11
    volatile bool auxRelay12;           // RELAY_12
    volatile bool auxRelay13;           // RELAY_13
    volatile bool auxRelay14;           // RELAY_14
} TelemetryData;

extern volatile TelemetryData telemetryData;

// === CAN MESSAGE ===
typedef struct {
    uint32_t identifier;
    uint8_t extd;
    uint8_t data_length_code;
    uint8_t data[8];
} can_message_t;

// === GLOBAL VARIABLES ===
extern bool demoModeActive;
extern uint32_t received_ids[MAX_IDS];
extern uint8_t received_ids_count;
extern volatile unsigned long last_id_timestamps[NUM_FILTERS];
extern volatile unsigned long last_receive_time, last_can_update;
extern unsigned long last_send_time;

extern bool manualUnlockPressed;
extern bool selfTestFailed;

extern QueueHandle_t canQueue;
extern HardwareSerial RS485Serial;

extern SemaphoreHandle_t serialMutex, dataMutex, idMutex;
extern bool serialMutexInitialized, dataMutexInitialized, idMutexInitialized;

//extern CRGB leds[1];

// === TWAI CONFIG ===
extern twai_general_config_t g_config;
extern twai_timing_config_t twai_timing_config;
extern twai_filter_config_t twai_filter_config;

#ifdef HARDWARE_T2CAN
    // ====================== LilyGo T-2CAN Hardware Configuration ======================
    // Internal ESP32-S3 TWAI (CAN B) - Isolated Transceiver
    #define PIN_CAN_TX              GPIO_NUM_7   // Internal ESP32-S3 TWAI
    #define PIN_CAN_RX              GPIO_NUM_6   // Internal ESP32-S3 TWAI

    // SPI Interface for MCP2515 (second CAN bus - CAN A)
    #define PIN_MCP_CS              GPIO_NUM_10   // SPI Chip Select
    #define PIN_MCP_RST             GPIO_NUM_9    // MCP2515 Reset (active LOW pulse)
    #define PIN_MCP_INT             GPIO_NUM_8    // MCP2515 Interrupt (optional)

     // Power and Transceiver Control
    #define PIN_5V_EN               GPIO_NUM_46   // Main 5V rail enable (powers transceivers, MCP2515, etc.)
    #define PIN_CANB_STBY           GPIO_NUM_45   // Internal TWAI isolated transceiver enable/standby
                                                  // LOW = Normal operation (most common on T-2CAN)

    // ====================== External Pins via 26-Pin Header (Hut V3) ======================
    // Type 2 Lock Control (DRV8871 H-Bridge)
    #define TYPE2_LOCK_IN1_PIN      GPIO_NUM_18  // CN1.23 DRV8871 IN1 (Lock)
    #define TYPE2_LOCK_IN2_PIN      GPIO_NUM_21  // CN1.20 DRV8871 IN2 (Unlock)
    #define TYPE2_FEEDBACK_PIN      GPIO_NUM_36  // CN1.11 Lock position feedback (input)
    #define TYPE2_MANUAL_UNLOCK_PIN GPIO_NUM_35  // CN1.5 Manual unlock button (input)

    // Pump Control (Relay + PWM)
    #define BAT_PUMP_RELAY_PIN      GPIO_NUM_37   // CN1.9 Battery pump power relay / MOSFET
    #define BAT_PUMP_PWM_PIN        GPIO_NUM_38   // CN1.7 Battery pump speed control (PWM)
    #define INV_PUMP_RELAY_PIN      GPIO_NUM_39   // CN1.6 Inverter pump power relay / MOSFET
    #define INV_PUMP_PWM_PIN        GPIO_NUM_40   // CN1.12 Inverter pump speed control (PWM)

    // Auxiliary Outputs
    #define PIEZO_PIN               GPIO_NUM_5    // CN1.16 Buzzer
    #define FAN_RELAY_PIN           GPIO_NUM_17   // CN1.22 Cooling fan relay
    #define WS2812_DATA_PIN         GPIO_NUM_16   // CN.13 Addressable RGB LED (status LED)

    // Dashboard Indicator LEDs
    #define LED_CHECK_OIL_PIN       GPIO_NUM_14    // CN1.14 Instrument Cluster "Check Oil" LED (via relay)
    #define LED_BATTERY_PIN         GPIO_NUM_15    // CN1.15 Instrument Cluster "Battery" LED (via relay)

    // General Purpose Relays
    #define RELAY_11_PIN             GPIO_NUM_42  // CN1.8 General Purpose Relay 11
    #define RELAY_12_PIN             GPIO_NUM_41  // CN1.10 General Purpose Relay 12
    #define RELAY_13_PIN             GPIO_NUM_47  // CN1.19 General Purpose Relay 13
    #define RELAY_14_PIN             GPIO_NUM_4   // CN1.21 General Purpose Relay 14

    // General Purpose Input
    #define INPUT_13_PIN             GPIO_NUM_3   // CN1.26 General Purpose Input  



#elif defined(HARDWARE_TCAN485)
    // ====================== Aliases for backward compatibility ======================
    #define TYPE2_LOCK_PIN          TYPE2_LOCK_IN1
    #define TYPE2_UNLOCK_PIN        TYPE2_LOCK_IN2
    #define TYPE2_MANUAL_UNLOCK_PIN TYPE2_MANUAL_UNLOCK
    #define LED_PIN                 WS2812_DATA_PIN   // External WS2812B 
    // --- Legacy T-CAN485 Hardware Configuration (Hut V2) ---
    #define PIN_5V_EN           GPIO_NUM_16
    #define CAN_SE_PIN          GPIO_NUM_23
    #define PIN_CAN_TX          GPIO_NUM_27 
    #define PIN_CAN_RX          GPIO_NUM_26
    #define RS485_EN_PIN        GPIO_NUM_17
    #define LED_PIN             GPIO_NUM_4 // Internal Addressable RGB LED
    
    #define TYPE2_LOCK_IN1      GPIO_NUM_25 
    #define TYPE2_LOCK_IN2      GPIO_NUM_5  
    #define TYPE2_FEEDBACK_PIN  GPIO_NUM_18
    #define BAT_PUMP            GPIO_NUM_32
    #define INV_PUMP            GPIO_NUM_33
    // External Relays Mapping
    #define RELAY_CHECK_OIL    1  // Instrument Cluster "Check Oil"
    #define RELAY_BUZZER       2  // Alarm Buzzer
    #define RELAY_INV_FAN      3  // Inverter Fan
    #define RELAY_PUMPS_PWR    4  // Main Power for Pumps (Kl.15 Bypass)
#endif

// --- Global Constants (Hardware Independent) ---
#define BAT_PWM_CHANNEL             (ledc_channel_t)0
#define BAT_PWM_FREQ                5000         
#define INV_PWM_CHANNEL             (ledc_channel_t)1
#define INV_PWM_FREQ                1000         

// Timing for Pump Afterrun (5 Minutes)
#define PUMP_AFTERRUN_MS   300000

// === LOCK TIMING ===
#define LOCK_TIME_MS            2000
#define UNLOCK_TIME_MS          2000
#define CAN_TIMEOUT_MS          1000         

// === MAKRO ===
#define WITH_DATA_MUTEX(code) \
    do { \
        if (dataMutexInitialized && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { \
            code; \
            xSemaphoreGive(dataMutex); \
        } \
    } while(0)

// === FUNKTIONS ===
void update_led();
void handleLockState();
void initWebServer();
void updateWebDashboard();

// --- Global state shared across modules ---
extern volatile TelemetryData telemetryData;
extern bool demoModeActive;
extern bool manualUnlockPressed;
extern Freenove_ESP32_WS2812 led;
// === MCP2515 Globals ===
extern MCP2515 mcp2515;
extern SemaphoreHandle_t mcpMutex;


#endif