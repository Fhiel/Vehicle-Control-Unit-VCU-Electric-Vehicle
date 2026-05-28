/**
 * @file main.ino
 * @brief Core logic for the Vehicle Control Unit (VCU) on ESP32.
 * @author Fhiel (X1/9e Project)
*/
// SPDX-License-Identifier: MIT

#define DEBUG
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <driver/ledc.h>

#include "main.h"
#include "can_process.h"
#include "lock_control.h"
#include "telemetry.h" // Renamed from rs485.h
#include "pump_control.h"
#include "relay_control.h"
#include "utils.h"
#include "self_test.h"
#include "web_server.h"

#include <LittleFS.h>   // For serving web assets from flash




#ifdef HARDWARE_T2CAN
#include <SPI.h>
#include <mcp2515.h>          // autowp Library
#include "driver/periph_ctrl.h"
#include "soc/usb_periph.h"
#include "soc/gpio_reg.h"

MCP2515 mcp2515(PIN_MCP_CS); 
SemaphoreHandle_t mcpMutex = nullptr;
Freenove_ESP32_WS2812 led(1, WS2812_DATA_PIN, 0, TYPE_GRB);
#else
    Freenove_ESP32_WS2812 led(1, LED_PIN, 0, TYPE_GRB);
#endif

// === DEFINITION ===
const uint32_t FILTERED_IDS[NUM_FILTERS] = {0x22, 0x37, 0x239, 0x240, 0x355, 0x356, 0x379, 0x35A}; // Added 0x240
const uint32_t FAST_IDS[] = {0x22, 0x239, 0x240, 0x355, 0x356, 0x379, 0x35A}; // Put TPDO 2 into the processing pipeline
const uint32_t SLOW_IDS[] = {0x37};

const uint32_t FAST_TIMEOUT_MS = 300;
const uint32_t SLOW_TIMEOUT_MS = 2000;
const uint32_t CURRENT_THRESHOLD = 5;
const uint32_t AUTO_UNLOCK_TIMEOUT_MS = 60000;

volatile TelemetryData telemetryData = {0};
bool demoModeActive = false;
bool manualUnlockPressed = false;

// Global VCU Override Tracking Registers
volatile uint32_t manualOverride = 0;        // Bitmask for tracking MAN (1) vs AUTO (0) profile states
volatile uint32_t manualOverride_Values = 0; // Tracks the actual user configuration state (ON vs OFF)

// Dedicated analog override value buffers for multi-stage cooling pumps
volatile uint8_t manual_invPumpPwm = 0;       // Holds manual target speed for Inverter loop (0, 51, 204)
volatile uint8_t manual_batPumpPwm = 0;       // Holds manual target speed for Battery loop (0, 51, 204)
//CRGB leds[1];

HardwareSerial RS485Serial(2);

SemaphoreHandle_t serialMutex = nullptr, dataMutex = nullptr, idMutex = nullptr;
bool serialMutexInitialized = false, dataMutexInitialized = false, idMutexInitialized = false;

// === TWAI CONFIG ===
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)PIN_CAN_TX, // GPIO 7 (T-2CAN Internal TWAI) or GPIO_NUM_27 (TCAN485 Internal TWAI)
    (gpio_num_t)PIN_CAN_RX, // GPIO 6 (T-2CAN Internal TWAI) or GPIO_NUM_26 (TCAN485 Internal TWAI)
    TWAI_MODE_NORMAL 
);
twai_timing_config_t twai_timing_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t twai_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


/**
 * @brief Arduino setup function - initializes hardware, peripherals, and tasks.
 */
void setup() {
#ifdef HARDWARE_T2CAN
    periph_module_disable(PERIPH_USB_MODULE);
    periph_module_reset(PERIPH_USB_MODULE);

    pinMode(19, INPUT_PULLDOWN);
    pinMode(20, INPUT_PULLDOWN);

    delay(100);
#endif

    Serial.begin(115200);
   
    uint32_t waitSerial = millis();
    while (!Serial && (millis() - waitSerial < 3000)); 

    Serial.println("\n\n=== INITIALIZING VEHICLE CONTROL UNIT ==="); 
    Serial.flush();

    // =========================================================================
    // 1. FILESYSTEM INITIALIZATION
    // =========================================================================
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] ERROR: LittleFS partition mounting failed! Formatting filesystem...");
    } else {
        Serial.println("[FS] LittleFS volume mounted successfully.");
    }
    
#ifdef HARDWARE_T2CAN
    Serial.println("[SYSTEM] Hardware Layer Detected: LilyGo T-2CAN (ESP32-S3)");
    Serial.printf("[SYSTEM] Flash Space: %d MB | PSRAM Matrix: %d KB\n",
                  ESP.getFlashChipSize() / (1024 * 1024),
                  ESP.getPsramSize() / 1024);
#endif

    // =========================================================================
    // 2. HARDWARE-SPECIFIC VOLTAGE AND INTERACTION LAYERS
    // =========================================================================
#ifdef HARDWARE_TCAN485
    // TCAN485 legacy architecture transceiver control configurations
    pinMode(PIN_5V_EN, OUTPUT);    digitalWrite(PIN_5V_EN, HIGH);
    pinMode(CAN_SE_PIN, OUTPUT);   digitalWrite(CAN_SE_PIN, LOW);
    pinMode(RS485_EN_PIN, OUTPUT); digitalWrite(RS485_EN_PIN, HIGH);
    pinMode(RS485_SE_PIN, OUTPUT); digitalWrite(RS485_SE_PIN, HIGH);

    pinMode(BAT_PUMP, OUTPUT_OPEN_DRAIN); digitalWrite(BAT_PUMP, HIGH);
    pinMode(TYPE2_LOCK_PIN, OUTPUT);      digitalWrite(TYPE2_LOCK_PIN, LOW);
    pinMode(TYPE2_UNLOCK_PIN, OUTPUT);    digitalWrite(TYPE2_UNLOCK_PIN, LOW);
#endif

#ifdef HARDWARE_T2CAN
    // LilyGo T-2CAN Power Delivery & MCP2515 SPI Infrastructure configuration
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);      // Enable main 5V buck rail for transceivers
    delay(100);

    pinMode(PIN_CANB_STBY, OUTPUT);
    digitalWrite(PIN_CANB_STBY, LOW);   // Move isolated internal transceiver out of standby

    // Assert crisp physical hardware reset down to the MCP2515 controller
    pinMode(PIN_MCP_RST, OUTPUT);
    digitalWrite(PIN_MCP_RST, LOW);     // Assert reset line
    delay(50);
    digitalWrite(PIN_MCP_RST, HIGH);    // Release reset line
    delay(150);                         // Required delay for stable 16MHz crystal oscillation sync

    SPI.begin(12, 13, 11);              // Bind SPI bus channels (SCK, MISO, MOSI)

    // Execute standard software initialization routines for MCP2515
    mcp2515.reset();

    // Bind communication bitrate (500 kbps matched to custom 16MHz oscillator limits)
    if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
        Serial.println("[SPI-CAN] CRITICAL ERROR: MCP2515 bitrate alignment failed! Halting system.");
        while (1) { delay(500); }
    }

    // === THE OFFICIAL API FIX ===
    // This native function activates normal mode AND sets the OSM bit internally.
    // No hacks, no pin conflicts, 100% stable.
    if (mcp2515.setNormalOneShotMode() != MCP2515::ERROR_OK) {
        Serial.println("[SPI-CAN] CRITICAL ERROR: Failed to switch MCP2515 to One-Shot mode! Halting.");
        while (1) { delay(500); }
    }

    Serial.println("[SPI-CAN] Native One-Shot Mode activated. Transmit retries disabled.");
    Serial.println("[SPI-CAN] External MCP2515 initialized successfully (500kbps @ 16MHz).");
#endif

    // =========================================================================
    // 3. COMMON HARDWARE INTERACTION INPUT AND OUTPUT PORTS
    // =========================================================================
    pinMode(TYPE2_MANUAL_UNLOCK_PIN, INPUT_PULLUP);
    pinMode(TYPE2_FEEDBACK_PIN, INPUT_PULLUP);

    // Call the clean combined English abstraction module for Hut V3 board layouts
    init_relay_hardware();

    // =========================================================================
    // 4. THREAD PRIVILEGES, METADATA CLEARING & SYSTEM WATCHDOGS
    // =========================================================================
    serialMutex = xSemaphoreCreateMutex();
    dataMutex   = xSemaphoreCreateMutex();
    idMutex     = xSemaphoreCreateMutex();
    mcpMutex    = xSemaphoreCreateMutex();

    serialMutexInitialized = (serialMutex != nullptr);
    dataMutexInitialized   = (dataMutex != nullptr);
    idMutexInitialized     = (idMutex != nullptr);

    // Enforce uniform clean slate across cross-core telemetry maps
    memset((void*)&telemetryData, 0, sizeof(TelemetryData));

    // Default configuration metrics safely aligned to zero to protect against boot alarms
    telemetryData.imdStatus = 0x00;  
    telemetryData.vifcStatus = 0x00; 
    telemetryData.mcuFaultLevel = 0x00;

    esp_task_wdt_init(10000, true);   // Set defensive 10-second FreeRTOS task runtime limit
    esp_task_wdt_add(NULL);

    // =========================================================================
    // 5. INTERNAL ESP32-S3 TWAI DRIVER CORE OPERATION LAYER
    // =========================================================================
    canQueue = xQueueCreate(100, sizeof(can_message_t));

    // Force internal hardware RX line to clear input artifacts
    pinMode(PIN_CAN_RX, INPUT_PULLUP);
    delay(50);

    esp_err_t install_err = twai_driver_install(&g_config, &twai_timing_config, &twai_filter_config);
    if (install_err != ESP_OK) {
        Serial.printf("[TWAI] Core driver setup failed: %s\n", esp_err_to_name(install_err));
    } else {
        esp_err_t start_err = twai_start();
        if (start_err == ESP_OK) {
            Serial.println("[TWAI] Internal automotive TWAI engine running successfully @ 500 kbps.");
        } else {
            Serial.printf("[TWAI] Core activation sequence failed: %s\n", esp_err_to_name(start_err));
        }
    }

    // =========================================================================
    // 6. SERIAL PERIPHERAL TESTING ROUTINES
    // =========================================================================
#ifdef HARDWARE_TCAN485
    RS485Serial.setRxBufferSize(256);
    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, GPIO_NUM_21, GPIO_NUM_22);
#endif

    // =========================================================================
    // 7. PWM SPEED REGULATION CHANNELS (LEDC TIMERS CORRECTION)
    // =========================================================================
#ifdef HARDWARE_T2CAN
    int batPumpPin = BAT_PUMP_PWM_PIN;
    int invPumpPin = INV_PUMP_PWM_PIN;
#else
    int batPumpPin = BAT_PUMP; // Fixed: Named matching legacy definition macro
    int invPumpPin = INV_PUMP; // Fixed: Named matching legacy definition macro
#endif

    // --- CRITICAL STEP: Configure hardware base timers ---
    ledc_timer_config_t bat_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT, // 0 - 255 tracking map
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = BAT_PWM_FREQ,     // 5kHz from configuration parameters
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&bat_timer);

    ledc_timer_config_t inv_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = INV_PWM_FREQ,     // 1kHz from configuration parameters
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&inv_timer);

    // --- Bind speed configuration channels over physical hardware paths ---
    ledc_channel_config_t bat_ch = {
        .gpio_num   = batPumpPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BAT_PWM_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&bat_ch);

    ledc_channel_config_t inv_ch = {
        .gpio_num   = invPumpPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = INV_PWM_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&inv_ch);

    // =========================================================================
    // 8. ASYNC STACKS SETUP & CORE RENDERING TARGETS
    // =========================================================================
    led.begin();
    led.setBrightness(0);

    delay(1000); // Short stabilization settling frame

    // Spawn processing loops into the FreeRTOS scheduling scheduler
    xTaskCreate(twai_receive_task,      "CAN_RX",    4096, NULL, 10, NULL);
    xTaskCreate(process_can_messages,   "CAN_PROC",  8192, NULL,  5, NULL);
    xTaskCreate(self_test_task,         "SELF_TEST", 2048, NULL,  4, NULL);
    xTaskCreatePinnedToCore(twai_monitor_task, "CAN_MON", 2048, NULL, 2, NULL, 1);

    initWebServer();
    Serial.println("=== SYSTEM INITIALIZATION COMPLETE - VCU OPERATIONAL ===");
}

// === loop() ===
void loop() {
    // 1. WATCHDOG & TIMING TRACKING
    esp_task_wdt_reset();
    unsigned long now = millis();

    // =========================================================================
    // A. UNTIMERED / INDEPENDENT CYCLE: CAN TRANSMISSION HEARTBEAT
    // =========================================================================
    // Moved outside of the slow cycle constraint block to ensure that internal 
    // timers inside the task don't jitter, preventing critical inverter or charger timeouts.
    CAN_Transmit_Task(); 


    // =========================================================================
    // B. FAST CYCLE: COCKPIT INDICATORS & TELEMETRY STREAM (200ms / 5Hz)
    // =========================================================================
    // Critical for smooth physical needle movement on the classic cluster gauges
    static unsigned long lastFastComm = 0;
    if (now - lastFastComm >= 200) {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            
            #ifdef HARDWARE_TCAN485
                send_rs485_telemetry(); 
            #endif

            #ifdef HARDWARE_T2CAN
                send_can_telemetry(); 
                // receive_can_telemetry(); // Uncomment if polling instead of interrupt driven
            #endif

            xSemaphoreGive(serialMutex);
        }
        lastFastComm = now;
    }


    // =========================================================================
    // C. MEDIUM CYCLE: WEB DASHBOARD ASYNC BROADCAST (1000ms / 1Hz)
    // =========================================================================
    // Throttled to 1000ms to maintain total WebSocket stability on the mobile UI
    static unsigned long lastWebUpdate = 0;
    if (now - lastWebUpdate >= 1000) {
        updateWebDashboard();
        lastWebUpdate = now;
    }


    // =========================================================================
    // D. SLOW CYCLE: VEHICLE SAFETY, MECHANICAL LOCKS & ALARMS (500ms / 2Hz)
    // =========================================================================
    static unsigned long lastSafetyCheck = 0;
    if (now - lastSafetyCheck >= 500) {
        
        // 1. Core Diagnostics & State Machine Tracking
        check_data_timeout(now);
        handleLockState();          // Coordinates Type 2 charge lock sequence
        
        // 2. UNBREAKABLE INTERCEPT SHUNT MATRIX (MOVED BEFORE COMMITS)
        // This intercepts and locks user selections into RAM right before the
        // low-level hardware functions execute and drive the physical pins.
        if (manualOverride > 0) {
            // --- Digital Indicators & Alerts ---
            if (manualOverride & (1UL << OVR_BIT_CHECK_OIL))  { telemetryData.ledCheckOil = (manualOverride_Values & (1UL << OVR_BIT_CHECK_OIL)) ? true : false; }
            if (manualOverride & (1UL << OVR_BIT_LED_BATT))   { telemetryData.ledBattery  = (manualOverride_Values & (1UL << OVR_BIT_LED_BATT)) ? true : false; }
            if (manualOverride & (1UL << OVR_BIT_FAN))        { telemetryData.fanRelay    = (manualOverride_Values & (1UL << OVR_BIT_FAN)) ? true : false; }
            if (manualOverride & (1UL << OVR_BIT_PIEZO))      { telemetryData.isPiezoOn   = (manualOverride_Values & (1UL << OVR_BIT_PIEZO)) ? true : false; }
            if (manualOverride & (1UL << OVR_BIT_ALARM))      { telemetryData.isAlarm     = (manualOverride_Values & (1UL << OVR_BIT_ALARM)) ? true : false; }

            // --- Hut V3 Auxiliary Board Relays ---
            if (manualOverride & (1UL << OVR_BIT_REL11))      { telemetryData.auxRelay11  = (manualOverride_Values & (1UL << OVR_BIT_REL11)) ? true : false; }
            if (manualOverride & (1UL << OVR_BIT_REL12))      { telemetryData.auxRelay12  = (manualOverride_Values & (1UL << OVR_BIT_REL12)) ? true : false; }
            if (manualOverride & (1UL << OVR_BIT_REL13))      { telemetryData.auxRelay13  = (manualOverride_Values & (1UL << OVR_BIT_REL13)) ? true : false; }
            if (manualOverride & (1UL << OVR_BIT_REL14))      { telemetryData.auxRelay14  = (manualOverride_Values & (1UL << OVR_BIT_REL14)) ? true : false; }
        }

        // 3. Physical Driver Commits: Driven safely with shielded override values
        update_led();               // Commits current battery lighting states to hardware pins
        update_auxiliary_relays();  // Commits Hut V3 relay states to expansion board hardware ports
        update_alarm_states();      // Commits oil lamp, radiator fan, and acoustic loops to pins

        // 4. Automated Edge Trigger for the Bender IMD Isolation Self-Test
        static uint8_t lastBmsStatus = BMS_STATUS_BOOT;
        uint8_t currentBmsStatus = BMS_STATUS_BOOT;
        bool bmsValid = false;

        WITH_DATA_MUTEX({
            currentBmsStatus = telemetryData.bmsStatus;
            bmsValid = telemetryData.bmsStatusValid;
        });

        if (bmsValid) {
            bool isOperational = (currentBmsStatus == BMS_STATUS_READY || 
                                  currentBmsStatus == BMS_STATUS_DRIVE || 
                                  currentBmsStatus == BMS_STATUS_CHARGE);

            bool imdSelfTestMissing = (telemetryData.vifcStatus & (1 << 12)) || (telemetryData.vifcStatus & (1 << 13));

            if (isOperational && imdSelfTestMissing && !telemetryData.selfTestRunning) {
                WITH_DATA_MUTEX({ telemetryData.selfTestRequested = true; });
                safe_printf("[SYSTEM] Compliance Trigger: IMD flags test missing (Bits 12/13). Initiating self-test.\n");
            }
            
            lastBmsStatus = currentBmsStatus;
        }
        
        lastSafetyCheck = now;
    }
    
    // =========================================================================
    // E. ULTRA-SLOW CYCLE: THERMAL PROPULSION LOOPS (2000ms / 0.5Hz)
    // =========================================================================
    static unsigned long lastThermalUpdate = 0;
    if (now - lastThermalUpdate >= 2000) {
        int8_t mcu_t = 0; bool mcu_v = false;
        float bat_t = 0;  bool bat_v = false;

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            mcu_t = telemetryData.mcuTemp;
            mcu_v = telemetryData.mcuTempValid;
            bat_t = telemetryData.batTemp;
            bat_v = telemetryData.batTempValid;
            xSemaphoreGive(dataMutex);
        }

        // 1. Run Baseline Automated Thermal Loop Updates Normally
        update_inv_pump(mcu_t, mcu_v);
        update_bat_pump(bat_t, bat_v);

        // 2. Dynamic Thermal Override Enforcer
        // Forces your manual multi-stage pump values back into the active registers
        if (manualOverride & (1UL << OVR_BIT_INV_PUMP)) {
            telemetryData.invPumpPwm   = manual_invPumpPwm;
            telemetryData.invPumpRelay = (manual_invPumpPwm > 0);
        }
        if (manualOverride & (1UL << OVR_BIT_BAT_PUMP)) {
            telemetryData.batPumpPwm   = manual_batPumpPwm;
            telemetryData.batPumpRelay = (manual_batPumpPwm > 0);
        }
        
        lastThermalUpdate = now;
    }
   
    // =========================================================================
    // F. FREE-RTOS CORES YIELD DELAY
    // =========================================================================
    // Crucial 10ms yield window. Prevents CPU starvation on Core 1 and leaves 
    // Core 0 completely open to process TCP/IP packets and async web transactions.
    vTaskDelay(pdMS_TO_TICKS(10));
}