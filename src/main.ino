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
#include <FastLED.h>

#include "main.h"
#include "can_process.h"
#include "lock_control.h"
#include "telemetry.h" // Renamed from rs485.h
#include "pump_control.h"
#include "relay_control.h"
#include "utils.h"
#include "self_test.h"
#include "web_server.h"

#ifdef HARDWARE_T2CAN
#include <mcp_can.h>
MCP_CAN CAN_Peripheral(PIN_MCP_CS); 
#endif

// === DEFINITION ===
const uint32_t FILTERED_IDS[NUM_FILTERS] = {0x01, 0x22, 0x37, 0x239, 0x355, 0x356, 0x379, 0x35A};
const uint32_t FAST_IDS[] = {0x01, 0x22,  0x239, 0x355, 0x356, 0x379, 0x35A};
const uint32_t SLOW_IDS[] = {0x37};

const uint32_t FAST_TIMEOUT_MS = 300;
const uint32_t SLOW_TIMEOUT_MS = 2000;
const uint32_t CURRENT_THRESHOLD = 5;
const uint32_t AUTO_UNLOCK_TIMEOUT_MS = 60000;

volatile TelemetryData telemetryData = {0};
bool demoModeActive = false;
bool manualUnlockPressed = false;
CRGB leds[1];

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
 * @brief Arduino setup function - initializes hardware, peripherals, and tasks
 */
void setup() {
    // ====================== PRE-INITIALIZATION: POWER AND TRANSCEIVER WAKE ======================
    // Enable main power rail and wake transceivers before installing drivers
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);        // Enable 5V rail for CAN transceivers and peripherals

    pinMode(PIN_CANB_STBY, OUTPUT);
    digitalWrite(PIN_CANB_STBY, LOW);     // Put internal TWAI transceiver into Normal mode (active)

    pinMode(PIN_MCP_RST, OUTPUT);
    digitalWrite(PIN_MCP_RST, LOW);   // Reset MCP2515
    delay(10);
    digitalWrite(PIN_MCP_RST, HIGH);  // Release reset (hold high for normal operation)

    delay(150);  // Short stabilization delay for power rails and transceivers

    // ====================== SERIAL MONITOR ======================
    Serial.begin(115200);

    // Wait for native USB on ESP32-S3 (important for reliable serial output)
    uint32_t timeout = millis();
    while (!Serial && millis() - timeout < 3000) {
        delay(10);
    }

    Serial.println("\n[SYSTEM] USB Serial Connected.");
    Serial.println("\n--- VCU BOOT START ---");

#ifdef HARDWARE_T2CAN
    Serial.println("Hardware: LilyGo T-2CAN (ESP32-S3)");
    Serial.printf("Flash: %d MB | PSRAM: %d KB\n",
                  ESP.getFlashChipSize() / (1024 * 1024),
                  ESP.getPsramSize() / 1024);
#endif

    // ====================== HARDWARE-SPECIFIC PIN CONFIGURATION ======================
#ifdef HARDWARE_TCAN485
    // TCAN485 specific pins
    pinMode(PIN_5V_EN, OUTPUT);    digitalWrite(PIN_5V_EN, HIGH);
    pinMode(CAN_SE_PIN, OUTPUT);   digitalWrite(CAN_SE_PIN, LOW);
    pinMode(RS485_EN_PIN, OUTPUT); digitalWrite(RS485_EN_PIN, HIGH);
    pinMode(RS485_SE_PIN, OUTPUT); digitalWrite(RS485_SE_PIN, HIGH);

    pinMode(BAT_PUMP, OUTPUT_OPEN_DRAIN); digitalWrite(BAT_PUMP, HIGH);
    pinMode(TYPE2_LOCK_PIN, OUTPUT);      digitalWrite(TYPE2_LOCK_PIN, LOW);
    pinMode(TYPE2_UNLOCK_PIN, OUTPUT);    digitalWrite(TYPE2_UNLOCK_PIN, LOW);
#endif

#ifdef HARDWARE_T2CAN
    // T-2CAN specific pins (relays, lock control, etc.)
    pinMode(BAT_PUMP_RELAY, OUTPUT);  digitalWrite(BAT_PUMP_RELAY, LOW);
    pinMode(INV_PUMP_RELAY, OUTPUT);  digitalWrite(INV_PUMP_RELAY, LOW);
    pinMode(FAN_RELAY_PIN, OUTPUT);   digitalWrite(FAN_RELAY_PIN, LOW);
    pinMode(TYPE2_LOCK_IN1, OUTPUT);  digitalWrite(TYPE2_LOCK_IN1, LOW);
    pinMode(TYPE2_LOCK_IN2, OUTPUT);  digitalWrite(TYPE2_LOCK_IN2, LOW);

    // MCP2515 (second CAN bus) reset and power
    pinMode(PIN_MCP_RST, OUTPUT);
    digitalWrite(PIN_MCP_RST, LOW);   // Reset pulse
    delay(10);
    digitalWrite(PIN_MCP_RST, HIGH);  // Release reset

    pinMode(PIN_5V_EN, OUTPUT);       digitalWrite(PIN_5V_EN, HIGH);
    pinMode(PIN_CANB_STBY, OUTPUT);   digitalWrite(PIN_CANB_STBY, LOW);  // Enable internal TWAI transceiver
    pinMode(PIN_MCP_CS, OUTPUT);      digitalWrite(PIN_MCP_CS, HIGH);
#endif

    // Common input pins
    pinMode(TYPE2_MANUAL_UNLOCK_PIN, INPUT_PULLUP);
    pinMode(TYPE2_FEEDBACK_PIN, INPUT_PULLUP);

    // ====================== MUTEXES AND WATCHDOG ======================
    serialMutex = xSemaphoreCreateMutex();
    dataMutex   = xSemaphoreCreateMutex();
    idMutex     = xSemaphoreCreateMutex();

    esp_task_wdt_init(10000, true);   // 10 second watchdog timeout
    esp_task_wdt_add(NULL);

    // ====================== INTERNAL TWAI (CAN B) INITIALIZATION ======================
    canQueue = xQueueCreate(100, sizeof(can_message_t));

    // Ensure RX pin is HIGH (recessive state) to prevent "Illegal Dominant" error on startup
    pinMode(PIN_CAN_RX, INPUT_PULLUP);
    delay(50);

    // Install and start TWAI driver (uses standard 500 kbps timing)
    esp_err_t install_err = twai_driver_install(&g_config, &twai_timing_config, &twai_filter_config);
    if (install_err != ESP_OK) {
        Serial.printf("TWAI driver install failed: %s\n", esp_err_to_name(install_err));
    } else {
        esp_err_t start_err = twai_start();
        if (start_err == ESP_OK) {
            Serial.println("TWAI (Internal CAN) started successfully @ 500 kbps");
        } else {
            Serial.printf("TWAI start failed: %s\n", esp_err_to_name(start_err));
        }
    }

    // ====================== TELEMETRY INTERFACES ======================
#ifdef HARDWARE_TCAN485
    RS485Serial.setRxBufferSize(256);
    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, GPIO_NUM_21, GPIO_NUM_22);
#endif

#ifdef HARDWARE_T2CAN
    // MCP2515 (second CAN bus via SPI)
    SPI.begin(12, 13, 11, 10);  // SCLK, MISO, MOSI, CS

    if (CAN_Peripheral.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
        CAN_Peripheral.setMode(MCP_NORMAL);
        Serial.println("MCP2515 (CANA) initialized successfully");
    } else {
        Serial.println("MCP2515 initialization failed");
    }
#endif

    // ====================== PWM CHANNELS FOR PUMPS ======================
#ifdef HARDWARE_T2CAN
    int batPumpPin = BAT_PUMP_PWM;
    int invPumpPin = INV_PUMP_PWM;
#else
    int batPumpPin = BAT_PUMP;
    int invPumpPin = INV_PUMP;
#endif

    // Battery pump PWM
    ledc_channel_config_t bat_ch = {
        .gpio_num   = batPumpPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BAT_PWM_CHANNEL,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&bat_ch);

    // Inverter pump PWM
    ledc_channel_config_t inv_ch = {
        .gpio_num   = invPumpPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = INV_PWM_CHANNEL,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&inv_ch);

    // ====================== LED AND WEB SERVER ======================
#ifdef HARDWARE_T2CAN
    FastLED.addLeds<WS2812, WS2812_DATA_PIN, GRB>(leds, 1);
#else
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, 1);
#endif
    FastLED.setBrightness(20);
    leds[0] = CRGB::Black;
    FastLED.show();

    initWebServer();

    // ====================== TASK CREATION ======================
    // Priorities tuned for stability: CAN RX highest, processing lower to avoid starving WiFi/core 0
    xTaskCreate(twai_receive_task,      "CAN_RX",   4096, NULL, 10, NULL);
    xTaskCreate(process_can_messages,   "CAN_PROC", 8192, NULL,  5, NULL);
    xTaskCreate(self_test_task,         "SELF_TEST",2048, NULL,  4, NULL);
    xTaskCreatePinnedToCore(twai_monitor_task, "CAN_MON", 2048, NULL, 2, NULL, 1);
}

// === loop() ===
void loop() {
    // 1. WATCHDOG & TIMING
    esp_task_wdt_reset();
    unsigned long now = millis();

    // --- FAST CYCLE: RS485 TELEMETRY & BMS PROXY (200ms / 5Hz) ---
    // Critical for smooth needle movement on the RS485 dashboard
    // Wrapped in a mutex check to prevent serial bus contention
    static unsigned long lastFastComm = 0;
    if (now - lastFastComm >= 200) {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            
            #ifdef HARDWARE_TCAN485
                send_rs485_telemetry(); 
            #endif

            #ifdef HARDWARE_T2CAN
                send_can_telemetry(); 
            #endif

            CAN_Transmit_Task(); 
            xSemaphoreGive(serialMutex);
        }
        lastFastComm = now;
    }

    // --- MEDIUM CYCLE: WEB DASHBOARD UPDATE (250ms / 4Hz) --- TEST with 1000ms
    // Provides steady updates to the browser without flooding the network
    static unsigned long lastWebUpdate = 0;
    if (now - lastWebUpdate >= 1000) {
        updateWebDashboard();
        lastWebUpdate = now;
    }

    // --- SLOW CYCLE: SAFETY, LOCKS & TIMEOUTS (500ms / 2Hz) ---
    // Lock logic and CAN timeouts don't need high-frequency processing
    static unsigned long lastSafetyCheck = 0;
    if (now - lastSafetyCheck >= 500) {
        check_data_timeout(now);
        handleLockState(); // Evaluates manual unlock, auto-lock, and safety conditions
        updateRelayAutomation(); // Evaluate sensor data and update relay targets
        CAN_Transmit_Task(); // Handles range updates and relay shadow transmission
        update_led(); // Update Status LED (INIT, READY, etc.)
        lastSafetyCheck = now;
    }

    // --- ULTRA-SLOW CYCLE: THERMAL MANAGEMENT (2000ms / 0.5Hz) ---
    // Pump control based on temperatures. Reducing Mutex calls here 
    // significantly improves WebSocket stability.
    static unsigned long lastThermalUpdate = 0;
    if (now - lastThermalUpdate >= 2000) {
        int8_t mcu_t = 0; bool mcu_v = false;
        float bat_t = 0; bool bat_v = false;

        // Take Mutex briefly to capture latest sensor data
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            mcu_t = telemetryData.mcu_temp;
            mcu_v = telemetryData.mcuTempValid;
            bat_t = telemetryData.bat_temp;
            bat_v = telemetryData.batTempValid;
            xSemaphoreGive(dataMutex);
        }

        update_inv_pump(mcu_t, mcu_v);
        update_bat_pump(bat_t, bat_v);
        lastThermalUpdate = now;
    }

    // --- SYSTEM STABILITY DELAY ---
    // Crucial: Gives Core 0 (WiFi/TCP) 10ms to handle pings and packets
    vTaskDelay(pdMS_TO_TICKS(10));
}