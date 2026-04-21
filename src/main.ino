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
#include "rs485.h"
#include "pump_control.h"
#include "relay_control.h"
#include "utils.h"
#include "self_test.h"
#include "web_server.h"

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
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_27, GPIO_NUM_26, TWAI_MODE_NORMAL);
twai_timing_config_t twai_timing_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t twai_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();



// === setup() ===
void setup() {
    // 1. Hardware Power & Pins
    pinMode(PIN_5V_EN, OUTPUT); digitalWrite(PIN_5V_EN, HIGH);
    pinMode(CAN_SE_PIN, OUTPUT); digitalWrite(CAN_SE_PIN, LOW);
    pinMode(RS485_EN_PIN, OUTPUT); digitalWrite(RS485_EN_PIN, HIGH);
    pinMode(RS485_SE_PIN, OUTPUT); digitalWrite(RS485_SE_PIN, HIGH);

    pinMode(BAT_PUMP, OUTPUT_OPEN_DRAIN); digitalWrite(BAT_PUMP, HIGH);

    pinMode(TYPE2_LOCK_PIN, OUTPUT); digitalWrite(TYPE2_LOCK_PIN, LOW);
    pinMode(TYPE2_UNLOCK_PIN, OUTPUT); digitalWrite(TYPE2_UNLOCK_PIN, LOW);
    pinMode(TYPE2_MANUAL_UNLOCK_PIN, INPUT_PULLUP);
    pinMode(TYPE2_FEEDBACK_PIN, INPUT_PULLUP);

    // 2. Telemetry Clean Start
    last_can_update = millis() -10000; // Ensure demo mode can start if no messages are received immediately

    // 3. Serial & Mutex Initialization
    Serial.begin(115200);
    
    serialMutex = xSemaphoreCreateMutex(); serialMutexInitialized = true;
    dataMutex = xSemaphoreCreateMutex(); dataMutexInitialized = true;
    idMutex = xSemaphoreCreateMutex(); idMutexInitialized = true;

    safe_printf("setup: Mutex Init OK\n");

    // 4. Watchdog Configuration
    esp_task_wdt_init(10000, true); 
    esp_task_wdt_add(NULL);

    // 5. CAN Bus Setup
    canQueue = xQueueCreate(100, sizeof(can_message_t));

    if (twai_driver_install(&g_config, &twai_timing_config, &twai_filter_config) != ESP_OK ||
        twai_start() != ESP_OK) {
        safe_printf("TWAI fail\n"); while(1);
    }

    // 6. RS485
    RS485Serial.setRxBufferSize(256);
    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, GPIO_NUM_21, GPIO_NUM_22);


    // 7. PWM Channels
    ledc_timer_config_t bat_timer = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
                                     .timer_num = LEDC_TIMER_0, .freq_hz = BAT_PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&bat_timer);
    ledc_channel_config_t bat_ch = {.gpio_num = BAT_PUMP, .speed_mode = LEDC_LOW_SPEED_MODE,
                                    .channel = BAT_PWM_CHANNEL, .timer_sel = LEDC_TIMER_0, .duty = 0};
    ledc_channel_config(&bat_ch);

    ledc_timer_config_t inv_timer = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
                                     .timer_num = LEDC_TIMER_1, .freq_hz = INV_PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&inv_timer);
    ledc_channel_config_t inv_ch = {.gpio_num = INV_PUMP, .speed_mode = LEDC_LOW_SPEED_MODE,
                                    .channel = INV_PWM_CHANNEL, .timer_sel = LEDC_TIMER_1, .duty = 0};
    ledc_channel_config(&inv_ch);

    // 8. LEDs & WebServer
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, 1);
    FastLED.setBrightness(20); leds[0] = CRGB::Black; FastLED.show();

    initWebServer();
  
    // 9. Task Creation (Priorities adjusted for Stability)
    // Lower the CAN priorities slightly so they don't starve the system
    // Keep CAN_RX highest to avoid hardware overflow, but bring PROC down.
    xTaskCreate(twai_receive_task, "CAN_RX", 4096, NULL, 10, NULL); 
    xTaskCreate(process_can_messages, "CAN_PROC", 8192, NULL, 5, NULL); 
    xTaskCreate(self_test_task, "SELF_TEST", 2048, NULL, 4, NULL);
    xTaskCreatePinnedToCore(twai_monitor_task, "CAN_MON", 2048, NULL, 2, NULL, 1);
}

// === loop() ===
void loop() {
    // 1. WATCHDOG & TIMING
    esp_task_wdt_reset();
    unsigned long now = millis();

    // --- HIGH PRIORITY: CAN BUS MONITORING (Every Loop) ---
    // These functions are non-blocking and essential for queue health
    // handleQueueMonitoring(now);
    // handleCanBus();

    // --- FAST CYCLE: RS485 TELEMETRY & BMS PROXY (200ms / 5Hz) ---
    // Critical for smooth needle movement on the RS485 dashboard
    // Wrapped in a mutex check to prevent serial bus contention
    static unsigned long lastFastComm = 0;
    if (now - lastFastComm >= 200) {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            send_rs485_telemetry(); 
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