// VCU_X19e_Hut2_ST_20251029105113.ino
#define DEBUG
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_task_wdt.h>
#include <driver/ledc.h>
#include <FastLED.h>

#include "main.h"
#include "can_process.h"
#include "lock_control.h"
#include "rs485.h"
#include "pump_control.h"
#include "proxy_bms.h"
#include "utils.h"
#include "self_test.h"

// === DEFINITIONEN ===
const uint32_t FILTERED_IDS[NUM_FILTERS] = {0x22, 0x37, 0x379, 0x239, 0x355, 0x356, 0x35A};
const uint32_t FAST_IDS[] = {0x22, 0x239, 0x355, 0x356, 0x379, 0x35A};
const uint32_t SLOW_IDS[] = {0x37};

const uint32_t FAST_TIMEOUT_MS = 300;
const uint32_t SLOW_TIMEOUT_MS = 2000;
const uint32_t CURRENT_THRESHOLD = 5;
const uint32_t AUTO_UNLOCK_TIMEOUT_MS = 60000;

volatile unsigned long last_id_timestamps[NUM_FILTERS] = {0};
uint32_t received_ids[MAX_IDS] = {0};
uint8_t received_ids_count = 0;

volatile TelemetryData telemetryData = {0};
CRGB leds[1];

unsigned long last_send_time = 0;
volatile unsigned long last_receive_time = 0;
volatile unsigned long last_can_update = 0;

bool selfTestRunning = false;
bool selfTestRequested = false;
uint8_t selfTestResult = 0;

bool isLocked = false, isLocking = false, isUnlocking = false, shouldLock = false;
bool is_charging = false;

QueueHandle_t canQueue;
HardwareSerial RS485Serial(2);

SemaphoreHandle_t serialMutex = nullptr, dataMutex = nullptr, idMutex = nullptr;
bool serialMutexInitialized = false, dataMutexInitialized = false, idMutexInitialized = false;

// === TWAI CONFIG ===
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_27, GPIO_NUM_26, TWAI_MODE_NORMAL);
twai_timing_config_t twai_timing_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t twai_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// === setup() ===
void setup() {
    pinMode(PIN_5V_EN, OUTPUT); digitalWrite(PIN_5V_EN, HIGH);
    pinMode(CAN_SE_PIN, OUTPUT); digitalWrite(CAN_SE_PIN, LOW);
    pinMode(GPIO_NUM_17, OUTPUT); digitalWrite(GPIO_NUM_17, LOW);
    pinMode(GPIO_NUM_19, OUTPUT); digitalWrite(GPIO_NUM_19, HIGH);
    pinMode(BAT_PUMP, OUTPUT_OPEN_DRAIN); digitalWrite(BAT_PUMP, HIGH);

    pinMode(TYPE2_LOCK_PIN, OUTPUT); digitalWrite(TYPE2_LOCK_PIN, LOW);
    pinMode(TYPE2_UNLOCK_PIN, OUTPUT); digitalWrite(TYPE2_UNLOCK_PIN, LOW);
    pinMode(TYPE2_MANUAL_UNLOCK_PIN, INPUT_PULLUP);
    pinMode(TYPE2_FEEDBACK_PIN, INPUT_PULLUP);

    telemetryData.imdIsoR = 50000;
    telemetryData.imdIsoRValid = false;

    Serial.begin(115200);
    safe_printf("setup: OK\n");

    esp_task_wdt_config_t wdt = {.timeout_ms = 10000, .trigger_panic = true};
    esp_task_wdt_init(&wdt);
    esp_task_wdt_add(NULL);

    esp_wifi_deinit(); WiFi.mode(WIFI_OFF); esp_bt_controller_deinit();

    serialMutex = xSemaphoreCreateMutex(); serialMutexInitialized = true;
    dataMutex = xSemaphoreCreateMutex(); dataMutexInitialized = true;
    idMutex = xSemaphoreCreateMutex(); idMutexInitialized = true;

    canQueue = xQueueCreate(50, sizeof(can_message_t));

    if (twai_driver_install(&g_config, &twai_timing_config, &twai_filter_config) != ESP_OK ||
        twai_start() != ESP_OK) {
        safe_printf("TWAI fail\n"); while(1);
    }

    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, GPIO_NUM_21, GPIO_NUM_22);
    RS485Serial.setRxBufferSize(256);

    // PWM
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

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, 1);
    FastLED.setBrightness(20); leds[0] = CRGB::Black; FastLED.show();

    xTaskCreate(process_can_messages, "CAN_PROC", 4096, NULL, 8, NULL);
    xTaskCreate(twai_receive_task, "CAN_RX", 3072, NULL, 12, NULL);
    xTaskCreatePinnedToCore(twai_monitor_task, "CAN_MON", 2048, NULL, 4, NULL, 1);
    xTaskCreate(self_test_task, "SELF_TEST", 2048, NULL, 5, NULL);
}

// === loop() ===
void loop() {
    esp_task_wdt_reset();
    unsigned long now = millis();

    handleQueueMonitoring(now);
    handleCanBus();

    static unsigned long t1 = 0;
    if (now - t1 >= 100) { check_data_timeout(now); t1 = now; }

    static unsigned long t2 = 0;
    if (now - t2 >= 50) { handleLockState(); t2 = now; }

    static int8_t mcu_temp = 0; static bool mcu_valid = false;
    WITH_DATA_MUTEX({
        mcu_temp = telemetryData.mcu_temp; mcu_valid = telemetryData.mcuTempValid;
    });
    update_inv_pump(mcu_temp, mcu_valid);

    static float bat_temp = 0; static bool bat_valid = false;
    WITH_DATA_MUTEX({
        bat_temp = telemetryData.bat_temp; bat_valid = telemetryData.batTempValid;
    });
    update_bat_pump(bat_temp, bat_valid);

    static unsigned long t3 = 0;
    if (now - t3 >= 500) { update_led(); t3 = now; }

    static unsigned long t4 = 0;
    if (now - t4 >= 200) { send_rs485_telemetry(); t4 = now; }

    static unsigned long t5 = 0;
    if (now - t5 >= 100) { send_proxy_bms_data(); t5 = now; }

    vTaskDelay(pdMS_TO_TICKS(20));
}