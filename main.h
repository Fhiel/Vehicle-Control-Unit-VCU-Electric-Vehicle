// main.h
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/twai.h>
#include <driver/gpio.h>
#include <FastLED.h>
#include <cstdint>
#include <esp_err.h>  // ← für esp_err_t

#include "utils.h"

using namespace std;

// === KONSTANTEN ===
#define NUM_FILTERS 7
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

// === PLAUSIBILITÄT ===
#define MOTOR_RPM_MAX       20000
#define TEMP_MIN            -20.0f
#define MCU_TEMP_MAX        85.0f
#define MOTOR_TEMP_MAX      100.0f
#define BMS_TEMP_MAX        60.0f
#define BMS_SOC_MAX         100
#define BMS_CURRENT_MAX     1000.0f
#define IMD_ISO_R_MAX       50000

// === TELEMETRY DATA ===
typedef struct {
    uint16_t motorRPM; bool motorRPMValid;
    int8_t motor_temp; bool motorTempValid;
    int8_t mcu_temp; bool mcuTempValid;
    uint16_t mcuFlags; bool mcuFlagsValid;
    uint8_t mcuFaultLevel; bool mcuFaultLevelValid;

    uint8_t bmsSoC; bool bmsSoCValid;
    int16_t bmsCurrent; bool bmsCurrentValid;
    float bat_temp; bool batTempValid;
    uint8_t bmsStatus; bool bmsStatusValid;
    uint16_t bmsWarnings; bool bmsWarningsValid;
    bool bmsLowVoltageWarn;
    bool bmsHighTempWarn;
    bool bmsLowTempWarn;
    bool bmsHighCurrentWarn;

    uint16_t imdIsoR; bool imdIsoRValid;
    uint16_t imdStatus; bool imdStatusValid;
    uint16_t vifcStatus; bool vifcStatusValid;
    uint16_t hv1Voltage; bool hv1VoltageValid;
    uint8_t imdNegRelay; bool imdNegRelayValid;
    uint8_t imdPosRelay; bool imdPosRelayValid;
    bool imdRelayOpen; bool imdRelayOpenValid;
    uint8_t selfTestResult; bool selfTestResultValid;
    bool selfTestFailed;

    uint8_t actual_bms_status;
    uint8_t proxy_bms_status;
    bool imd_stop_flag;
    bool isLocked, isUnLocking, isLocking, shouldLock;
    bool selfTestRequested, selfTestRunning;
    bool is_charging;
} TelemetryData;

extern volatile TelemetryData telemetryData;

// === CAN MESSAGE ===
typedef struct {
    uint32_t identifier;
    uint8_t extd;
    uint8_t data_length_code;
    uint8_t data[8];
} can_message_t;

// === GLOBALE VARIABLEN ===
extern uint32_t received_ids[MAX_IDS];
extern uint8_t received_ids_count;
extern volatile unsigned long last_id_timestamps[NUM_FILTERS];
extern volatile unsigned long last_receive_time, last_can_update;
extern unsigned long last_send_time;

extern bool selfTestRunning, selfTestRequested;
extern uint8_t selfTestResult;

extern bool isLocked, isLocking, isUnlocking, shouldLock;
extern bool is_charging;

extern QueueHandle_t canQueue;
extern HardwareSerial RS485Serial;

extern SemaphoreHandle_t serialMutex, dataMutex, idMutex;
extern bool serialMutexInitialized, dataMutexInitialized, idMutexInitialized;

extern CRGB leds[1];

// === TWAI CONFIG ===
extern twai_general_config_t g_config;
extern twai_timing_config_t twai_timing_config;
extern twai_filter_config_t twai_filter_config;

// === PINS ===
#define PIN_5V_EN                   GPIO_NUM_16
#define CAN_SE_PIN                  GPIO_NUM_23
#define LED_PIN                     GPIO_NUM_4
#define TYPE2_LOCK_PIN              GPIO_NUM_25
#define TYPE2_UNLOCK_PIN            GPIO_NUM_5
#define TYPE2_MANUAL_UNLOCK_PIN     GPIO_NUM_12  
#define TYPE2_FEEDBACK_PIN          GPIO_NUM_18
#define BAT_PUMP                    GPIO_NUM_32
#define BAT_PWM_CHANNEL             (ledc_channel_t)0
#define BAT_PWM_FREQ                5000         
#define INV_PUMP                    GPIO_NUM_33
#define INV_PWM_CHANNEL             (ledc_channel_t)1
#define INV_PWM_FREQ                1000         

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

// === FUNKTIONEN ===
void update_led();
void handleLockState();

#endif