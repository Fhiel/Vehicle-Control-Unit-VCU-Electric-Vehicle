/**
 * @file self_test.cpp
 * @brief Active IMD Self-Test and HV-Relay coordination.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#define DEBUG
#include "self_test.h"
#include <esp_task_wdt.h>
#include "main.h"

/**
 * @brief Sends a formatted command to the IMD (Bender/Isometer style).
 * CAN ID: 0x22, DLC: 5
 */
void send_imd_request(uint16_t cmd, uint8_t param) {
    twai_message_t message = {0};
    message.identifier = 0x22;
    message.data_length_code = 5;
    message.data[0] = (uint8_t)(cmd >> 8);   // CMD High
    message.data[1] = (uint8_t)(cmd & 0xFF); // CMD Low
    message.data[2] = param;
    
    twai_transmit(&message, pdMS_TO_TICKS(50));
}

void send_imd_self_test_start(bool full_test) {
    send_imd_request(0x00D0, full_test ? 1 : 0);
}

/**
 * @brief Controls the IMD internal HV coupling relays.
 */
void set_imd_hv_relays(bool open) {
    // CMD 0x00D2: 0 = NEG, 1 = POS
    // param 0x01 = Close, 0x00 = Open
    uint8_t state = open ? 0x00 : 0x01; 
    
    send_imd_request(0x00D2, 0); // NEG
    vTaskDelay(pdMS_TO_TICKS(20));
    send_imd_request(0x00D2, 1); // POS
    
    WITH_DATA_MUTEX({ telemetryData.imdRelayOpen = open; });
}

/**
 * @brief Global HV Release for the BMS (Master Interlock).
 * ID 0x309: 0x01 = Enable, 0x00 = Inhibit
 */
void send_bms_relay_release(bool enable) {
    static bool last_sent = false;
    twai_message_t msg = {0};
    msg.identifier = 0x309;
    msg.data_length_code = 1;
    msg.data[0] = enable ? 0x01 : 0x00;

    if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
        last_sent = enable;
    }
}

/**
 * @brief Task: Manages the active IMD Self-Test sequence.
 * This ensures HV isolation is verified before the car enters DRIVE or CHARGE.
 */
void self_test_task(void *parameter) {
    esp_task_wdt_add(NULL);
    
    enum SelfTestState {
        IDLE,
        OPENING_RELAYS,
        CHECKING_VOLTAGE,
        RUNNING_TEST,
        EVALUATING,
        CLEANUP_SUCCESS,
        CLEANUP_FAULT
    } state = IDLE;

    unsigned long stateTimer = 0;
    uint8_t retryCount = 0;

    while (1) {
        esp_task_wdt_reset();
        unsigned long now = millis();

        // Local copies of shared data
        bool requested = false;
        bool charging = false;
        uint16_t hv1 = 0;
        uint8_t result = 0;
        bool resultValid = false;

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            requested = telemetryData.selfTestRequested;
            charging = telemetryData.is_charging;
            hv1 = telemetryData.hv1Voltage;
            result = telemetryData.selfTestResult;
            resultValid = telemetryData.selfTestResultValid;
            xSemaphoreGive(dataMutex);
        }

        switch (state) {
            case IDLE:
                if (requested && !charging) {
                    state = OPENING_RELAYS;
                    stateTimer = now;
                    WITH_DATA_MUTEX({ 
                        telemetryData.selfTestRequested = false;
                        telemetryData.selfTestRunning = true; 
                        telemetryData.selfTestFailed = false;
                    });
                    set_imd_hv_relays(true); // Open relays for safety check
                    safe_printf("[IMD] Starting Active Self-Test...\n");
                }
                break;

            case OPENING_RELAYS:
                if (now - stateTimer >= 500) {
                    state = CHECKING_VOLTAGE;
                    stateTimer = now;
                }
                break;

            case CHECKING_VOLTAGE:
                // Check if HV1 (Load side) is actually low (relays worked)
                if (hv1 < 15) { // < 15V is considered safe/open
                    state = RUNNING_TEST;
                    stateTimer = now;
                    send_imd_self_test_start(false); // Quick test
                } else if (now - stateTimer > 2000) {
                    safe_printf("[IMD] Error: HV1 voltage still detected! Relays stuck?\n");
                    state = CLEANUP_FAULT;
                }
                break;

            case RUNNING_TEST:
                if (resultValid) {
                    state = EVALUATING;
                } else if (now - stateTimer > 8000) { // IMD Timeout
                    safe_printf("[IMD] Error: Self-Test Timeout.\n");
                    state = CLEANUP_FAULT;
                }
                break;

            case EVALUATING:
                if (result == 0) { // 0 = OK
                    safe_printf("[IMD] Self-Test PASSED.\n");
                    state = CLEANUP_SUCCESS;
                } else {
                    safe_printf("[IMD] Self-Test FAILED! Code: %d\n", result);
                    state = CLEANUP_FAULT;
                }
                break;

            case CLEANUP_SUCCESS:
                set_imd_hv_relays(false); // Close relays for normal operation
                send_bms_relay_release(true); // Allow High Voltage
                WITH_DATA_MUTEX({ telemetryData.selfTestRunning = false; });
                state = IDLE;
                retryCount = 0;
                break;

            case CLEANUP_FAULT:
                set_imd_hv_relays(false);
                send_bms_relay_release(false); // Inhibit High Voltage!
                WITH_DATA_MUTEX({ 
                    telemetryData.selfTestRunning = false; 
                    telemetryData.selfTestFailed = true;
                });
                state = IDLE;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}