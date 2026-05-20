/**
 * @file self_test.cpp
 * @brief Active IMD internal coupling relay and self-test sequence orchestration.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#define DEBUG
#include "self_test.h"
#include <esp_task_wdt.h>
#include "main.h"

void send_imd_request(uint16_t cmd, uint8_t param) {
    twai_message_t message = {0};
    message.identifier = 0x22;
    message.data_length_code = 5;
    message.data[0] = (uint8_t)(cmd >> 8);   
    message.data[1] = (uint8_t)(cmd & 0xFF); 
    message.data[2] = param;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    
    twai_transmit(&message, pdMS_TO_TICKS(50));
}

void send_imd_self_test_start(bool full_test) {
    // Command 0x00D0: Start self-test (0 = short test ~1-2s, 1 = long test ~10s)
    send_imd_request(0x00D0, full_test ? 1 : 0);
}

void set_imd_internal_coupling(bool connect) {
    // Command 0x00D2: Internal HV coupling path relays (0 = NEG, 1 = POS)
    // Parameter: 0x01 = Close (Connect for measuring), 0x00 = Open (Disconnect)
    uint8_t state = connect ? 0x01 : 0x00; 
    
    send_imd_request(0x00D2, 0); // Enforce state on NEG line
    vTaskDelay(pdMS_TO_TICKS(25));
    send_imd_request(0x00D2, 1); // Enforce state on POS line
    
    // Track internal coupling relay state in telemetry (true = connected/closed)
    WITH_DATA_MUTEX({ telemetryData.imdRelayOpen = !connect; });
    safe_printf("[IMD] Internal HV coupling relays commanded to: %s\n", connect ? "CLOSED (CONNECT)" : "OPEN (DISCONNECT)");
}

void send_bms_relay_release(bool enable) {
    // Global interlock release framework for the external BMS contactors
    twai_message_t msg = {0};
    msg.identifier = 0x309;
    msg.data_length_code = 1;
    msg.data[0] = enable ? 0x01 : 0x00;
    twai_transmit(&msg, pdMS_TO_TICKS(50));
}

void self_test_task(void *parameter) {
    esp_task_wdt_add(NULL);
    
    enum SelfTestState {
        IDLE,
        TRIGGERING_TEST,
        RUNNING_TEST,
        EVALUATING,
        CONNECTING_MEASUREMENT,
        CLEANUP_SUCCESS,
        CLEANUP_FAULT
    } state = IDLE;

    unsigned long stateTimer = 0;

    while (1) {
        esp_task_wdt_reset();
        unsigned long now = millis();

        bool requested = false;
        bool charging = false;
        uint16_t imdStatus = 0;
        uint16_t vifcStatus = 0;
        uint8_t result = 0;
        bool resultValid = false;

        // Thread-safe isolation snapshot
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            requested = telemetryData.selfTestRequested;
            charging = telemetryData.isCharging;
            imdStatus = telemetryData.imdStatus;   // D_IMC_STATUS
            vifcStatus = telemetryData.vifcStatus; // D_VIFC_STATUS
            result = telemetryData.selfTestResult;
            resultValid = telemetryData.selfTestResultValid;
            xSemaphoreGive(dataMutex);
        }

        switch (state) {
            case IDLE:
                // Trigger test on request (e.g. at boot when internal relays are still default open)
                if (requested && !charging) {
                    state = TRIGGERING_TEST;
                    stateTimer = now;
                    
                    WITH_DATA_MUTEX({ 
                        telemetryData.selfTestRequested = false;
                        telemetryData.selfTestRunning = true; 
                        telemetryData.selfTestFailed = false;
                        telemetryData.selfTestResultValid = false; 
                    });
                    
                    // Fire the short self-test immediately (requires internal relays to be OPEN)
                    send_imd_self_test_start(false); 
                    safe_printf("[IMD] Triggering internal short self-test sequence...\n");
                }
                break;

            case TRIGGERING_TEST:
                // Verify via D_IMC_STATUS Bit 4 that the device confirms the test run
                if (imdStatus & (1 << 4)) {
                    safe_printf("[IMD] Hardware confirms internal test is actively executing.\n");
                    state = RUNNING_TEST;
                    stateTimer = now;
                } else if (now - stateTimer > 2000) {
                    safe_printf("[IMD] WARNING: Trigger missed or relays were not open. Retrying test command...\n");
                    send_imd_self_test_start(false);
                    stateTimer = now;
                }
                break;

            case RUNNING_TEST: {
                // Verify test completion via D_VIFC_STATUS Bit 12 (0 = Executed/Done)
                bool testFinished = !(vifcStatus & (1 << 12)); 
                
                if (resultValid || testFinished) {
                    state = EVALUATING;
                } else if (now - stateTimer > 5000) { // Short test shouldn't take more than 2-3s
                    safe_printf("[IMD] ERROR: Internal self-test timed out.\n");
                    state = CLEANUP_FAULT;
                }
                break;
            }

            case EVALUATING: {
                // Evaluate internal diagnostic register bits (Image 7.3.5: Bits 0, 1, 2)
                bool internalIsoError = (imdStatus & (1 << 0));
                bool internalChassisError = (imdStatus & (1 << 1));
                bool internalSystemError = (imdStatus & (1 << 2));

                if (result == 0 && !internalIsoError && !internalChassisError && !internalSystemError) {
                    safe_printf("[IMD] Internal electronics check PASSED. Proceeding to close coupling relays.\n");
                    state = CONNECTING_MEASUREMENT;
                    stateTimer = now;
                    
                    // CRITICAL STEP: Close the internal Bender relays to connect to the actual car HV bus
                    set_imd_internal_coupling(true); 
                } else {
                    safe_printf("[IMD] CRITICAL: Internal electronics check FAILED! Status Register: 0x%X\n", imdStatus);
                    state = CLEANUP_FAULT;
                }
                break;
            }    

            case CONNECTING_MEASUREMENT:
                // Give the physical hardware internal relays 200ms to close cleanly
                if (now - stateTimer >= 200) {
                    state = CLEANUP_SUCCESS;
                }
                break;

            case CLEANUP_SUCCESS:
                // Isolation loop is running now. Safe to signal external BMS to release contactors.
                send_bms_relay_release(true); 
                
                WITH_DATA_MUTEX({ telemetryData.selfTestRunning = false; });
                state = IDLE;
                break;

            case CLEANUP_FAULT:
                // Lock down external contactors. Car stays safe and de-energized.
                set_imd_internal_coupling(false); // Disconnect internal tracking for safety
                send_bms_relay_release(false); 
                
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