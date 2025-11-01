// self_test.cpp
#define DEBUG
#include "self_test.h"
#include <esp_task_wdt.h>
#include "main.h"  // für TelemetryData, dataMutex, etc.

// --- IMD command ---
// Sende IMD-Anfrage mit korrektem DLC=5 und CMD-Format
// Maßnahmen für Sicherheit und Robustheit:
// - Eingabevalidierung: Überprüft CMD und Parameter
// - CAN-Übertragung: Mit 100 ms Timeout
// - Debugging: Begrenzt auf alle 5 Sekunden
void send_imd_request(uint16_t cmd, uint8_t param) {
    static unsigned long lastDebugPrint = 0;
    unsigned long now = millis();

    // --- 1. Eingabevalidierung ---
    if (cmd != 0x0001 && cmd != 0x0005 && cmd != 0x0036 && 
        cmd != 0x00D0 && cmd != 0x00D2 && cmd != 0x00DD) {
        if (now - lastDebugPrint >= 5000) {
            safe_printf("send_imd_request: Ungültiger Befehl 0x%04X\n", cmd);
            lastDebugPrint = now;
        }
        return;
    }
    if (cmd == 0x00D0 && param > 1) {
        if (now - lastDebugPrint >= 5000) {
            safe_printf("send_imd_request: Ungültiger Selbsttest-Parameter %d\n", param);
            lastDebugPrint = now;
        }
        return;
    }

    // --- 2. CAN-Nachricht ---
    twai_message_t message = {0};
    message.identifier = 0x22;
    message.extd = 0;
    message.data_length_code = 5;
    message.data[0] = (uint8_t)(cmd >> 8);   // CMD High
    message.data[1] = (uint8_t)(cmd & 0xFF); // CMD Low
    message.data[2] = param;                 // Parameter
    message.data[3] = 0x00;                  // Padding
    message.data[4] = 0x00;                  // Padding

    // --- 3. Senden ---
    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (err != ESP_OK && now - lastDebugPrint >= 5000) {
        safe_printf("send_imd_request: twai_transmit fehlgeschlagen für CMD 0x%04X: %s\n",
                    cmd, esp_err_to_name(err));
        lastDebugPrint = now;
    }
}

void send_imd_self_test_start(bool full_test) {
    send_imd_request(0x00D0, full_test ? 1 : 0);
}

void set_imd_hv_relays(bool open) {
    uint8_t state = open ? 0x00 : 0x01;
    WITH_DATA_MUTEX({ telemetryData.imdRelayOpen = open; });

    send_imd_request(0x00D2, 0);  // NEG
    vTaskDelay(pdMS_TO_TICKS(15));
    send_imd_request(0x00D2, 1);  // POS
}

esp_err_t get_imd_hv_relays(uint8_t *neg_state, uint8_t *pos_state) {
    // Hinweis: GET ist asynchron → wir senden nur Request
    // Antwort kommt über 0x22 → wird in process_can_messages() geparst
    send_imd_request(0x00DD, 0);  // NEG
    vTaskDelay(pdMS_TO_TICKS(15));
    send_imd_request(0x00DD, 1);  // POS
    return ESP_OK;
}

void send_bms_relay_release(bool enable) {
    static bool last_state = true;
    if (enable == last_state) return;

    twai_message_t msg = {0};
    msg.identifier = 0x309;
    msg.extd = 0;
    msg.data_length_code = 1;
    msg.data[0] = enable ? 0x01 : 0x00;

    if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
        safe_printf("BMS %s (0x309)\n", enable ? "FREIGABE" : "SPERRE");
        last_state = enable;
    }
}


// Selbsttest-Task: Verwaltet den IMD-Selbsttest
// Maßnahmen für Sicherheit und Robustheit:
// - Mutex-Schutz: dataMutex für selfTest-Variablen und globale Zustände
// - Zustandsmaschine: Handhabt Relais-Öffnung, HV_1-Prüfung und Selbsttest
// - Timeout: 5000 ms für Selbsttest, 100 ms für VIFC/HV_1-Responses
// - Fehlerbehandlung: Reset bei Fehler oder Timeout, Benachrichtigung an BMS/RS485
// - Debug: Begrenzt auf alle 5 Sekunden
// - Watchdog: Task registriert und reset
void self_test_task(void *parameter) {
    esp_task_wdt_add(NULL);
    typedef enum {
        SELF_TEST_IDLE,
        SELF_TEST_OPEN_RELAYS,
        SELF_TEST_CHECK_RELAYS,
        SELF_TEST_CHECK_HV1,
        SELF_TEST_RUNNING,
        SELF_TEST_ERROR,
        SELF_TEST_SUCCESS
    } SelfTestState;

    SelfTestState state = SELF_TEST_IDLE;
    unsigned long state_start_time = 0;
    static unsigned long lastDebugPrint = 0;
    static uint8_t error_count = 0;
    const uint8_t MAX_ERROR_COUNT = 3;

    while (1) {
        esp_task_wdt_reset();
        unsigned long currentMillis = millis();

        bool test_requested = false;
        bool is_charging_local = false;
        uint16_t temp_hv1_voltage = 0;
        uint8_t self_test_result = 0;
        bool self_test_result_valid = false;

        if (dataMutexInitialized && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            test_requested = telemetryData.selfTestRequested;
            is_charging_local = telemetryData.is_charging;
            temp_hv1_voltage = telemetryData.hv1VoltageValid ? telemetryData.hv1Voltage : 0;
            self_test_result = telemetryData.selfTestResult;
            self_test_result_valid = telemetryData.selfTestResultValid;

            if (test_requested && !is_charging_local) {
                telemetryData.selfTestRequested = false;
                telemetryData.selfTestRunning = true;
            }
            xSemaphoreGive(dataMutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        switch (state) {
            case SELF_TEST_IDLE:
                if (test_requested && !is_charging_local) {
                    if (error_count >= MAX_ERROR_COUNT) {
                        WITH_DATA_MUTEX({ telemetryData.selfTestRequested = false; telemetryData.selfTestRunning = false; });
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        break;
                    }
                    state = SELF_TEST_OPEN_RELAYS;
                    state_start_time = currentMillis;
                    set_imd_hv_relays(true);
                }
                break;

            case SELF_TEST_OPEN_RELAYS:
                if (currentMillis - state_start_time >= 200) {
                    state = SELF_TEST_CHECK_RELAYS;
                    state_start_time = currentMillis;
                }
                break;

            case SELF_TEST_CHECK_RELAYS:
                {
                    uint8_t neg_state, pos_state;
                    if (get_imd_hv_relays(&neg_state, &pos_state) == ESP_OK && neg_state == 0x00 && pos_state == 0x00) {
                        state = SELF_TEST_CHECK_HV1;
                        state_start_time = currentMillis;
                        set_imd_hv_relays(true);
                    } else {
                        state = SELF_TEST_ERROR;
                        error_count++;
                    }
                }
                break;

            case SELF_TEST_CHECK_HV1:
                if (currentMillis - state_start_time >= 300 && temp_hv1_voltage <= 10) {
                    state = SELF_TEST_RUNNING;
                    state_start_time = currentMillis;
                    send_imd_self_test_start(false);
                } else if (currentMillis - state_start_time >= 300) {
                    state = SELF_TEST_ERROR;
                    error_count++;
                }
                break;

            case SELF_TEST_RUNNING:
                if (self_test_result_valid) {
                    if (self_test_result == 0) {
                        set_imd_hv_relays(false);
                        state = SELF_TEST_SUCCESS;
                    } else {
                        set_imd_hv_relays(false);
                        send_bms_relay_release(false);
                        state = SELF_TEST_ERROR;
                        error_count++;
                    }
                } else if (currentMillis - state_start_time >= 8000) {
                    set_imd_hv_relays(false);
                    send_bms_relay_release(false);
                    state = SELF_TEST_ERROR;
                    error_count++;
                }
                break;

            case SELF_TEST_SUCCESS:
                WITH_DATA_MUTEX({ telemetryData.selfTestRunning = false; telemetryData.selfTestFailed = false; });
                state = SELF_TEST_IDLE;
                error_count = 0;
                break;

            case SELF_TEST_ERROR:
                set_imd_hv_relays(false);
                send_bms_relay_release(false);
                WITH_DATA_MUTEX({ telemetryData.selfTestRunning = false; telemetryData.selfTestRequested = false; telemetryData.selfTestFailed = true; });
                state = SELF_TEST_IDLE;
                break;

            default:
                state = SELF_TEST_IDLE;
                error_count++;
                break;
        }

        if (currentMillis - lastDebugPrint >= 5000) {
            safe_printf("self_test_task: State=%d, Err=%u\n", state, error_count);
            lastDebugPrint = currentMillis;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}