//proxy_bms.cpp
#define DEBUG
#include "proxy_bms.h"
#include "main.h"

// send_proxy_bms_data
// Sends Proxy BMS data over CAN (ID 0x244)
// Measures for safety and robustness:
// - Mutex protection: dataMutex with 200 ms timeout.
// - Error bit: selfTestResult in status bit 7.
// - Error handling: Logs twai_transmit failures.
// - Debug: Limited to every 5 seconds.
void send_proxy_bms_data() {
    // --- 1. Prüfe Mutex ---
    if (!dataMutexInitialized) {
        safe_printf("send_proxy_bms_data: dataMutex not initialized\n");
        return;
    }

    // --- 2. Kritischen Abschnitt mit Timeout ---
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        safe_printf("send_proxy_bms_data: Timeout waiting for dataMutex\n");
        return;
    }

    // --- 3. Lokale Kopien ---
    uint8_t status = 0;
    uint8_t limit_percent = 100;

    // --- 3.1 SoC (i1q15: 0.1%) ---
    int16_t soc_raw = telemetryData.bmsSoCValid ? (int16_t)(telemetryData.bmsSoC * 10) : 0;
    // --- 3.2 Current (dA: 0.1 A) ---
    int16_t current_raw = telemetryData.bmsCurrentValid ? (int16_t)(telemetryData.bmsCurrent * 10) : 0;

    // --- 3.3 Warnings aus 0x35A (müssen in telemetryData gespeichert sein!) ---
    bool low_volt_warn  = telemetryData.bmsLowVoltageWarn;
    bool high_temp_warn = telemetryData.bmsHighTempWarn;
    bool low_temp_warn  = telemetryData.bmsLowTempWarn;
    bool high_curr_warn = telemetryData.bmsHighCurrentWarn;

    bool any_warning = low_volt_warn || high_temp_warn || low_temp_warn || high_curr_warn;

    // --- 3.4 Status Byte bauen ---
    if (any_warning) {
        status |= (1 << 1);  // Limit aktiv

        // Dynamisches Limit
        if (high_curr_warn) {
            limit_percent = 30;
        } else if (high_temp_warn || low_volt_warn) {
            limit_percent = 50;
        } else if (low_temp_warn) {
            limit_percent = 70;
        } else {
            limit_percent = 100;
        }
    } else {
        status |= (1 << 2);  // Normal
    }

    // --- 3.5 Self-Test Fehler → Bit 7 ---
    if (selfTestResult != 0) {
        status |= (1 << 7);
    }

    // --- 4. CAN-Nachricht ---
    twai_message_t message = {0};
    message.identifier = 0x244;
    message.extd = 0;
    message.data_length_code = 8;

    message.data[0] = (soc_raw >> 8) & 0xFF;
    message.data[1] = soc_raw & 0xFF;
    message.data[2] = (current_raw >> 8) & 0xFF;
    message.data[3] = current_raw & 0xFF;
    message.data[4] = status;
    message.data[5] = limit_percent;  // Nur Byte 5!
    message.data[6] = 0;
    message.data[7] = 0;

    // --- 5. Mutex freigeben ---
    xSemaphoreGive(dataMutex);

    // --- 6. Senden ---
    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        safe_printf("send_proxy_bms_data: twai_transmit failed: %s\n", esp_err_to_name(err));
        return;
    }

    // --- 7. Debug alle 5 Sekunden ---
    static unsigned long lastLog = 0;
    unsigned long now = millis();
    if (now - lastLog >= 5000) {
        safe_printf("Proxy BMS sent: SoC=%.1f%%, I=%.1fA, Status=0x%02X, Limit=%d%%\n",
                    soc_raw / 10.0f, current_raw / 10.0f, status, limit_percent);
        lastLog = now;
    }
}
