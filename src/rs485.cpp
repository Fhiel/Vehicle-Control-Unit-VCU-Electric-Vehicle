// rs485.cpp
#define DEBUG
#include "rs485.h"
#include "main.h"  // für TelemetryData, dataMutex, etc.

// packTelemetryData
// Packs telemetry data into a 14+3 byte binary packet for RS485
// Measures for safety and robustness:
// - Mutex protection: dataMutex with 100 ms timeout, safe Give before return.
// - Plausibility checks: Limits on RPM, temperatures, and IsoR.
// - Error bit: selfTestResult included in payload bit 7.
// - Debug: Limited to every 5 seconds.
void packTelemetryData(uint8_t* payload_buffer) {
    // --- 1. Sicherheits-Check: Buffer gültig? ---
    if (payload_buffer == NULL) {
        safe_printf("packTelemetryData: NULL pointer for payload_buffer!\n");
        return;
    }

    // --- 2. Initialisiere mit sicheren Defaults ---
    memset(payload_buffer, 0, PACKET_PAYLOAD_LENGTH);

    // --- 3. Prüfe, ob dataMutex initialisiert ist ---
    if (!dataMutexInitialized) {
        safe_printf("packTelemetryData: dataMutex not initialized, using defaults\n");
        return;
    }

    // --- 4. Kritischen Abschnitt mit Timeout schützen ---
    bool mutex_taken = (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE);
    if (!mutex_taken) {
        safe_printf("packTelemetryData: Timeout waiting for dataMutex\n");
        return; // Buffer bleibt auf 0 (sichere Defaults)
    }

    // --- 5. Lokale Kopien der Telemetriedaten (nur gültige Werte) ---
    uint16_t temp_motorRPM        = telemetryData.motorRPMValid        ? telemetryData.motorRPM        : 0;
    int8_t   temp_motor_temp      = telemetryData.motorTempValid       ? telemetryData.motor_temp      : 0;
    int8_t   temp_mcu_temp        = telemetryData.mcuTempValid         ? telemetryData.mcu_temp        : 0;
    uint16_t temp_mcuFlags        = telemetryData.mcuFlagsValid        ? telemetryData.mcuFlags        : 0;
    uint8_t  temp_mcuFaultLevel   = telemetryData.mcuFaultLevelValid   ? telemetryData.mcuFaultLevel   : 0;
    uint16_t temp_imdIsoR         = telemetryData.imdIsoRValid         ? telemetryData.imdIsoR         : 50000;
    uint16_t temp_imdStatus       = telemetryData.imdStatusValid       ? telemetryData.imdStatus       : 0;
    uint16_t temp_vifcStatus      = telemetryData.vifcStatusValid      ? telemetryData.vifcStatus      : 0;
    uint16_t temp_selfTestResult = selfTestResult; // Global, kein Mutex nötig

    // --- 6. Plausibilitätsprüfung (vor dem Packen) ---
    if (temp_motorRPM > 10000) temp_motorRPM = 0;
    if (temp_motor_temp > 150 || temp_motor_temp < -50) temp_motor_temp = 0;
    if (temp_mcu_temp > 150 || temp_mcu_temp < -50) temp_mcu_temp = 0;
    if (temp_imdIsoR > 50000) temp_imdIsoR = 50000;

    // --- 7. Mutex freigeben (vor dem Packen – kein Zugriff mehr auf globale Variablen) ---
    xSemaphoreGive(dataMutex);
    mutex_taken = false;

    // --- 8. Binäres Packen in den Payload-Buffer ---
    uint8_t* buf = payload_buffer;

    buf[0]  = (temp_motorRPM >> 8) & 0xFF;
    buf[1]  = temp_motorRPM & 0xFF;
    buf[2]  = (uint8_t)temp_motor_temp;  // int8_t → uint8_t (2er-Komplement)
    buf[3]  = (uint8_t)temp_mcu_temp;
    buf[4]  = (temp_mcuFlags >> 8) & 0xFF;
    buf[5]  = temp_mcuFlags & 0xFF;
    buf[6]  = temp_mcuFaultLevel;
    buf[7]  = (temp_imdIsoR >> 8) & 0xFF;
    buf[8]  = temp_imdIsoR & 0xFF;
    buf[9]  = (temp_imdStatus >> 8) & 0xFF;
    buf[10] = temp_imdStatus & 0xFF;
    buf[11] = (temp_vifcStatus >> 8) & 0xFF;
    buf[12] = temp_vifcStatus & 0xFF;
    buf[13] = (uint8_t)(
        (telemetryData.imdIsoRValid ? (1 << 0) : 0) |
        (telemetryData.motorRPMValid ? (1 << 1) : 0) |
        (selfTestResult != 0 ? (1 << 7) : 0)
    );

    // --- 9. Debug-Ausgabe (alle 5 Sekunden) ---
    static unsigned long lastDebugPrint = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastDebugPrint >= 5000) {
        safe_printf("packTelemetryData: RPM=%u, T_motor=%d°C, T_mcu=%d°C, IsoR=%u kΩ, Valid(imd=%d,mcu=%d)\n",
                    temp_motorRPM, temp_motor_temp, temp_mcu_temp, temp_imdIsoR,
                    telemetryData.imdIsoRValid, telemetryData.motorRPMValid);
        lastDebugPrint = currentMillis;
    }
}

void send_rs485_telemetry() {
    static uint8_t payload[14];
    static uint8_t packet[17];  // 1 Start + 14 Payload + 1 CRC + 1 End

    // --- 1. Payload packen ---
    packTelemetryData(payload);

    // --- 2. Nur senden, wenn Daten gültig ---
    if (!telemetryData.motorRPMValid && !telemetryData.imdIsoRValid) {
        return; // Keine gültigen Daten → nichts senden
    }

    // --- 3. Paket bauen ---
    packet[0] = 0xAA;                                          // START
    memcpy(&packet[1], payload, 14);                           // Payload
    packet[15] = calculateChecksum(payload, 14);              // CRC
    packet[16] = 0x55;                                         // END

    // --- 4. Senden (mit serialMutex) ---
    if (serialMutexInitialized && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        RS485Serial.write(packet, 17);
        RS485Serial.flush();  // Wichtig: wartet, bis alles gesendet
        xSemaphoreGive(serialMutex);
    }

    // --- 5. DEBUG (<lle 10 Sekunden) ---
    static unsigned long lastDebug = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastDebug >= 10000) {
        char hex[50] = {0};
        for (int i = 0; i < 17; i++) snprintf(hex + strlen(hex), sizeof(hex), "%02X ", packet[i]);
        safe_printf("Sent: %s\n", hex);
        lastDebug = currentMillis;
    }
}
