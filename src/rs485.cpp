/**
 * @file rs485.cpp
 * @brief High-speed telemetry link between VCU (ESP32) and Dashboard (RP2040).
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include "rs485.h"
#include "main.h"

// Helper: MCU State logic
uint8_t calculateMcuStateID(uint16_t flags) {
    if (flags & 0x08) return 4; // WARN
    if (flags & 0x04) return 3; // LIMIT
    if (flags & 0x02) return 2; // STOP
    if (flags & 0x01) return 1; // BLOCK
    return 0;                   // OK
}

// Helper: IMD State logic
uint8_t calculateImdStateID(uint16_t status) {
    bool iso_error = (status & 0x03); // Bits 0 and 1
    if (iso_error) return 5;
    if (status & (1 << 2)) return 2; // ERR
    if (status & (1 << 5)) return 1; // WARN
    if (status & (1 << 4)) return 3; // TEST
    if (status & (1 << 3)) return 4; // CALIB
    return 0;
}

/**
 * @brief Translates VIFC status bits into a Dashboard State ID.
 * 0: OK, 1: COM ERR, 2: STALE, 3: TST ERR, 4: ISO OFF
 */
uint8_t calculateVifcStateID(uint16_t vifc) {
    // Bits 1, 2, 4 = Communication / Command Errors
    if (vifc & 0x0016) return 1; 
    
    // Bit 8 = Data Stale
    if (vifc & 0x0100) return 2; 
    
    // Bits 12, 13 = Self Test Errors
    if (vifc & 0x3000) return 3; 
    
    // Bit 0 = Measurement Inactive (ISO OFF)
    if (!(vifc & 0x0001)) return 4; 
    
    return 0; // Everything OK
}

// Helper: Gear selection
char calculateRndChar(uint16_t flags) {
    uint8_t rnd_value = (flags >> 2) & 0x03;
    switch (rnd_value) {
        case 0:  return 'N';
        case 1:  return 'R';
        case 2:  return 'D';
        default: return ' '; 
    }
}

void packOptimizedTelemetry(uint8_t* buf) {
    if (!dataMutexInitialized) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        // RPM (Big Endian)
        uint16_t rpm = telemetryData.motorRPMValid ? telemetryData.motorRPM : 0;
        buf[0] = (rpm >> 8) & 0xFF;
        buf[1] = rpm & 0xFF;

        // Gear
        buf[2] = (uint8_t)calculateRndChar(telemetryData.mcuFlags);

        // Temps
        buf[3] = (int8_t)telemetryData.motor_temp;
        buf[4] = (int8_t)telemetryData.mcu_temp;

        // State IDs for Dashboard String Lists
        buf[5] = calculateMcuStateID(telemetryData.mcuFlags);
        buf[6] = calculateImdStateID(telemetryData.imdStatus);
        buf[7] = calculateVifcStateID(telemetryData.vifcStatus);

        // Insulation
        uint16_t iso = telemetryData.imdIsoRValid ? telemetryData.imdIsoR : 50000;
        buf[8] = (iso >> 8) & 0xFF;
        buf[9] = iso & 0xFF;

        // Fault & Validity Mask
        buf[10] = telemetryData.mcuFaultLevel;
        buf[11] = (telemetryData.imdIsoRValid ? (1 << 0) : 0) | 
                  (telemetryData.motorRPMValid ? (1 << 1) : 0);

        // Padding
        buf[12] = 0; buf[13] = 0; buf[14] = 0;

        xSemaphoreGive(dataMutex);
    }
}

void send_rs485_telemetry() {
    static uint8_t payload[15]; 
    static uint8_t packet[18]; 

    packOptimizedTelemetry(payload);

    // Heartbeat: We send the packet even if data is invalid to keep 
    // the dashboard RP2040 from timing out.
    packet[0] = 0xAA; // START_BYTE
    memcpy(&packet[1], payload, 15);
    packet[16] = calculateChecksum(payload, 15);
    packet[17] = 0x55; // END_BYTE

    if (serialMutexInitialized && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        RS485Serial.write(packet, 18);
        RS485Serial.flush();
        xSemaphoreGive(serialMutex);
    }
}