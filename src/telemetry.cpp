/**
 * @file telemetry.cpp (formerly rs485.cpp)
 * @brief Telemetry link between VCU and Dashboard (Supports RS485 and CANA).
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include "telemetry.h" 
#include "main.h"

#ifdef HARDWARE_T2CAN
#include <SPI.h>
#include <mcp2515.h>
#endif

// --- Helper Logic (State Calculations remain identical) ---

uint8_t calculateMcuStateID(uint16_t flags, bool valid) {
    if (!valid) return 255;
    if (flags & 0x08) return 4; // WARN
    if (flags & 0x04) return 3; // LIMIT
    if (flags & 0x02) return 2; // STOP
    if (flags & 0x01) return 1; // BLOCK
    return 0;                   // OK
}

uint8_t calculateImdStateID(uint16_t status, bool valid) {
    if (!valid) return 255;
    bool iso_error = (status & 0x03); 
    if (iso_error) return 5;
    if (status & (1 << 2)) return 2; // ERR
    if (status & (1 << 5)) return 1; // WARN
    if (status & (1 << 4)) return 3; // TEST
    if (status & (1 << 3)) return 4; // CALIB
    return 0;
}

uint8_t calculateVifcStateID(uint16_t vifc, bool valid) {
    if (!valid) return 255;
    if (vifc & 0x0016) return 1; // COM ERR
    if (vifc & 0x0100) return 2; // STALE
    if (vifc & 0x3000) return 3; // TST ERR
    if (!(vifc & 0x0001)) return 4; // ISO OFF
    return 0; 
}

char calculateRndChar(uint16_t flags) {
    uint8_t rnd_value = (flags >> 2) & 0x03;
    switch (rnd_value) {
        case 0:  return 'N';
        case 1:  return 'R';
        case 2:  return 'D';
        default: return '-'; 
    }
}

/**
 * @brief Packs telemetry data into a 15-byte buffer for cross-platform use.
 */
bool packOptimizedTelemetry(uint8_t* buf) {
    if (!dataMutexInitialized) return false;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        uint16_t rpm = telemetryData.motorRPMValid ? telemetryData.motorRPM : 0;
        buf[0] = (rpm >> 8) & 0xFF;
        buf[1] = rpm & 0xFF;

        // Gear selection char
        buf[2] = telemetryData.motorRPMValid ? (uint8_t)calculateRndChar(telemetryData.mcuFlags) : (uint8_t)'-';

        // Temperatures
        buf[3] = telemetryData.motorTempValid ? (int8_t)telemetryData.motor_temp : -99;
        buf[4] = telemetryData.mcuTempValid   ? (int8_t)telemetryData.mcu_temp   : -99;

        // State IDs for Dashboard String Lists
        buf[5] = calculateMcuStateID(telemetryData.mcuFlags, telemetryData.motorRPMValid);
        buf[6] = calculateImdStateID(telemetryData.imdStatus, telemetryData.imdStatusValid);
        buf[7] = calculateVifcStateID(telemetryData.vifcStatus, telemetryData.vifcStatusValid);

        // Insulation resistance
        uint16_t iso = telemetryData.imdIsoRValid ? telemetryData.imdIsoR : 0;
        buf[8] = (iso >> 8) & 0xFF;
        buf[9] = iso & 0xFF;

        // Fault & Validity Mask
        buf[10] = telemetryData.mcuFaultLevel;

        uint8_t vMask = 0;
        if (telemetryData.imdIsoRValid)   vMask |= (1 << 0);
        if (telemetryData.motorRPMValid)  vMask |= (1 << 1);
        buf[11] = vMask;

        // Padding (reserved for future use)
        buf[12] = 0; buf[13] = 0; buf[14] = 0;

        xSemaphoreGive(dataMutex);
        return true; // Data packed successfully
    }
    return false; // Data not ready or mutex issue
}

/**
 * @brief T-CAN485 only: Sends telemetry via RS485.
 */
#ifdef HARDWARE_TCAN485
void send_rs485_telemetry() {
    static uint8_t payload[15]; 
    static uint8_t packet[18]; 

    packOptimizedTelemetry(payload);

    packet[0] = 0xAA; // START_BYTE
    memcpy(&packet[1], payload, 15);
    packet[16] = calculateChecksum(payload, 15); // Ensure this helper exists
    packet[17] = 0x55; // END_BYTE

    if (serialMutexInitialized && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        RS485Serial.write(packet, 18);
        RS485Serial.flush();
        xSemaphoreGive(serialMutex);
    }
}
#endif


/**
 * @brief T-2CAN only: Sends telemetry via 2nd CAN Interface (CANA / MCP2515).
 * Uses a sanitized buffer to prevent stack-garbage (dynamic noise) from appearing on the bus.
 */
#ifdef HARDWARE_T2CAN

void send_can_telemetry() {
    // 1. Lokaler Puffer (wie bisher)
    uint8_t buf[15] = {0};

    // 2. Daten packen (Mutex-Check intern in packOptimizedTelemetry)
    if (!packOptimizedTelemetry(buf)) {
        return; 
    }

    // 3. Vorbereiten der autowp-Strukturen
    struct can_frame frame1;
    struct can_frame frame2;

    // Frame 1 Setup (ID 0x100)
    frame1.can_id  = 0x100;
    frame1.can_dlc = 8;
    frame1.data[0] = buf[0];  // RPM High
    frame1.data[1] = buf[1];  // RPM Low
    frame1.data[2] = buf[2];  // Gear
    frame1.data[3] = buf[3];  // Motor Temp
    frame1.data[4] = buf[4];  // MCU Temp
    frame1.data[5] = buf[11]; // Validity Mask
    frame1.data[6] = 0;
    frame1.data[7] = 0;

    // Frame 2 Setup (ID 0x200)
    frame2.can_id  = 0x200;
    frame2.can_dlc = 8;
    frame2.data[0] = buf[5];  // McuStateID
    frame2.data[1] = buf[6];  // ImdStateID
    frame2.data[2] = buf[7];  // VifcStateID
    frame2.data[3] = buf[8];  // ISO High
    frame2.data[4] = buf[9];  // ISO Low
    frame2.data[5] = buf[10]; // McuFaultLevel
    frame2.data[6] = 0;
    frame2.data[7] = 0;

    // 5. Versand via SPI
    if (mcpMutex != nullptr && xSemaphoreTake(mcpMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mcp2515.sendMessage(&frame1);
        delayMicroseconds(500); // 1ms ist oft zu lang, 500us reicht dem Controller
        mcp2515.sendMessage(&frame2);
        xSemaphoreGive(mcpMutex);
    }
}
#endif


/**
 * @brief Liest eingehende CAN-Nachrichten auf dem ESP32, 
 * damit ACKs gesendet werden und der Puffer nicht blockiert.
void receive_can_telemetry() {
    struct can_frame rcvFrame;

    if (mcpMutex != nullptr && xSemaphoreTake(mcpMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // Solange Nachrichten im Puffer sind, auslesen
        while (mcp2515.readMessage(&rcvFrame) == MCP2515::ERROR_OK) {
            // Hier prüfen wir auf die ID vom RP2040
            if (rcvFrame.can_id == 0x300) {
                // Optional: Verarbeite Odometer Daten
                // uint32_t total_km = (rcvFrame.data[0] << 24) | ...
            }
        }
        xSemaphoreGive(mcpMutex);
    }

}
*/