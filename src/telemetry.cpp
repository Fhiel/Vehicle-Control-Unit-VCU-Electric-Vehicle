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
#include <mcp_can.h>
extern MCP_CAN CAN_Peripheral; // Defined in main.cpp or peripheral_can.cpp
#endif

// --- Helper Logic (State Calculations remain identical) ---

uint8_t calculateMcuStateID(uint16_t flags) {
    if (flags & 0x08) return 4; // WARN
    if (flags & 0x04) return 3; // LIMIT
    if (flags & 0x02) return 2; // STOP
    if (flags & 0x01) return 1; // BLOCK
    return 0;                   // OK
}

uint8_t calculateImdStateID(uint16_t status) {
    bool iso_error = (status & 0x03); 
    if (iso_error) return 5;
    if (status & (1 << 2)) return 2; // ERR
    if (status & (1 << 5)) return 1; // WARN
    if (status & (1 << 4)) return 3; // TEST
    if (status & (1 << 3)) return 4; // CALIB
    return 0;
}

uint8_t calculateVifcStateID(uint16_t vifc) {
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
        default: return ' '; 
    }
}

/**
 * @brief Packs telemetry data into a 15-byte buffer for cross-platform use.
 */
void packOptimizedTelemetry(uint8_t* buf) {
    if (!dataMutexInitialized) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        // RPM (Big Endian)
        uint16_t rpm = telemetryData.motorRPMValid ? telemetryData.motorRPM : 0;
        buf[0] = (rpm >> 8) & 0xFF;
        buf[1] = rpm & 0xFF;

        // Gear selection char
        buf[2] = (uint8_t)calculateRndChar(telemetryData.mcuFlags);

        // Temperatures
        buf[3] = (int8_t)telemetryData.motor_temp;
        buf[4] = (int8_t)telemetryData.mcu_temp;

        // State IDs for Dashboard String Lists
        buf[5] = calculateMcuStateID(telemetryData.mcuFlags);
        buf[6] = calculateImdStateID(telemetryData.imdStatus);
        buf[7] = calculateVifcStateID(telemetryData.vifcStatus);

        // Insulation resistance
        uint16_t iso = telemetryData.imdIsoRValid ? telemetryData.imdIsoR : 50000;
        buf[8] = (iso >> 8) & 0xFF;
        buf[9] = iso & 0xFF;

        // Fault & Validity Mask
        buf[10] = telemetryData.mcuFaultLevel;
        buf[11] = (telemetryData.imdIsoRValid ? (1 << 0) : 0) | 
                  (telemetryData.motorRPMValid ? (1 << 1) : 0);

        // Padding (reserved for future use)
        buf[12] = 0; buf[13] = 0; buf[14] = 0;

        xSemaphoreGive(dataMutex);
    }
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
 */
#ifdef HARDWARE_T2CAN
void send_can_telemetry() {
    uint8_t buf[15];
    packOptimizedTelemetry(buf); 

    // Frame 1: Dynamic/Fast Data - ID 0x100
    uint8_t frame1[8];
    frame1[0] = buf[0];  // RPM High
    frame1[1] = buf[1];  // RPM Low
    frame1[2] = buf[2];  // Gear
    frame1[3] = buf[3];  // Motor Temp
    frame1[4] = buf[4];  // MCU Temp
    frame1[5] = buf[11]; // Validity Mask
    frame1[6] = 0; 
    frame1[7] = 0; 
    
    // Frame 2: Status & Insulation - ID 0x200
    uint8_t frame2[8];
    frame2[0] = buf[5];  // McuStateID
    frame2[1] = buf[6];  // ImdStateID
    frame2[2] = buf[7];  // VifcStateID
    frame2[3] = buf[8];  // ISO High
    frame2[4] = buf[9];  // ISO Low
    frame2[5] = buf[10]; // McuFaultLevel
    frame2[6] = 0; 
    frame2[7] = 0; 

    // Transmit via MCP2515 on T-2CAN
    // sendMsgBuf arguments: ID, isExtended (0), length (8), buffer
    CAN_Peripheral.sendMsgBuf(0x100, 0, 8, frame1);
    
    // Small inter-frame delay to prevent buffer issues on the RP2040 receiver side
    delay(2); 
    
    CAN_Peripheral.sendMsgBuf(0x200, 0, 8, frame2);
}
#endif