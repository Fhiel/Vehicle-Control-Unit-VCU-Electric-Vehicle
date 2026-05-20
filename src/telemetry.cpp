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

/* @brief Calculates the IMD/Isolation state ID matched to Python string index mapping.
 * Aligned with: ["IMD OK", "ISO WARN", "ISO ERR", "IMD TST", "IMD CAL", "RDY-GO!"]
 * Index 0     Index 1     Index 2    Index 3    Index 4    Index 5
 */
uint8_t calculateImdStateID(uint16_t status, bool valid, bool master_alarm, uint8_t bms_status) {
    if (!valid) return 255;
    
    // PRIORITY 1: Master safety alert from central alarm manager (or web panel light test)
    if (master_alarm) {
        return 2; // Maps directly to IMD_STATES[2] -> "ISO ERR" (Unyielding Red Alarm)
    }

    // PRIORITY 2: Contactor readiness check. If BMS is READY (contactors open, no error),
    // the system is safe to start. We signal the erlösende "OK-GO!" state to the display.
    if (bms_status == BMS_STATUS_READY && status == 0) {
        return 5; // Maps directly to IMD_STATES[5] -> "RDY-GO!" -> Displays "OK - GO !"
    }

    // PRIORITY 3: Hardware insulation bitmask evaluation (Normal active driving corridor)
    if (status & (1 << 0)) return 2; // Bit 0: Isolation Error -> "ISO ERR"
    if (status & (1 << 1)) return 1; // Bit 1: Chassis Fault    -> "ISO WARN"
    if (status & (1 << 2)) return 2; // Bit 2: System Fault     -> "ISO ERR"
    if (status & (1 << 5)) return 1; // Bit 5: Dispersed Warning  -> "ISO WARN"
    if (status & (1 << 4)) return 3; // Bit 4: Insulation Test     -> "IMD TST"
    if (status & (1 << 3)) return 4; // Bit 3: Calibration Mode   -> "IMD CAL"
    
    return 0; // "IMD OK"
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
        buf[3] = telemetryData.motorTempValid ? (int8_t)telemetryData.motorTemp : -99;
        buf[4] = telemetryData.mcuTempValid   ? (int8_t)telemetryData.mcuTemp   : -99;

        // State IDs for Dashboard String Lists (Injected with verified context dependencies)
        buf[5] = calculateMcuStateID(telemetryData.mcuFlags, telemetryData.motorRPMValid);
        buf[6] = calculateImdStateID(telemetryData.imdStatus, telemetryData.imdStatusValid, telemetryData.isAlarm, telemetryData.bmsStatus);
        buf[7] = calculateVifcStateID(telemetryData.vifcStatus, telemetryData.vifcStatusValid);

        // Insulation resistance
        uint16_t iso = telemetryData.imdIsoRValid ? telemetryData.imdIsoR : 0;
        buf[8] = (iso >> 8) & 0xFF;
        buf[9] = iso & 0xFF;

        // Fault Level
        buf[10] = telemetryData.mcuFaultLevel;

        uint8_t vMask = 0;
        if (telemetryData.imdIsoRValid)   vMask |= (1 << 0);
        if (telemetryData.motorRPMValid)  vMask |= (1 << 1);
        buf[11] = vMask;

        // Padding
        buf[12] = 0; buf[13] = 0; buf[14] = 0;

        xSemaphoreGive(dataMutex);
        return true; 
    }
    return false; 
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


#ifdef HARDWARE_T2CAN
/**
 * @brief T-2CAN only: Sends telemetry via 2nd CAN Interface (CANA / MCP2515).
 * Streams RAW native hardware registers directly to the RP2040 display unit.
 */
void send_can_telemetry() {
    // Prepare autowp CAN-Structures
    struct can_frame frame1;
    struct can_frame frame2;

    // Locale Variables for safety
    uint16_t local_rpm = 0;
    uint8_t  local_gear = '-';
    int8_t   local_motor_temp = -99;
    int8_t   local_mcu_temp = -99;
    uint8_t  local_vMask = 0;

    uint16_t local_mcu_flags = 0;
    uint16_t local_imd_status = 0;
    uint16_t local_vifc_status = 0;
    uint16_t local_imd_iso_r = 0;
    uint8_t  local_mcu_fault = 0;
    uint8_t  local_bms_status = 0;
    bool     local_is_alarm = false;

    // --- THREAD-SAFE CONTEXT DATA ACQUISITION ---
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        local_rpm = telemetryData.motorRPMValid ? telemetryData.motorRPM : 0;
        local_gear = telemetryData.motorRPMValid ? (uint8_t)calculateRndChar(telemetryData.mcuFlags) : (uint8_t)'-';
        local_motor_temp = telemetryData.motorTempValid ? (int8_t)telemetryData.motorTemp : -99;
        local_mcu_temp = telemetryData.mcuTempValid ? (int8_t)telemetryData.mcuTemp : -99;
        
        if (telemetryData.imdIsoRValid)  local_vMask |= (1 << 0);
        if (telemetryData.motorRPMValid) local_vMask |= (1 << 1);

        // NATIVE REGISTER EXTRAKTION
        local_mcu_flags  = telemetryData.mcuFlags;
        local_imd_status = telemetryData.imdStatus;
        local_vifc_status = telemetryData.vifcStatus;
        local_imd_iso_r  = telemetryData.imdIsoRValid ? telemetryData.imdIsoR : 0;
        local_mcu_fault  = telemetryData.mcuFaultLevel;
        
        // ADDITIONAL DATAS FOR COCKPIT-STATEMACHINE 
        local_bms_status = telemetryData.bmsStatus;
        local_is_alarm   = telemetryData.isAlarm;

        xSemaphoreGive(dataMutex);
    } else return;

    // =========================================================================
    // FRAME 1: KINETIC RUNTIME MATRIX (ID 0x100)
    // =========================================================================
    frame1.can_id  = 0x100;
    frame1.can_dlc = 8;
    frame1.data[0] = (local_rpm >> 8) & 0xFF; // Motor RPM High
    frame1.data[1] = local_rpm & 0xFF;        // Motor RPM Low
    frame1.data[2] = local_gear;              // ASCII Gear Character ('N', 'D', 'R')
    frame1.data[3] = (uint8_t)local_motor_temp;
    frame1.data[4] = (uint8_t)local_mcu_temp;
    frame1.data[5] = local_vMask;             // Validity Mask
    frame1.data[6] = (local_mcu_flags >> 8) & 0xFF; // MCU Flags High (For RND backup verification)
    frame1.data[7] = local_mcu_flags & 0xFF;        // MCU Flags Low

    // =========================================================================
    // FRAME 2: DIAGNOSTIC SAFETY MATRIX (ID 0x200)
    // =========================================================================
    frame2.can_id  = 0x200;
    frame2.can_dlc = 8;
    frame2.data[0] = (local_imd_status >> 8) & 0xFF;  // Raw Bender status bits high
    frame2.data[1] = local_imd_status & 0xFF;         // Raw Bender status bits low
    frame2.data[2] = (local_vifc_status >> 8) & 0xFF; // Raw VIFC link register high
    frame2.data[3] = local_vifc_status & 0xFF;        // Raw VIFC link register low
    frame2.data[4] = (local_imd_iso_r >> 8) & 0xFF;   // Raw insulation tracking value high
    frame2.data[5] = local_imd_iso_r & 0xFF;          // Raw insulation tracking value low
    
    // BACKBONE TRANSITIONS FOR PYTHON PARSING
    frame2.data[6] = local_bms_status;                // Byte 6: Current BMS operational mode
    frame2.data[7] = local_is_alarm ? 1 : 0;          // Byte 7: Unified Master Alarm status

    // Send via SPI link to MCP2515 hardware shield
    if (mcpMutex != nullptr && xSemaphoreTake(mcpMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mcp2515.sendMessage(&frame1);
        delayMicroseconds(500); 
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