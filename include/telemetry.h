/**
 * @file telemetry.h
 * @brief Header for the VCU telemetry link (Supporting RS485 and CANA).
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <cstdint>

// --- Protocol Constants ---
#define TELEMETRY_PAYLOAD_LENGTH 15

// RS485 Specific (Legacy TCAN485)
#define RS485_START_BYTE 0xAA
#define RS485_END_BYTE   0x55
#define RS485_PACKET_TOTAL_LENGTH 18

// CAN Specific (New T-2CAN)
#define CAN_ID_TELEMETRY_FAST 0x100
#define CAN_ID_TELEMETRY_SLOW 0x200

// --- Logic Helpers (State Calculation) ---

/**
 * @brief Translates raw MCU flags into a priority-based state ID.
 */
uint8_t calculateMcuStateID(uint16_t flags);

/**
 * @brief Translates IMD status into a dashboard-ready state ID.
 */
uint8_t calculateImdStateID(uint16_t status);

/**
 * @brief Translates VIFC status into a dashboard-ready state ID.
 */
uint8_t calculateVifcStateID(uint16_t vifc);

/**
 * @brief Maps MCU gear flags to 'R', 'N', or 'D'.
 */
char calculateRndChar(uint16_t flags);

// --- Core Functions ---

/**
 * @brief Packs telemetry data into the standardized optimized payload.
 */
void packOptimizedTelemetry(uint8_t* buf);

/**
 * @brief Sends telemetry via the appropriate hardware interface.
 * Hardware selection is handled via build flags in the implementation.
 */
#ifdef HARDWARE_TCAN485
    void send_rs485_telemetry();
#endif

#ifdef HARDWARE_T2CAN
    void send_can_telemetry();
#endif

#endif // TELEMETRY_H