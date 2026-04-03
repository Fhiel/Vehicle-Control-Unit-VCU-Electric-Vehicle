/**
 * @file rs485.h
 * @brief Header for the RS485 communication link (VCU to Dashboard).
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#ifndef RS485_H
#define RS485_H

#include <Arduino.h>
#include <cstdint>

// --- RS485 Protocol Constants ---
#define RS485_START_BYTE 0xAA
#define RS485_END_BYTE   0x55
#define PACKET_PAYLOAD_LENGTH 15
#define PACKET_TOTAL_LENGTH   18

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
 * (Defined in rs485.cpp)
 */
uint8_t calculateVifcStateID(uint16_t vifc);

/**
 * @brief Maps MCU gear flags to 'R', 'N', or 'D'.
 */
char calculateRndChar(uint16_t flags);

// --- Core Functions ---

/**
 * @brief Internal: Packs telemetry data into the 15-byte optimized payload.
 */
void packOptimizedTelemetry(uint8_t* buf);

/**
 * @brief Main function: Sends the 18-byte RS485 packet to the dashboard.
 * Should be called periodically (e.g., every 200ms).
 */
void send_rs485_telemetry();

#endif // RS485_H