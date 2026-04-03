/**
 * @file CAN_Transmit.h
 * @author Your Name (X1/9e Project)
 * @brief Header for the Centralized CAN Transmission Manager.
 * * Defines global constants for battery capacity and estimated consumption,
 * and provides the interface for the VCU transmission tasks.
 * * Configuration for Tesla Model S Modules (5x 5.3kWh = 26.5kWh total).
 */

#ifndef CAN_TRANSMIT_H
#define CAN_TRANSMIT_H

#include <Arduino.h>

/* --- Vehicle Configuration --- */
// Total usable energy in kWh (5x Tesla Modules @ 5.2kWh usable each)
static const float BATT_CAPACITY_KWH = 26.0f; 

// Estimated average consumption for the Bertone X1/9 (kWh per 100km)
static const float AVG_CONSUMPTION_KWH_100KM = 14.5f;

/* --- External State Variables --- */
// Managed in CAN_Transmit.cpp, accessed by WebServer/Main
extern bool dailyModeActive;    // If true, charging stops at 80% SoC
extern uint16_t estimatedRange; // Current range prediction in kilometers

/* --- Public Function Prototypes --- */

/**
 * @brief Main execution task for all outgoing CAN messages.
 * Should be called in the main loop. Handles timing for:
 * - Hyper9 Proxy (0x244)
 * - Elcon Charger Control (0x1806E5F4)
 * - Relay Module Broadcast (0x101)
 */
void CAN_Transmit_Task();

/**
 * @brief Calculates the estimated driving range based on current SoC.
 * Internal calculation exposed for potential use in other modules.
 */
void updateRangeEstimation();

/**
 * @brief Sends a specific command frame to the Elcon/TC Charger.
 * @param stopCharging If true, sends the STOP command (0x01).
 */
void sendElconCommand(bool stopCharging);

/**
 * @brief Transmits the Hyper9 Proxy BMS frame (0x244).
 * Translates telemetryData into NetGain/Hyper9 readable format.
 */
void send_proxy_bms_data();

#endif // CAN_TRANSMIT_H