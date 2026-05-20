/**
 * @file logger.h
 * @brief Circular RAM-buffered system log engine for high-voltage vehicle monitoring.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

/**
 * @brief Pushes a formatted system or safety alert event into the volatile SRAM ring buffer.
 * Automatically mirrors the generated line to the serial debugging interface.
 * * @param code A short alphanumeric string categorizing the event (e.g., "IMD_ERR", "CHG_ST").
 * @param message A descriptive English detail string regarding the event signature.
 */
void log_system_event(const char* code, const char* message);

/**
 * @brief Commits all buffered runtime records from volatile RAM directly into the LittleFS log file.
 * Run this on critical vehicle system milestones (e.g., Charging Complete, Ignition Power-Down).
 * * @return true if write sequence completed successfully, false on filesystem write faults.
 */
bool commit_logs_to_flash();

#endif // LOGGER_H