/**
 * @file alarm_manager.h
 * @brief Central intelligence for powertrain alerts and high-voltage safety monitoring.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H

#include <Arduino.h>

// Shared global latch variable controlling acoustic alert suppression
extern volatile bool alarmPiezoMuted;

/**
 * @brief Evaluates all system telemetry to route and execute alarms.
 * Synchronizes isolation error registers, BMS statuses, and physical outputs.
 * Call this systematically inside the main processing loop or periodic safety cycle.
 */
void update_alarm_states();

#endif // ALARM_MANAGER_H