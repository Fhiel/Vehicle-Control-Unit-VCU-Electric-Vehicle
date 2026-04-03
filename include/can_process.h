/**
 * @file can_process.h
 * @brief Header for CAN processing and monitoring tasks.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#ifndef CAN_PROCESS_H
#define CAN_PROCESS_H

#include "main.h"
#include <cstdint>

// --- Constants ---
extern volatile unsigned long last_id_timestamps[NUM_FILTERS];
extern uint32_t received_ids[MAX_IDS];
extern uint8_t received_ids_count;
extern volatile unsigned long last_receive_time;
extern volatile unsigned long last_can_update;
extern QueueHandle_t canQueue;

// --- Core Tasks ---
/**
 * @brief Dedicated task to receive raw TWAI messages and push them to canQueue.
 */
void twai_receive_task(void *parameter);

/**
 * @brief Task to pull messages from canQueue and map them to telemetryData.
 * Also handles the simulation logic for Demo Mode.
 */
void process_can_messages(void *parameter);

/**
 * @brief Background task to monitor TWAI driver health and queue fill levels.
 * Performs automatic bus-off recovery.
 */
void twai_monitor_task(void *parameter);

// --- Maintenance & Logic ---
/**
 * @brief Checks for communication timeouts and marks data as invalid if nodes go silent.
 * @param currentMillis Current system time in ms.
 */
void check_data_timeout(unsigned long currentMillis);

/**
 * @brief Logs unknown CAN IDs to a list for debugging purposes.
 */
void add_received_id(uint32_t id);

/**
 * @brief Updates the live-timestamp for a specific CAN ID.
 */
void update_id_timestamp(uint32_t identifier, unsigned long now);

/**
 * @brief Updates all filtered timestamps (used by Demo Mode).
 */
void update_all_timestamps(unsigned long now);

#endif // CAN_PROCESS_H