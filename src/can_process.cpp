/**
 * @file can_process.cpp
 * @brief Main CAN processing and TWAI monitor logic for VCU X1/9e.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include "can_process.h"
#include "main.h"
#include <esp_task_wdt.h>
#include <cmath>

// === 1. DEFINITIONS ===
volatile unsigned long last_id_timestamps[NUM_FILTERS] = {0};
volatile unsigned long last_receive_time = 0;
volatile unsigned long last_can_update = 0;

uint32_t received_ids[MAX_IDS] = {0};
uint8_t received_ids_count = 0;

// Define timeouts for data validity (in ms)
QueueHandle_t canQueue = NULL; 

// === 2. Extern Variables  ===
extern SemaphoreHandle_t dataMutex, idMutex;
extern const uint32_t FILTERED_IDS[NUM_FILTERS];
extern const uint32_t FAST_TIMEOUT_MS, SLOW_TIMEOUT_MS;
extern void safe_printf(const char *fmt, ...);

// === 3. Functions ===

/**
 * @brief Task: Dedicated CAN Receiver
 * Priority: 10 (Highest) - Runs on Core 0/1
 */
void twai_receive_task(void *parameter) {
    esp_task_wdt_add(NULL); 
    twai_message_t message;

    while (1) {
        esp_task_wdt_reset(); 

        // Block for 50ms waiting for a message
        if (twai_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
            unsigned long currentMillis = millis();
            last_receive_time = currentMillis; 

            // Filter incoming IDs
            bool accepted = false;
            for (int i = 0; i < NUM_FILTERS; i++) {
                if (message.identifier == FILTERED_IDS[i]) {
                    accepted = true;
                    break;
                }
            }

            if (accepted) {
                // Forward to Queue with 5ms timeout to avoid blocking this high-prio task
                if (xQueueSend(canQueue, &message, pdMS_TO_TICKS(5)) != pdTRUE) {
                    // Only log overflow if absolutely necessary to avoid Serial lag
                }
            } else {
                add_received_id(message.identifier); 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

/**
 * @brief Map raw CAN frames to the global TelemetryData struct.
 */
void map_can_to_telemetry(const twai_message_t& msg) {
    uint8_t dlc = (msg.data_length_code > 8) ? 8 : msg.data_length_code;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    switch (msg.identifier) {
        // =====================================================================
        // SME AC-X1 INVERTER INTERFACE (TPDO 1 & TPDO 2)
        // =====================================================================

        case 0x239: // MCU TPDO 1: Fast Dynamic Powertrain Metrics (100ms)
            if (dlc >= 7) {
                WITH_DATA_MUTEX({
                    // 1. Motor Speed (RPM)
                    telemetryData.motorRPM = (msg.data[1] << 8) | msg.data[0];
                    telemetryData.motorRPMValid = (telemetryData.motorRPM <= MOTOR_RPM_MAX);

                    // 2. Motor Output Torque (Scale: -32768 to 32767 map to -100% to +100%)
                    int16_t raw_torque = (msg.data[3] << 8) | msg.data[2];
                    telemetryData.motorTorque = (float)raw_torque / 327.68f; // Converts directly to % percentage
                    
                    // 3. Thermal Snapshot
                    telemetryData.motorTemp = (int8_t)msg.data[4];
                    telemetryData.motorTempValid = (telemetryData.motorTemp >= TEMP_MIN && telemetryData.motorTemp <= MOTOR_TEMP_MAX);
                    
                    telemetryData.mcuTemp = (int8_t)msg.data[5];
                    telemetryData.mcuTempValid = (telemetryData.mcuTemp >= TEMP_MIN && telemetryData.mcuTemp <= MCU_TEMP_MAX);

                    // 4. Central Fault Level
                    telemetryData.mcuFaultLevel = msg.data[6];
                    telemetryData.mcuFaultLevelValid = true;
                });
            }
            break;

        case 0x240: // MCU TPDO 2: Medium System State & Diagnostics (250ms)
            if (dlc >= 5) {
                WITH_DATA_MUTEX({
                    // 1. System Flag Bitfield
                    telemetryData.systemFlags = (msg.data[1] << 8) | msg.data[0];
                    
                    // 2. Motor Flag Bitfield (New Diagnostic Layer)
                    telemetryData.motorFlags = (msg.data[3] << 8) | msg.data[2];
                    
                    // 3. System Lifetime Hours (Word)
                    telemetryData.systemKeyOntime = (msg.data[5] << 8) | msg.data[4];
                    
                    telemetryData.mcuFlagsValid = true;
                });
            }
            break;

        // =====================================================================
        // TESLA BMS V2 (0x355, 0x356, 0x375, 0x379)
        // =====================================================================
        case 0x379: // TeslaBMSV2 Core Status Info
            if (msg.data_length_code >= 5) {
                WITH_DATA_MUTEX({
                    telemetryData.bmsStatus = msg.data[4];
                    telemetryData.bmsStatusValid = true;
                    telemetryData.isCharging = (telemetryData.bmsStatus == BMS_STATUS_CHARGE); 
                    
                    if (telemetryData.bmsStatus == BMS_STATUS_CHARGE) {
                        telemetryData.bmsChargeAllowed = true;
                    } else {
                        telemetryData.bmsChargeAllowed = false;
                    }
                });
            }
            break;

        case 0x356: // BMS Dynamic Metrics: Current + Temp
            if (msg.data_length_code >= 6) {
                WITH_DATA_MUTEX({
                    // Systemspannung liegt primär auf der Isolationsüberwachung, 
                    // Strom und Temperatur ziehen wir hier ab:
                    int16_t raw_current = (msg.data[3] << 8) | msg.data[2];
                    telemetryData.bmsCurrent = raw_current / 10.0f;
                    telemetryData.bmsCurrentValid = (fabsf(telemetryData.bmsCurrent) <= BMS_CURRENT_MAX);
                    
                    int16_t raw_temp = (msg.data[5] << 8) | msg.data[4];
                    telemetryData.batTemp = raw_temp / 10.0f;
                    telemetryData.batTempValid = (telemetryData.batTemp >= TEMP_MIN && telemetryData.batTemp <= BMS_TEMP_MAX);
                });
            }
            break;

        case 0x355: // BMS: SoC
            if (msg.data_length_code >= 2) {
                WITH_DATA_MUTEX({
                    uint16_t soc_raw = (msg.data[1] << 8) | msg.data[0];
                    telemetryData.bmsSoC = soc_raw / 10.0f;
                    telemetryData.bmsSoCValid = true;
                });
            }
            break;
  

        // =====================================================================
        // BENDER iso165-C1 (0x37, 0x22)
        // =====================================================================    
        case 0x37: // Cyclical IMD_Info (Once per second)
            if (dlc >= 6) {
                // Little-endian extraction of isolation metrics and bitmasks
                telemetryData.imdIsoR = (msg.data[1] << 8) | msg.data[0];
                telemetryData.imdIsoRValid = true;
                
                telemetryData.imdStatus = (msg.data[3] << 8) | msg.data[2]; // D_IMC_STATUS
                telemetryData.imdStatusValid = true;
                
                telemetryData.vifcStatus = (msg.data[5] << 8) | msg.data[4]; // D_VIFC_STATUS
                telemetryData.vifcStatusValid = true;
            }
            break;

        case 0x22: // Active Command-Response Interface
            if (dlc >= 5) {
                uint16_t responseCmd = (msg.data[0] << 8) | msg.data[1];
                
                if (responseCmd == 0x00D0) { // Response to Self-Test Request
                    telemetryData.selfTestResult = msg.data[2]; // Fixed: Byte 2 holds data parameter
                    telemetryData.selfTestResultValid = true;
                } else if (responseCmd == 0x0036) { // Response to High Voltage Bus Request
                    telemetryData.hv1Voltage = (msg.data[2] << 8) | msg.data[3];
                    telemetryData.hv1VoltageValid = true;
                }
            }
            break;

    }
    xSemaphoreGive(dataMutex);
}

/**
 * @brief Task: Process CAN Queue and Handle Demo Mode
 */
void process_can_messages(void *parameter) {
    esp_task_wdt_add(NULL);
    twai_message_t message;
    static float demo_phase = 0;

    while (1) {
        esp_task_wdt_reset();
        unsigned long now = millis();

        // Try to get message from queue
        if (xQueueReceive(canQueue, &message, pdMS_TO_TICKS(10)) == pdTRUE) {
            last_can_update = now; 
            update_id_timestamp(message.identifier, now); 

            if (!demoModeActive) {
                map_can_to_telemetry(message);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

/**
 * @brief Task: Hardware Watchdog & Driver Recovery
 */
void twai_monitor_task(void *parameter) {
    esp_task_wdt_add(NULL);
    static uint8_t bus_off_count = 0;

    while (1) {
        esp_task_wdt_reset();
        
        UBaseType_t fill = uxQueueMessagesWaiting(canQueue);
        if (fill >= 90) {
            safe_printf("[CAN] Queue Critical (%u/100). Resetting.\n", fill);
            xQueueReset(canQueue);
        }

        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            if (status.state == TWAI_STATE_BUS_OFF) {
                bus_off_count++;
                safe_printf("[CAN] Bus-Off detected (%u). Recovering...\n", bus_off_count);
                
                twai_initiate_recovery();
                // Give the hardware time to clear the error state
                vTaskDelay(pdMS_TO_TICKS(500)); 
                twai_start();
                
                if (bus_off_count > 10) { 
                    safe_printf("[CAN] Persistent Failure. Entering Passive Monitor Mode.\n");
                    // Instead of return (which crashes), we enter a long sleep
                    while(1) {
                        esp_task_wdt_reset();
                        vTaskDelay(pdMS_TO_TICKS(5000));
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Helper functions for timestamps
void update_id_timestamp(uint32_t identifier, unsigned long now) {
    if (xSemaphoreTake(idMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < NUM_FILTERS; i++) {
            if (FILTERED_IDS[i] == identifier) {
                last_id_timestamps[i] = now;
                break;
            }
        }
        xSemaphoreGive(idMutex);
    }
}

void update_all_timestamps(unsigned long now) {
    if (xSemaphoreTake(idMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < NUM_FILTERS; i++) last_id_timestamps[i] = now;
        xSemaphoreGive(idMutex);
    }
}

void add_received_id(uint32_t id) {
    if (idMutexInitialized && xSemaphoreTake(idMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (received_ids_count < MAX_IDS) {
            bool exists = false;
            for (int i = 0; i < received_ids_count; i++) {
                if (received_ids[i] == id) { exists = true; break; }
            }
            if (!exists) received_ids[received_ids_count++] = id;
        }
        xSemaphoreGive(idMutex);
    }
}

/**
 * @brief Checks if CAN nodes have timed out and invalidates telemetry data.
 */
void check_data_timeout(unsigned long currentMillis) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    for (int i = 0; i < NUM_FILTERS; i++) {
        uint32_t id = FILTERED_IDS[i];
        unsigned long last_seen = last_id_timestamps[i];
        unsigned long timeout = (FILTERED_IDS[i] == 0x37) ? SLOW_TIMEOUT_MS : FAST_TIMEOUT_MS;

        if (last_seen == 0 || (currentMillis - last_seen > timeout)) {
            switch (FILTERED_IDS[i]) {
                case 0x37:
                    telemetryData.imdIsoRValid = false;
                    telemetryData.vifcStatusValid = false;
                    telemetryData.imdStatus = 0;
                    break;
                case 0x239:
                    telemetryData.motorRPMValid = false;
                    telemetryData.motorTempValid = false;
                    telemetryData.mcuFlags = 0x02; // MCU STOP Bit (ID 2)
                    break;
                case 0x355:
                    telemetryData.bmsSoCValid = false;
                    break;
                case 0x356:
                    telemetryData.bmsCurrentValid = false;
                    telemetryData.batTempValid = false;
                    break;
            }
        }
    }
    xSemaphoreGive(dataMutex);
}