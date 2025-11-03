// can_processor.cpp
#define DEBUG
#include "can_process.h"
#include "main.h"  // für TelemetryData, dataMutex, etc.
#include <esp_task_wdt.h>

// --- Externe Abhängigkeiten ---
extern SemaphoreHandle_t dataMutex, idMutex;
extern bool dataMutexInitialized, idMutexInitialized;
extern volatile unsigned long last_id_timestamps[NUM_FILTERS];
extern const uint32_t FILTERED_IDS[NUM_FILTERS];
extern const uint32_t FAST_TIMEOUT_MS, SLOW_TIMEOUT_MS;
extern bool is_charging;
extern void safe_printf(const char *fmt, ...);

// twai_receive_task
// Receives CAN messages and forwards them to the queue
// Measures for safety and robustness:
// - Mutex protection: last_id_timestamps with idMutex (10 ms timeout).
// - Queue safety: Blocking xQueueSend with 5 ms timeout to avoid message loss.
// - Error handling: Logs TWAI receive and queue failures.
// - Debug: Limited to every 5 seconds.
// - Watchdog: Task registered and reset.
void twai_receive_task(void *parameter) {
    esp_task_wdt_add(NULL); // Register task with watchdog
    twai_message_t message;

    while (1) {
        esp_task_wdt_reset(); // Feed watchdog

        // --- 1. Receive CAN message ---
        if (twai_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
            unsigned long currentMillis = millis();
            last_receive_time = currentMillis; // Update global receive time

            // --- 2. Filter message ---
            bool accepted = false;
            for (int i = 0; i < NUM_FILTERS; i++) {
                if (message.identifier == FILTERED_IDS[i]) {
                    accepted = true;
                    break;
                }
            }

            if (accepted) {
                // --- 3. Forward to queue ---
                if (xQueueSend(canQueue, &message, pdMS_TO_TICKS(5)) != pdTRUE) {
                    safe_printf("twai_receive_task: CAN queue full, dropped ID 0x%X\n", message.identifier);
                }
            } else {
                add_received_id(message.identifier); // Handle unknown IDs
            }

            // --- 4. Debug output (every 5 seconds) ---
            // static unsigned long lastDebugPrint = 0;
            // if (accepted && currentMillis - lastDebugPrint >= 5000) {
            //     safe_printf("twai_receive_task: Received ID 0x%X\n", message.identifier);
            //     lastDebugPrint = currentMillis;
            // }
        } else {
            safe_printf("twai_receive_task: TWAI receive timeout\n");
        }

        // --- 5. Task delay ---
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


// --- process_can_messages() ---
void process_can_messages(void *parameter) {
    esp_task_wdt_add(NULL);
    twai_message_t message;
    static unsigned long lastDebugPrint[NUM_FILTERS] = {0};

    while (1) {
        esp_task_wdt_reset();

        if (xQueueReceive(canQueue, &message, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        unsigned long currentMillis = millis();
        last_can_update = currentMillis;

        // --- 1. Filterung ---
        int filter_index = -1;
        for (int i = 0; i < NUM_FILTERS; i++) {
            if (FILTERED_IDS[i] == message.identifier) {
                filter_index = i;
                break;
            }
        }
        if (filter_index == -1) {
            add_received_id(message.identifier);
            continue;
        }

        // --- 2. Timestamp ---
        if (idMutexInitialized && xSemaphoreTake(idMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            last_id_timestamps[filter_index] = currentMillis;
            xSemaphoreGive(idMutex);
        }

        // --- 3. Daten kopieren ---
        uint8_t data_copy[8] = {0};
        uint8_t dlc = min(message.data_length_code, (uint8_t)8);
        memcpy(data_copy, message.data, dlc);

        // --- 4. Verarbeitung mit dataMutex ---
        if (dataMutexInitialized && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (message.identifier) {
                case 0x239: // MCU
                    if (dlc >= 8) {
                        uint16_t rpm = (data_copy[1] << 8) | data_copy[0];
                        int8_t motor_temp = (int8_t)data_copy[2] - 40;
                        int8_t mcu_temp = (int8_t)data_copy[3] - 40;
                        uint16_t flags = (data_copy[4] << 8) | data_copy[5];
                        uint8_t fault = data_copy[6];

                        telemetryData.motorRPM = (rpm <= MOTOR_RPM_MAX) ? rpm : 0;
                        telemetryData.motorRPMValid = (rpm <= MOTOR_RPM_MAX);
                        telemetryData.motor_temp = (motor_temp >= TEMP_MIN && motor_temp <= MOTOR_TEMP_MAX) ? motor_temp : 0;
                        telemetryData.motorTempValid = (motor_temp >= TEMP_MIN && motor_temp <= MOTOR_TEMP_MAX);
                        telemetryData.mcu_temp = (mcu_temp >= TEMP_MIN && mcu_temp <= MCU_TEMP_MAX) ? mcu_temp : 0;
                        telemetryData.mcuTempValid = (mcu_temp >= TEMP_MIN && mcu_temp <= MCU_TEMP_MAX);
                        telemetryData.mcuFlags = flags;
                        telemetryData.mcuFlagsValid = true;
                        telemetryData.mcuFaultLevel = fault;
                        telemetryData.mcuFaultLevelValid = true;
                    }
                    break;

                case 0x356: // BMS Current + Temp
                    if (dlc >= 6) {
                        int16_t raw_current = (data_copy[3] << 8) | data_copy[2];
                        float current = raw_current / 10.0f;
                        int16_t raw_temp = (data_copy[5] << 8) | data_copy[4];
                        float temp = raw_temp / 10.0f;

                        bool current_valid = (fabsf(current) <= BMS_CURRENT_MAX);
                        bool temp_valid = (temp >= TEMP_MIN && temp <= BMS_TEMP_MAX);

                        #define CURRENT_AVG_COUNT 3
                        static float current_avg_buffer[CURRENT_AVG_COUNT] = {0};
                        static uint8_t current_avg_index = 0, current_avg_count = 0;
                        current_avg_buffer[current_avg_index] = current;
                        current_avg_index = (current_avg_index + 1) % CURRENT_AVG_COUNT;
                        if (current_avg_count < CURRENT_AVG_COUNT) current_avg_count++;
                        float current_avg = 0;
                        for (uint8_t i = 0; i < current_avg_count; i++) current_avg += current_avg_buffer[i];
                        current_avg /= current_avg_count;

                        bool new_charging = (current_avg > 5.0f);
                        if (is_charging != new_charging) {
                            is_charging = new_charging;
                            if (currentMillis - lastDebugPrint[filter_index] >= 5000) {
                                safe_printf("CAN: is_charging=%d (avg=%.1fA)\n", is_charging, current_avg);
                                lastDebugPrint[filter_index] = currentMillis;
                            }
                        }

                        telemetryData.bmsCurrent = (int16_t)current;
                        telemetryData.bmsCurrentValid = current_valid;
                        telemetryData.bat_temp = temp;
                        telemetryData.batTempValid = temp_valid;
                    }
                    break;

                case 0x379: // BMS Status
                    if (dlc >= 1) {
                        telemetryData.bmsStatus = data_copy[0];
                        telemetryData.bmsStatusValid = true;
                    }
                    break;

                case 0x355: // BMS SoC
                    if (dlc >= 2) {
                        uint16_t soc_raw = (data_copy[1] << 8) | data_copy[0];
                        uint8_t soc = soc_raw / 10.0f;
                        telemetryData.bmsSoC = (soc <= BMS_SOC_MAX) ? soc : 0;
                        telemetryData.bmsSoCValid = (soc <= BMS_SOC_MAX);
                    }
                    break;

                case 0x37: // IMD + VIFC
                    if (dlc >= 6) {
                        uint16_t isoR = (data_copy[1] << 8) | data_copy[0];
                        uint16_t status = (data_copy[3] << 8) | data_copy[2];
                        uint16_t vifc = (data_copy[5] << 8) | data_copy[4];

                        telemetryData.imdIsoR = (isoR <= IMD_ISO_R_MAX) ? isoR : 50000;
                        telemetryData.imdIsoRValid = (isoR <= IMD_ISO_R_MAX);
                        telemetryData.imdStatus = status;
                        telemetryData.imdStatusValid = true;
                        telemetryData.vifcStatus = vifc;
                        telemetryData.vifcStatusValid = true;

                        bool needSelfTest = ((vifc & (1 << 12)) || (vifc & (1 << 13))) && !selfTestRunning && !selfTestRequested && !is_charging;
                        if (needSelfTest) {
                            selfTestRequested = true;
                        }
                    }
                    break;

                case 0x22: // IMD Response
                    if (dlc >= 5) {
                        uint16_t cmd = (data_copy[0] << 8) | data_copy[1];
                        if (cmd == 0x00D0) {
                            telemetryData.selfTestResult = data_copy[3];
                            telemetryData.selfTestResultValid = true;
                        } else if (cmd == 0x0036) {
                            telemetryData.hv1Voltage = (data_copy[2] << 8) | data_copy[3];
                            telemetryData.hv1VoltageValid = true;
                        }
                    }
                    break;
            }
            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- check_data_timeout() ---
void check_data_timeout(unsigned long currentMillis) {
    if (!idMutexInitialized || !dataMutexInitialized) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        safe_printf("check_data_timeout: dataMutex timeout\n");
        return;
    }

    for (int i = 0; i < NUM_FILTERS; i++) {
        unsigned long diff = (currentMillis >= last_id_timestamps[i]) ? (currentMillis - last_id_timestamps[i]) : 0;
        unsigned long timeout = (FILTERED_IDS[i] == 0x37) ? SLOW_TIMEOUT_MS : FAST_TIMEOUT_MS;

        if (diff > timeout && last_id_timestamps[i] != 0) {
            switch (FILTERED_IDS[i]) {
                case 0x37:
                    telemetryData.imdIsoRValid = false;
                    telemetryData.imdStatusValid = false;
                    telemetryData.vifcStatusValid = false;
                    break;
                case 0x239:
                    telemetryData.motorRPMValid = false;
                    telemetryData.motorTempValid = false;
                    telemetryData.mcuTempValid = false;
                    telemetryData.mcuFlagsValid = false;
                    telemetryData.mcuFaultLevelValid = false;
                    break;
                case 0x355:
                    telemetryData.bmsSoCValid = false;
                    break;
                case 0x356:
                    telemetryData.bmsCurrentValid = false;
                    telemetryData.batTempValid = false;
                    is_charging = false;
                    break;
                case 0x379:
                    telemetryData.bmsStatusValid = false;
                    break;
                case 0x22:
                    telemetryData.hv1VoltageValid = false;
                    break;
            }
        }
    }

    xSemaphoreGive(dataMutex);
}


// Monitors CAN queue fill level and prevents overflow
// Measures for safety and robustness:
// - Queue monitoring: Warns at 80% and resets at 90% capacity.
// - Mutex protection: serialMutex with 10 ms timeout for safe printing.
// - Timing control: Uses provided currentMillis to ensure consistent intervals.
// - Debug: Limited to every 5 seconds to reduce serial load.
// - Error handling: Checks for valid queue and logs reset events.
void handleQueueMonitoring(unsigned long currentMillis) {
    // --- 1. Timing control ---
    static unsigned long previousMillisQueue = 0;
    const unsigned long intervalQueue = 10000; // 10 seconds
    if (currentMillis - previousMillisQueue < intervalQueue) {
        return; // Skip if interval not reached
    }
    previousMillisQueue = currentMillis;

    // --- 2. Check queue validity ---
    if (canQueue == NULL) {
        safe_printf("handleQueueMonitoring: canQueue is NULL\n");
        return;
    }

    // --- 3. Check queue fill level ---
    UBaseType_t queueItems = uxQueueMessagesWaiting(canQueue);
    const UBaseType_t QUEUE_CAPACITY = 100; // Updated to match revised queue size
    const UBaseType_t WARN_THRESHOLD = QUEUE_CAPACITY * 0.8; // 80%
    const UBaseType_t RESET_THRESHOLD = QUEUE_CAPACITY * 0.9; // 90%

    // --- 4. Handle overflow ---
    bool queue_reset = false;
    if (queueItems >= RESET_THRESHOLD) {
        xQueueReset(canQueue);
        queue_reset = true;
    }

    // --- 5. Log queue status ---
    bool mutex_taken = false;
    if (serialMutexInitialized && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mutex_taken = true;
        if (queue_reset) {
            safe_printf("handleQueueMonitoring: Queue overflow, reset at %u/%u\n", queueItems, QUEUE_CAPACITY);
        } else if (queueItems >= WARN_THRESHOLD) {
            safe_printf("handleQueueMonitoring: Queue high at %u/%u\n", queueItems, QUEUE_CAPACITY);
        } else {
            safe_printf("handleQueueMonitoring: Queue at %u/%u\n", queueItems, QUEUE_CAPACITY);
        }
    } else {
        // Silent fail to avoid blocking
    }

    // --- 6. Release mutex ---
    if (mutex_taken) {
        xSemaphoreGive(serialMutex);
    }
}

// Monitors CAN bus activity and detects timeouts
// Measures for safety and robustness:
// - Plausibility checks: Validates time_diff calculation to prevent overflow errors.
// - Mutex protection: serialMutex with 10 ms timeout for safe printing.
// - Debug: Limited to every 5 seconds to reduce serial load.
// - Error handling: Logs invalid time states.
void handleCanBus() {
    // --- 1. Capture current time ---
    unsigned long localMillis = millis();

    // --- 2. Calculate time difference ---
    unsigned long time_diff;
    if (localMillis >= last_receive_time) {
        time_diff = localMillis - last_receive_time;
    } else {
        time_diff = 0; // Prevent overflow
        safe_printf("handleCanBus: localMillis=%lu < last_receive_time=%lu, setting time_diff=0\n",
                    localMillis, last_receive_time);
    }

    // --- 3. Log and check timeout (every 5 seconds) ---
    static unsigned long lastLog = 0;
    if (localMillis - lastLog >= 5000) {
        // --- 4. Mutex for serial output ---
        bool mutex_taken = false;
        if (serialMutexInitialized && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            mutex_taken = true;
            safe_printf("handleCanBus: localMillis=%lu, last_receive_time=%lu, time_diff=%lu\n",
                        localMillis, last_receive_time, time_diff);
            if (time_diff > CAN_TIMEOUT_MS) {
                safe_printf("handleCanBus: CAN Bus Timeout: No messages received for %lu ms\n", time_diff);
            }
        } else {
            // Silent fail to avoid blocking
        }

        // --- 5. Release mutex ---
        if (mutex_taken) {
            xSemaphoreGive(serialMutex);
        }

        lastLog = localMillis;
    }
}


// Task to monitor CAN queue and TWAI status
// Measures for safety and robustness:
// - Checks CAN queue fill level to prevent overflow
// - Monitors TWAI status with timeout handling
// - Resets TWAI driver on error with recovery delay
// - Resets TWAI driver after several bus off conditions
// - Debug output limited to every 5 seconds
// - Watchdog reset to prevent task hangs
void twai_monitor_task(void *parameter) {
    esp_task_wdt_add(NULL);
    static unsigned long lastDebugPrint = 0;
    static uint8_t bus_off_count = 0;
    const uint8_t MAX_BUS_OFF = 5;

    while (1) {
        esp_task_wdt_reset();
        unsigned long currentMillis = millis();

        UBaseType_t queue_fill = uxQueueMessagesWaiting(canQueue);
        if (queue_fill >= 90) {
            safe_printf("twai_monitor_task: Queue overflow (%u/100), resetting\n", queue_fill);
            xQueueReset(canQueue);
        }

        twai_status_info_t status_info;
        if (twai_get_status_info(&status_info) == ESP_OK) {
            if (status_info.state == TWAI_STATE_STOPPED || status_info.state == TWAI_STATE_BUS_OFF) {
                bus_off_count++;
                safe_printf("twai_monitor_task: TWAI stopped or bus-off, restarting (count=%u)\n", bus_off_count);
                twai_stop();
                vTaskDelay(pdMS_TO_TICKS(100));
                if (bus_off_count >= MAX_BUS_OFF) {
                    safe_printf("twai_monitor_task: Max bus-off reached, reinstalling TWAI driver\n");
                    twai_driver_uninstall();
                    if (twai_driver_install(&g_config, &twai_timing_config, &twai_filter_config) != ESP_OK) {
                        safe_printf("twai_monitor_task: Failed to reinstall TWAI driver\n");
                        esp_restart();
                    }
                    bus_off_count = 0;
                }
                twai_start();
            } else if (status_info.tx_error_counter > 128 || status_info.rx_error_counter > 128) {
                safe_printf("twai_monitor_task: High error counters (TX=%u, RX=%u), resetting\n",
                            status_info.tx_error_counter, status_info.rx_error_counter);
                twai_initiate_recovery();
                vTaskDelay(pdMS_TO_TICKS(100));
                twai_start();
            }
        }

        // if (currentMillis - lastDebugPrint >= 5000) {
        //     safe_printf("twai_monitor_task: Queue=%u/100, TWAI state=%d, TX_err=%u, RX_err=%u, bus_off_count=%u\n",
        //                 queue_fill, status_info.state, status_info.tx_error_counter, status_info.rx_error_counter, bus_off_count);
        //     lastDebugPrint = currentMillis;
        // }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Tracks and logs unknown CAN IDs
// Measures for safety and robustness:
// - Mutex protection: received_ids and received_ids_count with idMutex (10 ms timeout).
// - Plausibility checks: Validates ID against FILTERED_IDS and prevents duplicates.
// - Array bounds safety: Ensures received_ids does not overflow MAX_IDS.
// - Debug: Limited to every 5 seconds to reduce serial load.
// - Error handling: Logs mutex timeouts and array overflow.
void add_received_id(uint32_t id) {
    // --- 1. Check if ID is already in FILTERED_IDS ---
    for (int i = 0; i < NUM_FILTERS; i++) {
        if (id == FILTERED_IDS[i]) {
            return; // Known ID, no action needed
        }
    }

    // --- 2. Mutex for received_ids and received_ids_count ---
    bool mutex_taken = false;
    if (idMutexInitialized && xSemaphoreTake(idMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mutex_taken = true;

        // --- 3. Check for duplicate ID ---
        for (int i = 0; i < received_ids_count; i++) {
            if (received_ids[i] == id) {
                xSemaphoreGive(idMutex);
                return; // Duplicate ID
            }
        }

        // --- 4. Add new ID if space available ---
        if (received_ids_count < MAX_IDS) {
            received_ids[received_ids_count++] = id;
        }

        // --- 5. Release mutex ---
        xSemaphoreGive(idMutex);
    } else {
        safe_printf("add_received_id: Timeout waiting for idMutex\n");
        return;
    }

    // --- 6. Debug output (every 5 seconds) ---
    // static unsigned long lastDebugPrint = 0;
    // unsigned long currentMillis = millis();
    // if (currentMillis - lastDebugPrint >= 5000) {
    //     if (received_ids_count >= MAX_IDS) {
    //         safe_printf("add_received_id: List full, cannot add ID 0x%X\n", id);
    //     } else {
    //         safe_printf("add_received_id: Added new CAN ID: 0x%X, count=%d\n", id, received_ids_count);
    //     }
    //     lastDebugPrint = currentMillis;
    // }
}
