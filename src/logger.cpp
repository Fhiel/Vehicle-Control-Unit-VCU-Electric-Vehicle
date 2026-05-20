/**
* @file logger.cpp
* @brief Circular RAM-buffered system log engine for high-voltage vehicle monitoring.
* @author Fhiel (X1/9e Project)
* @license MIT
*/

#include "main.h"
#include "utils.h"
#include "logger.h"
#include <Arduino.h>
#include "FS.h"
#include "LittleFS.h"

#define MAX_LOG_ENTRIES 50
#define LOG_FILE_PATH "/log.json"

struct LogEntry {
    unsigned long timestamp;
    char code[8];
    char message[48];
};

static LogEntry ringBuffer[MAX_LOG_ENTRIES];
static int head = 0;
static int totalEntries = 0;
static SemaphoreHandle_t logMutex = xSemaphoreCreateMutex();

void log_system_event(const char* code, const char* message) {
    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ringBuffer[head].timestamp = millis();
        strncpy(ringBuffer[head].code, code, sizeof(ringBuffer[head].code) - 1);
        strncpy(ringBuffer[head].message, message, sizeof(ringBuffer[head].message) - 1);
        
        head = (head + 1) % MAX_LOG_ENTRIES;
        if (totalEntries < MAX_LOG_ENTRIES) totalEntries++;
        
        xSemaphoreGive(logMutex);
    }
    
    // Mirror to standard serial console for immediate debugging
    safe_printf("[LOG - %s] %s\n", code, message);
}

/**
 * @brief Commits the volatile memory ring buffer to the physical LittleFS partition.
 * Execute this on systematic boundaries like Charging Complete, or Drive Inhibit release.
 */
bool commit_logs_to_flash() {
    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;

    // Open file in append mode or create clean weekly files
    File logFile = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
    if (!logFile) {
        logFile = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
    }
    
    if (!logFile) {
        xSemaphoreGive(logMutex);
        return false;
    }

    // Export raw strings or JSON nodes sequentially
    int current = (head - totalEntries + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
    for (int i = 0; i < totalEntries; i++) {
        logFile.printf("{\"t\":%lu,\"c\":\"%s\",\"m\":\"%s\"}\n", 
                       ringBuffer[current].timestamp, 
                       ringBuffer[current].code, 
                       ringBuffer[current].message);
        current = (current + 1) % MAX_LOG_ENTRIES;
    }

    logFile.close();
    totalEntries = 0; // Clear memory loop since data is safely written to flash
    xSemaphoreGive(logMutex);
    return true;
}