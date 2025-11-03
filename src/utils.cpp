//utils.cpp
#define DEBUG
#include "utils.h"
#include "rs485.h"
#include "main.h"

void safe_printf(const char* format, ...) {
#ifdef DEBUG
    if (!serialMutexInitialized) return;

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        xSemaphoreGive(serialMutex);
    }
#else
    (void)format;  // vermeidet "unused parameter"
#endif
}

// Calculates XOR checksum for RS485 packet
// Measures for safety and robustness:
// - Null pointer check: Validates input buffer.
// - Length validation: Ensures len is non-zero and reasonable.
// - Debug: Limited to every 10 seconds for verification.
uint8_t calculateChecksum(const uint8_t* data, size_t len) {
    // --- 1. Input validation ---
    if (data == NULL || len == 0 || len > PACKET_PAYLOAD_LENGTH) {
        safe_printf("calculateChecksum: Invalid input (data=%p, len=%u)\n", data, len);
        return 0; // Safe default
    }

    // --- 2. Calculate XOR checksum ---
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }

    // --- 3. Debug output (every 10 seconds) ---
    // static unsigned long lastDebugPrint = 0;
    // unsigned long currentMillis = millis();
    // if (currentMillis - lastDebugPrint >= 10000) {
    //     safe_printf("calculateChecksum: len=%u, checksum=0x%02X\n", len, checksum);
    //     lastDebugPrint = currentMillis;
    // }

    return checksum;
}

// Maps a float value from one range to another
// Measures for safety and robustness:
// - Input validation: Checks for valid input ranges to prevent division by zero.
// - Plausibility checks: Clamps output to output range.
// - Error handling: Returns default value on invalid inputs.
// - Debug: Limited to every 5 seconds to reduce serial load.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    // --- 1. Input validation ---
    if (in_max == in_min) {
        safe_printf("mapFloat: Invalid input range (in_min=%f, in_max=%f)\n", in_min, in_max);
        return out_min; // Safe default
    }

    // --- 2. Calculate mapped value ---
    float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    // --- 3. Clamp to output range ---
    if (result < out_min) result = out_min;
    if (result > out_max) result = out_max;

    // --- 4. Debug output (every 5 seconds) ---
    // static unsigned long lastDebugPrint = 0;
    // unsigned long currentMillis = millis();
    // if (currentMillis - lastDebugPrint >= 5000) {
    //     safe_printf("mapFloat: x=%f, in=[%f,%f], out=[%f,%f], result=%f\n",
    //                 x, in_min, in_max, out_min, out_max, result);
    //     lastDebugPrint = currentMillis;
    // }

    return result;
}


// Logs RS485 packet contents for debugging
// Measures for safety and robustness:
// - Null pointer checks: Validates input buffer.
// - Mutex protection: serialMutex with 10 ms timeout for safe printing.
// - Debug: Limited to every 10 seconds to reduce serial load.
// - Buffer safety: Limits logging to PACKET_TOTAL_LENGTH.
void log_rs485_packet(const uint8_t* packet) {
    // --- 1. Null pointer check ---
    if (packet == NULL) {
        safe_printf("log_rs485_packet: NULL packet pointer\n");
        return;
    }

    // --- 2. Mutex for serial output ---
    if (!serialMutexInitialized || xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return; // Silent fail to avoid blocking
    }

    // --- 3. Log packet (every 10 seconds) ---
    static unsigned long lastDebugPrint = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastDebugPrint >= 10000) {
        char hex[64] = {0};
        size_t len = min((size_t)PACKET_TOTAL_LENGTH, (size_t)32);// Cap to avoid buffer overflow
        for (size_t i = 0; i < len; i++) {
            snprintf(hex + strlen(hex), sizeof(hex) - strlen(hex), "%02X ", packet[i]);
        }
        safe_printf("log_rs485_packet: Packet: %s\n", hex);
        lastDebugPrint = currentMillis;
    }

    // --- 4. Release mutex ---
    xSemaphoreGive(serialMutex);
}

