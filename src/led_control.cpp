/**
 * @file led_control.cpp
 * @brief Physical LED feedback aligned with Tesla-style charging states and shutdown loops.
 * @note Fixed variable naming bugs and expanded diagnostic status tracking IDs.
 * @author Frank Hielscher (X1/9e Project)
 * @license MIT
 */

#include "led_control.h"
#include "main.h"
#include <cmath>

void update_led() {
    bool isLocked = false;
    bool isUnlocking = false;
    bool isCharging = false;
    bool isBmsready = false; 
    bool selftestfailed = false;
    bool selftestrunning = false;
    bool manualstopactive = false; 
    float bmssoc = 0.0f;

    // --- 1. THREAD-SAFE CONTEXT DATA ACQUISITION ---
    WITH_DATA_MUTEX({
        isLocked = telemetryData.isLocked;
        isUnlocking = telemetryData.isUnLocking;
        isCharging = telemetryData.isCharging;
        isBmsready = (telemetryData.bmsStatusValid && telemetryData.bmsStatus == BMS_STATUS_READY);
        selftestfailed = telemetryData.selfTestFailed;
        selftestrunning = telemetryData.selfTestRunning;
        bmssoc = telemetryData.bmsSoC;
        manualstopactive = telemetryData.manualStopRequested; 
    });

    unsigned long now = millis();
    uint8_t r = 0, g = 0, b = 0, brightness = 0;
    uint8_t ui_status_id = 0; // Default: 0 = Amber/Standby
    
    // --- 2. CRITICAL ERROR: IMD FAILURE (Red Strobe) ---
    if (selftestfailed) {
        static unsigned long strobe_timer = 0;
        static bool strobe_on = true;
        if (now - strobe_timer >= (strobe_on ? 50 : 800)) {
            strobe_on = !strobe_on;
            strobe_timer = now;
        }
        if (strobe_on) { r = 255; brightness = 255; }
        ui_status_id = 2; // 2 = Red / Error
    }
    // --- 3. IMD INSULATION MEASUREMENT RUNNING (Solid Yellow) ---
    else if (selftestrunning) {
        r = 255; g = 255; brightness = 128; 
        ui_status_id = 4; // 4 = Yellow / Test Active
    } 
    // --- 4. CRITICAL CHARGER RAMP-DOWN SEQUENCE (Fast Cyan Pulsing) ---
    else if (manualstopactive) {
        float intensity = (sin(now / 125.0) + 1.0) / 2.0; 
        g = 255; b = 255; 
        brightness = 50 + (uint8_t)(intensity * 205);
        ui_status_id = 3; // 3 = Cyan / Ramp-Down stopping
    }
    // --- 5. UNMANAGED PLUG / CONNECTOR OPEN WARNING (Solid Red) ---
    else if (isUnlocking || (!isLocked && isCharging)) {
        r = 255; brightness = 255; 
        ui_status_id = 2; // 2 = Red / Danger Area
    } 
    // --- 6. ACTIVE CHARGING PROCESS (Tesla-Style Green Breathing) ---
    else if (isCharging) {
        if (bmssoc >= 80.0f && dailyModeActive) {
            b = 255; brightness = 60; // Solid Blue (Daily range ceiling achieved)
            ui_status_id = 5; // 5 = Blue / Limit Reached
        } else {
            float intensity = (sin(now / 500.0) + 1.0) / 2.0; 
            g = 255; brightness = 30 + (uint8_t)(intensity * 225);
            ui_status_id = 1; // 1 = Green / Charging
        }
    } 
    // --- 7. VEHICLE LOCKED & SECURED ---
    else if (isLocked) {
        if (isBmsready) { 
            g = 255; brightness = 60; // Solid Green: Propulsion loop closed, drive-ready
            ui_status_id = 1; 
        } else { 
            r = 255; g = 255; brightness = 40; // Amber: Waiting/Standby
            ui_status_id = 0; 
        }
    }

    // Command physical NeoPixel/Hardware layer
    led.setBrightness(brightness);
    led.setLedColorData(0, r, g, b);
    led.show();

    // --- 4. THREAD-SAFE STATUS WRITE-BACK TO TELEMETRY MATRIX ---
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        telemetryData.ws2812Status = ui_status_id;
        xSemaphoreGive(dataMutex);
    }

    #ifdef DEBUG
    static unsigned long last_print = 0;
    if (now - last_print >= 5000) {
        safe_printf("[LED] State ID: %d | BMS-Status: %d | SoC: %.1f%%\n", 
                    ui_status_id, telemetryData.bmsStatus, bmssoc); // FIXED: Variable name matching compiler footprint
        last_print = now;
    }
    #endif
}