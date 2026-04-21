/**
 * @file led_control.cpp
 * @brief Physical LED feedback aligned with Tesla-style charging states.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include "led_control.h"
#include "main.h"

void update_led() {
    bool is_locked = false;
    bool is_unlocking = false;
    bool is_charging = false;
    bool bms_ready = false;
    bool self_test_failed = false;
    bool self_test_running = false;
    float bms_soc = 0.0f;

    // 1. Thread-safe data acquisition
    WITH_DATA_MUTEX({
        is_locked = telemetryData.isLocked;
        is_unlocking = telemetryData.isUnLocking;
        is_charging = telemetryData.is_charging;
        bms_ready = (telemetryData.bmsStatusValid && telemetryData.bmsStatus == 3);
        self_test_failed = telemetryData.selfTestFailed;
        self_test_running = telemetryData.selfTestRunning;
        bms_soc = telemetryData.bmsSoC;
    });

    unsigned long now = millis();
    
    // --- 2. CRITICAL ERROR: IMD FAILURE (Red Strobe) ---
    if (self_test_failed) {
        static unsigned long strobe_timer = 0;
        static bool strobe_on = true;
        // Fast flash to indicate high-voltage hazard
        if (now - strobe_timer >= (strobe_on ? 50 : 800)) {
            strobe_on = !strobe_on;
            strobe_timer = now;
            FastLED.setBrightness(255);
            leds[0] = strobe_on ? CRGB::Red : CRGB::Black;
            FastLED.show();
        }
        return; 
    }

    // --- 3. STATE MACHINE (Tesla-Style States) ---
    if (self_test_running) {
        // Initializing / Checking isolation
        leds[0] = CRGB::Yellow;
        FastLED.setBrightness(128);
    } 
    else if (is_unlocking || (!is_locked && is_charging)) {
        // Red Solid: Charging without locking the Type 2 cable (Safety Warning)
        leds[0] = CRGB::Red; 
        FastLED.setBrightness(255);
    } 
    else if (is_charging) {
        // Check if daily limit (80%) is reached
        if (bms_soc >= 80.0f && dailyModeActive) {
            // Tesla Style: Solid Blue (Waiting / Limit Reached)
            leds[0] = CRGB::Blue;
            FastLED.setBrightness(60);
        } else {
            // Tesla Style: Pulsing Green (Active Charging)
            uint8_t pulse = beatsin8(15, 30, 255); 
            FastLED.setBrightness(pulse);
            leds[0] = CRGB::Green;
        }
    } 
    else if (is_locked) {
        // Cable is locked but not charging
        if (bms_ready) {
            leds[0] = CRGB::Green;
            FastLED.setBrightness(60);
        } else {
            leds[0] = CRGB::Yellow;
            FastLED.setBrightness(40);
        }
    } 
    else {
        // Cable removed (!is_locked)
        // LED OFF to indicate "Ready to Drive" state (Cable unplugged)
        leds[0] = CRGB::Black; 
        FastLED.setBrightness(0);
    }

    FastLED.show();

    #ifdef DEBUG
    static unsigned long last_print = 0;
    if (now - last_print >= 5000) {
        safe_printf("[LED] State: %s | SoC: %.1f%%\n", 
            self_test_failed ? "FAIL" : (is_charging ? "CHARGING" : "READY"), bms_soc);
        last_print = now;
    }
    #endif
}