/**
 * @file relay_control.cpp
 * @brief Logic for the 4-channel VCU Relay Module (ID 0x101).
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include "relay_control.h"
#include "main.h"
#include <driver/twai.h>

/* --- Global State --- */
uint8_t relayShadow = 0x00;     // Current relay bitmask
uint8_t manualOverride = 0x00;  // 1 = Manual/Web-Override active

static bool buzzerMuted = false;
static unsigned long afterrunStartTime = 0;
static unsigned long lastBuzzerToggle = 0;
static bool buzzerState = false;

/**
 * @brief Sends the relayShadow to the Remo-Module via CAN (ID 0x101).
 */
void sendRelayCommand() {
    twai_message_t msg = {0};
    msg.identifier = 0x101; 
    msg.data_length_code = 1; // Only 1 byte needed for 8 relays
    msg.data[0] = relayShadow; 

    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void setRelay(uint8_t channel, bool state) {
    if (channel < 1 || channel > 8) return;

    uint8_t oldShadow = relayShadow;
    if (state) relayShadow |= (1 << (channel - 1));
    else       relayShadow &= ~(1 << (channel - 1));
    
    if (oldShadow != relayShadow) {
        sendRelayCommand();
    }
}

void setRelayManual(uint8_t channel, bool state) {
    if (channel < 1 || channel > 4) return;
    manualOverride |= (1 << (channel - 1)); 
    setRelay(channel, state);
}

void releaseToAuto(uint8_t channel) {
    if (channel < 1 || channel > 4) return;
    manualOverride &= ~(1 << (channel - 1)); 
}

/**
 * @brief Main Relay Logic. Evaluates sensors and manages afterrun.
 */
void updateRelayAutomation() {
    unsigned long now = millis();
    
    // Safety copy of needed telemetry data to minimize Mutex holding time
    bool stRunning, isCharging, ignition;
    uint8_t stResult, pInv, pBat;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        stRunning = telemetryData.selfTestRunning;
        stResult  = telemetryData.selfTestResult;
        isCharging = telemetryData.is_charging;
        pInv      = telemetryData.inv_pump_pwm;
        pBat      = telemetryData.bat_pump_pwm;
        ignition  = (telemetryData.relayInputs & (1 << 0));
        xSemaphoreGive(dataMutex);
    } else return; // Skip this cycle if data is busy

    // --- Relay 1: CHECK OIL (Status Display) ---
    if (!(manualOverride & (1 << 0))) { 
        setRelay(1, (stRunning || stResult != 0));
    }

    // --- Relay 2: BUZZER (Acoustic Alert) ---
    if (!(manualOverride & (1 << 1))) {
        if (manualUnlockPressed && stResult != 0) buzzerMuted = true;
        if (stResult == 0) buzzerMuted = false; 

        if (stResult != 0 && !buzzerMuted) {
            if (now - lastBuzzerToggle >= 500) {
                buzzerState = !buzzerState;
                lastBuzzerToggle = now;
            }
            setRelay(2, buzzerState);
        } else {
            setRelay(2, false);
        }
    }

    // --- Relay 3: INVERTER FAN ---
    if (!(manualOverride & (1 << 2))) {
        setRelay(3, (pInv > 191)); // Activate Fan > 75% Load
    }

    // --- Relay 4: PUMP POWER & AFTERRUN ---
    if (!(manualOverride & (1 << 3))) {
        if (ignition || isCharging) {
            setRelay(4, true);
            afterrunStartTime = 0; 
        } else {
            // Trigger Afterrun if pumps were working hard (>25%)
            if (afterrunStartTime == 0 && (pInv > 64 || pBat > 64)) {
                afterrunStartTime = now;
                safe_printf("[RELAY] Starting 5min Cooling Afterrun...\n");
            }

            if (afterrunStartTime > 0 && (now - afterrunStartTime < PUMP_AFTERRUN_TIME)) {
                setRelay(4, true);
            } else {
                setRelay(4, false);
                afterrunStartTime = 0; 
            }
        }
    }
}