/**
 * @file relay_control.cpp
 * @brief Logic for Relay Management (Supports CAN-Remote and Onboard GPIOs).
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include "relay_control.h"
#include "main.h"

#ifdef HARDWARE_TCAN485
#include <driver/twai.h>
#endif

/* --- Global State --- */
uint8_t relayShadow = 0x00;     // Current relay bitmask
uint8_t manualOverride = 0x00;  // 1 = Manual/Web-Override active

static bool buzzerMuted = false;
static unsigned long afterrunStartTime = 0;
static unsigned long lastBuzzerToggle = 0;
static bool buzzerState = false;

/**
 * @brief Updates relay states based on the hardware platform.
 */
void sendRelayCommand() {
#ifdef HARDWARE_TCAN485
    // Legacy: Send relayShadow to the Remote-Module via CAN (ID 0x01)
    twai_message_t msg = {0};
    msg.identifier = 0x01;     
    msg.data_length_code = 8;  
    msg.data[0] = 0x01; // Function Code: Write DO
    msg.data[1] = 0x01; // Address
    msg.data[2] = relayShadow; 
    twai_transmit(&msg, pdMS_TO_TICKS(10));
#endif

#ifdef HARDWARE_T2CAN
    // New: Directly control onboard MOSFETs/Relays via GPIO
    digitalWrite(RELAY_1, (relayShadow & (1 << 0)));
    digitalWrite(RELAY_2, (relayShadow & (1 << 1)));
    digitalWrite(RELAY_3, (relayShadow & (1 << 2)));
    digitalWrite(RELAY_4, (relayShadow & (1 << 3)));
    
    // Additional T-2CAN onboard features
    digitalWrite(FAN_RELAY_PIN,   (relayShadow & (1 << 2))); // R3 is also the Fan
    digitalWrite(BAT_PUMP_RELAY,  (relayShadow & (1 << 3))); // R4 enables Pumps
    digitalWrite(INV_PUMP_RELAY,  (relayShadow & (1 << 3))); // R4 enables Pumps
    
    // Improved Piezo Interval Logic
    // If Relay 2 is "ON" in the shadow, we toggle the physical pin for the beep effect
    digitalWrite(PIEZO_PIN, (relayShadow & (1 << 1)) ? buzzerState : LOW);
#endif
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

/* --- Web Interface Manual Control --- */
/**
 * @brief Overrides automatic logic and sets a relay state manually from Web UI.
 */
void setRelayManual(uint8_t channel, bool state) {
    if (channel < 1 || channel > 4) return;
    manualOverride |= (1 << (channel - 1)); 
    setRelay(channel, state);
}

/**
 * @brief Releases a relay back to the automatic automation logic.
 */
void releaseToAuto(uint8_t channel) {
    if (channel < 1 || channel > 4) return;
    manualOverride &= ~(1 << (channel - 1)); 
    // State will be re-calculated in the next updateRelayAutomation() cycle
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

    // --- Relay 2: BUZZER LOGIC ---
    if (!(manualOverride & (1 << 1))) {
        if (manualUnlockPressed && stResult != 0) buzzerMuted = true;
        if (stResult == 0) buzzerMuted = false; 

        if (stResult != 0 && !buzzerMuted) {
            // This creates the "Beep... Beep..." interval
            if (now - lastBuzzerToggle >= 500) {
                buzzerState = !buzzerState;
                lastBuzzerToggle = now;
                // We force a hardware update to flip the Piezo pin immediately
                sendRelayCommand();
            }
            setRelay(2, true); // Keep logic state ON, updateHardwareRelays handles the pin toggling
        } else {
            setRelay(2, false);
        }
    }

    // --- Relay 3: INVERTER FAN ---
    if (!(manualOverride & (1 << 2))) {
        setRelay(3, (pInv > 191)); // Activate Fan > 75% Load
    }

    // --- Relay 4: PUMP POWER ---
    if (!(manualOverride & (1 << 3))) {
        bool shouldBeOn = false;
        if (ignition || isCharging) {
            shouldBeOn = true;
            afterrunStartTime = 0; 
        } else {
            if (afterrunStartTime == 0 && (pInv > 64 || pBat > 64)) {
                afterrunStartTime = now;
            }
            if (afterrunStartTime > 0 && (now - afterrunStartTime < 300000)) { // 5min
                shouldBeOn = true;
            }
        }
        setRelay(4, shouldBeOn);
    }

    // FORCE UPDATE: Falls das Modul den Status "vergisst", senden wir regelmäßig
    static unsigned long lastForceSend = 0;
    if (now - lastForceSend > 500) { 
        sendRelayCommand();
        lastForceSend = now;
    }
}