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
    msg.identifier = 0x01;     
    msg.extd = 0;              
    msg.rtr = 0;               
    msg.data_length_code = 8;  

    // Byte 0: Function Code (0x01 = Write DO)
    msg.data[0] = 0x01; 
    
    // Byte 1: Address Code (0x01 = Factory Setting)
    msg.data[1] = 0x01; 

    // Byte 2: Relais 1-4
    // Bit 0 = R1, Bit 1 = R2, Bit 2 = R3, Bit 3 = R4 ...
    msg.data[2] = relayShadow; 

    // Byte 3 bis 7: reserved / unused
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    // Transmit with 10ms Timeout
    esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(10));
    
    #ifdef DEBUG
    if (res != ESP_OK) {
        printf("CAN Relay Transmit Error: 0x%X\n", res);
    }
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