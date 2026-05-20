/**
 * @file relay_control.cpp
 * @brief Main processing worker to synchronize General Purpose Relays with structural overrides.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include "relay_control.h"
#include "main.h"

void init_relay_hardware() {
    // Vintage Instrument Cluster Indicators
    pinMode(LED_CHECK_OIL_PIN, OUTPUT);   digitalWrite(LED_CHECK_OIL_PIN, LOW);
    pinMode(LED_BATTERY_PIN, OUTPUT);     digitalWrite(LED_BATTERY_PIN, LOW);

    // High-Voltage Safety Acoustic Alarm
    pinMode(PIEZO_RELAY_PIN, OUTPUT);     digitalWrite(PIEZO_RELAY_PIN, LOW);

    // Thermal Management Actuators
    pinMode(FAN_RELAY_PIN, OUTPUT);       digitalWrite(FAN_RELAY_PIN, LOW);
    pinMode(BAT_PUMP_RELAY_PIN, OUTPUT);  digitalWrite(BAT_PUMP_RELAY_PIN, LOW);
    pinMode(INV_PUMP_RELAY_PIN, OUTPUT);  digitalWrite(INV_PUMP_RELAY_PIN, LOW);

    // Hut V3 Auxiliary High-Current Expansion Drivers
    pinMode(RELAY_11_PIN, OUTPUT);        digitalWrite(RELAY_11_PIN, LOW);
    pinMode(RELAY_12_PIN, OUTPUT);        digitalWrite(RELAY_12_PIN, LOW);
    pinMode(RELAY_13_PIN, OUTPUT);        digitalWrite(RELAY_13_PIN, LOW);
    pinMode(RELAY_14_PIN, OUTPUT);        digitalWrite(RELAY_14_PIN, LOW);

    // General Purpose Inputs
    pinMode(INPUT_13_PIN, INPUT_PULLUP);
}

void update_auxiliary_relays() {
    bool led_oil_target = false;
    bool led_batt_target = false;
    bool r11_target = false; 
    bool r12_target = false;
    bool r13_target = false; 
    bool r14_target = false;

    // =========================================================================
    // 1. DYNAMIC AUTOMATION LAYER (Calculated if loop runs on AUTO)
    // =========================================================================
    WITH_DATA_MUTEX({
        if (!telemetryData.bmsStatusValid || telemetryData.bmsStatus == BMS_STATUS_ERROR) {
            led_batt_target = true;
            led_oil_target = true; 
        }
    });

    // =========================================================================
    // 2. CRITICAL DETERMINISTIC HARDWARE DEPLOYMENT (Strict Isolated Registers)
    // =========================================================================
    
    // --- CHECK OIL LAMP (Bit 16) ---
    if (manualOverride & (1UL << OVR_BIT_CHECK_OIL)) {
        digitalWrite(LED_CHECK_OIL_PIN, (manualOverride & (1UL << 16)) ? HIGH : LOW);
    } else {
        digitalWrite(LED_CHECK_OIL_PIN, led_oil_target ? HIGH : LOW);
    }

    // --- VINTAGE BATTERY LAMP (Bit 23) ---
    if (manualOverride & (1UL << OVR_BIT_LED_BATT)) {
        digitalWrite(LED_BATTERY_PIN, (manualOverride & (1UL << 23)) ? HIGH : LOW);
    } else {
        digitalWrite(LED_BATTERY_PIN, led_batt_target ? HIGH : LOW);
    }

    // --- HUT LATCH V3: RELAY 11 (Bit 24) ---
    if (manualOverride & (1UL << OVR_BIT_REL11)) {
        digitalWrite(RELAY_11_PIN, (manualOverride & (1UL << 24)) ? HIGH : LOW);
    } else {
        digitalWrite(RELAY_11_PIN, r11_target ? HIGH : LOW);
    }

    // --- HUT LATCH V3: RELAY 12 (Bit 25) ---
    if (manualOverride & (1UL << OVR_BIT_REL12)) {
        digitalWrite(RELAY_12_PIN, (manualOverride & (1UL << 25)) ? HIGH : LOW);
    } else {
        digitalWrite(RELAY_12_PIN, r12_target ? HIGH : LOW);
    }

    // --- HUT LATCH V3: RELAY 13 (Bit 26) ---
    if (manualOverride & (1UL << OVR_BIT_REL13)) {
        digitalWrite(RELAY_13_PIN, (manualOverride & (1UL << 26)) ? HIGH : LOW);
    } else {
        digitalWrite(RELAY_13_PIN, r13_target ? HIGH : LOW);
    }

    // --- HUT LATCH V3: RELAY 14 (Bit 27) ---
    if (manualOverride & (1UL << OVR_BIT_REL14)) {
        digitalWrite(RELAY_14_PIN, (manualOverride & (1UL << 27)) ? HIGH : LOW);
    } else {
        digitalWrite(RELAY_14_PIN, r14_target ? HIGH : LOW);
    }

    // =========================================================================
    // 3. RE-READ AND SYNC TELEMETRY DIRECTLY FROM PHYSICAL PORTS
    // =========================================================================
    WITH_DATA_MUTEX({
        telemetryData.auxRelay11  = (digitalRead(RELAY_11_PIN) == HIGH);
        telemetryData.auxRelay12  = (digitalRead(RELAY_12_PIN) == HIGH);
        telemetryData.auxRelay13  = (digitalRead(RELAY_13_PIN) == HIGH);
        telemetryData.auxRelay14  = (digitalRead(RELAY_14_PIN) == HIGH);
        telemetryData.ledCheckOil = (digitalRead(LED_CHECK_OIL_PIN) == HIGH);
        telemetryData.ledBattery  = (digitalRead(LED_BATTERY_PIN) == HIGH);
        telemetryData.auxinput13  = (digitalRead(INPUT_13_PIN) == LOW);
    });
}