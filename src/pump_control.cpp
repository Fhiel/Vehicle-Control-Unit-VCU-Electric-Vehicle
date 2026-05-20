#define DEBUG
#include "pump_control.h"
#include "main.h"
#include <driver/ledc.h>

bool enablePumpDebug = false;

void update_inv_pump(int8_t mcu_temp, bool valid) {
    const int START_TEMP = 30;
    const int STOP_TEMP  = 28;
    const int MAX_TEMP   = MCU_TEMP_MAX;
    
    int duty_cycle = 0;
    bool relay_state = false;
    bool fan_state = false;

    // =========================================================================
    // A. INVERTER FLUID PUMP CONTROL (STRICT 1UL FIXED MASK: BITS 21 & 22)
    // =========================================================================
    if (manualOverride & (1UL << OVR_BIT_INV_PUMP)) {
        // Override Active: Extract the 2-bit stage profile window safely
        uint8_t p_stage = (manualOverride >> 21) & 0x03;
        
        if (p_stage == 1) {        // ECO Stage (20%)
            relay_state = true;
            duty_cycle = 51;      
        } else if (p_stage == 2) { // BOOST Stage (80%)
            relay_state = true;
            duty_cycle = 204;     
        } else {                   // OFF Stage
            relay_state = false;
            duty_cycle = 0;
        }
    } else {
        // AUTOMATION MODE: Closed-loop thermal tracking logic
        if (!valid) {
            relay_state = false;
            duty_cycle = 0;
        } else {
            if (mcu_temp >= START_TEMP) relay_state = true;
            else if (mcu_temp <= STOP_TEMP) relay_state = false;

            if (!relay_state) duty_cycle = 0;
            else if (mcu_temp <= START_TEMP) duty_cycle = 51;
            else if (mcu_temp >= MAX_TEMP) duty_cycle = 255;
            else duty_cycle = (int)map(mcu_temp, START_TEMP, MAX_TEMP, 51, 255);
        }
    }

    // CRITICAL FIX: Execute physical pin write OUTSIDE of the conditional loops!
    digitalWrite(INV_PUMP_RELAY_PIN, relay_state ? HIGH : LOW);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)INV_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)INV_PWM_CHANNEL);

    // =========================================================================
    // B. RADIATOR COOLING FAN (STRICT 1UL FIXED MASK: STATE BIT 18)
    // =========================================================================
    if (manualOverride & (1UL << OVR_BIT_FAN)) {
        // Override Active: Extract individual binary command from slot 18
        fan_state = (manualOverride & (1UL << 18)) ? true : false;
    } else {
        // Automation Mode: Follow closed-loop threshold rules
        const int FAN_START_TEMP = 45; const int FAN_STOP_TEMP = 40;
        if (!valid) fan_state = false;
        else if (mcu_temp >= FAN_START_TEMP) fan_state = true;
        else if (mcu_temp <= FAN_STOP_TEMP) fan_state = false;
    }

    // Direct physical actuator command execution
    digitalWrite(FAN_RELAY_PIN, fan_state ? HIGH : LOW);

    // Synchronize values safely back to the global telemetry tracking matrix
    WITH_DATA_MUTEX({ 
        telemetryData.invPumpRelay = (digitalRead(INV_PUMP_RELAY_PIN) == HIGH);
        telemetryData.invPumpPwm   = duty_cycle; 
        telemetryData.fanRelay     = (digitalRead(FAN_RELAY_PIN) == HIGH);  
    });
}

void update_bat_pump(float bat_temp, bool valid) {
    const float START_TEMP = 20.0f;
    const float STOP_TEMP  = 18.0f;
    const float MAX_TEMP   = 45.0f;
    
    int duty_cycle = 0;
    bool relay_state = false;

    // =========================================================================
    // A. BATTERY FLUID PUMP CONTROL (STRICT 1UL FIXED MASK: BITS 19 & 20)
    // =========================================================================
    if (manualOverride & (1UL << OVR_BIT_BAT_PUMP)) {
        // Override Active: Extract the 2-bit stage profile window safely
        uint8_t p_stage = (manualOverride >> 19) & 0x03;
        
        if (p_stage == 1) {        // ECO Stage (20%)
            relay_state = true;
            duty_cycle = 51;
        } else if (p_stage == 2) { // BOOST Stage (80%)
            relay_state = true;
            duty_cycle = 204;
        } else {                   // OFF Stage
            relay_state = false;
            duty_cycle = 0;
        }
    } else {
        // AUTOMATION MODE
        if (!valid) {
            relay_state = false;
            duty_cycle = 0;
        } else {
            if (bat_temp >= START_TEMP) relay_state = true;
            else if (bat_temp <= STOP_TEMP) relay_state = false;

            if (!relay_state) duty_cycle = 0;
            else if (bat_temp <= START_TEMP) duty_cycle = 51;
            else if (bat_temp >= MAX_TEMP) duty_cycle = 255;
            else duty_cycle = (int)mapFloat(bat_temp, START_TEMP, MAX_TEMP, 51.0f, 255.0f);
        }
    }

    // Unconditional hardware output sync block
    digitalWrite(BAT_PUMP_RELAY_PIN, relay_state ? HIGH : LOW);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)BAT_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)BAT_PWM_CHANNEL);

    WITH_DATA_MUTEX({ 
        telemetryData.batPumpRelay = (digitalRead(BAT_PUMP_RELAY_PIN) == HIGH);
        telemetryData.batPumpPwm   = duty_cycle; 
    });

    // Debug output engine
    static unsigned long lastDebug = 0;
    if (enablePumpDebug && (millis() - lastDebug >= 5000)) {
        safe_printf("[PUMP] Battery: %.1f°C | Relay: %s | Duty: %d%%\n", 
                    bat_temp, relay_state ? "ON" : "OFF", (duty_cycle * 100 / 255));
        lastDebug = millis();
    }
}