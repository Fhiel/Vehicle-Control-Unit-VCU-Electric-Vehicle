/**
 * @file pump_control.cpp
 * @brief Thermal management for Battery and Inverter cooling loops.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#define DEBUG
#include "pump_control.h"
#include "main.h"
#include <driver/ledc.h>

/**
 * @brief Controls Inverter Pump speed based on MCU/Motor temperature.
 * Logic: Linear ramp between MIN_TEMP and MAX_TEMP.
 */
void update_inv_pump(int8_t mcu_temp, bool valid) {
    const int MIN_TEMP = 30;              // Start ramping up at 30°C
    const int MAX_TEMP = MCU_TEMP_MAX;    // Full power at defined limit
    const int MIN_DUTY = 51;              // 20% Minimum to prevent stalling
    const int MAX_DUTY = 255;             // 100%
    int duty_cycle = 0;

    if (!valid) {
        duty_cycle = 0; // Failsafe: Off if no data (Relay 4 afterrun might still be active)
    } else if (mcu_temp <= MIN_TEMP) {
        duty_cycle = MIN_DUTY;
    } else if (mcu_temp >= MAX_TEMP) {
        duty_cycle = MAX_DUTY;
    } else {
        duty_cycle = (int)map(mcu_temp, MIN_TEMP, MAX_TEMP, MIN_DUTY, MAX_DUTY);
    }

    // Apply Hardware PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)INV_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)INV_PWM_CHANNEL);

    // Update Telemetry for Web Dashboard
    WITH_DATA_MUTEX({ telemetryData.inv_pump_pwm = duty_cycle; });

    // Debug output every 5 seconds
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 5000) {
        safe_printf("[PUMP] Inverter: %d°C, Duty: %d%%\n", mcu_temp, (duty_cycle * 100 / 255));
        lastDebug = millis();
    }
}

/**
 * @brief Controls Battery Pump speed based on Tesla Module temperatures.
 * Logic: Tesla modules like it cool (20-35°C). Full speed at 50°C.
 */
void update_bat_pump(float bat_temp, bool valid) {
    const float MIN_TEMP = 20.0f;
    const float MAX_TEMP = 45.0f;         // Full cooling earlier for battery longevity
    const int MIN_DUTY = 51;              // 20% Minimum
    const int MAX_DUTY = 255;
    int duty_cycle = 0;

    if (!valid) {
        duty_cycle = 0;
    } else if (bat_temp <= MIN_TEMP) {
        duty_cycle = MIN_DUTY;
    } else if (bat_temp >= MAX_TEMP) {
        duty_cycle = MAX_DUTY;
    } else {
        duty_cycle = (int)mapFloat(bat_temp, MIN_TEMP, MAX_TEMP, (float)MIN_DUTY, (float)MAX_DUTY);
    }

    // Apply Hardware PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)BAT_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)BAT_PWM_CHANNEL);

    // Update Telemetry for Web Dashboard
    WITH_DATA_MUTEX({ telemetryData.bat_pump_pwm = duty_cycle; });

    // Debug output every 5 seconds
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 5000) {
        safe_printf("[PUMP] Battery: %.1f°C, Duty: %d%%\n", bat_temp, (duty_cycle * 100 / 255));
        lastDebug = millis();
    }
}