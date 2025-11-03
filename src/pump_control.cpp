//pump_control.cpp
#define DEBUG
#include "pump_control.h"
#include "main.h"
#include <driver/ledc.h>

// update_inv_pump
// controls pump speed based on mcu_temp
// Measures for safety and robustness:
// - Plausibility checks: mcu_temp limited to -50°C to +150°C.
// - Safe default: Pump off for invalid values.
// - Debug: Limited to every 5 seconds
void update_inv_pump(int8_t mcu_temp, bool valid) {
    unsigned long currentMillis = millis();

    // --- 1. Validitätsprüfung ---
    const int MIN_TEMP = TEMP_MIN;      // z. B. 60
    const int MAX_TEMP = MCU_TEMP_MAX;  // z. B. 100
    const int MIN_DUTY = 51;
    const int MAX_DUTY = 255;
    int duty_cycle = 0;

    if (!valid) {
        duty_cycle = 0;
        safe_printf("update_inv_pump: Ungültige mcu_temp (valid=%d), pump off\n", valid);
    } else if (mcu_temp <= MIN_TEMP) {
        duty_cycle = MIN_DUTY;
    } else if (mcu_temp >= MAX_TEMP) {
        duty_cycle = MAX_DUTY;
    } else {
        duty_cycle = (int)map(mcu_temp, MIN_TEMP, MAX_TEMP, MIN_DUTY, MAX_DUTY);
    }

    // --- 2. PWM setzen ---
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)INV_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)INV_PWM_CHANNEL);

    // --- 3. Debug alle 5 Sekunden ---
    static unsigned long lastDebugPrint = 0;
    if (currentMillis - lastDebugPrint >= 5000) {
        safe_printf("update_inv_pump: mcu_temp=%d°C, valid=%d, duty_cycle=%d%%\n",
                    mcu_temp, valid, (duty_cycle * 100 / 255));
        lastDebugPrint = currentMillis;
    }
}


// update_bat_pump
// controls pump speed based on mcu_temp
// Measures for safety and robustness:
// - Plausibility checks: mcu_temp limited to -50°C to +150°C.
// - Safe default: Pump off for invalid values.
// - Debug: Limited to every 5 seconds
void update_bat_pump(float bat_temp, bool valid) {
    const float MIN_TEMP = 20.0f;
    const float MAX_TEMP = 50.0f;
    const int MIN_DUTY = 51;
    const int MAX_DUTY = 255;
    int duty_cycle = 0;

    if (!valid) {
        duty_cycle = 0;
    } else if (bat_temp <= MIN_TEMP) {
        duty_cycle = MIN_DUTY;
    } else if (bat_temp >= MAX_TEMP) {
        duty_cycle = MAX_DUTY;
    } else {
        duty_cycle = (int)mapFloat(bat_temp, MIN_TEMP, MAX_TEMP, MIN_DUTY, MAX_DUTY);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)BAT_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)BAT_PWM_CHANNEL);

    // Debug alle 5 Sekunden
    static unsigned long lastDebug = 0;
    unsigned long now = millis();
    if (now - lastDebug >= 5000) {
        safe_printf("update_bat_pump: T=%.1f°C, valid=%d, duty=%d%%\n",
                    bat_temp, valid, duty_cycle * 100 / 255);
        lastDebug = now;
    }
}
