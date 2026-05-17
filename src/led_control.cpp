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
    bool is_bms_ready = false; // Name korrigiert für echte Logik
    bool self_test_failed = false;
    bool self_test_running = false;
    float bms_soc = 0.0f;

    // 1. Thread-safe data acquisition (Unter Verwendung der neuen GitHub-Defines)
    WITH_DATA_MUTEX({
        is_locked = telemetryData.isLocked;
        is_unlocking = telemetryData.isUnLocking;
        is_charging = telemetryData.is_charging;
        
        // KORREKTUR: Nutzt jetzt den echten READY-Wert (1) statt CHARGE (3)
        is_bms_ready = (telemetryData.bmsStatusValid && telemetryData.bmsStatus == BMS_STATUS_READY);
        
        self_test_failed = telemetryData.selfTestFailed;
        self_test_running = telemetryData.selfTestRunning;
        bms_soc = telemetryData.bmsSoC;
    });

    unsigned long now = millis();
    uint8_t r = 0, g = 0, b = 0, brightness = 0;
    
    // --- 2. CRITICAL ERROR: IMD FAILURE (Red Strobe) ---
    if (self_test_failed) {
        static unsigned long strobe_timer = 0;
        static bool strobe_on = true;
        if (now - strobe_timer >= (strobe_on ? 50 : 800)) {
            strobe_on = !strobe_on;
            strobe_timer = now;
        }
        if (strobe_on) { r = 255; brightness = 255; }
    }
    // --- 3. STATE MACHINE (Tesla-Style States) ---
    else if (self_test_running) {
        r = 255; g = 255; brightness = 128; // Gelb (Messung läuft)
    } 
    else if (is_unlocking || (!is_locked && is_charging)) {
        r = 255; brightness = 255; // Rot Warnung (Stecker steckt, ist aber ungesteuert offen!)
    } 
    else if (is_charging) {
        // Tesla-Style: Blau bei erreichtem Limit, sonst grün pulsieren
        if (bms_soc >= 80.0f && dailyModeActive) {
            b = 255; brightness = 60; // Blau (Limit im Daily-Modus erreicht)
        } else {
            // Sinus-Pulsieren für aktives Laden
            float intensity = (sin(now / 500.0) + 1.0) / 2.0; // 0.0 bis 1.0
            g = 255; brightness = 30 + (uint8_t)(intensity * 225);
        }
    } 
    else if (is_locked) {
        // Kabel ist verriegelt, aber es fließt kein Ladestrom mehr
        if (is_bms_ready) { 
            g = 255; brightness = 60; // Dauergrün: Sicher verriegelt und fahrbereit
        } else { 
            r = 255; g = 255; brightness = 40; // Orange/Gelb: Verriegelt, wartet (z.B. Standby)
        }
    }

    // LED ansteuern
    led.setBrightness(brightness);
    led.setLedColorData(0, r, g, b);
    led.show();

    // Feedback an das Telemetrie-Struct für die Web-UI (Expert-Seite) übergeben
    // So weiß auch das Smartphone, welche Farbe die LED gerade anzeigt!
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (self_test_failed) telemetryData.ws2812Status = 2;       // Red/Error
        else if (is_charging) telemetryData.ws2812Status = 1;      // Green/Charging
        else if (is_locked && is_bms_ready) telemetryData.ws2812Status = 1; // Green/Ready
        else telemetryData.ws2812Status = 0;                       // Off/Idle
        xSemaphoreGive(dataMutex);
    }

    #ifdef DEBUG
    static unsigned long last_print = 0;
    if (now - last_print >= 5000) {
        safe_printf("[LED] State: %s | BMS-Status: %d | SoC: %.1f%%\n", 
            self_test_failed ? "FAIL" : (is_charging ? "CHARGING" : "READY"), 
            telemetryData.bmsStatus, bms_soc);
        last_print = now;
    }
    #endif
}