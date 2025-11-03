// led_control.cpp
#define DEBUG
#include "led_control.h"
#include "main.h"

void update_led() {
    // --- 1. Daten mit Mutex lesen ---
    bool is_locked = false;
    bool is_unlocking = false;
    bool is_charging = false;
    bool bms_status_ok = false;
    bool self_test_failed = false;
    bool self_test_running = false;

    WITH_DATA_MUTEX({
        is_locked = telemetryData.isLocked;
        is_unlocking = telemetryData.isUnLocking;
        is_charging = telemetryData.is_charging;
        bms_status_ok = telemetryData.bmsStatusValid && telemetryData.bmsStatus == 3;
        self_test_failed = telemetryData.selfTestFailed;
        self_test_running = telemetryData.selfTestRunning;
    });

    static CRGB current_color = CRGB::Black;
    static unsigned long blink_start = 0;
    static bool blink_on = false;
    static bool last_self_test_failed = false;
    CRGB target_color = CRGB::Black;

    // --- 2. SELBSTTEST FEHLER: STARKER ROTER BLITZ (50ms AN / 150ms AUS) ---
    if (self_test_failed) {
        unsigned long now = millis();
        unsigned long cycle_time = now - blink_start;

        // Neuer Fehler? → Blitz starten
        if (!last_self_test_failed) {
            blink_start = now;
            blink_on = true;
            FastLED.setBrightness(255);  // MAX HELLIGKEIT
            leds[0] = CRGB::Red;
            FastLED.show();
            last_self_test_failed = true;
            return;
        }

        // Blitz-Zyklus: 50ms AN → 150ms AUS
        if (blink_on && cycle_time >= 50) {
            blink_on = false;
            blink_start = now;
            leds[0] = CRGB::Black;
            FastLED.show();
        }
        else if (!blink_on && cycle_time >= 1000) {
            blink_on = true;
            blink_start = now;
            leds[0] = CRGB::Red;
            FastLED.show();
        }
        return;  // Blitz hat höchste Priorität
    }
    else {
        // Fehler vorbei → Reset + Helligkeit zurück
        if (last_self_test_failed) {
            FastLED.setBrightness(20);  // Normale Helligkeit
            last_self_test_failed = false;
            blink_on = false;
        }
    }

    // --- 3. NORMALE ZUSTÄNDE (nur wenn KEIN Fehler) ---
    if (self_test_running) {
        target_color = CRGB::Yellow;  // Selftest läuft
    }
    else if (is_unlocking || (!is_locked && is_charging)) {
        target_color = CRGB::Red;     // Entriegeln oder Laden ohne Verriegelung
    }
    else if (bms_status_ok) {
        target_color = is_charging ? CRGB::Blue : CRGB::Green;
    }
    else {
        target_color = CRGB::Yellow;  // BMS nicht OK
    }

    // --- 4. Nur updaten, wenn sich Farbe ändert ---
    if (target_color.r != current_color.r ||
        target_color.g != current_color.g ||
        target_color.b != current_color.b) {
        current_color = target_color;
        FastLED.setBrightness(20);  // Normale Helligkeit
        leds[0] = current_color;
        FastLED.show();
    }

    // --- 5. Debug alle 5 Sekunden ---
    #ifdef DEBUG
    static unsigned long last_print = 0;
    if (millis() - last_print >= 5000) {
        const char* state = self_test_failed ? "BLITZ ROT (Selbsttest Fehler!)" :
                           self_test_running ? "Gelb (Selftest läuft)" :
                           is_unlocking ? "Rot (Entriegeln)" :
                           (!is_locked && is_charging) ? "Rot (Laden ohne Lock)" :
                           is_charging ? "Blau (Laden)" :
                           bms_status_ok ? "Grün (BMS OK)" : "Gelb (BMS nicht OK)";
        safe_printf("LED: %s\n", state);
        last_print = millis();
    }
    #endif
}