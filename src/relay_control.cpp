/**
 * @file relay_control.cpp
 * @brief Logic for Direct Component Management (Hut V3 Shield Architecture).
 * @author Fhiel (X1/9e Project)
 * @license MIT
 *
 * @note Legacy CAN-Remote codes and old relay Shadows have been completely removed.
 * Automations now write directly to their dedicated GPIO pins and telemetry.
 */

#include "relay_control.h"
#include "main.h"

/* --- Global Manual Override Mask --- */
// Bit 0: Check Oil | Bit 1: Piezo | Bit 2: Fan | Bit 3: Bat Pump | Bit 4: Inv Pump | Bit 5-8: Aux 11-14
uint16_t manualOverride = 0x0000; 

static bool buzzerMuted = false;
static unsigned long afterrunStartTime = 0;
static unsigned long lastBuzzerToggle = 0;
static bool buzzerState = false;

/**
 * @brief Syncs all physical GPIO states back into the Telemetry struct for the Web UI.
 */
void syncHardwareToTelemetry() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        telemetryData.ledCheckOil   = digitalRead(LED_CHECK_OIL_PIN);
        telemetryData.ledBattery    = digitalRead(LED_BATTERY_PIN);
        telemetryData.isPiezoOn       = digitalRead(PIEZO_PIN);
        telemetryData.fanRelay      = digitalRead(FAN_RELAY_PIN);
        telemetryData.batPumpRelay  = digitalRead(BAT_PUMP_RELAY_PIN);
        telemetryData.invPumpRelay  = digitalRead(INV_PUMP_RELAY_PIN);
        
        telemetryData.auxRelay11    = digitalRead(RELAY_11_PIN);
        telemetryData.auxRelay12    = digitalRead(RELAY_12_PIN);
        telemetryData.auxRelay13    = digitalRead(RELAY_13_PIN);
        telemetryData.auxRelay14    = digitalRead(RELAY_14_PIN);
        
        telemetryData.auxinput13    = digitalRead(INPUT_13_PIN);
        xSemaphoreGive(dataMutex);
    }
}

/* ================================================================
   ===             WEB INTERFACE INTERACTION CONTROLS           ===
   ================================================================ */

void toggleOutput(uint8_t pin) {
    if (pin == 0) return;

    // Toggle the corresponding bit in the manual override mask
    if (pin == LED_CHECK_OIL_PIN) manualOverride |= (1 << 0);
    else if (pin == PIEZO_PIN)    manualOverride |= (1 << 1);
    else if (pin == FAN_RELAY_PIN) manualOverride |= (1 << 2);
    else if (pin == BAT_PUMP_RELAY_PIN) manualOverride |= (1 << 3);
    else if (pin == INV_PUMP_RELAY_PIN) manualOverride |= (1 << 4);
    else if (pin == RELAY_11_PIN) manualOverride |= (1 << 5);
    else if (pin == RELAY_12_PIN) manualOverride |= (1 << 6);
    else if (pin == RELAY_13_PIN) manualOverride |= (1 << 7);
    else if (pin == RELAY_14_PIN) manualOverride |= (1 << 8);
    else if (pin == LED_BATTERY_PIN)    manualOverride |= (1 << 9);

    bool current = digitalRead(pin);
    digitalWrite(pin, !current);

    syncHardwareToTelemetry();
}

void setOutput(uint8_t pin, bool state) {
    if (pin == 0) return;
    
    // If the command is to set the output to LOW, we clear the manual override bit to allow automation to take over again
    if (!state) {
        if (pin == LED_CHECK_OIL_PIN) manualOverride &= ~(1 << 0);
        else if (pin == PIEZO_PIN)    manualOverride &= ~(1 << 1);
        else if (pin == FAN_RELAY_PIN) manualOverride &= ~(1 << 2);
        else if (pin == BAT_PUMP_RELAY_PIN) manualOverride &= ~(1 << 3);
        else if (pin == INV_PUMP_RELAY_PIN) manualOverride &= ~(1 << 4);
        else if (pin == RELAY_11_PIN) manualOverride &= ~(1 << 5);
        else if (pin == RELAY_12_PIN) manualOverride &= ~(1 << 6);
        else if (pin == RELAY_13_PIN) manualOverride &= ~(1 << 7);
        else if (pin == RELAY_14_PIN) manualOverride &= ~(1 << 8);
        else if (pin == LED_BATTERY_PIN)    manualOverride &= ~(1 << 9);
        return; 
    }

    digitalWrite(pin, state);
    syncHardwareToTelemetry();
}

/* ================================================================
   ===                  AUTOMATION ENGINE                       ===
   ================================================================ */

void updateRelayAutomation() {
    unsigned long now = millis();
    
    bool stRunning, isCharging, ignition;
    uint8_t stResult, pInv, pBat;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        stRunning  = telemetryData.selfTestRunning;
        stResult   = telemetryData.selfTestResult;
        isCharging = telemetryData.is_charging;
        pInv       = telemetryData.inv_pump_pwm;
        pBat       = telemetryData.bat_pump_pwm;
        xSemaphoreGive(dataMutex);
    } else return; 

    // --- 1. COCKPIT LED: CHECK OIL ---
    if (!(manualOverride & (1 << 0))) { 
        // AUTOMATION (Nur wenn kein Web-Override aktiv ist)
        digitalWrite(LED_CHECK_OIL_PIN, (stRunning || stResult != 0) ? HIGH : LOW);
    }

    // --- 2. PIEZO BUZZER LOGIC (Erweitert für Schreibtisch-Test) ---
    // Ein Alarm ist aktiv, wenn der IMD einen Fehler meldet ODER wir das Bit manuell gesetzt haben
    bool muteActive = (manualOverride & (1 << 1));
    bool alarmActive = (stResult != 0 || muteActive); 

    if (alarmActive) {
        // Das Icon leuchtet im JSON (und damit auf der Hauptseite) IMMER rot,
        // sobald der Alarm aktiv ist!
        telemetryData.isPiezoOn = true;

        if (muteActive) {
            // Wenn das Bit gesetzt ist -> Physikalischer Krachmacher bleibt AUS (Stummgeschaltet)
            digitalWrite(PIEZO_PIN, LOW);
        } else {
            // Wenn der Fehler echt ist, aber noch nicht stummgeschaltet -> Piezo taktet rhythmisch
            if (now - lastBuzzerToggle >= 500) {
                buzzerState = !buzzerState;
                lastBuzzerToggle = now;
            }
            digitalWrite(PIEZO_PIN, buzzerState ? HIGH : LOW);
        }
    } else {
        // Alles ruhig
        telemetryData.isPiezoOn = false;
        digitalWrite(PIEZO_PIN, LOW);
    }

    // --- 3. INVERTER LÜFTER (FAN) ---
    if (!(manualOverride & (1 << 2))) {
        digitalWrite(FAN_RELAY_PIN, (pInv > 191) ? HIGH : LOW);
    }

    // --- 4. COOLANT PUMPS (Getrennte Automation ohne die alte ignition-Variable) ---
    bool runAutomationPumps = false;
    
    if (isCharging) {
        runAutomationPumps = true;
        afterrunStartTime = 0; // Reset Nachlauf-Timer während des Ladens
    } else {
        // Wenn nicht geladen wird, prüfen wir den temperaturabhängigen Nachlauf
        if (afterrunStartTime == 0 && (pInv > 64 || pBat > 64)) {
            afterrunStartTime = now;
        }
        // 5 Minuten (300.000 ms) Nachlaufzeit gewähren, falls der Timer läuft
        if (afterrunStartTime > 0 && (now - afterrunStartTime < 300000)) { 
            runAutomationPumps = true;
        } else if (afterrunStartTime > 0) {
            afterrunStartTime = 0; // Timer abgelaufen -> Reset
        }
    }

    // Nur zuweisen, wenn KEIN manuelles Override aktiv ist
    if (!(manualOverride & (1 << 3))) {
        digitalWrite(BAT_PUMP_RELAY_PIN, runAutomationPumps ? HIGH : LOW);
    }
    if (!(manualOverride & (1 << 4))) {
        digitalWrite(INV_PUMP_RELAY_PIN, runAutomationPumps ? HIGH : LOW);
    }

    // --- 5. COCKPIT LED: BATTERY ---
    // Hier nutzen wir jetzt ein EIGENES Bit (z.B. Bit 9), damit das Schalten der Batterie-LED 
    // nicht mit dem Piezo-Buzzer (Bit 1) kollidiert!
    if (!(manualOverride & (1 << 9))) { 
        if (ignition) {
            if (telemetryData.bmsStatus == BMS_STATUS_DRIVE || telemetryData.bmsStatus == BMS_STATUS_CHARGE) { 
                digitalWrite(LED_BATTERY_PIN, LOW);  
            } else {
                digitalWrite(LED_BATTERY_PIN, HIGH); 
            }
        } else {
            digitalWrite(LED_BATTERY_PIN, LOW);
        }
    }

    // Am Ende alle Pin-Zustände einsammeln und ins JSON-Struct spiegeln
    syncHardwareToTelemetry();
}