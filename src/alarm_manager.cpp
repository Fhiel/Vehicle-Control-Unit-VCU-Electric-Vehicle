/**
 * @file alarm_manager.cpp
 * @brief Central intelligence for powertrain alerts and high-voltage safety monitoring.
 * @note Fully decoupled visual alarm tracking (Channel 10) and physical acoustic buzzer routing (Channel 1).
 * @author Frank Hielscher (X1/9e Project)
 * @license MIT
 */

#include "alarm_manager.h"
#include "main.h"

// Shared mute latch synchronized via incoming WebSocket "PIEZO_TOGGLE" command
volatile bool alarmPiezoMuted = false;
static uint16_t lastAlarmSignature = 0; // Tracks active faults to handle automated un-muting routines

/**
 * @brief Evaluates system-wide telemetry matrices to route and execute safety alarms.
 * @note Periodic execution within the main tracking loop or slow-task scheduler (~10Hz).
 */
void update_alarm_states() {
    bool check_oil_required = false;
    bool piezo_alarm_required = false;
    uint16_t current_alarm_signature = 0;

    // --- 1. THREAD-SAFE CONTEXT DATA ACQUISITION ---
    uint8_t  mcu_fault_level = 0;
    uint8_t  bms_status = 0;
    bool     bms_hw_fault = false;
    uint16_t imd_status = 0;
    bool     imd_test_failed = false;
    uint16_t vifc_status = 0;

    WITH_DATA_MUTEX({
        mcu_fault_level = telemetryData.mcuFaultLevel; 
        bms_status      = telemetryData.bmsStatus;
        bms_hw_fault    = telemetryData.bmsHardwareFault;
        imd_status      = telemetryData.imdStatus;
        imd_test_failed = telemetryData.selfTestFailed;
        vifc_status     = telemetryData.vifcStatus;
    });

    // =========================================================================
    // SYSTEM CRITICAL TRANSITION: GALVANIC CONTACTOR SAFE-STATE (BMS == READY)
    // =========================================================================
    if (bms_status == BMS_STATUS_READY) {
        piezo_alarm_required = false;
        check_oil_required = false;
        alarmPiezoMuted = false;
        lastAlarmSignature = 0;
    } 
    else {
        // =====================================================================
        // ROUTE 1: POWERTRAIN ALERT (Check OIL Master Control)
        // =====================================================================
        if (mcu_fault_level > 0 || bms_hw_fault || bms_status == BMS_STATUS_ERROR) {
            check_oil_required = true;
        }

        // =====================================================================
        // ROUTE 2: HIGH-VOLTAGE SAFETY ALERT (Insulation Warning System)
        // =====================================================================
        bool imd_isolation_error = (imd_status & (1 << 0)); // Bit 0: Isolation Error
        bool imd_chassis_fault    = (imd_status & (1 << 1)); // Bit 1: Chassis Fault
        bool imd_system_fault     = (imd_status & (1 << 2)); // Bit 2: System Fault
        
        if (imd_isolation_error || imd_chassis_fault || imd_system_fault || imd_test_failed) {
            piezo_alarm_required = true;
            current_alarm_signature |= (imd_status & 0x07); 
            if (imd_test_failed) current_alarm_signature |= (1 << 7);
        }

        if (vifc_status & 0x00FF) { 
            piezo_alarm_required = true;
            current_alarm_signature |= (vifc_status << 8);
        }

        // AUTO-UNMUTE LOOP
        if (piezo_alarm_required && (current_alarm_signature != lastAlarmSignature)) {
            alarmPiezoMuted = false; 
            lastAlarmSignature = current_alarm_signature;
        }
    }

    // =========================================================================
    // ROUTE 3: HARDWARE MAPPING LAYER WITH UNBIASED MANUAL OVERRIDE MATRIX
    // =========================================================================
    
    // --- 3.1 CHECK OIL DIGITAL PIN LOGIC ---
    if (manualOverride & (1UL << OVR_BIT_CHECK_OIL)) {
        bool manual_oil_state = (manualOverride & (1UL << 16)) ? true : false;
        digitalWrite(LED_CHECK_OIL_PIN, manual_oil_state ? HIGH : LOW);
    } else {
        digitalWrite(LED_CHECK_OIL_PIN, check_oil_required ? HIGH : LOW);
    }

    // --- 3.2 ACOUSTIC PIEZO HORN DIGITAL PIN LOGIC (CHANNEL 1) ---
    bool fire_buzzer = piezo_alarm_required && !alarmPiezoMuted;

    if (manualOverride & (1UL << OVR_BIT_PIEZO)) {
        // Handshake: Web-UI explicitly triggers the physical horn pin via Bit 17
        bool manual_piezo_state = (manualOverride & (1UL << 17)) ? true : false;
        digitalWrite(PIEZO_RELAY_PIN, manual_piezo_state ? HIGH : LOW);
    } else {
        // Automation: Horn follows safety loop unless muted by driver acknowledgement
        digitalWrite(PIEZO_RELAY_PIN, fire_buzzer ? HIGH : LOW);
    }

    // --- 3.3 VISUAL MASTER ALARM STATE OVERRIDE (CHANNEL 10) ---
    bool final_visual_alarm_state = piezo_alarm_required;

    if (manualOverride & (1UL << OVR_BIT_ALARM)) {
        // Handshake: Web-UI enforces manual visual lamp test via Bit 28
        final_visual_alarm_state = (manualOverride & (1UL << 28)) ? true : false;
    }

    // =========================================================================
    // ROUTE 4: DIAGNOSTIC RETRIEVAL & TELEMETRY STORAGE RECORDING
    // =========================================================================
    WITH_DATA_MUTEX({
        telemetryData.ledCheckOil = (digitalRead(LED_CHECK_OIL_PIN) == HIGH);
        telemetryData.isPiezoOn   = (digitalRead(PIEZO_RELAY_PIN) == HIGH);
        
        // The visual flag tracks the decoupled result (allowing manual dashboard light tests)
        telemetryData.isAlarm     = final_visual_alarm_state; 
    });
}