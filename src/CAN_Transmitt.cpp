/**
 * @file CAN_Transmit.cpp
 * @brief Outgoing CAN Manager (Hyper9 Proxy, Elcon Charger, Relay Control).
 * @note Fixed multithreading data leaks and optimized charging sequence lockouts.
 * @author Frank Hielscher (X1/9e Project)
 * @license MIT
 */

#include "CAN_Transmit.h"
#include "main.h"
#include "relay_control.h" 
#include <driver/twai.h>

/* --- Global State Variables --- */
bool dailyModeActive = true;       // Default: 80% SoC Cap
uint16_t estimatedRange = 0;       // Remaining distance in km

/* --- Local Constants --- */
static const uint32_t ELCON_CHARGER_ID = 0x1806E5F4; 
static const uint32_t HYPER9_PROXY_ID  = 0x244;      
static const unsigned long TX_INTERVAL_MS = 500;     

#define CHARGER_MAX_V_HEX 0x04E2 // 125.0V
#define CHARGER_MAX_A_HEX 0x0096 // 15.0A

void updateRangeEstimation() {
    // Math is computed locally within the interval to keep mutex hold times minimal
    float currentSoC = 0.0f;
    bool socValid = false;

    WITH_DATA_MUTEX({
        currentSoC = telemetryData.bmsSoC;
        socValid = telemetryData.bmsSoCValid;
    });

    if (socValid && currentSoC > 0) {
        float currentEnergyKWh = BATT_CAPACITY_KWH * (currentSoC / 100.0f);
        estimatedRange = (uint16_t)((currentEnergyKWh / AVG_CONSUMPTION_KWH_100KM) * 100.0f);
    } else {
        estimatedRange = 0; 
    }
}

void send_proxy_bms_data() {
    if (demoModeActive) return;

    uint16_t soc_hyper9 = 0;
    int16_t current_da = 0;
    uint8_t status = 0;
    uint8_t limit_percent = 100;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        soc_hyper9 = (uint16_t)((telemetryData.bmsSoC / 100.0f) * 32768.0f);
        current_da = (int16_t)(telemetryData.bmsCurrent * 10.0f);
        
        if (telemetryData.bmsHighTempWarn) limit_percent = 50;
        else if (telemetryData.bmsLowVoltageWarn) limit_percent = 40;

        // --- DRIVE INHIBIT (INTERLOCK) LOGIC ---
        // Bit 5 set = Motor Disabled / Inverter Interlock active
        bool cableConnected = (telemetryData.bmsStatus == 0x04 || telemetryData.isCharging);
        bool systemFault = (telemetryData.selfTestResult != 0 || telemetryData.bmsHardwareFault);
        
        if (cableConnected || telemetryData.isLocked || systemFault) {
            status |= (1 << 5); // DRIVE INHIBIT ACTIVATED
        } else if (limit_percent < 100) {
            status |= (1 << 1); // LIMIT MODE
        } else {
            status |= (1 << 2); // NORMAL MODE
        }

        if (telemetryData.isCharging) status |= (1 << 3);
        
        xSemaphoreGive(dataMutex);
    }

    twai_message_t msg = {0};
    msg.identifier = HYPER9_PROXY_ID;
    msg.data_length_code = 8;
    msg.data[0] = (uint8_t)(soc_hyper9 >> 8);
    msg.data[1] = (uint8_t)(soc_hyper9 & 0xFF);
    msg.data[2] = (uint8_t)(current_da >> 8);
    msg.data[3] = (uint8_t)(current_da & 0xFF);
    msg.data[4] = status;
    msg.data[5] = limit_percent;
    twai_transmit(&msg, pdMS_TO_TICKS(5));
}

void sendElconCommand(bool stopCharging) {
    twai_message_t msg = {0};
    msg.identifier = ELCON_CHARGER_ID;
    msg.extd = 1; 
    msg.data_length_code = 8;

    msg.data[0] = (uint8_t)(CHARGER_MAX_V_HEX >> 8);
    msg.data[1] = (uint8_t)(CHARGER_MAX_V_HEX & 0xFF);
    msg.data[2] = (uint8_t)(CHARGER_MAX_A_HEX >> 8);
    msg.data[3] = (uint8_t)(CHARGER_MAX_A_HEX & 0xFF);
    
    // Safety: Byte 4 = 0x00 for Charge, 0x01 for Stop
    msg.data[4] = stopCharging ? 0x01 : 0x00; 

    twai_transmit(&msg, pdMS_TO_TICKS(5));
}

void CAN_Transmit_Task() {
    unsigned long now = millis();

    // =========================================================================
    // 1. PERIODIC TRANSMISSION ENGINE (2Hz / 500ms Loop)
    // =========================================================================
    static unsigned long lastUpdate = 0;
    if (now - lastUpdate >= TX_INTERVAL_MS) {
        lastUpdate = now;

        // Local snapshot variables for thread-safe isolation
        float currentSoC = 0.0f;
        bool hardwareLocked = false;
        bool bmsWantsCharge = false;
        bool manualStopActive = false;

        // --- THREAD-SAFE SNAPSHOT ---
        WITH_DATA_MUTEX({
            currentSoC       = telemetryData.bmsSoC;
            hardwareLocked   = telemetryData.isLocked;
            bmsWantsCharge   = telemetryData.bmsChargeAllowed;
            manualStopActive = telemetryData.manualStopRequested;
        });

        // Compute range metrics based on the fresh data snapshot
        updateRangeEstimation();
        
        // 1. Dispatch proxy matrix to Netgain Hyper9 controller (keeps interlock synchronized)
        send_proxy_bms_data(); 

        // 2. Evaluate Safeties & Charging Boundaries
        bool socLimitReached = (dailyModeActive && currentSoC >= 80.0f);

        // The Charger must immediately request a ramp-down/stop if:
        // - Custom 80% longevity cap is achieved in daily mode
        // - OR the TeslaBMS drops out of charge ready state (cell protection)
        // - OR the operator clicks the Web-UI manual stop command
        bool chargerStopReq = socLimitReached || !bmsWantsCharge || manualStopActive;

        // HARDWARE INTERLOCK RECOVERY: We ONLY communicate with the Elcon hardware
        // if the Type 2 connector pin is physical, structurally CLOSED and LOCKED.
        // This completely prevents pre-lock CAN command noise.
        if (hardwareLocked) {
            sendElconCommand(chargerStopReq);
        }
    }
}