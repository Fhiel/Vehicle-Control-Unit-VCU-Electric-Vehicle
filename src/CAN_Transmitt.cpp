/**
 * @file CAN_Transmit.cpp
 * @brief Outgoing CAN Manager (Hyper9 Proxy, Elcon Charger, Relay Control).
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
    if (telemetryData.bmsSoCValid && telemetryData.bmsSoC > 0) {
        float currentEnergyKWh = BATT_CAPACITY_KWH * (telemetryData.bmsSoC / 100.0f);
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
        // Bit 5 set = Motor Disabled
        bool cableConnected = (telemetryData.bmsStatus == 0x04 || telemetryData.is_charging);
        bool systemFault = (telemetryData.selfTestResult != 0 || telemetryData.bmsHardwareFault);
        
        if (cableConnected || telemetryData.isLocked || systemFault) {
            status |= (1 << 5); // DRIVE INHIBIT
        } else if (limit_percent < 100) {
            status |= (1 << 1); // LIMIT MODE
        } else {
            status |= (1 << 2); // NORMAL MODE
        }

        if (telemetryData.is_charging) status |= (1 << 3);
        
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
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();

    // 1. SAFETY CHECKS
    bool socLimitReached = (dailyModeActive && telemetryData.bmsSoC >= 80);
    
    // CRITICAL SAFETY: If cable is detected but NOT locked, force Charger STOP!
    bool chargerStopReq = socLimitReached || !telemetryData.isLocked;

    // 2. PERIODIC TRANSMISSION (2Hz)
    if (now - lastUpdate >= TX_INTERVAL_MS) {
        lastUpdate = now;

        updateRangeEstimation();
        send_proxy_bms_data();
        sendRelayCommand(); 
        
        // Charger heartbeat
        if (telemetryData.is_charging || chargerStopReq) {
            sendElconCommand(chargerStopReq);
        }
    }
}