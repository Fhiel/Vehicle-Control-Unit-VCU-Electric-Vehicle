/**
 * @file web_server.cpp
 * @brief Web Interface and WebSocket handling for the VCU X1/9e.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
// CRITICAL: This flag tells ElegantOTA to use AsyncWebServer instead of WebServer
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
#include <ElegantOTA.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

#include "main.h"
#include "web_server.h"
#include "web_ui.h"
#include "CAN_Transmit.h"
#include "relay_control.h" 
#include "lock_control.h"  
#include "secrets.h"
#include "pwa_data.h"

// Global instances
AsyncWebServer server(80);
AsyncWebSocket webSocket("/ws");

// --- ArduinoJson V7 Optimized Buffer ---
static JsonDocument doc;
// Use PSRAM for the buffer on T-2CAN to save internal Heap
static char jsonBuffer[1536];

// Forward Declaration
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, 
               void *arg, uint8_t *data, size_t len);

/**
 * @brief Initializes WiFi (AP + Station), WebSocket and ElegantOTA.
 */
void initWebServer() {
    // 1. WiFi Mode Setup (Dual mode for car-internal and home network access)
    WiFi.mode(WIFI_AP_STA);
    WiFi.setHostname(DEVICE_HOSTNAME); 

    // 2. Start Access Point (Dashboard link for S10e inside the car)
    WiFi.softAP(AP_SSID, AP_PASS); 
    
    // 3. Connect to Home WiFi (Garage / Maintenance link)
    WiFi.begin(HOME_SSID, HOME_PASS);

    // 3.1 Start mDNS Responder (access via http://hostname.local)
    if (!MDNS.begin(DEVICE_HOSTNAME)) {
        safe_printf("[NET] Error setting up MDNS responder!\n");
    } else {
        safe_printf("[NET] mDNS started: http://%s.local\n", DEVICE_HOSTNAME);
        MDNS.addService("http", "tcp", 80);
    }
    
    Serial.println("[NET] WiFi Dual-Mode initialized.");

    // 4. WebSocket & Server Config
    webSocket.onEvent(onWsEvent);
    server.addHandler(&webSocket);

    // Root route delivering the web_ui.h content
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", index_html); 
    });

    // PWA Manifest route
    server.on("/manifest.json", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "application/json", manifest_json);
    });

    // Service Worker route (required for PWA installability/standalone mode)
    server.on("/sw.js", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "application/javascript", 
        "self.addEventListener('install', (e) => { self.skipWaiting(); }); "
        "self.addEventListener('fetch', (e) => { e.respondWith(fetch(e.request)); });");
    });

    // 5. ElegantOTA Setup (Secure Over-the-Air updates)
    ElegantOTA.begin(&server, OTA_USER, OTA_PASS);
    
    ElegantOTA.onStart([]() { Serial.println("[OTA] Critical: Update started!"); });
    ElegantOTA.onEnd([](bool success) { 
        if (success) Serial.println("[OTA] Update successful. Rebooting...");
        else Serial.println("[OTA] Update failed!");
    });

    // 6. Start Server
    server.begin();
    Serial.println("[NET] WebServer & OTA initialized");
}

/**
 * @brief Handles incoming WebSocket commands from the dashboard.
 */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, 
               void *arg, uint8_t *data, size_t len) {
    
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            
            data[len] = 0; 
            const char* cmd = (const char*)data;

            // --- RELAY CONTROLS (Common to all Hardware) ---
            if      (strcmp(cmd, "REL1_ON") == 0)   setRelayManual(1, true);
            else if (strcmp(cmd, "REL1_OFF") == 0)  setRelayManual(1, false);
            else if (strcmp(cmd, "REL1_AUTO") == 0) releaseToAuto(1);

            else if (strcmp(cmd, "REL2_ON") == 0)   setRelayManual(2, true);
            else if (strcmp(cmd, "REL2_OFF") == 0)  setRelayManual(2, false);
            else if (strcmp(cmd, "REL2_AUTO") == 0) releaseToAuto(2);

            else if (strcmp(cmd, "REL3_ON") == 0)   setRelayManual(3, true);
            else if (strcmp(cmd, "REL3_OFF") == 0)  setRelayManual(3, false);
            else if (strcmp(cmd, "REL3_AUTO") == 0) releaseToAuto(3);

            else if (strcmp(cmd, "REL4_ON") == 0)   setRelayManual(4, true);
            else if (strcmp(cmd, "REL4_OFF") == 0)  setRelayManual(4, false);
            else if (strcmp(cmd, "REL4_AUTO") == 0) releaseToAuto(4);

            // --- CHARGE MODES ---
            else if (strcmp(cmd, "MODE_DAILY") == 0) dailyModeActive = true;
            else if (strcmp(cmd, "MODE_TRIP") == 0)  dailyModeActive = false;
            // ELCON Charger commands are sent via CANB (TWAI) - works on both boards!
            else if (strcmp(cmd, "CHARGE_STOP_REQ") == 0) sendElconCommand(true);

            // --- LOCKING & DIAGNOSTICS ---
            // These interact with the state machine in loop()
            else if (strcmp(cmd, "LOCK_REQ") == 0)   WITH_DATA_MUTEX({ telemetryData.shouldLock = true; });
            else if (strcmp(cmd, "UNLOCK_REQ") == 0) manualUnlockPressed = true;
            else if (strcmp(cmd, "IMD_TEST_START") == 0) WITH_DATA_MUTEX({ telemetryData.selfTestRequested = true; });
            else if (strcmp(cmd, "DEMO_TOGGLE") == 0) demoModeActive = !demoModeActive;
        }
    }
}

/**
 * @brief Telemetry Broadcast to Browser (ArduinoJson V7 compatible)
 */
void updateWebDashboard() {
    if (webSocket.count() == 0) return;
    doc.clear();

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Group 1: MCU
        JsonObject mcu = doc["mcu"].to<JsonObject>();
        mcu["rpm"]   = (int32_t)telemetryData.motorRPM;
        mcu["rpmV"]  = (bool)telemetryData.motorRPMValid; // Valid Flag
        mcu["mt"]    = (int)telemetryData.motor_temp;
        mcu["mtV"]   = (bool)telemetryData.motorTempValid;
        mcu["it"]    = (int)telemetryData.mcu_temp;
        mcu["itV"]   = (bool)telemetryData.mcuTempValid;
        mcu["flt"]   = (int)telemetryData.mcuFaultLevel;

        // Group 2: BMS
        JsonObject bms = doc["bms"].to<JsonObject>();
        bms["soc"]   = (int)telemetryData.bmsSoC;
        bms["socV"]  = (bool)telemetryData.bmsSoCValid;
        bms["a"]     = (float)telemetryData.bmsCurrent;
        bms["aV"]    = (bool)telemetryData.bmsCurrentValid;
        bms["v"]     = (int)telemetryData.hv1Voltage;
        bms["vV"]    = (bool)telemetryData.hv1VoltageValid;

        // Group 3: IMD
        JsonObject imd = doc["imd"].to<JsonObject>();
        imd["r"]     = (int)telemetryData.imdIsoR;
        imd["rV"]    = (bool)telemetryData.imdIsoRValid;
        imd["st"]    = telemetryData.selfTestRunning ? 3 : 1; 

        // Group 4: VCU
        JsonObject vcu = doc["vcu"].to<JsonObject>();
        vcu["range"] = (int)estimatedRange;      
        vcu["trip"]  = !dailyModeActive;        
        vcu["relO"]  = (int)relayShadow;
        vcu["mOv"]   = (int)manualOverride;
        vcu["batP"] = (int)telemetryData.bat_pump_pwm;
        vcu["invP"] = (int)telemetryData.inv_pump_pwm;
        vcu["chg"]   = (bool)telemetryData.is_charging;
        vcu["unl"]   = (bool)telemetryData.isUnLocking;
        vcu["err"]   = (bool)telemetryData.selfTestFailed;
        vcu["run"]   = (bool)telemetryData.selfTestRunning;
        vcu["soc"]   = (int)telemetryData.bmsSoC;

        // Group 5: Proxy BMS (Hyper9 Interface)
        JsonObject proxy = doc["proxy"].to<JsonObject>();
        bool cableConnected = (telemetryData.bmsStatus == 0x04 || telemetryData.is_charging);
        bool systemFault = (telemetryData.selfTestResult != 0 || telemetryData.bmsHardwareFault);
        bool driveInhibit = (cableConnected || telemetryData.isLocked || systemFault);

        proxy["soc"] = (int)((telemetryData.bmsSoC / 100.0f) * 32768.0f); // Raw SoC for Hyper9
        proxy["inh"] = (bool)driveInhibit; // Drive Inhibit Bit
        proxy["lim"] = (telemetryData.bmsHighTempWarn) ? 50 : (telemetryData.bmsLowVoltageWarn ? 40 : 100);
        proxy["flt"] = (bool)systemFault;

        xSemaphoreGive(dataMutex);
    }

    #ifdef HARDWARE_T2CAN
    // For T-2CAN (PSRAM pointer): Explicitly pass the pointer and the buffer size
        size_t len = serializeJson(doc, jsonBuffer, 2048); 
    #else
        // For TCAN485 (Fixed array): The compiler knows the size automatically
        size_t len = serializeJson(doc, jsonBuffer);
    #endif
    webSocket.textAll(jsonBuffer, len);
}