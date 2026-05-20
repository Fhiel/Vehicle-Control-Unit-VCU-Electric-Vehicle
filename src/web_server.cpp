/**
 * @file web_server.cpp
 * @brief Web Interface and WebSocket handling for the VCU X1/9e.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>     // Erst diese
#include <ElegantOTA.h>            // Dann ElegantOTA
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// CRITICAL: ElegantOTA soll AsyncWebServer benutzen
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1

#include "main.h"
#include "web_server.h"
#include "web_ui.h"
#include "CAN_Transmit.h"
#include "relay_control.h" 
#include "lock_control.h"  
#include "secrets.h"

#include <LittleFS.h>
//#include "pwa_data.h"  replaced by manifest.json in LittleFS for better maintainability and separation of concerns

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

    // Load the main dashboard page from LittleFS (index.html)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html"); 
    });

    // Load Logo from LittleFS (logo.png)
    server.on("/logo.png", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/logo.png", "image/png");
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(LittleFS, "/style.css", "text/css"); });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(LittleFS, "/script.js", "application/javascript"); });

    // Load Manifest (PWA Metadata) from LittleFS (manifest.json)
    server.on("/manifest.json", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/manifest.json", "application/json");
    });

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(204); 
    });

    server.on("/favicon.png", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/favicon.png", "image/png");
    });

    // Service Worker route for PWA offline capabilities
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

    Serial.println("[NET] Waiting for network interfaces to stabilize...");
    delay(1000); // 1 second delay to allow WiFi to initialize and mDNS to propagate

    // 6. Start Server
    server.begin();
    Serial.println("[NET] WebServer & OTA initialized");

    // --- IP Addresses  ---
    safe_printf("\n--- VCU NETWORKS ---\n");
    safe_printf("  [AP]  S10e Dashboard IP: %s\n", WiFi.softAPIP().toString().c_str());
    
    uint32_t startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 4000) {
        delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
        safe_printf("  [STA] Garage/Home IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        safe_printf("  [STA] Garage/Home: Connecting... (IP follows when ready)\n");
    }
    safe_printf("---------------------\n\n");
}

/**
 * @brief Main WebSocket event routing dispatcher
 * @note Handles incoming asynchronous control frames from client interfaces
 */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
    
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client connection established from #%u\n", client->id());
        updateWebDashboard();
    } 
    else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client connection severed from #%u\n", client->id());
    } 
    else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            
            char cmd[64];
            size_t copyLen = (len < 63) ? len : 63;
            memcpy(cmd, data, copyLen);
            cmd[copyLen] = '\0'; 

            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                
                // --- LABOR SIMULATION ROUTINES ---
                if (strcmp(cmd, "DEMO_MODE_ON") == 0) {
                    demoModeActive = true;
                    safe_printf("[WEBSERVER] Labor Dial-Demo view lock forced via dashboard.\n");
                }
                else if (strcmp(cmd, "DEMO_MODE_OFF") == 0) {
                    demoModeActive = false;
                    safe_printf("[WEBSERVER] Labor Dial-Demo view released.\n");
                }
                
                // --- TRIP MANAGEMENT & DRIVE PROFILES ---
                else if (strcmp(cmd, "MODE_DAILY") == 0) {
                    dailyModeActive = true;
                }
                else if (strcmp(cmd, "MODE_TRIP") == 0) {
                    dailyModeActive = false;
                }
                else if (strcmp(cmd, "CHARGE_STOP_REQ") == 0) {
                    manualUnlockPressed = true; 
                    safe_printf("[WEBSERVER] Charging cooldown sequence forced via dashboard interface.\n");
                }
                
                // --- HARDWARE ACOUSTIC MUTING ---
                else if (strcmp(cmd, "PIEZO_TOGGLE") == 0) {
                    if (telemetryData.isAlarm) {
                        alarmPiezoMuted = true;
                        safe_printf("[NET] Alarm acoustics acknowledged by operator. Output muted.\n");
                    }
                }
                
                // --- DIGITAL OVERRIDES (MANUAL VS AUTOMATION TWIN-REGISTERS) ---
                else if (strcmp(cmd, "OIL_MODE_MANUAL") == 0)   { manualOverride |= (1UL << OVR_BIT_CHECK_OIL); }
                else if (strcmp(cmd, "OIL_MODE_AUTO") == 0)     { manualOverride &= ~(1UL << OVR_BIT_CHECK_OIL); }
                else if (strcmp(cmd, "OIL_CMD_ON") == 0)        { manualOverride_Values |= (1UL << OVR_BIT_CHECK_OIL); }
                else if (strcmp(cmd, "OIL_CMD_OFF") == 0)       { manualOverride_Values &= ~(1UL << OVR_BIT_CHECK_OIL); }

                else if (strcmp(cmd, "FAN_MODE_MANUAL") == 0)   { manualOverride |= (1UL << OVR_BIT_FAN); }
                else if (strcmp(cmd, "FAN_MODE_AUTO") == 0)     { manualOverride &= ~(1UL << OVR_BIT_FAN); }
                else if (strcmp(cmd, "FAN_CMD_ON") == 0)        { manualOverride_Values |= (1UL << OVR_BIT_FAN); }
                else if (strcmp(cmd, "FAN_CMD_OFF") == 0)       { manualOverride_Values &= ~(1UL << OVR_BIT_FAN); }

                else if (strcmp(cmd, "BAT_MODE_MANUAL") == 0)   { manualOverride |= (1UL << OVR_BIT_LED_BATT); }
                else if (strcmp(cmd, "BAT_MODE_AUTO") == 0)     { manualOverride &= ~(1UL << OVR_BIT_LED_BATT); }
                else if (strcmp(cmd, "BAT_CMD_ON") == 0)        { manualOverride_Values |= (1UL << OVR_BIT_LED_BATT); }
                else if (strcmp(cmd, "BAT_CMD_OFF") == 0)       { manualOverride_Values &= ~(1UL << OVR_BIT_LED_BATT); }

                else if (strcmp(cmd, "PIEZO_MODE_MANUAL") == 0) { manualOverride |= (1UL << OVR_BIT_PIEZO); }
                else if (strcmp(cmd, "PIEZO_MODE_AUTO") == 0)   { manualOverride &= ~(1UL << OVR_BIT_PIEZO); }
                else if (strcmp(cmd, "PIEZO_CMD_ON") == 0)      { manualOverride_Values |= (1UL << OVR_BIT_PIEZO); }
                else if (strcmp(cmd, "PIEZO_CMD_OFF") == 0)     { manualOverride_Values &= ~(1UL << OVR_BIT_PIEZO); }

                else if (strcmp(cmd, "ALARM_MODE_MANUAL") == 0){ manualOverride |= (1UL << OVR_BIT_ALARM); }
                else if (strcmp(cmd, "ALARM_MODE_AUTO") == 0)  { manualOverride &= ~(1UL << OVR_BIT_ALARM); }
                else if (strcmp(cmd, "ALARM_CMD_ON") == 0)     { manualOverride_Values |= (1UL << OVR_BIT_ALARM); }
                else if (strcmp(cmd, "ALARM_CMD_OFF") == 0)    { manualOverride_Values &= ~(1UL << OVR_BIT_ALARM); }

                // --- COOLING LOOPS OPERATION MANAGEMENT (RELIABLE ANALOG VALUE TRACKING) ---
                else if (strcmp(cmd, "INVP_MODE_MANUAL") == 0) { manualOverride |= (1UL << OVR_BIT_INV_PUMP); }
                else if (strcmp(cmd, "INVP_MODE_AUTO") == 0)   { manualOverride &= ~(1UL << OVR_BIT_INV_PUMP); }
                else if (strcmp(cmd, "INVP_CMD_OFF") == 0)     { manual_invPumpPwm = 0;   telemetryData.invPumpPwm = 0; }
                else if (strcmp(cmd, "INVP_CMD_20") == 0)      { manual_invPumpPwm = 51;  telemetryData.invPumpPwm = 51; }
                else if (strcmp(cmd, "INVP_CMD_80") == 0)      { manual_invPumpPwm = 204; telemetryData.invPumpPwm = 204; }

                else if (strcmp(cmd, "BATP_MODE_MANUAL") == 0) { manualOverride |= (1UL << OVR_BIT_BAT_PUMP); }
                else if (strcmp(cmd, "BATP_MODE_AUTO") == 0)   { manualOverride &= ~(1UL << OVR_BIT_BAT_PUMP); }
                else if (strcmp(cmd, "BATP_CMD_OFF") == 0)     { manual_batPumpPwm = 0;   telemetryData.batPumpPwm = 0; }
                else if (strcmp(cmd, "BATP_CMD_20") == 0)      { manual_batPumpPwm = 51;  telemetryData.batPumpPwm = 51; }
                else if (strcmp(cmd, "BATP_CMD_80") == 0)      { manual_batPumpPwm = 204; telemetryData.batPumpPwm = 204; }

                // --- HUT V3 AUXILIARY EXPANSION BUS CHANNELS ---
                else if (strcmp(cmd, "REL11_MODE_MANUAL") == 0){ manualOverride |= (1UL << OVR_BIT_REL11); }
                else if (strcmp(cmd, "REL11_MODE_AUTO") == 0)  { manualOverride &= ~(1UL << OVR_BIT_REL11); }
                else if (strcmp(cmd, "REL11_CMD_ON") == 0)     { manualOverride_Values |= (1UL << OVR_BIT_REL11); }
                else if (strcmp(cmd, "REL11_CMD_OFF") == 0)    { manualOverride_Values &= ~(1UL << OVR_BIT_REL11); }

                else if (strcmp(cmd, "REL12_MODE_MANUAL") == 0){ manualOverride |= (1UL << OVR_BIT_REL12); }
                else if (strcmp(cmd, "REL12_MODE_AUTO") == 0)  { manualOverride &= ~(1UL << OVR_BIT_REL12); }
                else if (strcmp(cmd, "REL12_CMD_ON") == 0)     { manualOverride_Values |= (1UL << OVR_BIT_REL12); }
                else if (strcmp(cmd, "REL12_CMD_OFF") == 0)    { manualOverride_Values &= ~(1UL << OVR_BIT_REL12); }

                else if (strcmp(cmd, "REL13_MODE_MANUAL") == 0){ manualOverride |= (1UL << OVR_BIT_REL13); }
                else if (strcmp(cmd, "REL13_MODE_AUTO") == 0)  { manualOverride &= ~(1UL << OVR_BIT_REL13); }
                else if (strcmp(cmd, "REL13_CMD_ON") == 0)     { manualOverride_Values |= (1UL << OVR_BIT_REL13); }
                else if (strcmp(cmd, "REL13_CMD_OFF") == 0)    { manualOverride_Values &= ~(1UL << OVR_BIT_REL13); }

                else if (strcmp(cmd, "REL14_MODE_MANUAL") == 0){ manualOverride |= (1UL << OVR_BIT_REL14); }
                else if (strcmp(cmd, "REL14_MODE_AUTO") == 0)  { manualOverride &= ~(1UL << OVR_BIT_REL14); }
                else if (strcmp(cmd, "REL14_CMD_ON") == 0)     { manualOverride_Values |= (1UL << OVR_BIT_REL14); }
                else if (strcmp(cmd, "REL14_CMD_OFF") == 0)    { manualOverride_Values &= ~(1UL << OVR_BIT_REL14); }

                xSemaphoreGive(dataMutex);
            }
            updateWebDashboard();
        }
    }
}

/**
 * @brief Telemetry Broadcast to Browser (ArduinoJson V7 compatible)
 */
void updateWebDashboard() {
    if (webSocket.count() == 0) return;

    JsonDocument doc; 

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Group 1: MCU / Powertrain
        JsonObject mcu = doc["mcu"].to<JsonObject>();
        mcu["rpm"]   = (int32_t)telemetryData.motorRPM;
        mcu["rpmV"]  = (bool)telemetryData.motorRPMValid;
        mcu["trq"]   = (float)telemetryData.motorTorque;
        mcu["mt"]    = (int)telemetryData.motorTemp;
        mcu["mtV"]   = (bool)telemetryData.motorTempValid;
        mcu["it"]    = (int)telemetryData.mcuTemp;
        mcu["itV"]   = (bool)telemetryData.mcuTempValid;
        mcu["flt"]   = (int)telemetryData.mcuFaultLevel;
        mcu["fltV"]  = (bool)telemetryData.mcuFaultLevelValid;

        // Group 2: BMS & Labor Viewport Interceptor
        JsonObject bms = doc["bms"].to<JsonObject>();
        bms["soc"]   = (int)telemetryData.bmsSoC;
        bms["socV"]  = (bool)telemetryData.bmsSoCValid;
        bms["a"]     = (float)telemetryData.bmsCurrent;
        bms["aV"]    = (bool)telemetryData.bmsCurrentValid;
        bms["v"]     = (int)telemetryData.hv1Voltage;
        bms["vV"]    = (bool)telemetryData.hv1VoltageValid;
        
        if (demoModeActive && telemetryData.bmsStatus == 0) {
            bms["st"] = 2; 
        } else {
            bms["st"] = (int)telemetryData.bmsStatus;     
        }
        bms["stV"]   = (bool)telemetryData.bmsStatusValid;

        // Group 3: IMD / Insulation Monitor
        JsonObject imd = doc["imd"].to<JsonObject>();
        imd["r"]     = (int)telemetryData.imdIsoR;
        imd["rV"]    = (bool)telemetryData.imdIsoRValid;
        imd["st"]    = (int)telemetryData.imdStatus;     
        imd["stV"]   = (bool)telemetryData.imdStatusValid;  

        // Group 4: VCU Control & Advanced Diagnostics
        JsonObject vcu = doc["vcu"].to<JsonObject>();
        vcu["range"]       = (int)estimatedRange;      
        vcu["trip"]        = (bool)!dailyModeActive;
        vcu["demo_active"] = (bool)demoModeActive;        
        vcu["mOv"]         = (uint32_t)manualOverride; 
        vcu["chg"]         = (bool)telemetryData.isCharging;
        vcu["soc"]         = (int)telemetryData.bmsSoC;
        vcu["err"]         = (bool)telemetryData.selfTestFailed;
        vcu["run"]         = (bool)telemetryData.selfTestRunning;
        
        vcu["sysF"]  = (uint16_t)telemetryData.systemFlags;    
        vcu["motF"]  = (uint16_t)telemetryData.motorFlags;     
        vcu["ontm"]  = (uint8_t)telemetryData.systemKeyOntime; 
        vcu["flgV"]  = (bool)telemetryData.mcuFlagsValid;
        
        vcu["lok"]   = (bool)telemetryData.isLocked;            
        vcu["lkg"]   = (bool)telemetryData.isLocking;           
        vcu["unl"]   = (bool)telemetryData.isUnLocking;         
        vcu["stp"]   = (bool)telemetryData.manualStopRequested; 

        // Group 5: Proxy BMS (Hyper9 Interface)
        JsonObject proxy = doc["proxy"].to<JsonObject>();
        bool cableConnected = (telemetryData.bmsStatus == 3 || telemetryData.isCharging); 
        bool systemFault = (telemetryData.selfTestResult != 0 || telemetryData.bmsHardwareFault);
        bool driveInhibit = (cableConnected || telemetryData.isLocked || systemFault);

        proxy["soc"] = (int)((telemetryData.bmsSoC / 100.0f) * 32768.0f); 
        proxy["inh"] = (bool)driveInhibit; 
        proxy["lim"] = (telemetryData.bmsHighTempWarn) ? 50 : (telemetryData.bmsLowVoltageWarn ? 40 : 100);
        proxy["flt"] = (bool)systemFault;

        // ====================================================================
        // EXPANDED CHANNEL ROUTING ENGINE (MAN/AUTO PROTECTION INTERLOCK)
        // ====================================================================
        JsonObject ovr = doc["ovr"].to<JsonObject>();
        
        // Symmetrically serializes from the manual values mask to protect frontend states
        auto packBinaryChannel = [&](JsonObject& parent, const char* key, uint8_t modeBit, bool hardwareState) {
            JsonObject ch = parent[key].to<JsonObject>();
            bool isManual = (manualOverride & (1UL << modeBit)) ? true : false;
            ch["m"] = isManual ? 1 : 0;
            
            if (isManual) {
                ch["s"] = (manualOverride_Values & (1UL << modeBit)) ? 1 : 0; // Reads straight from stable override mask
            } else {
                ch["s"] = hardwareState ? 1 : 0; 
            }
        };

        auto packPumpChannel = [&](JsonObject& parent, const char* key, uint8_t modeBit, uint8_t hardwarePwmValue) {
            JsonObject ch = parent[key].to<JsonObject>();
            bool isManual = (manualOverride & (1UL << modeBit)) ? true : false;
            ch["m"] = isManual ? 1 : 0;
            
            uint8_t dynamicPwmState = hardwarePwmValue;
            if (isManual) {
                // Read from our protected manual value buffers to shield the UI from background drift
                if (modeBit == OVR_BIT_BAT_PUMP)      dynamicPwmState = manual_batPumpPwm;
                else if (modeBit == OVR_BIT_INV_PUMP) dynamicPwmState = manual_invPumpPwm;
            }
            
            if (dynamicPwmState == 204)     ch["s"] = 2; // BOOST
            else if (dynamicPwmState == 51) ch["s"] = 1; // ECO
            else                            ch["s"] = 0; // OFF
        };

        packBinaryChannel(ovr, "oil",  OVR_BIT_CHECK_OIL, telemetryData.ledCheckOil);
        packBinaryChannel(ovr, "fan",  OVR_BIT_FAN,       telemetryData.fanRelay);
        packBinaryChannel(ovr, "bat",  OVR_BIT_LED_BATT,  telemetryData.ledBattery);
        
        packPumpChannel(ovr,   "batp", OVR_BIT_BAT_PUMP,  telemetryData.batPumpPwm);
        packPumpChannel(ovr,   "invp", OVR_BIT_INV_PUMP,  telemetryData.invPumpPwm);
        
        packBinaryChannel(ovr, "r11",  OVR_BIT_REL11,     telemetryData.auxRelay11);
        packBinaryChannel(ovr, "r12",  OVR_BIT_REL12,     telemetryData.auxRelay12);
        packBinaryChannel(ovr, "r13",  OVR_BIT_REL13,     telemetryData.auxRelay13);
        packBinaryChannel(ovr, "r14",  OVR_BIT_REL14,     telemetryData.auxRelay14);

        JsonObject chBuzz = ovr["buzz"].to<JsonObject>();
        bool piezoManual = (manualOverride & (1UL << OVR_BIT_PIEZO)) ? true : false;
        chBuzz["m"] = piezoManual ? 1 : 0;
        chBuzz["s"] = piezoManual ? ((manualOverride_Values & (1UL << OVR_BIT_PIEZO)) ? 1 : 0) : (telemetryData.isPiezoOn ? 1 : 0);

        JsonObject chAlarm = ovr["alarm"].to<JsonObject>();
        bool alarmManual = (manualOverride & (1UL << OVR_BIT_ALARM)) ? true : false;
        chAlarm["m"] = alarmManual ? 1 : 0;
        chAlarm["s"] = alarmManual ? ((manualOverride_Values & (1UL << OVR_BIT_ALARM)) ? 1 : 0) : (telemetryData.isAlarm ? 1 : 0);
        
        // ==================== HARDWARE / PHYSICAL IO-PORTS (T-2CAN + Hut V3) ====================
        JsonObject hw = doc["hw"].to<JsonObject>();
        hw["5v_en"]          = (bool)telemetryData.fiveVEnabled;
        hw["canb_stby"]      = (bool)telemetryData.canBStby;
        hw["lock_in1"]       = (bool)telemetryData.lockIn1;
        hw["lock_in2"]       = (bool)telemetryData.lockIn2;
        hw["lock_fb"]        = (bool)telemetryData.lockFeedback;
        hw["manual_unlock"]  = (bool)telemetryData.manualUnlockBtn;

        hw["bat_pump_relay"] = (bool)telemetryData.batPumpRelay;
        hw["bat_pump_pwm"]   = (int)telemetryData.batPumpPwm;
        hw["inv_pump_relay"] = (bool)telemetryData.invPumpRelay;
        hw["inv_pump_pwm"]   = (int)telemetryData.invPumpPwm;
        hw["fan_relay"]      = (bool)telemetryData.fanRelay;

        hw["aux_rel11"]      = (bool)telemetryData.auxRelay11;
        hw["aux_rel12"]      = (bool)telemetryData.auxRelay12;
        hw["aux_rel13"]      = (bool)telemetryData.auxRelay13;
        hw["aux_rel14"]      = (bool)telemetryData.auxRelay14;

        hw["is_alarm"]       = (bool)telemetryData.isAlarm;   
        hw["piezo"]          = (bool)telemetryData.isPiezoOn; 
        hw["led_oil"]        = (bool)telemetryData.ledCheckOil;
        hw["led_battery"]    = (bool)telemetryData.ledBattery;
        hw["ws2812"]         = (int)telemetryData.ws2812Status;
        hw["aux_in13"]       = (bool)telemetryData.auxinput13;

        xSemaphoreGive(dataMutex);
    }

    String responseString;
    serializeJson(doc, responseString);
    webSocket.textAll(responseString);
}