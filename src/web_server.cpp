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
 * @brief Handles incoming WebSocket commands from the dashboard.
 */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, 
               void *arg, uint8_t *data, size_t len) {
    
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            
            data[len] = 0; 
            const char* cmd = (const char*)data;
            safe_printf("[WS RECV] Empfangener Befehl: %s\n", cmd);

            // ====================== MANUAL OVERRIDES ======================

            // Indicator LEDs
            if      (strcmp(cmd, "LED_CHECK_OIL_TOGGLE") == 0) toggleOutput(LED_CHECK_OIL_PIN);
            else if (strcmp(cmd, "LED_CHECK_OIL_AUTO") == 0)   setOutput(LED_CHECK_OIL_PIN, false);   // oder eigene Auto-Logik

            else if (strcmp(cmd, "LED_BATTERY_TOGGLE") == 0)   toggleOutput(LED_BATTERY_PIN);
            else if (strcmp(cmd, "LED_BATTERY_AUTO") == 0)     setOutput(LED_BATTERY_PIN, false);

            // Alarm / Mute Steuerung
            else if (strcmp(cmd, "PIEZO_TOGGLE") == 0) {
                // Toggeln des Mute-Bits (Bit 1)
                manualOverride ^= (1 << 1); 
                safe_printf("[ALARM] Mute-condition changed! Current override register: 0x%X\n", manualOverride);
            }
            else if (strcmp(cmd, "PIEZO_AUTO") == 0) {
                manualOverride &= ~(1 << 1); // deactivate manual override for alarm (Bit 1)
            }

            // Cooling
            else if (strcmp(cmd, "FAN_TOGGLE") == 0)           toggleOutput(FAN_RELAY_PIN);
            else if (strcmp(cmd, "FAN_AUTO") == 0)             setOutput(FAN_RELAY_PIN, false);

            // Pumps
            else if (strcmp(cmd, "BAT_PUMP_TOGGLE") == 0)      toggleOutput(BAT_PUMP_RELAY_PIN);
            else if (strcmp(cmd, "BAT_PUMP_AUTO") == 0)        setOutput(BAT_PUMP_RELAY_PIN, false);

            else if (strcmp(cmd, "INV_PUMP_TOGGLE") == 0)      toggleOutput(INV_PUMP_RELAY_PIN);
            else if (strcmp(cmd, "INV_PUMP_AUTO") == 0)        setOutput(INV_PUMP_RELAY_PIN, false);

            // General Purpose Relays
            else if (strcmp(cmd, "REL11_TOGGLE") == 0) toggleOutput(RELAY_11_PIN);
            else if (strcmp(cmd, "REL11_AUTO") == 0)   setOutput(RELAY_11_PIN, false);
            else if (strcmp(cmd, "REL12_TOGGLE") == 0) toggleOutput(RELAY_12_PIN);
            else if (strcmp(cmd, "REL12_AUTO") == 0)   setOutput(RELAY_12_PIN, false);
            else if (strcmp(cmd, "REL13_TOGGLE") == 0) toggleOutput(RELAY_13_PIN);
            else if (strcmp(cmd, "REL13_AUTO") == 0)   setOutput(RELAY_13_PIN, false);
            else if (strcmp(cmd, "REL14_TOGGLE") == 0) toggleOutput(RELAY_14_PIN);
            else if (strcmp(cmd, "REL14_AUTO") == 0)   setOutput(RELAY_14_PIN, false);

            // VCU & Safety
            else if (strcmp(cmd, "LOCK_REQ") == 0)           WITH_DATA_MUTEX({ telemetryData.shouldLock = true; });
            else if (strcmp(cmd, "UNLOCK_REQ") == 0)         manualUnlockPressed = true;
            else if (strcmp(cmd, "IMD_TEST_START") == 0)     WITH_DATA_MUTEX({ telemetryData.selfTestRequested = true; });

            // Main Page Controls
            else if (strcmp(cmd, "MODE_DAILY") == 0) dailyModeActive = true;
            else if (strcmp(cmd, "MODE_TRIP") == 0)  dailyModeActive = false;
            else if (strcmp(cmd, "CHARGE_STOP_REQ") == 0) sendElconCommand(true);
        }
    }
}

/**
 * @brief Telemetry Broadcast to Browser (ArduinoJson V7 compatible)
 * @note Cleaned up to perfectly match the web_ui.h keys and fixed BMS status logic.
*/
void updateWebDashboard() {
    if (webSocket.count() == 0) return;

    // ArduinoJson V7: Lokales Dokument leert sich automatisch und verhindert Heap-Müll
    JsonDocument doc; 

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Group 1: MCU
        JsonObject mcu = doc["mcu"].to<JsonObject>();
        mcu["rpm"]   = (int32_t)telemetryData.motorRPM;
        mcu["rpmV"]  = (bool)telemetryData.motorRPMValid;
        mcu["mt"]    = (int)telemetryData.motor_temp;
        mcu["mtV"]   = (bool)telemetryData.motorTempValid;
        mcu["it"]    = (int)telemetryData.mcu_temp;
        mcu["itV"]   = (bool)telemetryData.mcuTempValid;
        mcu["flt"]   = (int)telemetryData.mcuFaultLevel;
        mcu["fltV"]  = (bool)telemetryData.mcuFaultLevelValid;

        // Group 2: BMS
        JsonObject bms = doc["bms"].to<JsonObject>();
        bms["soc"]   = (int)telemetryData.bmsSoC;
        bms["socV"]  = (bool)telemetryData.bmsSoCValid;
        bms["a"]     = (float)telemetryData.bmsCurrent;
        bms["aV"]    = (bool)telemetryData.bmsCurrentValid;
        bms["v"]     = (int)telemetryData.hv1Voltage;
        bms["vV"]    = (bool)telemetryData.hv1VoltageValid;
        bms["st"]    = (int)telemetryData.bmsStatus;     
        bms["stV"]   = (bool)telemetryData.bmsStatusValid;

        // Group 3: IMD
        JsonObject imd = doc["imd"].to<JsonObject>();
        imd["r"]     = (int)telemetryData.imdIsoR;
        imd["rV"]    = (bool)telemetryData.imdIsoRValid;
        imd["st"]    = (int)telemetryData.imdStatus;     
        imd["stV"]   = (bool)telemetryData.imdStatusValid;  

        // Group 4: VCU
        JsonObject vcu = doc["vcu"].to<JsonObject>();
        vcu["range"] = (int)estimatedRange;      
        vcu["trip"]  = !dailyModeActive;        
        vcu["mOv"]   = (int)manualOverride;
        vcu["chg"]   = (bool)telemetryData.is_charging;
        vcu["unl"]   = (bool)telemetryData.isUnLocking;
        vcu["err"]   = (bool)telemetryData.selfTestFailed;
        vcu["run"]   = (bool)telemetryData.selfTestRunning;
        vcu["soc"]   = (int)telemetryData.bmsSoC;

        // Group 5: Proxy BMS (Hyper9 Interface) - Logik auf echten GitHub-Status 3 korrigiert
        JsonObject proxy = doc["proxy"].to<JsonObject>();
        bool cableConnected = (telemetryData.bmsStatus == BMS_STATUS_CHARGE || telemetryData.is_charging);
        bool systemFault = (telemetryData.selfTestResult != 0 || telemetryData.bmsHardwareFault);
        bool driveInhibit = (cableConnected || telemetryData.isLocked || systemFault);

        proxy["soc"] = (int)((telemetryData.bmsSoC / 100.0f) * 32768.0f); 
        proxy["inh"] = (bool)driveInhibit; 
        proxy["lim"] = (telemetryData.bmsHighTempWarn) ? 50 : (telemetryData.bmsLowVoltageWarn ? 40 : 100);
        proxy["flt"] = (bool)systemFault;

        // ==================== HARDWARE / IO (T-2CAN + Hut V3) ====================
        JsonObject hw = doc["hw"].to<JsonObject>();

        hw["5v_en"]          = (bool)telemetryData.fiveVEnabled;
        hw["canb_stby"]      = (bool)telemetryData.canBStby;

        hw["lock_in1"]       = (bool)telemetryData.lockIn1;
        hw["lock_in2"]       = (bool)telemetryData.lockIn2;
        hw["lock_fb"]        = (bool)telemetryData.lockFeedback;
        hw["manual_unlock"]  = (bool)telemetryData.manualUnlockBtn;

        // Keys exakt an das JS-Dashboard angepasst:
        hw["bat_pump_relay"] = (bool)telemetryData.batPumpRelay;
        hw["bat_pump_pwm"]   = (int)telemetryData.bat_pump_pwm;
        hw["inv_pump_relay"] = (bool)telemetryData.invPumpRelay;
        hw["inv_pump_pwm"]   = (int)telemetryData.inv_pump_pwm;
        hw["fan_relay"]      = (bool)telemetryData.fanRelay;

        hw["aux_rel11"]      = (bool)telemetryData.auxRelay11;
        hw["aux_rel12"]      = (bool)telemetryData.auxRelay12;
        hw["aux_rel13"]      = (bool)telemetryData.auxRelay13;
        hw["aux_rel14"]      = (bool)telemetryData.auxRelay14;

        hw["is_alarm"]       = (bool)telemetryData.isAlarm;   // Schaltet das Icon auf der Hauptseite ROT
        hw["piezo"]          = (bool)telemetryData.isPiezoOn; // Zeigt auf der Expert-Seite, ob der Pin Lärm macht
        hw["led_oil"]        = (bool)telemetryData.ledCheckOil;
        hw["led_battery"]    = (bool)telemetryData.ledBattery;
        hw["ws2812"]         = (int)telemetryData.ws2812Status;

        hw["aux_in13"]       = (bool)telemetryData.auxinput13;

        xSemaphoreGive(dataMutex);
    }

    // ====================================================================
    // FINISH & SEND
    // ====================================================================

    // 1. Speicher-Überlauf fixen: Nutze sizeof(jsonBuffer), um exakt die deklarierte Größe zu löschen (1536)
    memset(jsonBuffer, 0, sizeof(jsonBuffer));

    size_t len = 0;
    #ifdef HARDWARE_T2CAN
        // Auf dem T-2CAN nutzen wir den PSRAM-Puffer mit seiner definierten Größe
        len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer)); 
    #else
        // Auf dem festen Array ermittelt der Compiler die Größe automatisch
        len = serializeJson(doc, jsonBuffer);
    #endif

    // 2. Core-Panic verhindern: Nur senden, wenn die Puffer-Länge gültig ist UND Clients aktiv sind
    if (len > 0 && len < sizeof(jsonBuffer)) {
        // Sicherstellen, dass das webSocket-Objekt existiert und bereit ist
        if (webSocket.count() > 0) {
            webSocket.textAll(jsonBuffer, len);
        }
    } else {
        // Optional: Fehler-Logging für ungültige Puffergrößen
        Serial.printf("[NET] Warning: Invalid JSON buffer length (%d bytes). Data not sent.\n", len);
    }
}