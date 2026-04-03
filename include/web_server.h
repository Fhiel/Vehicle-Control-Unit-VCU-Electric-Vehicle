#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>

// External declarations for global server objects
extern AsyncWebServer server;
extern AsyncWebSocket webSocket;

/**
 * Starts the WiFi Access Point and initializes the AsyncWebServer
 */
void initWebServer();

/**
 * Gathers current telemetryData and broadcasts JSON via WebSocket
 */
void updateWebDashboard();

#endif