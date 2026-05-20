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

// Forward declaration of the asynchronous WebSocket event handler callback
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

/**
 * Gathers current telemetryData and broadcasts JSON via WebSocket
 */
void updateWebDashboard();

#endif