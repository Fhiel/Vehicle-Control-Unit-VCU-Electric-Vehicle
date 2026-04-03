#ifndef PWA_DATA_H
#define PWA_DATA_H

#include <Arduino.h>

/**
 * @file pwa_data.h
 * @brief Progressive Web App Manifest for X1/9e VCU.
 * Minimal configuration without external icon assets.
 */

const char manifest_json[] PROGMEM = R"rawliteral(
{
  "name": "X1/9e VCU Expert",
  "short_name": "X19e VCU",
  "start_url": "/",
  "display": "standalone",
  "background_color": "#000000",
  "theme_color": "#008cff",
  "description": "Vehicle Control Unit Dashboard for Fiat X1/9 Electric Conversion",
  "orientation": "landscape"
}
)rawliteral";

#endif // PWA_DATA_H