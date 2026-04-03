/**
 * @file secrets_example.h
 * @brief Template for network credentials and device identity.
 * * @instructions:
 * 1. Copy this file to "secrets.h"
 * 2. Fill in your actual WiFi credentials and passwords.
 * 3. Make sure "secrets.h" is added to your .gitignore to keep your data private!
 */

#ifndef SECRETS_H
#define SECRETS_H

// --- Device Identity ---
// The hostname used for mDNS (e.g., http://vcu-x19e.local)
#define DEVICE_HOSTNAME "vcu-x19e"

// --- Access Point Settings (Car Internal Network) ---
// Credentials for the ESP32's own WiFi network (AP Mode)
#define AP_SSID       "VCU_X19E_AP"
#define AP_PASS       "your_secure_ap_password"

// --- Station Settings (Home/Garage Network) ---
// Credentials to connect to your existing WiFi (Station Mode)
#define HOME_SSID     "YOUR_HOME_SSID"
#define HOME_PASS     "YOUR_HOME_PASSWORD"

// --- OTA Security ---
// Credentials for the ElegantOTA web interface (Updates via Browser)
#define OTA_USER       "admin"
#define OTA_PASS       "admin_password"

#endif // SECRETS_H