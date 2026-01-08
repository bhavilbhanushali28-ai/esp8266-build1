// ============================================================
// WiFiConfig.h - WiFi Configuration
// ============================================================

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <Arduino.h>

// ===================== HARDCODED NETWORKS STRUCTURE =====================
struct HardcodedNetwork {
    const char* ssid;
    const char* password;
};

// Declare as extern - definition will be in WiFiConfig.cpp
extern const HardcodedNetwork hardcodedNetworks[];
extern const size_t NUM_HARDCODED_NETWORKS;

// ===================== OPTIONAL OVERRIDES =====================
// Uncomment and modify to override defaults

// Connection timeouts
// #define WIFIMGR_MAX_CONNECT_TIMEOUT_MS 15000
// #define WIFIMGR_MAX_STA_ATTEMPTS 3

// Telegram delay (increase if crashing)
// #define WIFIMGR_TELEGRAM_DELAY_MS 5000
// #define WIFIMGR_MIN_HEAP_FOR_TELEGRAM 12000
// #define WIFIMGR_CONNECTION_STABLE_MS 3000

// Hunt mode
// #define WIFIMGR_HUNT_LIGHT_SLEEP_MS 5000
// #define WIFIMGR_HUNT_CONNECT_TIMEOUT_MS 12000

// AP Mode
// #define WIFIMGR_AP_MODE_DURATION_MS 300000

#endif // WIFI_CONFIG_H