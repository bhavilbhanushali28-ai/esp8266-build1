// ============================================================
// WiFiManager.h - Intermittent WiFi with Smart AP Fallback
// Version: 2.2 - Fixed for ArduinoDroid compatibility
// ============================================================

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "WiFiConfig.h"  // Include for HardcodedNetwork struct

// ===================== CONFIGURATION DEFAULTS =====================

#ifndef WIFIMGR_MAX_CONNECT_TIMEOUT_MS
#define WIFIMGR_MAX_CONNECT_TIMEOUT_MS 15000
#endif

#ifndef WIFIMGR_MAX_STA_ATTEMPTS
#define WIFIMGR_MAX_STA_ATTEMPTS 3
#endif

#ifndef WIFIMGR_RECONNECT_INTERVAL_MS
#define WIFIMGR_RECONNECT_INTERVAL_MS 5000
#endif

#ifndef WIFIMGR_MAX_BACKOFF_MULTIPLIER
#define WIFIMGR_MAX_BACKOFF_MULTIPLIER 8
#endif

#ifndef WIFIMGR_MAX_WRONG_PASSWORD_ATTEMPTS
#define WIFIMGR_MAX_WRONG_PASSWORD_ATTEMPTS 3
#endif

#ifndef WIFIMGR_MAX_AUTH_FAILURES
#define WIFIMGR_MAX_AUTH_FAILURES 5
#endif

#ifndef WIFIMGR_POOR_SIGNAL_THRESHOLD_DBM
#define WIFIMGR_POOR_SIGNAL_THRESHOLD_DBM -80
#endif

#ifndef WIFIMGR_MAX_POOR_SIGNAL_DISCONNECTS
#define WIFIMGR_MAX_POOR_SIGNAL_DISCONNECTS 3
#endif

#ifndef WIFIMGR_POOR_SIGNAL_WINDOW_MS
#define WIFIMGR_POOR_SIGNAL_WINDOW_MS 300000
#endif

#ifndef WIFIMGR_GOOD_SIGNAL_RESET_MS
#define WIFIMGR_GOOD_SIGNAL_RESET_MS 60000
#endif

#ifndef WIFIMGR_AP_MODE_DURATION_MS
#define WIFIMGR_AP_MODE_DURATION_MS 300000
#endif

#ifndef WIFIMGR_AP_AUTO_EXIT_ENABLED
#define WIFIMGR_AP_AUTO_EXIT_ENABLED true
#endif

#ifndef WIFIMGR_HUNT_SCAN_INTERVAL_MS
#define WIFIMGR_HUNT_SCAN_INTERVAL_MS 10000
#endif

#ifndef WIFIMGR_HUNT_LIGHT_SLEEP_MS
#define WIFIMGR_HUNT_LIGHT_SLEEP_MS 5000
#endif

#ifndef WIFIMGR_HUNT_CONNECT_TIMEOUT_MS
#define WIFIMGR_HUNT_CONNECT_TIMEOUT_MS 12000
#endif

#ifndef WIFIMGR_HUNT_MAX_QUICK_RETRIES
#define WIFIMGR_HUNT_MAX_QUICK_RETRIES 3
#endif

#ifndef WIFIMGR_TELEGRAM_DELAY_MS
#define WIFIMGR_TELEGRAM_DELAY_MS 3500
#endif

#ifndef WIFIMGR_MIN_HEAP_FOR_TELEGRAM
#define WIFIMGR_MIN_HEAP_FOR_TELEGRAM 10000
#endif

#ifndef WIFIMGR_CONNECTION_STABLE_MS
#define WIFIMGR_CONNECTION_STABLE_MS 2000
#endif

#ifndef WIFIMGR_MIN_HEAP_FOR_SCAN
#define WIFIMGR_MIN_HEAP_FOR_SCAN 6000
#endif

#ifndef WIFIMGR_HEAP_CRITICAL
#define WIFIMGR_HEAP_CRITICAL 4000
#endif

// ===================== ENUMERATIONS =====================

typedef enum {
    WIFI_MGR_IDLE = 0,
    WIFI_MGR_SCANNING,
    WIFI_MGR_CONNECTING_SAVED,
    WIFI_MGR_CONNECTING_HARDCODED,
    WIFI_MGR_CONNECTED,
    WIFI_MGR_AP_MODE,
    WIFI_MGR_WAITING_BACKOFF,
    WIFI_MGR_HUNT_SCANNING,
    WIFI_MGR_HUNT_CONNECTING,
    WIFI_MGR_HUNT_LIGHT_SLEEP,
    WIFI_MGR_STATE_COUNT
} WifiMgrState;

typedef enum {
    WIFI_OP_MODE_NORMAL = 0,
    WIFI_OP_MODE_INTERMITTENT_HUNT
} WifiMgrOperationMode;

typedef enum {
    WIFI_FAIL_NONE = 0,
    WIFI_FAIL_TIMEOUT,
    WIFI_FAIL_WRONG_PASSWORD,
    WIFI_FAIL_SSID_NOT_FOUND,
    WIFI_FAIL_CONNECTION_LOST,
    WIFI_FAIL_POOR_SIGNAL,
    WIFI_FAIL_UNKNOWN
} WifiMgrFailureType;

// ===================== STRUCTURES =====================

typedef struct {
    WifiMgrState state;
    WifiMgrState previousState;
    WifiMgrOperationMode operationMode;
    WifiMgrFailureType lastFailureType;
    
    uint32_t lastAttemptTime;
    uint32_t connectionStartTime;
    uint32_t lastScanTime;
    uint32_t lastScanCompleteTime;
    uint32_t huntStartTime;
    uint32_t lastSuccessfulConnection;
    uint32_t lastTargetSeenTime;
    uint32_t apModeStartTime;
    uint32_t lightSleepStartTime;
    uint32_t goodSignalStartTime;
    
    uint8_t savedNetworkAttempts;
    uint8_t currentHardcodedIndex;
    uint8_t backoffMultiplier;
    uint8_t quickRetryCount;
    uint8_t wrongPasswordCount;
    uint8_t authFailureCount;
    uint8_t consecutiveScanMisses;
    uint8_t consecutiveDisconnects;
    uint16_t totalScanCount;
    
    uint8_t poorSignalDisconnectCount;
    uint32_t poorSignalWindowStart;
    int8_t lastConnectedRSSI;
    bool lastDisconnectWasPoorSignal;
    uint32_t lastDisconnectTime;
    
    bool scanResultsCached;
    bool wifiEventRegistered;
    bool targetNetworkEverSeen;
    bool apModeTriggeredByAuthFail;
    bool apModeTriggeredByPoorSignal;
    bool scanInProgress;
    bool isConnecting;
    
    uint8_t availableNetworkCount;
    int8_t lastRSSI;
    int8_t bestSeenRSSI;
    
    uint8_t cachedNetworkIndices[8];
    char targetSSID[33];
} WifiMgrContext;

typedef struct {
    uint32_t crc;
    char lastConnectedSSID[33];
    uint8_t failedAttemptsSinceSuccess;
    bool targetEverSeen;
} WifiMgrRTCData;

// ===================== PUBLIC FUNCTION DECLARATIONS =====================

#ifdef __cplusplus
extern "C" {
#endif

// Initialization
void initWiFiManager(void);

// Main loop function
void manageWiFiConnection(void);

// State getters
WifiMgrState getWiFiState(void);
const char* getWiFiStateName(void);
const char* getWiFiStateNameByState(WifiMgrState state);
WifiMgrFailureType getLastFailureType(void);
const char* getFailureTypeName(WifiMgrFailureType type);

// Mode control
void setWiFiOperationMode(WifiMgrOperationMode mode);
WifiMgrOperationMode getWiFiOperationMode(void);
void enableIntermittentHuntMode(void);
void disableIntermittentHuntMode(void);
bool isInHuntMode(void);

// Connection status
bool isWiFiConnected(void);
uint32_t getTimeSinceLastConnection(void);
uint32_t getTimeSinceTargetSeen(void);
bool hasTargetBeenSeen(void);

// Signal info
int8_t getLastRSSI(void);
int8_t getCurrentRSSI(void);

// Statistics
uint16_t getTotalScanCount(void);
uint8_t getPoorSignalDisconnectCount(void);
uint8_t getWrongPasswordCount(void);
uint8_t getAuthFailureCount(void);

// Control functions
void resetWiFiManager(void);
void resetFailureCounters(void);
void forceAPMode(const char* reason);
void exitAPModeAndResume(void);
void setTargetSSID(const char* ssid);

// Debug
void printWiFiManagerStatus(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_Hnt(void);
uint8_t getPoorSignalDisconnectCount(void);
uint8_t getWrongPasswordCount(void);
uint8_t getAuthFailureCount(void);

// Control functions
void resetWiFiManager(void);
void resetFailureCounters(void);
void forceAPMode(const char* reason);
void exitAPModeAndResume(void);
void setTargetSSID(const char* ssid);

// Debug
void printWiFiManagerStatus(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H