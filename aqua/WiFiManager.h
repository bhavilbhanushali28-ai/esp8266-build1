// ============================================================
// WiFiManager.h - Intermittent WiFi with Smart AP Fallback
// Version: 2.1 - Fixed for ArduinoDroid compatibility
// ============================================================

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <ESP8266WiFi.h>

// ===================== OPERATION MODES =====================
// Using regular enum with prefix to avoid conflicts
typedef enum {
    WIFI_OP_MODE_NORMAL = 0,
    WIFI_OP_MODE_INTERMITTENT_HUNT = 1
} WifiMgrOperationMode;

// ===================== FAILURE TYPES =====================
typedef enum {
    WIFI_FAIL_NONE = 0,
    WIFI_FAIL_TIMEOUT,
    WIFI_FAIL_WRONG_PASSWORD,
    WIFI_FAIL_SSID_NOT_FOUND,
    WIFI_FAIL_CONNECTION_LOST,
    WIFI_FAIL_POOR_SIGNAL,
    WIFI_FAIL_UNKNOWN
} WifiMgrFailureType;

// ===================== STATE MACHINE =====================
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

// ===================== CONFIGURATION =====================
#define WIFIMGR_RECONNECT_INTERVAL_MS       5000
#define WIFIMGR_MAX_CONNECT_TIMEOUT_MS      15000
#define WIFIMGR_SCAN_CACHE_VALIDITY_MS      30000
#define WIFIMGR_NTP_SYNC_INTERVAL_MS        300000
#define WIFIMGR_MAX_STA_ATTEMPTS            3
#define WIFIMGR_MAX_BACKOFF_MULTIPLIER      6

#define WIFIMGR_HUNT_SCAN_INTERVAL_MS       5000
#define WIFIMGR_HUNT_CONNECT_TIMEOUT_MS     10000
#define WIFIMGR_HUNT_LIGHT_SLEEP_MS         4000
#define WIFIMGR_HUNT_MAX_QUICK_RETRIES      3

#define WIFIMGR_MAX_WRONG_PASSWORD_ATTEMPTS 2
#define WIFIMGR_MAX_AUTH_FAILURES           3

#define WIFIMGR_POOR_SIGNAL_THRESHOLD_DBM   (-80)
#define WIFIMGR_VERY_POOR_SIGNAL_DBM        (-85)
#define WIFIMGR_MAX_POOR_SIGNAL_DISCONNECTS 5
#define WIFIMGR_POOR_SIGNAL_WINDOW_MS       300000
#define WIFIMGR_GOOD_SIGNAL_RESET_MS        300000

#define WIFIMGR_AP_MODE_DURATION_MS         300000
#define WIFIMGR_AP_AUTO_EXIT_ENABLED        true

#define WIFIMGR_LIGHT_SLEEP_BETWEEN_SCANS_MS 4000
#define WIFIMGR_LIGHT_SLEEP_IDLE_MS          100

// ===================== CONNECTION CONTEXT =====================
struct WifiMgrContext {
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
    
    int8_t cachedNetworkIndices[8];
    uint8_t availableNetworkCount;
    int8_t lastRSSI;
    int8_t bestSeenRSSI;
    
    char targetSSID[33];
};

// ===================== RTC PERSISTENCE =====================
struct WifiMgrRTCData {
    uint32_t crc;
    uint8_t failedAttemptsSinceSuccess;
    uint8_t wrongPasswordFlags;
    uint32_t totalHuntTimeSeconds;
    uint32_t lastConnectionTimestamp;
    char lastConnectedSSID[33];
    bool targetEverSeen;
};

// ===================== PUBLIC FUNCTIONS =====================
void initWiFiManager(void);
void manageWiFiConnection(void);

WifiMgrState getWiFiState(void);
const char* getWiFiStateName(void);
const char* getWiFiStateNameByState(WifiMgrState state);
WifiMgrFailureType getLastFailureType(void);
const char* getFailureTypeName(WifiMgrFailureType type);

void setWiFiOperationMode(WifiMgrOperationMode mode);
WifiMgrOperationMode getWiFiOperationMode(void);
void enableIntermittentHuntMode(void);
void disableIntermittentHuntMode(void);
bool isInHuntMode(void);

bool isWiFiConnected(void);
uint32_t getTimeSinceLastConnection(void);
uint32_t getTimeSinceTargetSeen(void);
bool hasTargetBeenSeen(void);
int8_t getLastRSSI(void);
int8_t getCurrentRSSI(void);
uint16_t getTotalScanCount(void);
uint8_t getPoorSignalDisconnectCount(void);
uint8_t getWrongPasswordCount(void);
uint8_t getAuthFailureCount(void);

void resetWiFiManager(void);
void resetFailureCounters(void);
void forceAPMode(const char* reason);
void exitAPModeAndResume(void);
void setTargetSSID(const char* ssid);

void printWiFiManagerStatus(void);

#endif