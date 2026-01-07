// ============================================================
// WiFiManager.cpp - Intermittent WiFi with Smart AP Fallback
// Version: 2.1 - Fixed for ArduinoDroid compatibility
// ============================================================

#include "WiFiManager.h"
#include <ESP8266WiFi.h>
#include "WiFiConfig.h"
// Remove the struct and array definitions from here
/*
// Hardcoded networks
struct HardcodedNetwork {
    const char* ssid;
    const char* password;
};
const HardcodedNetwork hardcodedNetworks[] = {
    {"realme 5g", "bhavil23!"},
    {"ishwar", "12345689"}, // Example 2
    {"note 5pro", "bhavil23!"},    // Example 3
    {"lenovo", "12345678"}  
};
//const size_t NUM_HARDCODED_NETWORKS = sizeof(hardcodedNetworks) / sizeof(hardcodedNetworks[0]);

const size_t NUM_HARDCODED_NETWORKS = sizeof(hardcodedNetworks) / sizeof(hardcodedNetworks[0]);
*/


// ===================== EXTERNAL REFERENCES =====================
extern struct Settings {
    char staSsid[33];
    char staPassword[65];
} settings;
/*
extern struct HardcodedNetwork {
    const char* ssid;
    const char* password;
} hardcodedNetworks[];
*/

extern const size_t NUM_HARDCODED_NETWORKS;

extern bool wifiConnected;
extern bool isApModeActive;
extern bool isMidCycleWake;
extern bool telegramTriggeredForThisConnection;
extern bool telegramSendPending;
extern bool telegramDelayActive;
extern bool telegramSending;
extern uint8_t telegramRetryCount;
extern unsigned long lastTimeSync;

extern void setupAP(void);
extern void checkAPTimeout(void);
extern void syncTimeFromNTP(void);
extern void saveSettings(void);

// ===================== MODULE VARIABLES =====================
static WifiMgrContext ctx;
static WifiMgrRTCData rtcWiFiData;

static WiFiEventHandler onConnectedHandler;
static WiFiEventHandler onDisconnectedHandler;
static WiFiEventHandler onGotIPHandler;

// ===================== STATE NAMES =====================
static const char* STATE_NAMES[] = {
    "IDLE", "SCANNING", "CONNECTING_SAVED", "CONNECTING_HARDCODED",
    "CONNECTED", "AP_MODE", "WAITING_BACKOFF", "HUNT_SCANNING",
    "HUNT_CONNECTING", "HUNT_LIGHT_SLEEP"
};

static const char* FAILURE_NAMES[] = {
    "NONE", "TIMEOUT", "WRONG_PASSWORD", "SSID_NOT_FOUND",
    "CONNECTION_LOST", "POOR_SIGNAL", "UNKNOWN"
};

// ===================== FORWARD DECLARATIONS =====================
static void onWiFiStationConnected(const WiFiEventStationModeConnected& event);
static void onWiFiStationDisconnected(const WiFiEventStationModeDisconnected& event);
static void onWiFiGotIP(const WiFiEventStationModeGotIP& event);

static void handleNormalMode(uint32_t now);
static void handleIntermittentMode(uint32_t now);

static void handleIdleState(uint32_t now);
static void handleScanningState(uint32_t now);
static void handleConnectingSavedState(uint32_t now);
static void handleConnectingHardcodedState(uint32_t now);
static void handleConnectedState(uint32_t now);
static void handleBackoffState(uint32_t now);
static void handleAPModeState(uint32_t now);
static void handleHuntScanningState(uint32_t now);
static void handleHuntConnectingState(uint32_t now);
static void handleHuntLightSleepState(uint32_t now);

static void transitionState(WifiMgrState newState, const char* reason);
static bool tryConnectToNetwork(const char* ssid, const char* password);
static void tryConnectToHardcodedNetwork(uint8_t index);
static bool startAsyncScan(void);
static bool isTargetInScanResults(int scanCount);
static void cacheAvailableNetworks(int scanCount);
static uint32_t calculateBackoffDelay(void);

static void handleWrongPassword(void);
static void handleAuthFailure(void);
static void handleSSIDNotFound(void);
static void handlePoorSignalDisconnect(void);
static bool shouldFallbackToAP(void);
static void enterAPMode(const char* reason, bool authFailure, bool poorSignal);
static void exitAPMode(void);

static void onConnectionEstablished(void);
static void onConnectionLost(void);
static void onHuntConnectionEstablished(void);
static void saveConnectedCredentialsIfNew(void);

static void exitLightSleep(void);
static void yieldWithLightSleep(uint32_t ms);

static void loadRTCWiFiData(void);
static void saveRTCWiFiData(void);
static uint32_t calculateCRC32(const uint8_t* data, size_t length);

static const char* getSignalQuality(int8_t rssi);

// ===================== CONTEXT INITIALIZATION =====================
static void initContext(void) {
    ctx.state = WIFI_MGR_IDLE;
    ctx.previousState = WIFI_MGR_IDLE;
    ctx.operationMode = WIFI_OP_MODE_NORMAL;
    ctx.lastFailureType = WIFI_FAIL_NONE;
    
    ctx.lastAttemptTime = 0;
    ctx.connectionStartTime = 0;
    ctx.lastScanTime = 0;
    ctx.lastScanCompleteTime = 0;
    ctx.huntStartTime = 0;
    ctx.lastSuccessfulConnection = 0;
    ctx.lastTargetSeenTime = 0;
    ctx.apModeStartTime = 0;
    ctx.lightSleepStartTime = 0;
    ctx.goodSignalStartTime = 0;
    
    ctx.savedNetworkAttempts = 0;
    ctx.currentHardcodedIndex = 0;
    ctx.backoffMultiplier = 1;
    ctx.quickRetryCount = 0;
    ctx.wrongPasswordCount = 0;
    ctx.authFailureCount = 0;
    ctx.consecutiveScanMisses = 0;
    ctx.consecutiveDisconnects = 0;
    ctx.totalScanCount = 0;
    
    ctx.poorSignalDisconnectCount = 0;
    ctx.poorSignalWindowStart = 0;
    ctx.lastConnectedRSSI = -100;
    ctx.lastDisconnectWasPoorSignal = false;
    ctx.lastDisconnectTime = 0;
    
    ctx.scanResultsCached = false;
    ctx.wifiEventRegistered = false;
    ctx.targetNetworkEverSeen = false;
    ctx.apModeTriggeredByAuthFail = false;
    ctx.apModeTriggeredByPoorSignal = false;
    ctx.scanInProgress = false;
    ctx.isConnecting = false;
    
    ctx.availableNetworkCount = 0;
    ctx.lastRSSI = -100;
    ctx.bestSeenRSSI = -100;
    
    memset(ctx.cachedNetworkIndices, 0, sizeof(ctx.cachedNetworkIndices));
    memset(ctx.targetSSID, 0, sizeof(ctx.targetSSID));
}

// ===================== INITIALIZATION =====================
void initWiFiManager(void) {
    Serial.println(F("\nðŸ”§ ===== Initializing WiFi Manager ====="));
    
    // Initialize context
    initContext();
    
    // Register event handlers once
    if (!ctx.wifiEventRegistered) {
        onConnectedHandler = WiFi.onStationModeConnected(onWiFiStationConnected);
        onDisconnectedHandler = WiFi.onStationModeDisconnected(onWiFiStationDisconnected);
        onGotIPHandler = WiFi.onStationModeGotIP(onWiFiGotIP);
        ctx.wifiEventRegistered = true;
        Serial.println(F("   âœ“ Event handlers registered"));
    }
    
    // Load RTC data
    loadRTCWiFiData();
    
    // Configure WiFi
    WiFi.setAutoReconnect(false);
    WiFi.persistent(false);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
    WiFi.setOutputPower(20.5);
    
    // Set target SSID
    if (strlen(settings.staSsid) > 0) {
        strncpy(ctx.targetSSID, settings.staSsid, sizeof(ctx.targetSSID) - 1);
        ctx.targetSSID[sizeof(ctx.targetSSID) - 1] = '\0';
        Serial.print(F("   Target SSID: "));
        Serial.println(ctx.targetSSID);
    }
    
    ctx.targetNetworkEverSeen = rtcWiFiData.targetEverSeen;
    ctx.huntStartTime = millis();
    
    transitionState(WIFI_MGR_IDLE, "init");
    
    Serial.print(F("   Mode: "));
    Serial.println(ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT 
                   ? "INTERMITTENT_HUNT" : "NORMAL");
    Serial.println(F("========================================\n"));
}

// ===================== PUBLIC FUNCTIONS =====================
WifiMgrState getWiFiState(void) {
    return ctx.state;
}

const char* getWiFiStateName(void) {
    return getWiFiStateNameByState(ctx.state);
}

const char* getWiFiStateNameByState(WifiMgrState state) {
    if (state < WIFI_MGR_STATE_COUNT) {
        return STATE_NAMES[state];
    }
    return "UNKNOWN";
}

WifiMgrFailureType getLastFailureType(void) {
    return ctx.lastFailureType;
}

const char* getFailureTypeName(WifiMgrFailureType type) {
    if (type <= WIFI_FAIL_UNKNOWN) {
        return FAILURE_NAMES[type];
    }
    return "UNKNOWN";
}

void setWiFiOperationMode(WifiMgrOperationMode mode) {
    WifiMgrOperationMode oldMode = ctx.operationMode;
    ctx.operationMode = mode;
    
    if (mode != oldMode) {
        if (mode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
            Serial.println(F("\nðŸŽ¯ Mode: INTERMITTENT_HUNT"));
            ctx.huntStartTime = millis();
            ctx.backoffMultiplier = 1;
            
            if (ctx.state != WIFI_MGR_CONNECTED && ctx.state != WIFI_MGR_AP_MODE) {
                transitionState(WIFI_MGR_HUNT_SCANNING, "mode change");
            }
        } else {
            Serial.println(F("\nðŸ“¶ Mode: NORMAL"));
            if (ctx.state != WIFI_MGR_CONNECTED && ctx.state != WIFI_MGR_AP_MODE) {
                transitionState(WIFI_MGR_IDLE, "mode change");
            }
        }
    }
}

WifiMgrOperationMode getWiFiOperationMode(void) {
    return ctx.operationMode;
}

void enableIntermittentHuntMode(void) {
    setWiFiOperationMode(WIFI_OP_MODE_INTERMITTENT_HUNT);
}

void disableIntermittentHuntMode(void) {
    setWiFiOperationMode(WIFI_OP_MODE_NORMAL);
}

bool isInHuntMode(void) {
    return ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT;
}

bool isWiFiConnected(void) {
    return ctx.state == WIFI_MGR_CONNECTED && WiFi.status() == WL_CONNECTED;
}

uint32_t getTimeSinceLastConnection(void) {
    if (ctx.lastSuccessfulConnection == 0) return 0xFFFFFFFF;
    return millis() - ctx.lastSuccessfulConnection;
}

uint32_t getTimeSinceTargetSeen(void) {
    if (ctx.lastTargetSeenTime == 0) return 0xFFFFFFFF;
    return millis() - ctx.lastTargetSeenTime;
}

bool hasTargetBeenSeen(void) {
    return ctx.targetNetworkEverSeen;
}

int8_t getLastRSSI(void) {
    return ctx.lastRSSI;
}

int8_t getCurrentRSSI(void) {
    if (WiFi.status() == WL_CONNECTED) return WiFi.RSSI();
    return ctx.lastRSSI;
}

uint16_t getTotalScanCount(void) {
    return ctx.totalScanCount;
}

uint8_t getPoorSignalDisconnectCount(void) {
    return ctx.poorSignalDisconnectCount;
}

uint8_t getWrongPasswordCount(void) {
    return ctx.wrongPasswordCount;
}

uint8_t getAuthFailureCount(void) {
    return ctx.authFailureCount;
}

void resetWiFiManager(void) {
    Serial.println(F("ðŸ”„ Resetting WiFi Manager..."));
    
    WiFi.disconnect(true);
    WiFi.scanDelete();
    delay(100);
    
    WifiMgrOperationMode savedMode = ctx.operationMode;
    initContext();
    ctx.operationMode = savedMode;
    ctx.huntStartTime = millis();
    
    wifiConnected = false;
    isApModeActive = false;
}

void resetFailureCounters(void) {
    ctx.wrongPasswordCount = 0;
    ctx.authFailureCount = 0;
    ctx.poorSignalDisconnectCount = 0;
    ctx.poorSignalWindowStart = 0;
    ctx.lastFailureType = WIFI_FAIL_NONE;
}

void forceAPMode(const char* reason) {
    enterAPMode(reason, false, false);
}

void exitAPModeAndResume(void) {
    exitAPMode();
}

void setTargetSSID(const char* ssid) {
    strncpy(ctx.targetSSID, ssid, sizeof(ctx.targetSSID) - 1);
    ctx.targetSSID[sizeof(ctx.targetSSID) - 1] = '\0';
    ctx.targetNetworkEverSeen = false;
    ctx.lastTargetSeenTime = 0;
    ctx.wrongPasswordCount = 0;
    ctx.authFailureCount = 0;
}

// ===================== MAIN STATE MACHINE =====================
void manageWiFiConnection(void) {
    if (isMidCycleWake) return;
    
    uint32_t now = millis();
    
    if (ctx.state == WIFI_MGR_AP_MODE) {
        handleAPModeState(now);
        return;
    }
    
    if (WiFi.getMode() != WIFI_STA && ctx.state != WIFI_MGR_AP_MODE) {
        WiFi.mode(WIFI_STA);
        delay(10);
    }
    
    if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
        handleIntermittentMode(now);
    } else {
        handleNormalMode(now);
    }
}

// ===================== MODE HANDLERS =====================
static void handleNormalMode(uint32_t now) {
    switch (ctx.state) {
        case WIFI_MGR_IDLE: handleIdleState(now); break;
        case WIFI_MGR_SCANNING: handleScanningState(now); break;
        case WIFI_MGR_CONNECTING_SAVED: handleConnectingSavedState(now); break;
        case WIFI_MGR_CONNECTING_HARDCODED: handleConnectingHardcodedState(now); break;
        case WIFI_MGR_CONNECTED: handleConnectedState(now); break;
        case WIFI_MGR_WAITING_BACKOFF: handleBackoffState(now); break;
        default: transitionState(WIFI_MGR_IDLE, "invalid"); break;
    }
}

static void handleIntermittentMode(uint32_t now) {
    switch (ctx.state) {
        case WIFI_MGR_IDLE: transitionState(WIFI_MGR_HUNT_SCANNING, "hunt"); break;
        case WIFI_MGR_HUNT_SCANNING: handleHuntScanningState(now); break;
        case WIFI_MGR_HUNT_CONNECTING: handleHuntConnectingState(now); break;
        case WIFI_MGR_HUNT_LIGHT_SLEEP: handleHuntLightSleepState(now); break;
        case WIFI_MGR_CONNECTED: handleConnectedState(now); break;
        case WIFI_MGR_AP_MODE: handleAPModeState(now); break;
        default: transitionState(WIFI_MGR_HUNT_SCANNING, "recovery"); break;
    }
}

// ===================== STATE HANDLERS =====================
static void handleIdleState(uint32_t now) {
    ctx.connectionStartTime = now;
    ctx.lastAttemptTime = now;
    
    if (strlen(settings.staSsid) > 0) {
        Serial.println(F("ðŸ“¡ Trying saved network..."));
        tryConnectToNetwork(settings.staSsid, settings.staPassword);
        transitionState(WIFI_MGR_CONNECTING_SAVED, "saved");
    } else {
        transitionState(WIFI_MGR_SCANNING, "no saved");
    }
}

static void handleScanningState(uint32_t now) {
    if (!ctx.scanInProgress) {
        if (startAsyncScan()) ctx.scanInProgress = true;
        return;
    }
    
    int8_t scanResult = WiFi.scanComplete();
    if (scanResult == -1) return;
    
    ctx.scanInProgress = false;
    
    if (scanResult < 0) {
        transitionState(WIFI_MGR_WAITING_BACKOFF, "scan fail");
        return;
    }
    
    ctx.totalScanCount++;
    cacheAvailableNetworks(scanResult);
    WiFi.scanDelete();
    
    if (ctx.availableNetworkCount > 0) {
        ctx.currentHardcodedIndex = 0;
        tryConnectToHardcodedNetwork(0);
        transitionState(WIFI_MGR_CONNECTING_HARDCODED, "found");
    } else {
        handleSSIDNotFound();
    }
}

static void handleConnectingSavedState(uint32_t now) {
    wl_status_t status = WiFi.status();
    
    if (status == WL_CONNECTED) return;
    
    // WL_CONNECT_FAILED usually means auth failure
    if (status == WL_CONNECT_FAILED) { 
        handleAuthFailure(); 
        return; 
    }
    
    if (status == WL_NO_SSID_AVAIL) { 
        handleSSIDNotFound(); 
        return; 
    }
    
    if (now - ctx.lastAttemptTime >= WIFIMGR_MAX_CONNECT_TIMEOUT_MS) {
        ctx.savedNetworkAttempts++;
        Serial.printf(" Timeout (%d/%d)\n", ctx.savedNetworkAttempts, WIFIMGR_MAX_STA_ATTEMPTS);
        
        if (ctx.savedNetworkAttempts >= WIFIMGR_MAX_STA_ATTEMPTS) {
            transitionState(WIFI_MGR_SCANNING, "max attempts");
        } else {
            transitionState(WIFI_MGR_WAITING_BACKOFF, "timeout");
        }
    }
}

static void handleConnectingHardcodedState(uint32_t now) {
    wl_status_t status = WiFi.status();
    
    if (status == WL_CONNECTED) return;
    
    if (status == WL_CONNECT_FAILED) { 
        handleAuthFailure(); 
        return; 
    }
    
    if (now - ctx.lastAttemptTime >= WIFIMGR_MAX_CONNECT_TIMEOUT_MS) {
        ctx.currentHardcodedIndex++;
        if (ctx.currentHardcodedIndex < ctx.availableNetworkCount) {
            tryConnectToHardcodedNetwork(ctx.currentHardcodedIndex);
            ctx.lastAttemptTime = now;
        } else {
            if (!shouldFallbackToAP()) {
                transitionState(WIFI_MGR_WAITING_BACKOFF, "all tried");
            }
        }
    }
}
/*
static void handleConnectingHardcodedState(uint32_t now) {
    wl_status_t status = WiFi.status();
    
    if (status == WL_CONNECTED) return;
    if (status == WL_WRONG_PASSWORD) { handleWrongPassword(); return; }
    if (status == WL_CONNECT_FAILED) { handleAuthFailure(); return; }
    
    if (now - ctx.lastAttemptTime >= WIFIMGR_MAX_CONNECT_TIMEOUT_MS) {
        ctx.currentHardcodedIndex++;
        if (ctx.currentHardcodedIndex < ctx.availableNetworkCount) {
            tryConnectToHardcodedNetwork(ctx.currentHardcodedIndex);
            ctx.lastAttemptTime = now;
        } else {
            if (!shouldFallbackToAP()) {
                transitionState(WIFI_MGR_WAITING_BACKOFF, "all tried");
            }
        }
    }
}
*/

static void handleConnectedState(uint32_t now) {
    if (WiFi.status() != WL_CONNECTED) {
        onConnectionLost();
        if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
            transitionState(WIFI_MGR_HUNT_SCANNING, "lost");
        } else {
            transitionState(WIFI_MGR_IDLE, "lost");
        }
        return;
    }
    
    // Track RSSI
    static uint32_t lastRSSICheck = 0;
    if (now - lastRSSICheck >= 1000) {
        lastRSSICheck = now;
        ctx.lastConnectedRSSI = WiFi.RSSI();
        ctx.lastRSSI = ctx.lastConnectedRSSI;
        if (ctx.lastRSSI > ctx.bestSeenRSSI) ctx.bestSeenRSSI = ctx.lastRSSI;
    }
    
    // Reset poor signal counter after stable good connection
    if (ctx.lastConnectedRSSI >= WIFIMGR_POOR_SIGNAL_THRESHOLD_DBM) {
        if (ctx.goodSignalStartTime == 0) {
            ctx.goodSignalStartTime = now;
        } else if (now - ctx.goodSignalStartTime >= WIFIMGR_GOOD_SIGNAL_RESET_MS) {
            if (ctx.poorSignalDisconnectCount > 0) {
                ctx.poorSignalDisconnectCount = 0;
                ctx.poorSignalWindowStart = 0;
            }
            ctx.goodSignalStartTime = now;
        }
    } else {
        ctx.goodSignalStartTime = 0;
    }
    
    ctx.consecutiveDisconnects = 0;
}

static void handleBackoffState(uint32_t now) {
    uint32_t backoffDelay = calculateBackoffDelay();
    yieldWithLightSleep(10);
    
    if (now - ctx.lastAttemptTime >= backoffDelay) {
        transitionState(WIFI_MGR_IDLE, "backoff done");
    }
}

static void handleAPModeState(uint32_t now) {
    checkAPTimeout();
    
    if (WIFIMGR_AP_AUTO_EXIT_ENABLED && 
        ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
        uint32_t apTimeout = ctx.apModeTriggeredByAuthFail 
                             ? WIFIMGR_AP_MODE_DURATION_MS * 2 
                             : WIFIMGR_AP_MODE_DURATION_MS;
        
        if (now - ctx.apModeStartTime >= apTimeout) {
            if (WiFi.softAPgetStationNum() == 0) {
                exitAPMode();
            }
        }
    }
}

// ===================== HUNT MODE HANDLERS =====================
static void handleHuntScanningState(uint32_t now) {
    // Check AP fallback first
    if (shouldFallbackToAP()) return;
    
    Serial.println(F("\n ===== HUNT SCAN ====="));
    
    // Make sure WiFi is in correct mode
    if (WiFi.getMode() != WIFI_STA) {
        Serial.println(F(" WiFi not in STA mode! Fixing..."));
        WiFi.disconnect(true);
        delay(100);
        WiFi.mode(WIFI_STA);
        delay(500);
    }
    
    Serial.printf("WiFi Mode: %d (1=STA)\n", WiFi.getMode());
    
    // Use BLOCKING scan (more reliable than async after sleep issues)
    Serial.println(F("Scanning..."));
    int scanResult = WiFi.scanNetworks(false, true);  // blocking, show hidden
    
    ctx.totalScanCount++;
    Serial.printf("Found %d networks:\n", scanResult);
    
    if (scanResult <= 0) {
        Serial.println(F("   (none found)"));
        Serial.println(F("========================\n"));
        
        WiFi.scanDelete();
        ctx.consecutiveScanMisses++;
        
        // Wait and try again
        transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "no networks");
        return;
    }
    
    // Print all found networks
    bool targetFound = false;
    const char* targetSSID = strlen(ctx.targetSSID) > 0 ? ctx.targetSSID : settings.staSsid;
    
    for (int i = 0; i < scanResult; i++) {
        String ssid = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);
        
        bool isTarget = (strlen(targetSSID) > 0 && ssid.equals(targetSSID));
        bool isHardcoded = false;
        
        // Check hardcoded networks
        for (size_t h = 0; h < NUM_HARDCODED_NETWORKS; h++) {
            if (ssid.equals(hardcodedNetworks[h].ssid)) {
                isHardcoded = true;
                if (!targetFound) {
                    // Use first hardcoded network found if no specific target
                    strncpy(ctx.targetSSID, hardcodedNetworks[h].ssid, sizeof(ctx.targetSSID) - 1);
                }
                break;
            }
        }
        
        Serial.printf("   %d: [%s] %d dBm", i, ssid.c_str(), rssi);
        if (isTarget) {
            Serial.print(F(" <-- TARGET"));
            targetFound = true;
            ctx.lastRSSI = rssi;
        }
        if (isHardcoded) {
            Serial.print(F(" (known)"));
            if (!targetFound) {
                targetFound = true;
                ctx.lastRSSI = rssi;
            }
        }
        Serial.println();
    }
    
    WiFi.scanDelete();
    Serial.println(F("========================\n"));
    
    if (targetFound) {
        Serial.println(F("TARGET FOUND! Connecting..."));
        
        ctx.targetNetworkEverSeen = true;
        ctx.lastTargetSeenTime = now;
        ctx.consecutiveScanMisses = 0;
        ctx.quickRetryCount = 0;
        
        rtcWiFiData.targetEverSeen = true;
        saveRTCWiFiData();
        
        // Try to connect
        const char* ssidToUse = NULL;
        const char* passToUse = NULL;
        
        // First try saved credentials
        if (strlen(settings.staSsid) > 0) {
            ssidToUse = settings.staSsid;
            passToUse = settings.staPassword;
        }
        
        // If no saved credentials or target is different, check hardcoded
        if (ssidToUse == NULL || !String(ssidToUse).equals(ctx.targetSSID)) {
            for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                if (String(hardcodedNetworks[i].ssid).equals(ctx.targetSSID)) {
                    ssidToUse = hardcodedNetworks[i].ssid;
                    passToUse = hardcodedNetworks[i].password;
                    break;
                }
            }
        }
        
        if (ssidToUse != NULL) {
            Serial.printf("   SSID: [%s]\n", ssidToUse);
            Serial.printf("   Pass: [%s]\n", passToUse);
            tryConnectToNetwork(ssidToUse, passToUse);
            transitionState(WIFI_MGR_HUNT_CONNECTING, "found");
        } else {
            Serial.println(F("    No credentials found!"));
            transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "no creds");
        }
        
    } else {
        ctx.consecutiveScanMisses++;
        Serial.printf("Target not found (miss #%d)\n", ctx.consecutiveScanMisses);
        transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "not found");
    }
}
/*
static void handleHuntScanningState(uint32_t now) {
    if (shouldFallbackToAP()) return;
    
    if (!ctx.scanInProgress) {
        Serial.println(F("ðŸ” Hunt scan..."));
        if (startAsyncScan()) ctx.scanInProgress = true;
        return;
    }
    
    int8_t scanResult = WiFi.scanComplete();
    if (scanResult == -1) { yieldWithLightSleep(10); return; }
    
    ctx.scanInProgress = false;
    ctx.totalScanCount++;
    
    if (scanResult < 0) {
        transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "scan fail");
        return;
    }
    
    Serial.printf("   Found %d networks\n", scanResult);
    ctx.lastScanCompleteTime = now;
    
    bool targetFound = isTargetInScanResults(scanResult);
    WiFi.scanDelete();
    
    if (targetFound) {
        Serial.println(F("ðŸŽ¯ TARGET FOUND!"));
        ctx.targetNetworkEverSeen = true;
        ctx.lastTargetSeenTime = now;
        ctx.consecutiveScanMisses = 0;
        ctx.quickRetryCount = 0;
        
        rtcWiFiData.targetEverSeen = true;
        saveRTCWiFiData();
        
        if (strlen(settings.staSsid) > 0) {
            tryConnectToNetwork(settings.staSsid, settings.staPassword);
        } else {
            for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                if (strcmp(hardcodedNetworks[i].ssid, ctx.targetSSID) == 0) {
                    tryConnectToNetwork(hardcodedNetworks[i].ssid, 
                                        hardcodedNetworks[i].password);
                    break;
                }
            }
        }
        transitionState(WIFI_MGR_HUNT_CONNECTING, "found");
    } else {
        ctx.consecutiveScanMisses++;
        transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "not found");
    }
}
*/
static void handleHuntConnectingState(uint32_t now) {
    wl_status_t status = WiFi.status();
    
    // Print status every second
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.printf("   Connect status: %d ", status);
        switch(status) {
            case WL_IDLE_STATUS: Serial.println(F("(idle)")); break;
            case WL_NO_SSID_AVAIL: Serial.println(F("(no SSID)")); break;
            case WL_CONNECTED: Serial.println(F("(connected!)")); break;
            case WL_CONNECT_FAILED: Serial.println(F("(failed)")); break;
            case WL_DISCONNECTED: Serial.println(F("(disconnected/wrong pw)")); break;
            default: Serial.printf("(code %d)\n", status); break;
        }
    }
    
    // Connected - handled by event callback
    if (status == WL_CONNECTED) return;
    
    // Connection failed
    if (status == WL_CONNECT_FAILED) {
        Serial.println(F(" Connection failed!"));
        ctx.authFailureCount++;
        if (ctx.authFailureCount >= WIFIMGR_MAX_AUTH_FAILURES) {
            enterAPMode("Auth failed", true, false);
            return;
        }
        transitionState(WIFI_MGR_HUNT_SCANNING, "connect failed");
        return;
    }
    
    // SSID not available (disappeared during connect)
    if (status == WL_NO_SSID_AVAIL) {
        Serial.println(F(" SSID disappeared!"));
        transitionState(WIFI_MGR_HUNT_SCANNING, "ssid gone");
        return;
    }
    
    // Timeout
    if (now - ctx.lastAttemptTime >= WIFIMGR_HUNT_CONNECT_TIMEOUT_MS) {
        ctx.quickRetryCount++;
        Serial.printf(" Timeout! Quick retry %d/%d\n", 
                      ctx.quickRetryCount, WIFIMGR_HUNT_MAX_QUICK_RETRIES);
        
        if (ctx.quickRetryCount < WIFIMGR_HUNT_MAX_QUICK_RETRIES) {
            // Quick retry
            if (strlen(settings.staSsid) > 0) {
                tryConnectToNetwork(settings.staSsid, settings.staPassword);
            } else if (strlen(ctx.targetSSID) > 0) {
                // Find password for target
                for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                    if (String(hardcodedNetworks[i].ssid).equals(ctx.targetSSID)) {
                        tryConnectToNetwork(hardcodedNetworks[i].ssid, 
                                           hardcodedNetworks[i].password);
                        break;
                    }
                }
            }
            ctx.lastAttemptTime = now;
        } else {
            Serial.println(F("   Max retries reached, scanning again..."));
            ctx.quickRetryCount = 0;
            transitionState(WIFI_MGR_HUNT_SCANNING, "max retries");
        }
    }
    
    yield();
}
/*
static void handleHuntConnectingState(uint32_t now) {
    wl_status_t status = WiFi.status();
    
    if (status == WL_CONNECTED) return;
    if (status == WL_WRONG_PASSWORD) { handleWrongPassword(); return; }
    
    if (status == WL_CONNECT_FAILED) {
        ctx.authFailureCount++;
        if (ctx.authFailureCount >= WIFIMGR_MAX_AUTH_FAILURES) {
            enterAPMode("Auth failed", true, false);
            return;
        }
    }
    
    if (now - ctx.lastAttemptTime >= WIFIMGR_HUNT_CONNECT_TIMEOUT_MS) {
        ctx.quickRetryCount++;
        if (ctx.quickRetryCount < WIFIMGR_HUNT_MAX_QUICK_RETRIES) {
            if (strlen(settings.staSsid) > 0) {
                tryConnectToNetwork(settings.staSsid, settings.staPassword);
            }
            ctx.lastAttemptTime = now;
        } else {
            ctx.quickRetryCount = 0;
            transitionState(WIFI_MGR_HUNT_SCANNING, "retries done");
        }
    }
}
*/
static void handleHuntLightSleepState(uint32_t now) {
    // OPTION 1: Don't use forceSleep at all (more reliable)
    // Just wait a few seconds before scanning again
    
    if (ctx.lightSleepStartTime == 0) {
        ctx.lightSleepStartTime = now;
        Serial.printf(" Waiting %d ms before next scan...\n", WIFIMGR_HUNT_LIGHT_SLEEP_MS);
        
        // DON'T use forceSleepBegin - it breaks WiFi scanning!
        // WiFi.forceSleepBegin();  // COMMENTED OUT!
    }
    
    if (now - ctx.lightSleepStartTime >= WIFIMGR_HUNT_LIGHT_SLEEP_MS) {
        ctx.lightSleepStartTime = 0;
        
        // Make sure WiFi is ready for scanning
        if (WiFi.getMode() != WIFI_STA) {
            Serial.println(F("   Fixing WiFi mode..."));
            WiFi.mode(WIFI_STA);
            delay(200);
        }
        
        transitionState(WIFI_MGR_HUNT_SCANNING, "wait done");
    } else {
        // Just delay, don't sleep
        delay(100);
        yield();
    }
}
/*
static void handleHuntLightSleepState(uint32_t now) {
    if (ctx.lightSleepStartTime == 0) {
        ctx.lightSleepStartTime = now;
        WiFi.forceSleepBegin();
        delay(1);
    }
    
    if (now - ctx.lightSleepStartTime >= WIFIMGR_HUNT_LIGHT_SLEEP_MS) {
        exitLightSleep();
        ctx.lightSleepStartTime = 0;
        transitionState(WIFI_MGR_HUNT_SCANNING, "sleep done");
    } else {
        delay(100);
    }
}
*/
// ===================== EVENT CALLBACKS =====================
static void onWiFiStationConnected(const WiFiEventStationModeConnected& event) {
    Serial.print(F("ðŸ“¶ Associated: "));
    Serial.println(event.ssid.c_str());
    ctx.isConnecting = true;
}

static void onWiFiStationDisconnected(const WiFiEventStationModeDisconnected& event) {
    uint32_t now = millis();
    
    Serial.println(F("\nâŒ DISCONNECTED"));
    Serial.printf("   Reason: %d, RSSI: %d\n", event.reason, ctx.lastConnectedRSSI);
    
    bool wasPasswordIssue = false;
    bool wasPoorSignal = false;
    
    switch (event.reason) {
        case WIFI_DISCONNECT_REASON_AUTH_FAIL:
        case WIFI_DISCONNECT_REASON_AUTH_EXPIRE:
        case WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:
            ctx.lastFailureType = WIFI_FAIL_WRONG_PASSWORD;
            wasPasswordIssue = true;
            break;
        case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:
            ctx.lastFailureType = WIFI_FAIL_CONNECTION_LOST;
            if (ctx.lastConnectedRSSI < WIFIMGR_POOR_SIGNAL_THRESHOLD_DBM) {
                wasPoorSignal = true;
                ctx.lastFailureType = WIFI_FAIL_POOR_SIGNAL;
            }
            break;
        case WIFI_DISCONNECT_REASON_NO_AP_FOUND:
            ctx.lastFailureType = WIFI_FAIL_SSID_NOT_FOUND;
            break;
        default:
            ctx.lastFailureType = WIFI_FAIL_UNKNOWN;
            if (ctx.lastConnectedRSSI < WIFIMGR_POOR_SIGNAL_THRESHOLD_DBM) {
                wasPoorSignal = true;
            }
            break;
    }
    
    if (wasPoorSignal && ctx.state == WIFI_MGR_CONNECTED) {
        handlePoorSignalDisconnect();
    }
    
    if (wasPasswordIssue) {
        handleWrongPassword();
        return;
    }
    
    if (ctx.state == WIFI_MGR_CONNECTED) {
        onConnectionLost();
        ctx.lastDisconnectTime = now;
        ctx.consecutiveDisconnects++;
        
        if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
            ctx.quickRetryCount = 0;
            transitionState(WIFI_MGR_HUNT_SCANNING, "disconnect");
        } else {
            transitionState(WIFI_MGR_IDLE, "disconnect");
        }
    }
    
    ctx.isConnecting = false;
}

static void onWiFiGotIP(const WiFiEventStationModeGotIP& event) {
    Serial.println(F("\nâœ… CONNECTED"));
    Serial.print(F("   IP: ")); Serial.println(event.ip);
    Serial.printf("   RSSI: %d dBm\n", WiFi.RSSI());
    
    ctx.isConnecting = false;
    transitionState(WIFI_MGR_CONNECTED, "got IP");
    
    // Reset all counters
    ctx.savedNetworkAttempts = 0;
    ctx.currentHardcodedIndex = 0;
    ctx.backoffMultiplier = 1;
    ctx.quickRetryCount = 0;
    ctx.wrongPasswordCount = 0;
    ctx.authFailureCount = 0;
    ctx.consecutiveScanMisses = 0;
    ctx.consecutiveDisconnects = 0;
    ctx.lastFailureType = WIFI_FAIL_NONE;
    rtcWiFiData.failedAttemptsSinceSuccess = 0;
    
    ctx.lastSuccessfulConnection = millis();
    ctx.lastConnectedRSSI = WiFi.RSSI();
    ctx.lastRSSI = ctx.lastConnectedRSSI;
    ctx.goodSignalStartTime = 0;
    
    if (ctx.lastRSSI > ctx.bestSeenRSSI) ctx.bestSeenRSSI = ctx.lastRSSI;
    
    strncpy(rtcWiFiData.lastConnectedSSID, WiFi.SSID().c_str(), 
            sizeof(rtcWiFiData.lastConnectedSSID) - 1);
    saveRTCWiFiData();
    
    if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
        onHuntConnectionEstablished();
    } else {
        onConnectionEstablished();
    }
}

// ===================== FAILURE HANDLERS =====================
static void handleWrongPassword(void) {
    ctx.wrongPasswordCount++;
    ctx.lastFailureType = WIFI_FAIL_WRONG_PASSWORD;
    
    Serial.printf("ðŸ” Wrong password: %d/%d\n", 
                  ctx.wrongPasswordCount, WIFIMGR_MAX_WRONG_PASSWORD_ATTEMPTS);
    
    if (ctx.wrongPasswordCount >= WIFIMGR_MAX_WRONG_PASSWORD_ATTEMPTS) {
        enterAPMode("Wrong password", true, false);
    } else {
        if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
            transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "wrong pw");
        } else {
            transitionState(WIFI_MGR_WAITING_BACKOFF, "wrong pw");
        }
    }
}

static void handleAuthFailure(void) {
    ctx.authFailureCount++;
    ctx.lastFailureType = WIFI_FAIL_WRONG_PASSWORD;
    
    Serial.printf("ðŸ” Auth failure: %d/%d\n", 
                  ctx.authFailureCount, WIFIMGR_MAX_AUTH_FAILURES);
    
    if (ctx.authFailureCount >= WIFIMGR_MAX_AUTH_FAILURES) {
        enterAPMode("Auth failed", true, false);
    } else {
        if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
            transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "auth fail");
        } else {
            transitionState(WIFI_MGR_WAITING_BACKOFF, "auth fail");
        }
    }
}

static void handleSSIDNotFound(void) {
    ctx.consecutiveScanMisses++;
    ctx.lastFailureType = WIFI_FAIL_SSID_NOT_FOUND;
    
    if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
        transitionState(WIFI_MGR_HUNT_LIGHT_SLEEP, "not found");
    } else {
        transitionState(WIFI_MGR_WAITING_BACKOFF, "not found");
    }
}

static void handlePoorSignalDisconnect(void) {
    uint32_t now = millis();
    
    if (ctx.poorSignalWindowStart > 0 && 
        (now - ctx.poorSignalWindowStart) > WIFIMGR_POOR_SIGNAL_WINDOW_MS) {
        ctx.poorSignalDisconnectCount = 0;
        ctx.poorSignalWindowStart = 0;
    }
    
    if (ctx.poorSignalWindowStart == 0) ctx.poorSignalWindowStart = now;
    
    ctx.poorSignalDisconnectCount++;
    ctx.lastDisconnectWasPoorSignal = true;
    ctx.lastFailureType = WIFI_FAIL_POOR_SIGNAL;
    
    Serial.printf("ðŸ“¶ Poor signal: %d/%d (RSSI: %d)\n",
                  ctx.poorSignalDisconnectCount,
                  WIFIMGR_MAX_POOR_SIGNAL_DISCONNECTS,
                  ctx.lastConnectedRSSI);
}

static bool shouldFallbackToAP(void) {
    uint32_t now = millis();
    
    // Wrong password
    if (ctx.wrongPasswordCount >= WIFIMGR_MAX_WRONG_PASSWORD_ATTEMPTS) {
        enterAPMode("Wrong password", true, false);
        return true;
    }
    
    // Auth failures
    if (ctx.authFailureCount >= WIFIMGR_MAX_AUTH_FAILURES) {
        enterAPMode("Auth failed", true, false);
        return true;
    }
    
    // Poor signal window reset
    if (ctx.poorSignalWindowStart > 0 && 
        (now - ctx.poorSignalWindowStart) > WIFIMGR_POOR_SIGNAL_WINDOW_MS) {
        ctx.poorSignalDisconnectCount = 0;
        ctx.poorSignalWindowStart = 0;
    }
    
    // Poor signal disconnects
    if (ctx.poorSignalDisconnectCount >= WIFIMGR_MAX_POOR_SIGNAL_DISCONNECTS) {
        enterAPMode("Poor signal", false, true);
        return true;
    }
    
    // Protect Telegram
    if (telegramSendPending || telegramDelayActive || telegramSending) {
        return false;
    }
    
    return false;
}

static void enterAPMode(const char* reason, bool authFailure, bool poorSignal) {
    Serial.println(F("\nðŸ“¡ ===== ENTERING AP MODE ====="));
    Serial.print(F("   Reason: ")); Serial.println(reason);
    
    ctx.apModeTriggeredByAuthFail = authFailure;
    ctx.apModeTriggeredByPoorSignal = poorSignal;
    ctx.apModeStartTime = millis();
    
    WiFi.disconnect(true);
    WiFi.scanDelete();
    ctx.scanInProgress = false;
    ctx.isConnecting = false;
    
    delay(100);
    setupAP();
    
    isApModeActive = true;
    transitionState(WIFI_MGR_AP_MODE, reason);
}

static void exitAPMode(void) {
    Serial.println(F("\nðŸ“¡ Exiting AP mode...\n"));
    
    isApModeActive = false;
    ctx.apModeTriggeredByAuthFail = false;
    ctx.apModeTriggeredByPoorSignal = false;
    
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    delay(100);
    
    ctx.consecutiveScanMisses = 0;
    ctx.huntStartTime = millis();
    
    if (ctx.apModeTriggeredByPoorSignal) {
        ctx.poorSignalDisconnectCount = 0;
        ctx.poorSignalWindowStart = 0;
    }
    
    if (ctx.operationMode == WIFI_OP_MODE_INTERMITTENT_HUNT) {
        transitionState(WIFI_MGR_HUNT_SCANNING, "AP exit");
    } else {
        transitionState(WIFI_MGR_IDLE, "AP exit");
    }
}

// ===================== CONNECTION HANDLERS =====================
static void onConnectionEstablished(void) {
    wifiConnected = true;
    isApModeActive = false;
    
    if (!telegramTriggeredForThisConnection) {
        Serial.println(F("\nðŸ“± >>> TELEGRAM TRIGGERED <<<\n"));
        telegramTriggeredForThisConnection = true;
        telegramSendPending = true;
        telegramDelayActive = false;
        telegramSending = false;
        telegramRetryCount = 0;
    }
    
    saveConnectedCredentialsIfNew();
    
    if (lastTimeSync == 0) {
        syncTimeFromNTP();
        lastTimeSync = millis();
    }
}

static void onHuntConnectionEstablished(void) {
    Serial.println(F("\nðŸŽ¯ ===== HUNT SUCCESS! ====="));
    
    wifiConnected = true;
    isApModeActive = false;
    
    ctx.lastConnectedRSSI = WiFi.RSSI();
    ctx.lastRSSI = ctx.lastConnectedRSSI;
    ctx.huntStartTime = millis();
    ctx.consecutiveDisconnects = 0;
    
    Serial.printf("   RSSI: %d dBm %s\n", ctx.lastConnectedRSSI, 
                  getSignalQuality(ctx.lastConnectedRSSI));
    
    if (!telegramTriggeredForThisConnection) {
        Serial.println(F("ðŸ“± >>> IMMEDIATE TELEGRAM <<<"));
        telegramTriggeredForThisConnection = true;
        telegramSendPending = true;
        telegramDelayActive = false;
        telegramSending = false;
        telegramRetryCount = 0;
    }
    
    syncTimeFromNTP();
    lastTimeSync = millis();
    
    saveConnectedCredentialsIfNew();
    Serial.println(F("============================\n"));
}

static void onConnectionLost(void) {
    wifiConnected = false;
    telegramTriggeredForThisConnection = false;
    ctx.goodSignalStartTime = 0;
}

static void saveConnectedCredentialsIfNew(void) {
    String currentSSID = WiFi.SSID();
    
    if (!currentSSID.equals(settings.staSsid)) {
        currentSSID.toCharArray(settings.staSsid, sizeof(settings.staSsid));
        
        for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
            if (currentSSID.equals(hardcodedNetworks[i].ssid)) {
                strncpy(settings.staPassword, hardcodedNetworks[i].password,
                        sizeof(settings.staPassword) - 1);
                settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
                break;
            }
        }
        saveSettings();
    }
    
    strncpy(ctx.targetSSID, settings.staSsid, sizeof(ctx.targetSSID) - 1);
}

// ===================== HELPER FUNCTIONS =====================
static void transitionState(WifiMgrState newState, const char* reason) {
    if (ctx.state != newState) {
        ctx.previousState = ctx.state;
        Serial.printf("ðŸ”„ %s â†’ %s", getWiFiStateNameByState(ctx.state), 
                      getWiFiStateNameByState(newState));
        if (reason && strlen(reason) > 0) Serial.printf(" (%s)", reason);
        Serial.println();
        ctx.state = newState;
        ctx.lastAttemptTime = millis();
    }
}

static bool tryConnectToNetwork(const char* ssid, const char* password) {
    WiFi.disconnect(true);
    delay(10);
    Serial.print(F("ðŸ“¶ Connecting: ")); Serial.println(ssid);
    WiFi.begin(ssid, password);
    ctx.connectionStartTime = millis();
    ctx.lastAttemptTime = millis();
    ctx.isConnecting = true;
    return true;
}

static void tryConnectToHardcodedNetwork(uint8_t index) {
    if (index >= ctx.availableNetworkCount) return;
    uint8_t actualIndex = ctx.cachedNetworkIndices[index];
    tryConnectToNetwork(hardcodedNetworks[actualIndex].ssid,
                        hardcodedNetworks[actualIndex].password);
}

static bool startAsyncScan(void) {
    int8_t status = WiFi.scanComplete();
    if (status == -1) return true;
    if (status >= 0) WiFi.scanDelete();
    WiFi.scanNetworks(true, true);
    ctx.lastScanTime = millis();
    return true;
}

static bool isTargetInScanResults(int scanCount) {
    const char* target = strlen(ctx.targetSSID) > 0 ? ctx.targetSSID : settings.staSsid;
    if (strlen(target) == 0) return false;
    
    for (int i = 0; i < scanCount; i++) {
        if (WiFi.SSID(i).equals(target)) {
            ctx.lastRSSI = WiFi.RSSI(i);
            return true;
        }
    }
    
    for (size_t h = 0; h < NUM_HARDCODED_NETWORKS; h++) {
        for (int i = 0; i < scanCount; i++) {
            if (WiFi.SSID(i).equals(hardcodedNetworks[h].ssid)) {
                ctx.lastRSSI = WiFi.RSSI(i);
                return true;
            }
        }
    }
    return false;
}

static void cacheAvailableNetworks(int scanCount) {
    ctx.availableNetworkCount = 0;
    for (uint8_t h = 0; h < NUM_HARDCODED_NETWORKS && 
                        ctx.availableNetworkCount < sizeof(ctx.cachedNetworkIndices); h++) {
        for (int i = 0; i < scanCount; i++) {
            if (WiFi.SSID(i).equals(hardcodedNetworks[h].ssid)) {
                ctx.cachedNetworkIndices[ctx.availableNetworkCount++] = h;
                break;
            }
        }
    }
    ctx.scanResultsCached = true;
}

static uint32_t calculateBackoffDelay(void) {
    uint32_t backoff = WIFIMGR_RECONNECT_INTERVAL_MS * ctx.backoffMultiplier;
    if (ctx.backoffMultiplier < WIFIMGR_MAX_BACKOFF_MULTIPLIER) {
        ctx.backoffMultiplier *= 2;
    }
    return backoff;
}

// ===================== POWER MANAGEMENT =====================
static void exitLightSleep(void) {
    Serial.println(F(" Waking WiFi from sleep..."));
    
    // Wake up WiFi
    WiFi.forceSleepWake();
    delay(100);  // IMPORTANT: Wait for radio to stabilize
    
    // Re-enable STA mode (forceSleep may have changed it)
    WiFi.mode(WIFI_STA);
    delay(200);  // IMPORTANT: Wait for mode to apply
    
    Serial.printf("   WiFi Mode: %d (should be 1)\n", WiFi.getMode());
}
static void yieldWithLightSleep(uint32_t ms) {
    // DON'T use forceSleep - just regular delay
    if (ms > 0) {
        delay(ms);
    }
    yield();
}
/*
static void yieldWithLightSleep(uint32_t ms) {
    if (ms >= 10) {
        WiFi.forceSleepBegin();
        delay(ms);
        WiFi.forceSleepWake();
        delay(1);
    } else {
        delay(ms);
    }
    yield();
}
*/

// ===================== RTC PERSISTENCE =====================
static uint32_t calculateCRC32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    while (length--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

static void loadRTCWiFiData(void) {
    ESP.rtcUserMemoryRead(64, (uint32_t*)&rtcWiFiData, sizeof(rtcWiFiData));
    
    uint32_t expectedCRC = calculateCRC32(
        ((uint8_t*)&rtcWiFiData) + sizeof(rtcWiFiData.crc),
        sizeof(rtcWiFiData) - sizeof(rtcWiFiData.crc));
    
    if (expectedCRC != rtcWiFiData.crc) {
        memset(&rtcWiFiData, 0, sizeof(rtcWiFiData));
    }
}

static void saveRTCWiFiData(void) {
    rtcWiFiData.crc = calculateCRC32(
        ((uint8_t*)&rtcWiFiData) + sizeof(rtcWiFiData.crc),
        sizeof(rtcWiFiData) - sizeof(rtcWiFiData.crc));
    ESP.rtcUserMemoryWrite(64, (uint32_t*)&rtcWiFiData, sizeof(rtcWiFiData));
}

// ===================== UTILITY =====================
static const char* getSignalQuality(int8_t rssi) {
    if (rssi >= -50) return "(Excellent)";
    if (rssi >= -60) return "(Good)";
    if (rssi >= -70) return "(Fair)";
    if (rssi >= -80) return "(Weak)";
    return "(Very Weak)";
}

// ===================== DEBUG =====================
void printWiFiManagerStatus(void) {
    Serial.println(F("\n========== WiFi Manager =========="));
    Serial.printf("State: %s\n", getWiFiStateName());
    Serial.printf("Mode: %s\n", isInHuntMode() ? "HUNT" : "NORMAL");
    Serial.printf("Connected: %s\n", wifiConnected ? "YES" : "NO");
    
    if (wifiConnected) {
        Serial.printf("RSSI: %d dBm %s\n", WiFi.RSSI(), getSignalQuality(WiFi.RSSI()));
        Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
    }
    
    Serial.printf("Target: %s\n", strlen(ctx.targetSSID) > 0 ? ctx.targetSSID : "(none)");
    Serial.printf("Scans: %d\n", ctx.totalScanCount);
    Serial.printf("Wrong PW: %d/%d\n", ctx.wrongPasswordCount, WIFIMGR_MAX_WRONG_PASSWORD_ATTEMPTS);
    Serial.printf("Auth fail: %d/%d\n", ctx.authFailureCount, WIFIMGR_MAX_AUTH_FAILURES);
    Serial.printf("Poor signal: %d/%d\n", ctx.poorSignalDisconnectCount, WIFIMGR_MAX_POOR_SIGNAL_DISCONNECTS);
    Serial.printf("Heap: %u\n", ESP.getFreeHeap());
    Serial.println(F("==================================\n"));
}
