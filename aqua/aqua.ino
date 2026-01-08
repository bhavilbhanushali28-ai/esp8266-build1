#include "rtc_data.h"
RTCData rtcData;
uint32_t rtcChecksum(const RTCData &d) {
  return d.magic ^ d.remainingSleepSeconds;
}
#include "WiFiConfig.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h> 
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h> 
#include <Ticker.h>
#include <DNSServer.h>
#include <MD5Builder.h>
#include <vector>
#include <algorithm>
#include <time.h> 
#include <functional> 
#include <pgmspace.h>
#include <TimeLib.h> 
#include <AsyncJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <core_version.h>
#include "WiFiManager.h"

// ===================== TELEGRAM CONFIGURATION =====================
#define BOT_TOKEN "8069292937:AAGleYiuXQjYCr0K24k6tEazagmqfCXlud8"  
#define CHAT_ID   "5348635862"

// Telegram timing and safety constants
#define TELEGRAM_DELAY_MS 4000
#define TELEGRAM_MIN_HEAP 15000
#define TELEGRAM_STABILITY_MS 2500
#define TELEGRAM_SEND_TIMEOUT_MS 15000

// Telegram objects - use pointer for lazy initialization
WiFiClientSecure secureClient;
UniversalTelegramBot* bot = nullptr;

// Telegram state variables
bool telegramSending = false;
bool telegramDelayActive = false;
bool telegramSendPending = false;
bool telegramTriggeredForThisConnection = false;
bool telegramClientConfigured = false;
uint32_t telegramSendStartTime = 0;
uint32_t telegramConnectionStableTime = 0;
uint8_t telegramRetryCount = 0;
const uint8_t MAX_TELEGRAM_RETRIES = 2;

// Web server state
bool webServerStartPending = false;
unsigned long bootStart = 0;
bool pendingAPSetup = false;

extern time_t relayOnEpochTime;  
time_t relayOnEpochTime = 0;
const long gmtOffset_sec = 19800;        
const int daylightOffset_sec = 0;        
char pendingFailsafeReason[48] = {0};
time_t estimatedEpochTime = 0;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); 
unsigned long lastEpoch = 0;
unsigned long lastSyncMillis = 0;
unsigned long lastTimeSync = 0;
volatile bool fsLock = false;
volatile bool shouldSaveSettingsFlag = false;
volatile bool relayTimedOffPending = false;
uint32_t relayStartMillis = 0;
bool resumeSleepAfterBoot = false;
WiFiClient espClient; 
bool wifiConnected = false;
unsigned long lastWifiConnectAttemptMillis = 0;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 30000; 
unsigned long lastScanStartTime = 0;
const unsigned long WIFI_SCAN_INTERVAL_MS = 60000; 
bool scanInProgress = false; 
bool isTemporaryRun = false;
bool isMidCycleWake = false;        
bool serverRunning = false;

#define FAILSAFE_LOG_FILE "/failsafe.bin"
#define RELAY_LOG_FILE    "/relay.bin"
#define MAX_LOGS          10
#define LOG_ENTRY_LENGTH  128
#define DEBUG_MODE 1

const unsigned long DEBOUNCE_DELAY = 100;
unsigned long apStartTime = 0;
unsigned long sleepRequestTime = 0;
volatile unsigned long lastEmergencyPressTime = 0;
extern AsyncCallbackJsonWebHandler* saveSettingsHandler;
const byte DNS_PORT = 53;
DNSServer dnsServer;
bool dnsServerActive = false;
uint8_t relayLogIndex = 0;
uint8_t failsafeLogIndex = 0;
#define SWITCH_PIN D2
unsigned long lastSwitchStateChangeTime = 0;
const unsigned long SWITCH_CHANGE_COOLDOWN = 10000; 
bool lastSwitchReading = HIGH;          
bool lastSwitchStableState = HIGH;      
unsigned long lastSwitchDebounceTime = 0;
unsigned long lastSwitchActionTime = 0;
const int EMERGENCY_PIN = D5; 
bool deepSleepPending = false;
unsigned long deepSleepDurationSec = 0;
uint32_t remainingSleepSeconds = 0;
bool tryingHardcodedNetworks = false;
int currentHardcodedNetworkIndex = 0;
unsigned long hardcodedConnectAttemptStart = 0;
const unsigned long HARDCODED_CONNECT_TIMEOUT_MS = 15000; 
const int MAX_STA_CONNECT_ATTEMPTS = 5; 
bool relayState = false;
unsigned long relayOnStartTime = 0;
bool failsafeActive = false; 
bool emergencyStop = false;
unsigned long relayDuration = 0; 
bool loggedIn = false;
bool isApModeActive = false; 
unsigned long bootCount = 0;
unsigned long lastLoginTime = 0;
unsigned long sessionStart = 0;
const unsigned long sessionTimeout = 300000;
bool deepSleepRequested = false;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_COOLDOWN_MS = 2000;
volatile bool pendingRestart = false; 
const unsigned long AUTO_DEEP_SLEEP_DELAY_MS = 8 * 60 * 1000;   
bool autoDeepSleepScheduled = false; 
Ticker autoDeepSleepDelayTicker; 
Ticker deferredServerDelete;
AsyncWebServer server(80);
AsyncEventSource events("/events");
char currentSessionToken[33] = {0}; 
const size_t SETTINGS_DOC_CAPACITY = 2048; 

struct Settings {
  char staSsid[33];
  char staPassword[65];
  char hostname[32]; 
  bool useDHCP;
  IPAddress staticIp;
  IPAddress staticGateway;
  IPAddress staticSubnet;
  IPAddress staticDns1;
  IPAddress staticDns2;
  bool wifiAPModeEnabled; 
  char apSsid[32];
  char apPassword[64];
  char adminUsername[32];
  char adminPasswordHash[33]; 
  float calibrationFactor;
  unsigned long failsafeDurationHours;
  bool failsafeEnabled; 
  int failsafePin;
  int relayPin;    
  unsigned long defaultRelayDuration;
  unsigned long maxRelayRuntime;
  bool wasTimedRunInterrupted;
  unsigned int interruptedRunDuration;
  unsigned long interruptedRunStartTime;
  unsigned long bootUnixTime = 0;
  unsigned long deepSleepDurationSeconds;
  bool autoDeepSleepEnabled; 
  bool autoDeepSleepDelayAfterRun;
};
Settings settings;

struct BootRecoveryData {
  unsigned long lastBootMillis;
  uint8_t bootCount;
};
BootRecoveryData bootRecovery;

struct KnownNetwork {
  char ssid[33];
  char password[65];
};
std::vector<KnownNetwork> knownNetworks;

Ticker failsafeTicker;    
Ticker watchdogTicker;
Ticker relayTicker;       
Ticker logCleanupTicker;  
Ticker statusTicker;
Ticker wifiScanTicker;    
Ticker emergencyStopDelay;
volatile unsigned long lastLoopMillis = 0;
Ticker softwareWatchdogTicker; 
unsigned long lastLoginAttemptMillis = 0;
int failedLoginAttempts = 0;
const int MAX_FAILED_LOGIN_ATTEMPTS = 5;
const unsigned long LOGIN_COOLDOWN_TIME = 60000;
bool serverStarted = false;
const char* SETTINGS_FILE = "/settings.json";
const char* SETTINGS_BACKUP_FILE = "/settings.bak";
const char* BOOT_RECOVERY_FILE = "/boot_recovery.json";
volatile bool watchdogFeedRequested = false;
volatile bool shouldBroadcastAfterEmergencyClear = false;
volatile bool shouldLogEmergencyClear = false;
// Forward declarations
void setupLittleFS();
void initWiFi();
void startWebServer();
void stopWebServer(); 
void setupAP();
void checkAPTimeout();
void checkSwitchPress();
void generateSessionToken();
bool authenticate(const char* user, const char* pass);
bool isLoggedIn(AsyncWebServerRequest *request);
void clearEmergencyStopAndFailsafe();  
void applyDefaultSettings();
bool loadSettings(); 
bool saveSettings();
bool loadBootRecovery(); 
void saveBootRecovery();
void checkRecovery();
void factoryResetPreserveIndex(); 
void toggleRelayInternal();
void emergencyStopInternal(); 
void turnOffRelayTimed();
void failsafeTriggered();
void resetFailsafe(); 
void initiateAutoDeepSleep(); 
void handleLogin(AsyncWebServerRequest *request);
void handleUpdateLogin(AsyncWebServerRequest *request);
void handleClearFailsafe(AsyncWebServerRequest *request); 
void handleRelayControl(AsyncWebServerRequest *request); 
void handleEmergencyStop(AsyncWebServerRequest *request);
void handleRelayLogs(AsyncWebServerRequest *request);
void handleFailsafeLogs(AsyncWebServerRequest *request);
void handleClearRelayLogs(AsyncWebServerRequest *request);
void handleClearFailsafeLogs(AsyncWebServerRequest *request);
void handleScanNetworks(AsyncWebServerRequest *request); 
void handleKnownNetworks(AsyncWebServerRequest *request);
void handleSaveKnownNetworks(AsyncWebServerRequest *request);
void handleClearKnownNetworks(AsyncWebServerRequest *request);
void handleAPModeToggle(AsyncWebServerRequest *request);
void handleRestart(AsyncWebServerRequest *request);
void handleRestoreFactorySettings(AsyncWebServerRequest *request);
void handleUpdateWifi(AsyncWebServerRequest *request);
void handleDisconnectWifi(AsyncWebServerRequest *request); 
void handleSetDeepSleepDuration(AsyncWebServerRequest *request);
void handleDeepSleep(AsyncWebServerRequest *request);            
void handleFormatFS(AsyncWebServerRequest *request);
void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void handleStatus(AsyncWebServerRequest *request);
void handleGetSettings(AsyncWebServerRequest *request); 
void handleFsCheck(AsyncWebServerRequest *request); 
void handleAutoDeepSleepToggle(AsyncWebServerRequest *request); 
void broadcastStatus();
void broadcastScanResults();
void IRAM_ATTR feedWatchdog();
void setupWatchdog();
void checkLoopWatchdog();
void circularLogWrite(const char* filename, const char* logMsg, uint8_t& logIndex);
void safeRestart();
void startNonBlockingWiFiScan();
void processScanResults();

// ===================== TELEGRAM HELPER FUNCTIONS =====================

bool isTelegramHeapSafe() {
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < TELEGRAM_MIN_HEAP) {
        Serial.printf("Telegram: Heap too low: %u < %u\n", freeHeap, TELEGRAM_MIN_HEAP);
        return false;
    }
    return true;
}

bool initTelegramBot() {
    if (bot != nullptr) {
        return true;
    }
    
    if (!isTelegramHeapSafe()) {
        return false;
    }
    
    Serial.println(F("Initializing Telegram bot..."));
    Serial.printf("Free heap before: %u\n", ESP.getFreeHeap());
    
    secureClient.setInsecure();
    secureClient.setTimeout(10000);
    
    bot = new UniversalTelegramBot(BOT_TOKEN, secureClient);
    if (bot == nullptr) {
        Serial.println(F("Failed to create bot!"));
        return false;
    }
    
    bot->waitForResponse = 5000;
    telegramClientConfigured = true;
    
    Serial.printf("Telegram bot ready. Heap: %u\n", ESP.getFreeHeap());
    return true;
}

void cleanupTelegramBot() {
    if (bot != nullptr) {
        delete bot;
        bot = nullptr;
        telegramClientConfigured = false;
        Serial.println(F("Telegram bot cleaned up"));
        Serial.printf("Heap after cleanup: %u\n", ESP.getFreeHeap());
    }
}

bool sendTelegramMessage(const char* message) {
    if (!isTelegramHeapSafe()) {
        return false;
    }
    
    if (!initTelegramBot()) {
        return false;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("Telegram: WiFi not connected"));
        return false;
    }
    
    Serial.println(F("Sending Telegram message..."));
    ESP.wdtFeed();
    
    bool success = bot->sendMessage(CHAT_ID, message, "");
    
    ESP.wdtFeed();
    
    if (success) {
        Serial.println(F("Telegram: Message sent!"));
    } else {
        Serial.println(F("Telegram: Send failed"));
    }
    
    return success;
}

void resetTelegramState() {
    telegramSendPending = false;
    telegramDelayActive = false;
    telegramSending = false;
    telegramConnectionStableTime = 0;
    telegramSendStartTime = 0;
    telegramTriggeredForThisConnection = false;
    telegramRetryCount = 0;
    cleanupTelegramBot();
    Serial.println(F("Telegram state reset"));
}

void processTelegramQueue() {
    if (!telegramSendPending || telegramSending) {
        return;
    }
    
    if (isMidCycleWake) {
        return;
    }
    
    if (WiFi.status() != WL_CONNECTED || !wifiConnected) {
        return;
    }
    
    uint32_t now = millis();
    
    // PHASE 1: Start stability timer
    if (!telegramDelayActive && telegramConnectionStableTime == 0) {
        telegramConnectionStableTime = now;
        Serial.println(F("Telegram: Starting stability check..."));
        return;
    }
    
    // PHASE 2: Wait for connection stability
    if (!telegramDelayActive && telegramConnectionStableTime > 0) {
        if (now - telegramConnectionStableTime < TELEGRAM_STABILITY_MS) {
            return;
        }
        telegramDelayActive = true;
        telegramSendStartTime = now;
        Serial.printf("Telegram: Stable! Waiting %d ms...\n", TELEGRAM_DELAY_MS);
        return;
    }
    
    // PHASE 3: Wait for delay
    if (telegramDelayActive && !telegramSending) {
        if (now - telegramSendStartTime < TELEGRAM_DELAY_MS) {
            return;
        }
        
        if (!isTelegramHeapSafe()) {
            Serial.println(F("Telegram: Heap low, extending delay..."));
            telegramSendStartTime = now;
            delay(100);
            yield();
            return;
        }
        
        telegramSending = true;
        Serial.println(F("Telegram: Sending now..."));
    }
    
    // PHASE 4: Send message
    if (telegramSending) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println(F("Telegram: WiFi lost!"));
            telegramSending = false;
            telegramDelayActive = false;
            telegramConnectionStableTime = 0;
            return;
        }
        
        IPAddress ip = WiFi.localIP();
        char ipMsg[200];
        snprintf(ipMsg, sizeof(ipMsg), 
                 "ESP8266 Online!\n"
                 "IP: %d.%d.%d.%d\n"
                 "SSID: %s\n"
                 "RSSI: %d dBm\n"
                 "Heap: %u\n"
                 "Mode: %s", 
                 ip[0], ip[1], ip[2], ip[3], 
                 WiFi.SSID().c_str(),
                 WiFi.RSSI(),
                 ESP.getFreeHeap(),
                 isInHuntMode() ? "HUNT" : "NORMAL");
        
        ESP.wdtFeed();
        bool success = sendTelegramMessage(ipMsg);
        ESP.wdtFeed();
        
        if (success) {
            Serial.println(F("Telegram: SUCCESS!"));
            telegramSendPending = false;
            telegramTriggeredForThisConnection = true;
            telegramRetryCount = 0;
        } else {
            telegramRetryCount++;
            Serial.printf("Telegram: FAILED (%d/%d)\n", telegramRetryCount, MAX_TELEGRAM_RETRIES);
            
            if (telegramRetryCount >= MAX_TELEGRAM_RETRIES) {
                Serial.println(F("Telegram: Max retries, giving up"));
                telegramSendPending = false;
            } else {
                telegramDelayActive = false;
                telegramConnectionStableTime = 0;
            }
        }
        
        telegramSending = false;
        telegramDelayActive = false;
        
        if (!telegramSendPending && !serverRunning) {
            webServerStartPending = true;
        }
        
        cleanupTelegramBot();
    }
}

// ===================== FS LOCK FUNCTIONS =====================

bool acquireFSLock(unsigned long timeout = 3000) { 
  unsigned long start = millis();
  while (fsLock) {
    if (millis() - start > timeout) {
      Serial.println(F("FS lock timeout"));
      return false;
    }
    delay(1);  
  }
  fsLock = true;
  return true;
}

void releaseFSLock() {
  fsLock = false;
}

class FSLockGuard {
public:
  FSLockGuard() { locked = acquireFSLock(); }
  ~FSLockGuard() { if (locked) releaseFSLock(); }
  bool isLocked() const { return locked; }
private:
  bool locked;
};

void IRAM_ATTR feedWatchdog() {
  watchdogFeedRequested = true; 
}

void startNonBlockingWiFiScan() {
  if (scanInProgress) return;
  Serial.println(F("Starting WiFi scan..."));
  WiFi.setOutputPower(17.0);
  WiFi.scanNetworks(true); 
  scanInProgress = true;
  lastScanStartTime = millis();
}

void processScanResults() {
  if (!scanInProgress) return; 
  int n = WiFi.scanComplete();
  if (n >= 0) { 
    scanInProgress = false; 
    Serial.printf("%d networks found\n", n);
    JsonDocument doc;
    JsonArray networks = doc.to<JsonArray>();
    for (int i = 0; i < n; ++i) {
      JsonObject network = networks.add<JsonObject>();
      network["ssid"] = WiFi.SSID(i);
      network["rssi"] = WiFi.RSSI(i);
      network["encryption"] = (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted";
    }
    String jsonString;
    serializeJson(doc, jsonString);
    events.send(jsonString.c_str(), "wifiScanResults");
    WiFi.scanDelete(); 
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n\n========================================"));
  Serial.println(F("   AquaMaster SSE 101 AI Starting..."));
  Serial.println(F("========================================"));
  Serial.printf("Core: %s | Heap: %u\n", ESP.getCoreVersion().c_str(), ESP.getFreeHeap());
  Serial.printf("Reset: %s\n", ESP.getResetReason().c_str());
  
  while (!Serial && millis() < 3000);
  
  bootStart = millis();
  
  // Initialize WiFi Manager
  WiFi.mode(WIFI_STA);
  initWiFiManager();
  enableIntermittentHuntMode();
  
  // DON'T configure secureClient here - do it lazily
  
  bool rtcValid = loadRTC();
  if (rtcValid) {
    Serial.println(F("RTC load SUCCESS"));
    if (rtcData.remainingSleepSeconds > 86400UL) {
      Serial.println(F("RTC corrupted. Clearing."));
      clearRTC();
      rtcValid = false;
    }
  } else {
    Serial.println(F("RTC load FAILED"));
    clearRTC();
  }
  
  if (rtcValid && rtcData.remainingSleepSeconds > 0) {
    Serial.printf("MID-CYCLE WAKE: %u sec remaining\n", rtcData.remainingSleepSeconds);
    isMidCycleWake = true;
  } else {
    Serial.println(F("FINAL WAKE: Normal boot"));
    isMidCycleWake = false;
    clearRTC();
  }
  
  if (isMidCycleWake) {
    Serial.println(F("===== MID-CYCLE: Minimal setup ====="));
    return;
  }
  
  Serial.println(F("===== FULL INITIALIZATION ====="));
  
  if (!LittleFS.begin()) {
    Serial.println(F("LittleFS mount failed."));
    factoryResetPreserveIndex();
  } else {
    Serial.println(F("LittleFS mounted."));
  }
  
  applyDefaultSettings();
  if (!loadSettings()) {
    Serial.println(F("Settings failed. Factory reset."));
    factoryResetPreserveIndex();
  }
  if (!loadBootRecovery()) {
    factoryResetPreserveIndex();
  }
  checkRecovery();
  
  pinMode(settings.relayPin, OUTPUT);
  digitalWrite(settings.relayPin, HIGH);
  relayState = false;
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastSwitchReading = digitalRead(SWITCH_PIN);
  lastSwitchStableState = lastSwitchReading;
  
  timeClient.begin();
  setupWatchdog();
  softwareWatchdogTicker.attach_ms(5000, checkLoopWatchdog);
  statusTicker.attach_ms(2000, broadcastStatus);
  
  applyEstimatedTimeFromSnapshot();
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
  
  // Reset all flags
  autoDeepSleepScheduled = false;
  deepSleepRequested = false;
  sleepRequestTime = 0;
  telegramSendPending = false;
  telegramDelayActive = false;
  telegramSending = false;
  telegramTriggeredForThisConnection = false;
  telegramConnectionStableTime = 0;
  webServerStartPending = false;
  serverRunning = false;
  wifiConnected = false;
  
  Serial.printf("Setup complete. Heap: %u\n", ESP.getFreeHeap());
}
void loop() {
    lastLoopMillis = millis();
    ESP.wdtFeed();
    
    manageWiFiConnection();
    
    // Mid-cycle wake handling
    if (isMidCycleWake && rtcData.remainingSleepSeconds > 0) {
        static unsigned long midCycleBootTime = millis();
        if (millis() - midCycleBootTime > 500) {
            Serial.println(F("Mid-cycle: Going back to sleep..."));
            uint32_t sleepNow = rtcData.remainingSleepSeconds;
            if (sleepNow > 3600) {
                rtcData.remainingSleepSeconds = sleepNow - 3600;
                sleepNow = 3600;
                saveRTC();
            } else {
                clearRTC();
            }
            WiFi.mode(WIFI_OFF);
            delay(100);
            ESP.deepSleep(sleepNow * 1000000UL);
        }
        return;
    }
    
    // Debug output every 5 seconds
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        lastDebug = millis();
        Serial.printf("=== WiFi:%s Heap:%u Tg:p%d/d%d/s%d Web:p%d/r%d ===\n",
                      getWiFiStateName(), ESP.getFreeHeap(),
                      telegramSendPending, telegramDelayActive, telegramSending,
                      webServerStartPending, serverRunning);
    }
    
    // Emergency pin
    static uint32_t lastEmergencyCheck = 0;
    if (millis() - lastEmergencyCheck > 10) {
        lastEmergencyCheck = millis();
        static bool lastEmergencyState = HIGH;
        bool currentEmergencyState = digitalRead(EMERGENCY_PIN);
        if (lastEmergencyState == HIGH && currentEmergencyState == LOW) {
            logFailsafeEvent("Button:EmergencyStop");
            emergencyStopInternal();
        }
        if (lastEmergencyState == LOW && currentEmergencyState == HIGH) {
            clearEmergencyStopAndFailsafe();
        }
        lastEmergencyState = currentEmergencyState;
    }
    
    if (pendingFailsafeReason[0]) {
        logFailsafeEvent(pendingFailsafeReason);
        pendingFailsafeReason[0] = 0;
    }
    
    ESP.wdtFeed();
    checkSwitchPress();
    ESP.wdtFeed();
    
    if (pendingAPSetup) {
        pendingAPSetup = false;
        setupAP();
    }
    
    if (pendingRestart) {
        pendingRestart = false;
        safeRestart();
    }
    
    if (watchdogFeedRequested) {
        watchdogFeedRequested = false;
        ESP.wdtFeed();
    }
    
    if (dnsServerActive) {
        dnsServer.processNextRequest();
    }
    
    // ===================== TELEGRAM PROCESSING =====================
    processTelegramQueue();
    
    // ===================== WEB SERVER START =====================
    if (webServerStartPending && !serverRunning) {
        if (!telegramSendPending && !telegramDelayActive && !telegramSending) {
            if (isWiFiConnected() || isApModeActive) {
                Serial.println(F("Starting web server..."));
                startWebServer();
                serverRunning = true;
                webServerStartPending = false;
                Serial.printf("Web server started! Heap: %u\n", ESP.getFreeHeap());
            }
        }
    }
    
    // Deep sleep handling
    if (deepSleepRequested) {
        if (telegramSendPending || telegramDelayActive || telegramSending) {
            return;
        }
        
        if (sleepRequestTime == 0) {
            sleepRequestTime = millis();
        }
        
        if (millis() - sleepRequestTime > 500) {
            Serial.println(F("Deep sleep starting..."));
            cleanupTelegramBot();
            
            failsafeTicker.detach();
            watchdogTicker.detach();
            relayTicker.detach();
            statusTicker.detach();
            softwareWatchdogTicker.detach();
            
            if (serverRunning) {
                stopWebServer();
                serverRunning = false;
            }
            
            WiFi.mode(WIFI_OFF);
            delay(100);
            
            uint32_t totalSleep = settings.deepSleepDurationSeconds;
            if (totalSleep == 0) {
                deepSleepRequested = false;
                sleepRequestTime = 0;
                return;
            }
            
            uint32_t sleepNow = totalSleep;
            if (sleepNow > 3600) {
                rtcData.remainingSleepSeconds = sleepNow - 3600;
                sleepNow = 3600;
                saveRTC();
            } else {
                clearRTC();
            }
            
            Serial.printf("Sleeping %u seconds...\n", sleepNow);
            ESP.deepSleep(sleepNow * 1000000UL);
        }
    }
    
    checkAPTimeout();
    processScanResults();
    
    // Post-connection scan
    static bool postConnectScanDone = false;
    if (!postConnectScanDone && WiFi.status() == WL_CONNECTED && !scanInProgress) {
        startNonBlockingWiFiScan();
        postConnectScanDone = true;
    }
    if (WiFi.status() != WL_CONNECTED) {
        postConnectScanDone = false;
        if (telegramTriggeredForThisConnection) {
            resetTelegramState();
        }
    }
    
    if (shouldSaveSettingsFlag) {
        shouldSaveSettingsFlag = false;
        saveSettings();
    }
    
    // Relay timed off
    if (relayTimedOffPending) {
        relayTimedOffPending = false;
        if (relayState) {
            digitalWrite(settings.relayPin, HIGH);
            relayState = false;
            if (relayTicker.active()) relayTicker.detach();
            if (failsafeTicker.active()) failsafeTicker.detach();
            
            if (!isTemporaryRun) {
                logRelayRunTime();
                saveSettings();
            } else {
                isTemporaryRun = false;
            }
            broadcastStatus();
            
            if (settings.autoDeepSleepEnabled && !autoDeepSleepScheduled && 
                !failsafeActive && !emergencyStop) {
                autoDeepSleepDelayTicker.once_ms(600000, initiateAutoDeepSleep);
                autoDeepSleepScheduled = true;
            }
        }
    }
    
    yield();
}
void safeRestart() {
  Serial.println(F("Graceful restart..."));
  cleanupTelegramBot();
  
  if (serverRunning) stopWebServer();
  
  failsafeTicker.detach();
  watchdogTicker.detach();
  relayTicker.detach();
  statusTicker.detach();
  softwareWatchdogTicker.detach();
  autoDeepSleepDelayTicker.detach();
  wifiScanTicker.detach();
  
  dnsServer.stop();
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  
  ESP.restart();
}

void setupWatchdog() {
  watchdogTicker.attach_ms(5000, feedWatchdog);
}

void checkLoopWatchdog() {
  if (millis() - lastLoopMillis > 10000) { 
    Serial.println(F("Watchdog triggered!"));
    safeRestart();
  }
}

bool loadRTC() {
  ESP.rtcUserMemoryRead(RTC_SLOT, (uint32_t*)&rtcData, sizeof(rtcData));
  if (rtcData.magic != RTC_MAGIC) return false;
  if (rtcData.checksum != rtcChecksum(rtcData)) return false;
  return true;
}

void saveRTC() {
  rtcData.magic = RTC_MAGIC;
  rtcData.checksum = rtcChecksum(rtcData);
  ESP.rtcUserMemoryWrite(RTC_SLOT, (uint32_t*)&rtcData, sizeof(rtcData));
}

void clearRTC() {
  rtcData.magic = 0;
  rtcData.remainingSleepSeconds = 0;
  rtcData.checksum = 0;
  ESP.rtcUserMemoryWrite(RTC_SLOT, (uint32_t*)&rtcData, sizeof(rtcData));
}

void failsafeTriggered() {
  failsafeActive = true;
  digitalWrite(settings.relayPin, HIGH);
  relayState = false;
  logFailsafeEvent("Duration exceeded");
  failsafeTicker.detach(); 
  settings.wasTimedRunInterrupted = false;
  saveSettings();
  broadcastStatus(); 
}

void resetFailsafe() {
  failsafeActive = false;
  emergencyStop = false;
  failsafeTicker.detach();
  if (relayState) {
    digitalWrite(settings.relayPin, HIGH);
    relayState = false;
  }
  broadcastStatus();
}

void initiateAutoDeepSleep() {
  deepSleepRequested = true;
  sleepRequestTime = 0;
  autoDeepSleepScheduled = false;
  if (rtcData.remainingSleepSeconds == 0) {
    rtcData.remainingSleepSeconds = settings.deepSleepDurationSeconds;
    saveRTC();
  }
}

void turnOffRelayTimed() {
  if (relayState) {
    relayTimedOffPending = true;
  }
}

void toggleRelayInternal() {
  if (failsafeActive || emergencyStop) return;
  
  if (relayState) { 
    digitalWrite(settings.relayPin, HIGH); 
    relayState = false;
    if (relayTicker.active()) relayTicker.detach();
    autoDeepSleepDelayTicker.detach();
    autoDeepSleepScheduled = false;
  } else { 
    relayOnEpochTime = time(nullptr);
    digitalWrite(settings.relayPin, LOW); 
    relayState = true;
    relayOnStartTime = millis();
    if (settings.defaultRelayDuration > 0) {
      relayTicker.once(settings.defaultRelayDuration, turnOffRelayTimed);
    }
    if (settings.failsafeEnabled && settings.maxRelayRuntime > 0) {
      failsafeTicker.attach(settings.maxRelayRuntime, failsafeTriggered);
    }
    autoDeepSleepDelayTicker.detach();
    autoDeepSleepScheduled = false;
  }
  broadcastStatus();
}

void emergencyStopInternal() {
  if (emergencyStop) return;
  ESP.wdtFeed();
  emergencyStop = true;
  failsafeActive = true;
  digitalWrite(settings.relayPin, HIGH);
  relayState = false;
  if (relayTicker.active()) relayTicker.detach();
  if (failsafeTicker.active()) failsafeTicker.detach();
  autoDeepSleepDelayTicker.detach();
  autoDeepSleepScheduled = false;
  strncpy(pendingFailsafeReason, "Emergency Stop", sizeof(pendingFailsafeReason) - 1);
  broadcastStatus();
}

void clearEmergencyStopAndFailsafe() {
  if (emergencyStop || failsafeActive) {
    emergencyStop = false;
    failsafeActive = false;
    broadcastStatus();
  }
}
void getMD5Hash(const char* input, char* output, size_t outputSize) {
  if (outputSize < 33) return;
  MD5Builder md5;
  md5.begin();
  md5.add(input);
  md5.calculate();
  uint8_t digest[16];
  md5.getBytes(digest);
  for (uint8_t i = 0; i < 16; i++) {
    sprintf(output + (i * 2), "%02x", digest[i]);
  }
  output[32] = '\0';
}

void getMD5Hash_P(const __FlashStringHelper* flashStr, char* output, size_t outputSize) {
  if (outputSize < 33) return;
  char inputBuffer[64];
  strncpy_P(inputBuffer, (PGM_P)flashStr, 63);
  inputBuffer[63] = '\0';
  MD5Builder md5;
  md5.begin();
  md5.add(inputBuffer);
  md5.calculate();
  uint8_t digest[16];
  md5.getBytes(digest);
  for (uint8_t i = 0; i < 16; i++) {
    sprintf(output + (i * 2), "%02x", digest[i]);
  }
  output[32] = '\0';
}

void applyEstimatedTimeFromSnapshot() {
  File f = LittleFS.open("/time_sync.dat", "r");
  if (!f) {
    Serial.println(F("No saved time snapshot."));
    return;
  }
  String line = f.readStringUntil('\n');
  f.close();
  int comma = line.indexOf(',');
  if (comma == -1) return;
  uint32_t savedEpoch = line.substring(0, comma).toInt();
  uint32_t savedMillis = line.substring(comma + 1).toInt();
  estimatedEpochTime = savedEpoch + ((millis() - savedMillis) / 1000);
  struct timeval tv = { (time_t)estimatedEpochTime, 0 };
  settimeofday(&tv, nullptr);
  Serial.printf("Applied estimated time: %lu\n", estimatedEpochTime);
}

void syncTimeFromNTP() {
  if (WiFi.status() == WL_CONNECTED) {
    if (timeClient.forceUpdate()) {
      time_t epochTime = timeClient.getEpochTime();
      struct timeval tv;
      tv.tv_sec = epochTime;
      tv.tv_usec = 0;
      settimeofday(&tv, nullptr);
      lastEpoch = epochTime;
      lastSyncMillis = millis();
      Serial.printf("Time synced: %lu\n", lastEpoch);
      File f = LittleFS.open("/time_sync.dat", "w");
      if (f) {
        f.printf("%lu,%lu\n", (uint32_t)epochTime, millis());
        f.close();
      }
    }
  }
}

void logRelayRunTime() {
  time_t now = time(nullptr);
  if (now <= 1600000000UL || relayOnEpochTime == 0) {
    Serial.println(F("Time not synced. Skipping relay log."));
    return;
  }
  uint32_t duration = now - relayOnEpochTime;
  if (duration == 0) duration = 1;
  if (duration > 86400UL) {
    Serial.printf("Invalid duration (%lu s). Skipping.\n", duration);
    return;
  }
  char entry[LOG_ENTRY_LENGTH] = {0};
  snprintf(entry, sizeof(entry), "%lu,%lu\n", (uint32_t)relayOnEpochTime, duration);
  ESP.wdtFeed();
  circularLogWrite(RELAY_LOG_FILE, entry, relayLogIndex);
  ESP.wdtFeed();
  Serial.printf("Logged Relay: Epoch %lu, Duration %lu s\n", relayOnEpochTime, duration);
}

void logFailsafeEvent(const char* reason) {
  ESP.wdtFeed();
  char logLine[128];
  time_t now = time(nullptr);
  if (now < 1600000000UL) {
    now = estimatedEpochTime;
  }
  char cleanReason[48] = "Unknown";
  if (reason && reason[0]) {
    strncpy(cleanReason, reason, sizeof(cleanReason) - 1);
    cleanReason[sizeof(cleanReason) - 1] = '\0';
    for (int i = 0; cleanReason[i]; i++) {
      if (cleanReason[i] == ',' || cleanReason[i] == '\n' || cleanReason[i] == '\r') {
        cleanReason[i] = '_';
      }
    }
  }
  int len = snprintf(logLine, sizeof(logLine), "%lu,Failsafe,%s\n", (unsigned long)now, cleanReason);
  ESP.wdtFeed();
  {
    FSLockGuard guard;
    if (!guard.isLocked()) return;
    File f = LittleFS.open(FAILSAFE_LOG_FILE, "a");
    if (!f) return;
    f.write((const uint8_t*)logLine, len);
    f.flush();
    f.close();
  }
  ESP.wdtFeed();
}

void circularLogWrite(const char* filename, const char* logMsg, uint8_t& logIndex) {
  FSLockGuard lock;
  if (!lock.isLocked()) return;
  ESP.wdtFeed();
  if (!filename || !logMsg || strlen(logMsg) == 0) return;
  if (logIndex >= MAX_LOGS) logIndex = 0;

  File logFile = LittleFS.open(filename, "r+");
  if (!logFile) {
    logFile = LittleFS.open(filename, "w+");
    if (!logFile) return;
    char blank[LOG_ENTRY_LENGTH] = {0};
    for (uint8_t i = 0; i < MAX_LOGS; i++) {
      logFile.write((const uint8_t*)blank, LOG_ENTRY_LENGTH);
      yield();
    }
  }

  char padded[LOG_ENTRY_LENGTH] = {0};
  size_t len = strnlen(logMsg, LOG_ENTRY_LENGTH - 2);
  strncpy(padded, logMsg, len);
  for (size_t i = 0; i < len; i++) {
    if (padded[i] == '\r' || padded[i] == '\n') padded[i] = ' ';
  }
  padded[len] = '\n';

  size_t offset = (size_t)logIndex * LOG_ENTRY_LENGTH;
  logFile.seek(offset, SeekSet);
  logFile.write((const uint8_t*)padded, LOG_ENTRY_LENGTH);
  logFile.flush();
  logFile.close();

  logIndex = (logIndex + 1) % MAX_LOGS;
  ESP.wdtFeed();
}

void applyDefaultSettings() {
  strncpy(settings.staSsid, hardcodedNetworks[0].ssid, sizeof(settings.staSsid) - 1);
  settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';
  strncpy(settings.staPassword, hardcodedNetworks[0].password, sizeof(settings.staPassword) - 1);
  settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
  strncpy(settings.hostname, "aquamaster", sizeof(settings.hostname) - 1);
  settings.hostname[sizeof(settings.hostname) - 1] = '\0';
  settings.useDHCP = true;
  settings.staticIp.fromString("192.168.1.100");
  settings.staticGateway.fromString("192.168.1.1");
  settings.staticSubnet.fromString("255.255.255.0");
  settings.staticDns1.fromString("8.8.8.8");
  settings.staticDns2.fromString("8.8.4.4");
  settings.wifiAPModeEnabled = false;
  strncpy(settings.apSsid, "AquaMaster_AP", sizeof(settings.apSsid) - 1);
  settings.apSsid[sizeof(settings.apSsid) - 1] = '\0';
  strncpy(settings.apPassword, "aquapass", sizeof(settings.apPassword) - 1);
  settings.apPassword[sizeof(settings.apPassword) - 1] = '\0';
  strncpy(settings.adminUsername, "admin", sizeof(settings.adminUsername) - 1);
  settings.adminUsername[sizeof(settings.adminUsername) - 1] = '\0';
  char hash[33];
  getMD5Hash_P(F("admin"), hash, sizeof(hash));
  strncpy(settings.adminPasswordHash, hash, sizeof(settings.adminPasswordHash));
  settings.adminPasswordHash[sizeof(settings.adminPasswordHash) - 1] = '\0';
  settings.calibrationFactor = 1.0;
  settings.failsafeDurationHours = 12;
  settings.failsafeEnabled = true;
  settings.failsafePin = D5;
  settings.relayPin = D1;
  settings.defaultRelayDuration = 720;
  settings.maxRelayRuntime = 960;
  settings.wasTimedRunInterrupted = false;
  settings.interruptedRunDuration = 0;
  settings.interruptedRunStartTime = 0;
  settings.bootUnixTime = 0;
  settings.deepSleepDurationSeconds = 15 * 3600;
  settings.autoDeepSleepEnabled = true;
  settings.autoDeepSleepDelayAfterRun = 600;
  currentSessionToken[0] = '\0';
  loggedIn = false;
}

bool loadSettings() {
  FSLockGuard lock;
  if (!lock.isLocked()) return false;
  
  File configFile = LittleFS.open(SETTINGS_FILE, "r");
  if (!configFile) {
    configFile = LittleFS.open(SETTINGS_BACKUP_FILE, "r");
    if (!configFile) return false;
  }
  
  StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();
  if (error) return false;

  strncpy(settings.staSsid, doc["staSsid"] | "", sizeof(settings.staSsid) - 1);
  strncpy(settings.staPassword, doc["staPassword"] | "", sizeof(settings.staPassword) - 1);
  strncpy(settings.hostname, doc["hostname"] | "aquamaster", sizeof(settings.hostname) - 1);
  settings.useDHCP = doc["useDHCP"] | true;
  settings.staticIp.fromString(doc["staticIp"] | "192.168.1.100");
  settings.staticGateway.fromString(doc["staticGateway"] | "192.168.1.1");
  settings.staticSubnet.fromString("255.255.255.0");
  settings.staticDns1.fromString(doc["staticDns1"] | "8.8.8.8");
  settings.staticDns2.fromString(doc["staticDns2"] | "8.8.4.4");
  settings.wifiAPModeEnabled = doc["wifiAPModeEnabled"] | false;
  strncpy(settings.apSsid, doc["apSsid"] | "AquaMaster_AP", sizeof(settings.apSsid) - 1);
  strncpy(settings.apPassword, doc["apPassword"] | "aquapass", sizeof(settings.apPassword) - 1);
  strncpy(settings.adminUsername, doc["adminUsername"] | "admin", sizeof(settings.adminUsername) - 1);
  strncpy(settings.adminPasswordHash, doc["adminPasswordHash"] | "", sizeof(settings.adminPasswordHash) - 1);
  
  if (strlen(settings.adminPasswordHash) == 0) {
    char hash[33];
    getMD5Hash_P(F("admin"), hash, sizeof(hash));
    strncpy(settings.adminPasswordHash, hash, sizeof(settings.adminPasswordHash));
  }
  
  settings.calibrationFactor = doc["calibrationFactor"] | 1.0;
  settings.failsafeDurationHours = doc["failsafeDurationHours"] | 12;
  settings.failsafeEnabled = doc["failsafeEnabled"] | true;
  settings.failsafePin = doc["failsafePin"] | D5;
  settings.relayPin = doc["relayPin"] | D1;
  settings.defaultRelayDuration = doc["defaultRelayDuration"] | 720;
  settings.maxRelayRuntime = doc["maxRelayRuntime"] | 960;
  settings.wasTimedRunInterrupted = doc["wasTimedRunInterrupted"] | false;
  settings.interruptedRunDuration = doc["interruptedRunDuration"] | 0;
  settings.interruptedRunStartTime = doc["interruptedRunStartTime"] | 0;
  settings.bootUnixTime = doc["bootUnixTime"] | 0;
  settings.deepSleepDurationSeconds = doc["deepSleepDurationSeconds"] | (15 * 3600);
  settings.autoDeepSleepEnabled = doc["autoDeepSleepEnabled"] | true;
  settings.autoDeepSleepDelayAfterRun = doc["autoDeepSleepDelayAfterRun"] | 600;
  
  return true;
}

bool saveSettings() {
  FSLockGuard lock;
  if (!lock.isLocked()) return false;
  
  File configFile = LittleFS.open("/settings.tmp", "w");
  if (!configFile) return false;
  
  ESP.wdtFeed();
  StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
  doc["staSsid"] = settings.staSsid;
  doc["staPassword"] = settings.staPassword;
  doc["hostname"] = settings.hostname;
  doc["useDHCP"] = settings.useDHCP;
  doc["staticIp"] = settings.staticIp.toString();
  doc["staticGateway"] = settings.staticGateway.toString();
  doc["staticSubnet"] = settings.staticSubnet.toString();
  doc["staticDns1"] = settings.staticDns1.toString();
  doc["staticDns2"] = settings.staticDns2.toString();
  doc["wifiAPModeEnabled"] = settings.wifiAPModeEnabled;
  doc["apSsid"] = settings.apSsid;
  doc["apPassword"] = settings.apPassword;
  doc["adminUsername"] = settings.adminUsername;
  doc["adminPasswordHash"] = settings.adminPasswordHash;
  doc["calibrationFactor"] = settings.calibrationFactor;
  doc["failsafeDurationHours"] = settings.failsafeDurationHours;
  doc["failsafeEnabled"] = settings.failsafeEnabled;
  doc["failsafePin"] = settings.failsafePin;
  doc["relayPin"] = settings.relayPin;
  doc["defaultRelayDuration"] = settings.defaultRelayDuration;
  doc["maxRelayRuntime"] = settings.maxRelayRuntime;
  doc["wasTimedRunInterrupted"] = settings.wasTimedRunInterrupted;
  doc["interruptedRunDuration"] = settings.interruptedRunDuration;
  doc["interruptedRunStartTime"] = settings.interruptedRunStartTime;
  doc["bootUnixTime"] = settings.bootUnixTime;
  doc["deepSleepDurationSeconds"] = settings.deepSleepDurationSeconds;
  doc["autoDeepSleepEnabled"] = settings.autoDeepSleepEnabled;
  doc["autoDeepSleepDelayAfterRun"] = settings.autoDeepSleepDelayAfterRun;
  
  ESP.wdtFeed();
  if (serializeJson(doc, configFile) == 0) {
    configFile.close();
    return false;
  }
  configFile.close();
  
  if (LittleFS.exists(SETTINGS_BACKUP_FILE)) LittleFS.remove(SETTINGS_BACKUP_FILE);
  if (LittleFS.exists(SETTINGS_FILE)) LittleFS.rename(SETTINGS_FILE, SETTINGS_BACKUP_FILE);
  bool success = LittleFS.rename("/settings.tmp", SETTINGS_FILE);
  ESP.wdtFeed();
  return success;
}

bool loadBootRecovery() {
  FSLockGuard lock;
  if (!lock.isLocked()) return false;
  
  File file = LittleFS.open(BOOT_RECOVERY_FILE, "r");
  if (!file) {
    bootRecovery.lastBootMillis = 0;
    bootRecovery.bootCount = 0;
    saveBootRecovery();
    return true;
  }
  
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) return false;
  
  bootRecovery.lastBootMillis = doc["lastBootMillis"] | 0;
  bootRecovery.bootCount = doc["bootCount"] | 0;
  return true;
}

void saveBootRecovery() {
  FSLockGuard lock;
  if (!lock.isLocked()) return;
  
  File file = LittleFS.open(BOOT_RECOVERY_FILE, "w");
  if (!file) return;
  
  StaticJsonDocument<128> doc;
  doc["lastBootMillis"] = bootRecovery.lastBootMillis;
  doc["bootCount"] = bootRecovery.bootCount;
  serializeJson(doc, file);
  file.close();
}

void checkRecovery() {
  bootRecovery.lastBootMillis = millis();
  bootRecovery.bootCount++;
  saveBootRecovery();
  
  String reason = ESP.getResetReason();
  Serial.printf("Boot count: %u, Reset: %s\n", bootRecovery.bootCount, reason.c_str());
  
  if ((bootRecovery.bootCount >= 3 && reason == "Exception") || reason.indexOf("Software Watchdog") != -1) {
    Serial.println(F("Critical error. Factory reset..."));
    factoryResetPreserveIndex();
  }
}

void factoryResetPreserveIndex() {
  Serial.println(F("Factory Reset: Preserving index.html.gz"));
  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    String filenameStr = dir.fileName();
    if (filenameStr.indexOf("index.html.gz") >= 0) continue;
    LittleFS.remove(filenameStr);
  }
  applyDefaultSettings();
  saveSettings();
  bootRecovery.bootCount = 0;
  saveBootRecovery();
  
  relayState = false;
  failsafeActive = false;
  emergencyStop = false;
  loggedIn = false;
  isApModeActive = false;
  deepSleepPending = false;
  deepSleepRequested = false;
  serverRunning = false;
  scanInProgress = false;
  autoDeepSleepScheduled = false;
  
  pendingRestart = true;
}
void generateSessionToken() {
  static const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  const size_t charsetLength = sizeof(charset) - 1;
  for (uint8_t i = 0; i < 32; i++) {
    currentSessionToken[i] = charset[random(charsetLength)];
  }
  currentSessionToken[32] = '\0';
  sessionStart = millis();
}

bool authenticate(const char* user, const char* pass) {
  char hash[33];
  getMD5Hash(pass, hash, sizeof(hash));
  return (strcmp(user, settings.adminUsername) == 0 && strcmp(hash, settings.adminPasswordHash) == 0);
}

bool isLoggedIn(AsyncWebServerRequest *request) {
  if (!request->hasHeader("Cookie")) return false;
  
  const AsyncWebHeader* cookieHdr = request->getHeader("Cookie");
  if (!cookieHdr) return false;
  
  char cookieBuffer[128] = {0};
  size_t len = cookieHdr->value().length();
  if (len >= sizeof(cookieBuffer)) len = sizeof(cookieBuffer) - 1;
  for (size_t i = 0; i < len; ++i) {
    cookieBuffer[i] = cookieHdr->value()[i];
  }
  cookieBuffer[len] = '\0';
  
  const char* tokenStart = strstr(cookieBuffer, "authToken=");
  if (!tokenStart) return false;
  
  tokenStart += strlen("authToken=");
  char receivedToken[33] = {0};
  size_t i = 0;
  while (*tokenStart && *tokenStart != ';' && i < 32) {
    receivedToken[i++] = *tokenStart++;
  }
  receivedToken[i] = '\0';
  
  if (strcmp(receivedToken, currentSessionToken) != 0) return false;
  if (millis() - sessionStart > sessionTimeout) {
    currentSessionToken[0] = '\0';
    return false;
  }
  
  sessionStart = millis();
  return true;
}

void checkSwitchPress() {
  bool currentReading = digitalRead(SWITCH_PIN);
  if (currentReading != lastSwitchReading) {
    lastSwitchDebounceTime = millis();
  }
  if ((millis() - lastSwitchDebounceTime) > DEBOUNCE_DELAY) {
    if (currentReading != lastSwitchStableState) {
      lastSwitchStableState = currentReading;
      if (millis() - lastSwitchActionTime >= SWITCH_CHANGE_COOLDOWN) {
        if (currentReading == HIGH && relayState) {
          toggleRelayInternal();
        } else if (currentReading == LOW && !relayState) {
          toggleRelayInternal();
        }
        lastSwitchActionTime = millis();
      }
    }
  }
  lastSwitchReading = currentReading;
}
void initWiFi() {
  Serial.println(F("Initializing WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.hostname(settings.hostname);
  WiFi.disconnect();
  if (strlen(settings.staSsid) > 0) {
    if (settings.useDHCP) {
      WiFi.begin(settings.staSsid, settings.staPassword);
    } else {
      WiFi.config(settings.staticIp, settings.staticGateway, settings.staticSubnet, settings.staticDns1, settings.staticDns2);
      WiFi.begin(settings.staSsid, settings.staPassword);
    }
    lastWifiConnectAttemptMillis = millis();
    wifiConnected = false;
  } else {
    setupAP();
    isApModeActive = true;
  }
}

void setupAP() {
  if (isApModeActive) return;
  
  Serial.println(F("Starting AP mode..."));
  if (serverRunning) {
    stopWebServer();
  }
  
  WiFi.disconnect(true);
  delay(100);
  
  if (strlen(settings.apSsid) == 0 || strlen(settings.apSsid) > 31) {
    strncpy(settings.apSsid, "AquaMaster_AP", sizeof(settings.apSsid) - 1);
  }
  if (strlen(settings.apPassword) < 8 || strlen(settings.apPassword) > 63) {
    strncpy(settings.apPassword, "aquapass", sizeof(settings.apPassword) - 1);
  }
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(settings.apSsid, settings.apPassword);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print(F("AP IP: "));
  Serial.println(apIP);
  
  dnsServer.stop();
  dnsServerActive = false;
  dnsServer.setTTL(300);
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  if (dnsServer.start(DNS_PORT, "*", apIP)) {
    dnsServerActive = true;
    Serial.println(F("DNS server started."));
  }
  
  isApModeActive = true;
  apStartTime = millis();
  startWebServer();
}

void checkAPTimeout() {
  if (!isApModeActive) return;
  
  const unsigned long AP_NO_CLIENT_TIMEOUT_MS = 5UL * 60UL * 1000UL;
  const unsigned long AP_CLIENT_TIMEOUT_MS = 14UL * 60UL * 1000UL;
  
  int stationNum = WiFi.softAPgetStationNum();
  unsigned long currentTimeout = (stationNum > 0) ? AP_CLIENT_TIMEOUT_MS : AP_NO_CLIENT_TIMEOUT_MS;
  
  if (millis() - apStartTime >= currentTimeout) {
    if (relayState) {
      Serial.println(F("AP timeout but relay ON - delaying."));
      return;
    }
    Serial.println(F("AP timeout. Restarting..."));
    WiFi.disconnect(true);
    delay(100);
    pendingRestart = true;
  }
}

void stopAP() {
  if (dnsServerActive) {
    dnsServer.stop();
    dnsServerActive = false;
  }
  if (isApModeActive) {
    WiFi.softAPdisconnect(true);
    isApModeActive = false;
  }
  stopWebServer();
}
void startWebServer() {
  if (serverRunning) return;
  
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/updateLogin", HTTP_POST, handleUpdateLogin);
  server.on("/relay", HTTP_POST, handleRelayControl);
  server.on("/emergencyStop", HTTP_POST, handleEmergencyStop);
  server.on("/scan", HTTP_GET, handleScanNetworks);
  server.on("/knownNetworks", HTTP_GET, handleKnownNetworks);
  server.on("/saveKnownNetworks", HTTP_POST, handleSaveKnownNetworks);
  server.on("/clearKnownNetworks", HTTP_POST, handleClearKnownNetworks);
  server.on("/apModeToggle", HTTP_POST, handleAPModeToggle);
  server.on("/restart", HTTP_POST, handleRestart);
  server.on("/restoreFactorySettings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!isLoggedIn(request)) { request->send(403, "text/plain", "Forbidden"); return; }
    handleRestoreFactorySettings(request);
  });
  server.on("/failsafeLogs", HTTP_GET, handleFailsafeLogs);
  server.on("/relayLogs", HTTP_GET, handleRelayLogs);
  server.on("/clearFailsafeLogs", HTTP_POST, handleClearFailsafeLogs);
  server.on("/clearRelayLogs", HTTP_POST, handleClearRelayLogs);
  server.on("/updateWifi", HTTP_POST, handleUpdateWifi);
  server.on("/disconnectWifi", HTTP_POST, handleDisconnectWifi);
  server.on("/setDeepSleepDuration", HTTP_POST, handleSetDeepSleepDuration);
  server.on("/deepSleep", HTTP_POST, handleDeepSleep);
  server.on("/clearFailsafe", HTTP_POST, handleClearFailsafe);
  server.on("/fs-check", HTTP_GET, handleFsCheck);
  server.on("/autoDeepSleepToggle", HTTP_POST, handleAutoDeepSleepToggle);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/settings", HTTP_GET, handleGetSettings);
  
  server.onFileUpload(handleFileUpload);
  server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html",
      "<form method='POST' action='/upload' enctype='multipart/form-data'>"
      "<input type='file' name='upload'><input type='submit' value='Upload'></form>");
  });
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){}, handleFileUpload);
  
  server.on("/files", HTTP_GET, [](AsyncWebServerRequest *request) {
    Dir dir = LittleFS.openDir("/");
    String html = "<h3>Files:</h3><ul>";
    while (dir.next()) {
      html += "<li>" + dir.fileName() + " (" + String(dir.fileSize()) + ")</li>";
    }
    html += "</ul>";
    request->send(200, "text/html", html);
  });
  
  server.on("/format", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!isLoggedIn(request)) { request->send(403, "text/plain", "Forbidden"); return; }
    handleFormatFS(request);
  });
  
  server.addHandler(new AsyncCallbackJsonWebHandler("/saveSettings", [](AsyncWebServerRequest *request, JsonVariant &json) {
    if (!isLoggedIn(request)) { request->send(403, "text/plain", "Forbidden"); return; }
    StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
    doc.set(json);
    
    strncpy(settings.hostname, doc["hostname"] | "aquamaster", sizeof(settings.hostname) - 1);
    strncpy(settings.staSsid, doc["staSsid"] | "", sizeof(settings.staSsid) - 1);
    strncpy(settings.staPassword, doc["staPassword"] | "", sizeof(settings.staPassword) - 1);
    settings.useDHCP = doc["useDHCP"] | true;
    IPAddress ip;
    if (ip.fromString(doc["staticIp"] | "192.168.1.100")) settings.staticIp = ip;
    if (ip.fromString(doc["staticGateway"] | "192.168.1.1")) settings.staticGateway = ip;
    if (ip.fromString(doc["staticDns1"] | "8.8.8.8")) settings.staticDns1 = ip;
    if (ip.fromString(doc["staticDns2"] | "8.8.4.4")) settings.staticDns2 = ip;
    settings.staticSubnet = IPAddress(255, 255, 255, 0);
    settings.wifiAPModeEnabled = doc["wifiAPModeEnabled"] | false;
    strncpy(settings.apSsid, doc["apSsid"] | "AquaMaster_AP", sizeof(settings.apSsid) - 1);
    strncpy(settings.apPassword, doc["apPassword"] | "aquapass", sizeof(settings.apPassword) - 1);
    settings.calibrationFactor = doc["calibrationFactor"] | 1.0;
    settings.failsafeDurationHours = doc["failsafeDurationHours"] | 12;
    settings.failsafeEnabled = doc["failsafeEnabled"] | true;
    settings.failsafePin = doc["failsafePin"] | D5;
    settings.relayPin = doc["relayPin"] | D1;
    if (doc.containsKey("defaultRelayDuration")) settings.defaultRelayDuration = doc["defaultRelayDuration"].as<int>();
    if (doc.containsKey("maxRelayRuntime")) settings.maxRelayRuntime = doc["maxRelayRuntime"].as<int>();
    if (doc.containsKey("deepSleepDurationSeconds")) settings.deepSleepDurationSeconds = doc["deepSleepDurationSeconds"].as<int>();
    settings.autoDeepSleepEnabled = doc["autoDeepSleepEnabled"] | true;
    
    if (saveSettings()) {
      request->send(200, "text/plain", "Settings saved.");
      broadcastStatus();
      pendingRestart = true;
    } else {
      request->send(500, "text/plain", "Failed to save.");
    }
  }));
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/index.html.gz")) {
      AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/index.html.gz", "text/html");
      response->addHeader("Content-Encoding", "gzip");
      request->send(response);
    } else {
      request->send(500, "text/plain", "index.html.gz not found");
    }
  });
  
  events.onConnect([](AsyncEventSourceClient *client) {
    client->send("connected", "event");
  });
  server.addHandler(&events);
  
  server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest *request) { request->send(204); });
  server.on("/gen_204", HTTP_ANY, [](AsyncWebServerRequest *request) { request->send(204); });
  server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
  server.on("/library/test/success.html", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
  server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
  
  server.onNotFound([](AsyncWebServerRequest *request) {
    if (!WiFi.isConnected() && isApModeActive) {
      AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/index.html.gz", "text/html");
      response->addHeader("Content-Encoding", "gzip");
      request->send(response);
    } else {
      request->send(404, "text/plain", "Not found");
    }
  });
  
  server.begin();
  serverRunning = true;
  Serial.println(F("Web server started."));
}

void stopWebServer() {
  if (!serverRunning) return;
  events.close();
  events.onConnect(nullptr);
  server.end();
  serverRunning = false;
  Serial.println(F("Web server stopped."));
}
void handleLogin(AsyncWebServerRequest *request) {
  if (failedLoginAttempts >= MAX_FAILED_LOGIN_ATTEMPTS && (millis() - lastLoginAttemptMillis < LOGIN_COOLDOWN_TIME)) {
    request->send(429, "text/plain", "Too many attempts. Wait.");
    return;
  }
  if (request->hasArg("username") && request->hasArg("password")) {
    char user[32], pass[64];
    strncpy(user, request->arg("username").c_str(), 31);
    strncpy(pass, request->arg("password").c_str(), 63);
    if (authenticate(user, pass)) {
      generateSessionToken();
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Login successful");
      response->addHeader("Set-Cookie", String("authToken=") + currentSessionToken + "; Path=/; Max-Age=" + (sessionTimeout / 1000));
      request->send(response);
      loggedIn = true;
      failedLoginAttempts = 0;
      broadcastStatus();
    } else {
      failedLoginAttempts++;
      lastLoginAttemptMillis = millis();
      request->send(401, "text/plain", "Unauthorized");
    }
  } else {
    request->send(400, "text/plain", "Missing credentials");
  }
}

void handleUpdateLogin(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  if (!request->hasArg("newUsername") || !request->hasArg("newPassword") || !request->hasArg("oldPassword")) {
    request->send(400, "text/plain", "Missing parameters");
    return;
  }
  char oldPass[64], newUser[32], newPass[64];
  strncpy(oldPass, request->arg("oldPassword").c_str(), 63);
  strncpy(newUser, request->arg("newUsername").c_str(), 31);
  strncpy(newPass, request->arg("newPassword").c_str(), 63);
  
  if (authenticate(settings.adminUsername, oldPass)) {
    strncpy(settings.adminUsername, newUser, sizeof(settings.adminUsername) - 1);
    char hash[33];
    getMD5Hash(newPass, hash, sizeof(hash));
    strncpy(settings.adminPasswordHash, hash, sizeof(settings.adminPasswordHash));
    if (saveSettings()) {
      request->send(200, "text/plain", "Credentials updated.");
      currentSessionToken[0] = '\0';
      loggedIn = false;
    } else {
      request->send(500, "text/plain", "Failed to save.");
    }
  } else {
    request->send(401, "text/plain", "Old password incorrect.");
  }
}

void handleRelayControl(AsyncWebServerRequest *request) {
  char state[16] = {0};
  if (!request->hasArg("state")) {
    request->send(400, "text/plain", "Missing state");
    return;
  }
  strncpy(state, request->arg("state").c_str(), 15);
  
  if (millis() - lastCommandTime < COMMAND_COOLDOWN_MS) {
    request->send(429, "text/plain", "Cooldown active.");
    return;
  }
  if (failsafeActive || emergencyStop) {
    request->send(403, "text/plain", "Failsafe/Emergency active.");
    return;
  }
  
  bool success = false;
  
  if (strcmp(state, "on") == 0 || strcmp(state, "toggle") == 0) {
    if (relayState && strcmp(state, "toggle") == 0) {
      digitalWrite(settings.relayPin, HIGH);
      relayState = false;
      if (relayTicker.active()) relayTicker.detach();
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    } else if (!relayState) {
      relayOnEpochTime = time(nullptr);
      digitalWrite(settings.relayPin, LOW);
      relayState = true;
      relayOnStartTime = millis();
      relayDuration = settings.defaultRelayDuration;
      relayTicker.once(relayDuration, turnOffRelayTimed);
      if (settings.failsafeEnabled && settings.maxRelayRuntime > 0)
        failsafeTicker.attach(settings.maxRelayRuntime, failsafeTriggered);
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }
    success = true;
  } else if (strcmp(state, "off") == 0) {
    if (relayState) {
      digitalWrite(settings.relayPin, HIGH);
      relayState = false;
      if (failsafeTicker.active()) failsafeTicker.detach();
      if (relayTicker.active()) relayTicker.detach();
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }
    success = true;
  } else if (strcmp(state, "timed") == 0 && request->hasArg("duration")) {
    unsigned long duration = strtoul(request->arg("duration").c_str(), nullptr, 10);
    if (duration == 0 || duration > settings.maxRelayRuntime) {
      request->send(400, "text/plain", "Invalid duration.");
      return;
    }
    if (relayState) {
      digitalWrite(settings.relayPin, HIGH);
      relayState = false;
      if (relayTicker.active()) relayTicker.detach();
    }
    relayOnEpochTime = time(nullptr);
    digitalWrite(settings.relayPin, LOW);
    relayState = true;
    relayOnStartTime = millis();
    relayDuration = duration;
    relayTicker.once(duration, turnOffRelayTimed);
    if (settings.failsafeEnabled && settings.maxRelayRuntime > 0)
      failsafeTicker.attach(settings.maxRelayRuntime, failsafeTriggered);
    if (settings.autoDeepSleepEnabled) {
      unsigned long sleepDelay = settings.autoDeepSleepDelayAfterRun;
      if (sleepDelay == 0) sleepDelay = 10;
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepDelayTicker.once(duration + sleepDelay, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
    }
    success = true;
  } else if (strcmp(state, "temporary") == 0) {
    unsigned long duration = 600;
    if (request->hasArg("duration")) {
      duration = strtoul(request->arg("duration").c_str(), nullptr, 10);
      if (duration == 0 || duration > 3600UL) {
        request->send(400, "text/plain", "Invalid duration.");
        return;
      }
    }
    if (relayState) {
      digitalWrite(settings.relayPin, HIGH);
      relayState = false;
      if (relayTicker.active()) relayTicker.detach();
      if (failsafeTicker.active()) failsafeTicker.detach();
    }
    relayOnEpochTime = time(nullptr);
    relayOnStartTime = millis();
    relayDuration = duration;
    digitalWrite(settings.relayPin, LOW);
    relayState = true;
    isTemporaryRun = true;
    relayTicker.once(duration, turnOffRelayTimed);
    if (duration >= 600 && settings.autoDeepSleepEnabled) {
      unsigned long sleepDelay = settings.autoDeepSleepDelayAfterRun;
      if (sleepDelay == 0) sleepDelay = 10;
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepDelayTicker.once(duration + sleepDelay, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
    }
    success = true;
  } else {
    request->send(400, "text/plain", "Invalid state.");
    return;
  }
  
  if (success) {
    request->send(200, "text/plain", "OK");
    lastCommandTime = millis();
    broadcastStatus();
  } else {
    request->send(500, "text/plain", "Failed.");
  }
}

void handleEmergencyStop(AsyncWebServerRequest *request) {
  request->send(200, "application/json", "{\"status\":\"emergency_triggered\"}");
  emergencyStopDelay.once_ms(50, emergencyStopInternal);
}

void handleClearFailsafe(AsyncWebServerRequest *request) {
  clearEmergencyStopAndFailsafe();
  resetFailsafe();
  request->send(200, "text/plain", "Failsafe cleared.");
}

void handleScanNetworks(AsyncWebServerRequest *request) {
  if (scanInProgress) {
    request->send(409, "text/plain", "Scan in progress.");
    return;
  }
  WiFi.mode(WIFI_STA);
  scanInProgress = true;
  WiFi.scanNetworks(true, true);
  request->send(202, "text/plain", "Scan started.");
  wifiScanTicker.attach_ms(1000, broadcastScanResults);
}

void handleKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  StaticJsonDocument<2048> doc;
  JsonArray networks = doc.to<JsonArray>();
  for (const auto& kn : knownNetworks) {
    JsonObject network = networks.add<JsonObject>();
    network["ssid"] = kn.ssid;
  }
  char buf[2048];
  serializeJson(doc, buf);
  request->send(200, "application/json", buf);
}

void handleSaveKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  if (!request->hasArg("plain")) { request->send(400, "text/plain", "Missing JSON."); return; }
  
  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, request->arg("plain"))) {
    request->send(400, "text/plain", "Invalid JSON.");
    return;
  }
  JsonArray arr = doc.as<JsonArray>();
  if (arr.isNull()) { request->send(400, "text/plain", "Not an array."); return; }
  
  knownNetworks.clear();
  for (JsonObject net : arr) {
    KnownNetwork kn;
    strncpy(kn.ssid, net["ssid"] | "", 32);
    strncpy(kn.password, net["password"] | "", 64);
    knownNetworks.push_back(kn);
  }
  request->send(200, "text/plain", "Networks updated.");
}

void handleClearKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  knownNetworks.clear();
  request->send(200, "text/plain", "Networks cleared.");
}

void handleAPModeToggle(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  if (!request->hasArg("enable")) { request->send(400, "text/plain", "Missing enable."); return; }
  
  bool enable = strcasecmp(request->arg("enable").c_str(), "true") == 0;
  settings.wifiAPModeEnabled = enable;
  if (saveSettings()) {
    if (enable) {
      pendingAPSetup = true;
      request->send(200, "text/plain", "AP mode enabled.");
    } else {
      request->send(200, "text/plain", "AP mode disabled. Restarting...");
      pendingRestart = true;
    }
  } else {
    request->send(500, "text/plain", "Failed to save.");
  }
}

void handleRestart(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", "Restart scheduled.");
  pendingRestart = true;
}

void handleRestoreFactorySettings(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  request->send(200, "text/plain", "Restoring factory settings...");
  factoryResetPreserveIndex();
}

void handleUpdateWifi(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  if (!request->hasArg("ssid") || !request->hasArg("password")) {
    request->send(400, "text/plain", "Missing SSID or Password.");
    return;
  }
  strncpy(settings.staSsid, request->arg("ssid").c_str(), 32);
  strncpy(settings.staPassword, request->arg("password").c_str(), 64);
  if (saveSettings()) {
    request->send(200, "text/plain", "WiFi updated. Restarting...");
    pendingRestart = true;
  } else {
    request->send(500, "text/plain", "Failed to save.");
  }
}

void handleDisconnectWifi(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  WiFi.disconnect(true);
  settings.staSsid[0] = '\0';
  settings.staPassword[0] = '\0';
  saveSettings();
  request->send(200, "text/plain", "Disconnected.");
  broadcastStatus();
}

void handleSetDeepSleepDuration(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  if (request->hasArg("duration")) {
    settings.deepSleepDurationSeconds = strtoul(request->arg("duration").c_str(), NULL, 10);
    if (saveSettings()) {
      request->send(200, "text/plain", "Duration updated.");
      broadcastStatus();
    } else {
      request->send(500, "text/plain", "Failed to save.");
    }
  } else {
    request->send(400, "text/plain", "Missing duration.");
  }
}

void handleDeepSleep(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  if (settings.deepSleepDurationSeconds == 0) {
    request->send(400, "text/plain", "Duration is 0.");
    return;
  }
  initiateAutoDeepSleep();
  request->send(200, "text/plain", "Deep sleep initiated.");
}

void handleFormatFS(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  request->send(200, "text/plain", "Formatting...");
  factoryResetPreserveIndex();
  pendingRestart = true;
}

void handleAutoDeepSleepToggle(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  if (!request->hasArg("enable")) { request->send(400, "text/plain", "Missing enable."); return; }
  
  settings.autoDeepSleepEnabled = request->arg("enable").equalsIgnoreCase("true");
  if (saveSettings()) {
    request->send(200, "text/plain", "Setting updated.");
    broadcastStatus();
  } else {
    request->send(500, "text/plain", "Failed to save.");
  }
}

void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  FSLockGuard lock;
  if (!lock.isLocked()) return;
  
  const char *targetFile = "/index.html.gz";
  if (index == 0) {
    if (LittleFS.exists(targetFile)) LittleFS.remove(targetFile);
    request->_tempFile = LittleFS.open(targetFile, "w");
    if (!request->_tempFile) {
      request->send(500, "text/plain", "Failed to open file");
      return;
    }
  }
  if (len && request->_tempFile) {
    request->_tempFile.write(data, len);
  }
  if (final) {
    if (request->_tempFile) request->_tempFile.close();
    request->send(200, "text/plain", "Upload successful");
  }
}
void handleStatus(AsyncWebServerRequest *request) {
  broadcastStatus();
  StaticJsonDocument<512> doc;
  doc["relayState"] = relayState;
  doc["failsafeActive"] = failsafeActive;
  doc["emergencyStop"] = emergencyStop;
  doc["loggedIn"] = isLoggedIn(request);
  doc["wifiStatus"] = WiFi.status();
  doc["ipAddress"] = WiFi.localIP().toString();
  doc["ssid"] = WiFi.SSID();
  doc["hostname"] = settings.hostname;
  doc["apModeActive"] = isApModeActive;
  doc["scanInProgress"] = scanInProgress;
  doc["deepSleepRequested"] = deepSleepRequested;
  doc["deepSleepDurationSeconds"] = settings.deepSleepDurationSeconds;
  doc["autoDeepSleepScheduled"] = autoDeepSleepScheduled;
  doc["autoDeepSleepEnabled"] = settings.autoDeepSleepEnabled;
  doc["heap"] = ESP.getFreeHeap();
  
  char buf[512];
  serializeJson(doc, buf);
  request->send(200, "application/json", buf);
}

void handleGetSettings(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  
  StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
  doc["staSsid"] = settings.staSsid;
  doc["hostname"] = settings.hostname;
  doc["useDHCP"] = settings.useDHCP;
  doc["staticIp"] = settings.staticIp.toString();
  doc["staticGateway"] = settings.staticGateway.toString();
  doc["staticSubnet"] = settings.staticSubnet.toString();
  doc["staticDns1"] = settings.staticDns1.toString();
  doc["staticDns2"] = settings.staticDns2.toString();
  doc["wifiAPModeEnabled"] = settings.wifiAPModeEnabled;
  doc["apSsid"] = settings.apSsid;
  doc["calibrationFactor"] = settings.calibrationFactor;
  doc["failsafeDurationHours"] = settings.failsafeDurationHours;
  doc["failsafeEnabled"] = settings.failsafeEnabled;
  doc["failsafePin"] = settings.failsafePin;
  doc["relayPin"] = settings.relayPin;
  doc["defaultRelayDuration"] = settings.defaultRelayDuration;
  doc["maxRelayRuntime"] = settings.maxRelayRuntime;
  doc["deepSleepDurationSeconds"] = settings.deepSleepDurationSeconds;
  doc["autoDeepSleepEnabled"] = settings.autoDeepSleepEnabled;
  
  char buf[SETTINGS_DOC_CAPACITY];
  serializeJson(doc, buf);
  request->send(200, "application/json", buf);
}

void handleFsCheck(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) { request->send(401, "text/plain", "Unauthorized"); return; }
  
  FSInfo fs_info;
  if (!LittleFS.info(fs_info)) {
    request->send(500, "text/plain", "Failed to read FS info.");
    return;
  }
  
  StaticJsonDocument<256> doc;
  doc["totalBytes"] = fs_info.totalBytes;
  doc["usedBytes"] = fs_info.usedBytes;
  doc["blockSize"] = fs_info.blockSize;
  doc["pageSize"] = fs_info.pageSize;
  
  char buf[256];
  serializeJson(doc, buf);
  request->send(200, "application/json", buf);
}

void handleRelayLogs(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->print("[");
  
  FSLockGuard lock;
  if (!lock.isLocked() || !LittleFS.exists(RELAY_LOG_FILE)) {
    response->print("]");
    request->send(response);
    return;
  }
  
  File logFile = LittleFS.open(RELAY_LOG_FILE, "r");
  if (!logFile) {
    response->print("]");
    request->send(response);
    return;
  }
  
  bool first = true;
  char buffer[LOG_ENTRY_LENGTH + 1] = {0};
  
  for (uint8_t i = 0; i < MAX_LOGS; i++) {
    uint8_t idx = (relayLogIndex + i) % MAX_LOGS;
    logFile.seek(idx * LOG_ENTRY_LENGTH, SeekSet);
    memset(buffer, 0, sizeof(buffer));
    logFile.readBytes(buffer, LOG_ENTRY_LENGTH);
    
    if (buffer[0] == '\0') continue;
    
    char* nl = strchr(buffer, '\n');
    if (nl) *nl = '\0';
    char* comma = strchr(buffer, ',');
    if (!comma) continue;
    
    *comma = '\0';
    uint32_t epoch = strtoul(buffer, nullptr, 10);
    uint32_t duration = strtoul(comma + 1, nullptr, 10);
    
    if (epoch < 1600000000UL || duration == 0 || duration > 86400UL) continue;
    
    if (!first) response->print(",");
    response->printf("{\"epoch\":%lu,\"duration\":%lu}", epoch, duration);
    first = false;
  }
  
  logFile.close();
  response->print("]");
  request->send(response);
}

void handleFailsafeLogs(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->print("[");
  
  FSLockGuard lock;
  if (!lock.isLocked()) {
    response->print("]");
    request->send(response);
    return;
  }
  
  File logFile = LittleFS.open(FAILSAFE_LOG_FILE, "r");
  if (!logFile) {
    response->print("]");
    request->send(response);
    return;
  }
  
  bool first = true;
  char buffer[LOG_ENTRY_LENGTH + 1] = {0};
  
  for (uint8_t i = 0; i < MAX_LOGS; i++) {
    uint8_t idx = (failsafeLogIndex + i) % MAX_LOGS;
    logFile.seek(idx * LOG_ENTRY_LENGTH, SeekSet);
    logFile.readBytes(buffer, LOG_ENTRY_LENGTH);
    buffer[LOG_ENTRY_LENGTH] = '\0';
    
    char *epochStr = strtok(buffer, ",");
    char *category = strtok(NULL, ",");
    char *reasonStr = strtok(NULL, ",");
    
    if (reasonStr) reasonStr[strcspn(reasonStr, "\r\n")] = '\0';
    
    if (epochStr && reasonStr) {
      uint32_t epoch = strtoul(epochStr, nullptr, 10);
      if (!first) response->print(",");
      response->printf("{\"timestamp\":%lu,\"reason\":\"%s\"}", epoch, reasonStr);
      first = false;
    }
  }
  
  logFile.close();
  response->print("]");
  request->send(response);
}

void handleClearRelayLogs(AsyncWebServerRequest *request) {
  FSLockGuard lock;
  if (!lock.isLocked()) return;
  LittleFS.remove(RELAY_LOG_FILE);
  relayLogIndex = 0;
  request->send(200, "text/plain", "Relay logs cleared.");
}

void handleClearFailsafeLogs(AsyncWebServerRequest *request) {
  FSLockGuard lock;
  if (!lock.isLocked()) return;
  LittleFS.remove(FAILSAFE_LOG_FILE);
  failsafeLogIndex = 0;
  request->send(200, "text/plain", "Failsafe logs cleared.");
}

void broadcastStatus() {
  if (events.count() == 0) return;
  
  StaticJsonDocument<768> doc;
  doc["relayState"] = relayState;
  doc["failsafeActive"] = failsafeActive;
  doc["emergencyStop"] = emergencyStop;
  doc["loggedIn"] = loggedIn;
  doc["scanInProgress"] = scanInProgress;
  doc["relayOnStartTime"] = relayOnStartTime;
  doc["currentDeviceMillis"] = millis();
  doc["currentRelayDuration"] = relayDuration;
  
  wl_status_t ws = WiFi.status();
  doc["wifiStatus"] = ws;
  doc["ipAddress"] = WiFi.localIP().toString();
  doc["ssid"] = WiFi.SSID();
  doc["hostname"] = settings.hostname;
  doc["apModeActive"] = isApModeActive;
  
  const char* wifiStatusStr = "Unknown";
  switch (ws) {
    case WL_IDLE_STATUS: wifiStatusStr = "Idle"; break;
    case WL_NO_SSID_AVAIL: wifiStatusStr = "No SSID"; break;
    case WL_SCAN_COMPLETED: wifiStatusStr = "Scan Done"; break;
    case WL_CONNECTED: wifiStatusStr = "Connected"; break;
    case WL_CONNECT_FAILED: wifiStatusStr = "Failed"; break;
    case WL_CONNECTION_LOST: wifiStatusStr = "Lost"; break;
    case WL_DISCONNECTED: wifiStatusStr = "Disconnected"; break;
  }
  doc["wifiStatusText"] = wifiStatusStr;
  
  if (WiFi.getMode() & WIFI_AP) {
    doc["apIP"] = WiFi.softAPIP().toString();
    doc["apSSID"] = WiFi.softAPSSID();
  }
  
  doc["deepSleepRequested"] = deepSleepRequested;
  doc["deepSleepDurationSeconds"] = settings.deepSleepDurationSeconds;
  doc["autoDeepSleepScheduled"] = autoDeepSleepScheduled;
  doc["autoDeepSleepEnabled"] = settings.autoDeepSleepEnabled;
  doc["defaultRelayDuration"] = settings.defaultRelayDuration;
  doc["maxRelayRuntime"] = settings.maxRelayRuntime;
  doc["heap"] = ESP.getFreeHeap();
  
  String out;
  serializeJson(doc, out);
  events.send(out.c_str(), "status");
}

void broadcastScanResults() {
  if (events.count() == 0) return;
  
  int n = WiFi.scanComplete();
  if (n >= 0) {
    wifiScanTicker.detach();
    scanInProgress = false;
    
    StaticJsonDocument<4096> doc;
    JsonArray networks = doc.to<JsonArray>();
    
    for (int i = 0; i < n; ++i) {
      JsonObject network = networks.add<JsonObject>();
      network["ssid"] = WiFi.SSID(i);
      network["rssi"] = WiFi.RSSI(i);
      network["encryption"] = (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted";
    }
    
    WiFi.scanDelete();
    
    char buf[4096];
    serializeJson(doc, buf);
    events.send(buf, "scanResults");
  }
}
AsyncCallbackJsonWebHandler* saveSettingsHandler = nullptr;