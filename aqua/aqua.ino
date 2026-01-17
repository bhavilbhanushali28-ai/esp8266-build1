#include "rtc_data.h"
RTCData rtcData;
uint32_t rtcChecksum(const RTCData &d) {
  return d.magic ^ d.remainingSleepSeconds;
}
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
#include <DNSServer.h>
#include <pgmspace.h>
#include <TimeLib.h> 
#include <AsyncJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <core_version.h>
#define BOT_TOKEN "8069292937:AAGleYiuXQjYCr0K24k6tEazagmqfCXlud8"  
#define CHAT_ID   "-1003543724337"                            
BearSSL::WiFiClientSecure secureClient;
UniversalTelegramBot bot(BOT_TOKEN, secureClient);
bool telegramSending = false;
bool telegramDelayActive = false;
bool telegramSendPending = false;
bool telegramIpSent = false;
bool telegramAttemptFinished = false;
bool telegramTriggeredForThisConnection = false;
bool webStartPending = false;
bool webStartScheduled = false;
bool webServerStartPending = false;
bool tlsConfigured = false;
uint32_t telegramSendStartTime = 0;
uint32_t webStartAt = 0;
unsigned long bootStart = 0;
uint8_t telegramRetryCount = 0;
const uint8_t MAX_TELEGRAM_RETRIES = 3;
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
void startNonBlockingWiFiScan();
void processScanResults();
#define ARDUINOJSON_USE_DEPRECATED 0
#define FAILSAFE_LOG_FILE "/failsafe.bin"
#define RELAY_LOG_FILE    "/relay.bin"
#define MAX_LOGS          10
#define LOG_ENTRY_LENGTH  128
#define DEBUG_MODE 1
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
// Add near top of file:
#define MIN_HEAP_THRESHOLD 8000  // Minimum heap before refusing operations

bool checkHeapBeforeOperation(const char* operation) {
  uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < MIN_HEAP_THRESHOLD) {
    Serial.printf("⚠️ Low heap (%u) - skipping %s\n", freeHeap, operation);
    return false;
  }
  return true;
}

const unsigned long DEBOUNCE_DELAY = 100;
unsigned long apStartTime = 0;
unsigned long sleepRequestTime = 0;
volatile unsigned long lastEmergencyPressTime = 0;
//extern AsyncCallbackJsonWebHandler* saveSettingsHandler;
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
struct PredefinedNetwork {
    const char ssid[32];
    const char password[64];
};
const PredefinedNetwork hardcodedNetworks[] = {
    {"realme 5g", "bhavil23!"},
    {"ishwar", "12345689"}, 
    {"note 5pro", "bhavil23!"},    
    {"lenovo", "12345678"}      
};
const size_t NUM_HARDCODED_NETWORKS = sizeof(hardcodedNetworks) / sizeof(hardcodedNetworks[0]);
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
const size_t SETTINGS_DOC_CAPACITY = 1024; 
struct Settings {
  char staSsid[32];
  char staPassword[64];
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
Settings defaultSettings;
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
bool otaEnabled = false;
bool serverStarted = false;
const char* SETTINGS_FILE = "/settings.json";
const char* SETTINGS_BACKUP_FILE = "/settings.bak";
const char* BOOT_COUNT_FILE = "/boot_count.json";
const char* BOOT_RECOVERY_FILE = "/boot_recovery.json";
volatile bool watchdogFeedRequested = false;
volatile bool shouldBroadcastAfterEmergencyClear = false;
volatile bool shouldLogEmergencyClear = false;
void setupLittleFS();
void initWiFi();
void manageWiFiConnection();
bool isWiFiStable(AsyncWebServerRequest *request);
void startWebServer();
void stopWebServer(); 
void setupAP();
void checkAPTimeout();
void checkSwitchPress();
void generateSessionToken();
bool authenticate(const char* user, const char* pass);
bool isLoggedIn(AsyncWebServerRequest *request);
const char* getContentType(const char* filename);
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
void handleNotFound(AsyncWebServerRequest *request);
void handleUpdateWifi(AsyncWebServerRequest *request);
void handleDisconnectWifi(AsyncWebServerRequest *request); 
void handleSetDeepSleepDuration(AsyncWebServerRequest *request);
void handleDeepSleep(AsyncWebServerRequest *request);            
void handleFormatFS(AsyncWebServerRequest *request);
void handleFileUpload(AsyncWebServerRequest *request,
                      String filename,
                      size_t index,
                      uint8_t *data,
                      size_t len,
                      bool final);
void handleStatus(AsyncWebServerRequest *request);
void handleGetSettings(AsyncWebServerRequest *request); 
void handleFsCheck(AsyncWebServerRequest *request); 
void handleAutoDeepSleepToggle(AsyncWebServerRequest *request); 
void broadcastStatus();
void broadcastScanResults();
void IRAM_ATTR feedWatchdog();
void setupWatchdog();
void checkLoopWatchdog();
void startNonBlockingWiFiScan() {
  if (scanInProgress) {
    return;
  }
  Serial.println(F("Starting non-blocking WiFi scan..."));
 WiFi.setOutputPower(17.0);
  WiFi.scanNetworks(true); 
  scanInProgress = true;
  lastScanStartTime = millis();
}
bool acquireFSLock(unsigned long timeout = 3000) { 
  unsigned long start = millis();
  while (fsLock) {
    if (millis() - start > timeout) {
      Serial.println(F("âš ï¸ FS lock wait timed out"));
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
  FSLockGuard() {
    locked = acquireFSLock();
  }
  ~FSLockGuard() {
    if (locked) {
      releaseFSLock();
    }
  }
  bool isLocked() const {
    return locked;
  }
private:
  bool locked;
};
void IRAM_ATTR feedWatchdog() {
  watchdogFeedRequested = true; 
}
void processScanResults() {
  if (!scanInProgress) return; 
  int n = WiFi.scanComplete();
  if (n >= 0) { 
    scanInProgress = false; 
    Serial.printf_P(PSTR("%d networks found:\n"), n);
    
    // Use static buffer instead of JsonDocument
    static char jsonBuffer[1024];
    int offset = 0;
    offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "[");
    
    const int maxNetworks = min(n, 10);
    for (int i = 0; i < maxNetworks; ++i) {
      if (i > 0) {
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, ",");
      }
      offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
        "{\"ssid\":\"%s\",\"rssi\":%d,\"encryption\":\"%s\"}",
        WiFi.SSID(i).c_str(),
        WiFi.RSSI(i),
        (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted"
      );
      
      Serial.printf_P(PSTR("  %d: %s (%d dBm) %s\n"),
                      i + 1,
                      WiFi.SSID(i).c_str(),
                      WiFi.RSSI(i),
                      (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted");
    }
    offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "]");
    
    events.send(jsonBuffer, "wifiScanResults");
    Serial.println(F("WiFi scan complete and results broadcasted."));
    WiFi.scanDelete(); 
    lastScanStartTime = millis(); 
  }
}
void handleRelayControl(AsyncWebServerRequest *request) {
  char state[16] = {0};
  if (!request->hasArg("state")) {
    request->send(400, "text/plain", "Missing state parameter (on/off/toggle/timed/temporary)");
    return;
  }
  strncpy(state, request->arg("state").c_str(), sizeof(state) - 1);
  if (millis() - lastCommandTime < COMMAND_COOLDOWN_MS) {
    request->send(429, "text/plain", "Command cooldown active. Please wait.");
    return;
  }
  if (failsafeActive || emergencyStop) {
    request->send(403, "text/plain", "Forbidden: Failsafe or Emergency Stop is active.");
    return;
  }
  bool success = false;
  if (strcmp(state, "on") == 0 || strcmp(state, "toggle") == 0) {
    if (relayState) {
      if (strcmp(state, "toggle") == 0) {
        digitalWrite(settings.relayPin, HIGH);
        relayState = false;
        Serial.println(F("Relay OFF (toggle)."));
        if (relayTicker.active()) relayTicker.detach();
        settings.wasTimedRunInterrupted = false;
        settings.interruptedRunDuration = 0;
        settings.interruptedRunStartTime = 0;
        shouldSaveSettingsFlag = true;
        if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
        autoDeepSleepScheduled = false;
      }
      success = true;
    } else {
      relayOnEpochTime = time(nullptr);
      digitalWrite(settings.relayPin, LOW);
      relayState = true;
      relayOnStartTime = millis();
      relayDuration = settings.defaultRelayDuration;
      relayTicker.once(relayDuration, turnOffRelayTimed);
      Serial.printf_P(PSTR("Relay ON (timed for %lu sec).\n"), relayDuration);
      if (settings.failsafeEnabled && settings.maxRelayRuntime > 0)
        failsafeTicker.attach(settings.maxRelayRuntime, failsafeTriggered);
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
      success = true;
    }
  } else if (strcmp(state, "off") == 0) {
    if (relayState) {
      digitalWrite(settings.relayPin, HIGH);
      relayState = false;
      Serial.println(F("Relay OFF (via API)."));
      if (failsafeTicker.active()) failsafeTicker.detach();
      if (relayTicker.active()) relayTicker.detach();
      settings.wasTimedRunInterrupted = false;
      settings.interruptedRunDuration = 0;
      settings.interruptedRunStartTime = 0;
      shouldSaveSettingsFlag = true;
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }
    success = true;
  } else if (strcmp(state, "timed") == 0 && request->hasArg("duration")) {
    unsigned long duration = strtoul(request->arg("duration").c_str(), nullptr, 10);
    if (duration == 0 || duration > settings.maxRelayRuntime) {
      request->send(400, "text/plain", "Invalid or too long duration.");
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
    Serial.printf_P(PSTR("Relay ON (timed override: %lu sec).\n"), duration);
    if (settings.failsafeEnabled && settings.maxRelayRuntime > 0)
      failsafeTicker.attach(settings.maxRelayRuntime, failsafeTriggered);
    if (settings.autoDeepSleepEnabled) {
      unsigned long sleepDelay = settings.autoDeepSleepDelayAfterRun;
      if (sleepDelay == 0) sleepDelay = 10;
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepDelayTicker.once(duration + sleepDelay, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
      Serial.printf_P(PSTR("Auto deep sleep scheduled after timed run (delay %lu sec).\n"), sleepDelay);
    } else {
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }
    success = true;
  } else if (strcmp(state, "temporary") == 0) {
    unsigned long duration = 600;  
    if (request->hasArg("duration")) {
      duration = strtoul(request->arg("duration").c_str(), nullptr, 10);
      if (duration == 0 || duration > 3600UL) {
        request->send(400, "text/plain", "Invalid duration (max 3600 sec).");
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
    Serial.printf_P(PSTR("Relay ON (temporary for %lu sec, no flash writes).\n"), duration);
    if (duration >= 600 && settings.autoDeepSleepEnabled) {
      unsigned long sleepDelay = settings.autoDeepSleepDelayAfterRun;
      if (sleepDelay == 0) sleepDelay = 10;
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepDelayTicker.once(duration + sleepDelay, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
      Serial.printf_P(
        PSTR("Auto deep sleep scheduled after temporary run (duration %lu sec, delay %lu sec).\n"),
        duration, sleepDelay
      );
    } else {
      if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }
    success = true;
  } else {
    request->send(400, "text/plain", "Invalid state parameter.");
    return;
  }
  if (success) {
    request->send(200, "text/plain", "Command processed.");
    lastCommandTime = millis();
    broadcastStatus();
  } else {
    request->send(500, "text/plain", "Failed to process command.");
  }
}
void logRelayRunTime() {
  time_t now = time(nullptr);
  if (now <= 1600000000UL || relayOnEpochTime == 0) {
    Serial.println(F("âš ï¸ Time not synced or relayOnEpochTime is 0. Skipping relay log."));
    return;
  }
  uint32_t duration = now - relayOnEpochTime;
  if (duration == 0) duration = 1;
  if (duration > 86400UL) {
    Serial.printf("âš ï¸ Invalid relay log: duration too long (%lu s). Skipping.\n", duration);
    return;
  }
  char entry[LOG_ENTRY_LENGTH] = {0};
  snprintf(entry, sizeof(entry), "%lu,%lu\n", (uint32_t)relayOnEpochTime, duration);
  entry[LOG_ENTRY_LENGTH - 1] = '\0';
  Serial.printf("ðŸ“ Log Entry: '%s'\n", entry);
  ESP.wdtFeed();
  circularLogWrite(RELAY_LOG_FILE, entry, relayLogIndex);
  ESP.wdtFeed();
  Serial.printf("âœ… Logged Relay Run: Epoch %lu, Duration %lu s\n", relayOnEpochTime, duration);
#ifdef DEBUG_MODE
  File file = LittleFS.open(RELAY_LOG_FILE, "r");
  if (file) {
    Serial.println("ðŸ”Ž Validating log content:");
    while (file.available()) {
      String line = file.readStringUntil('\n');
      Serial.println(line);
    }
    file.close();
  } else {
    Serial.println("âŒ Failed to open log file for validation.");
  }
#endif
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
  int len = snprintf(logLine, sizeof(logLine),
                     "%lu,Failsafe,%s\n", (unsigned long)now, cleanReason);
  ESP.wdtFeed();  
  {
    FSLockGuard guard;
    File f = LittleFS.open(FAILSAFE_LOG_FILE, "a");
    if (!f) {
      Serial.println(F("❌ Could not open failsafe log for writing!"));
      return;
    }
    f.write((const uint8_t*)logLine, len);  
    f.flush(); 
    f.close();
  }
  ESP.wdtFeed();  
}
void applyEstimatedTimeFromSnapshot() {
  File f = LittleFS.open("/time_sync.dat", "r");
  if (!f) {
    Serial.println(F("⚠️ No saved time snapshot found."));
    return;
  }
  String line = f.readStringUntil('\n');
  f.close();
  int comma = line.indexOf(',');
  if (comma == -1) {
    Serial.println(F("❌ Invalid time snapshot format."));
    return;
  }
  uint32_t savedEpoch = line.substring(0, comma).toInt();
  uint32_t savedMillis = line.substring(comma + 1).toInt();
  estimatedEpochTime = savedEpoch + ((millis() - savedMillis) / 1000);  
  struct timeval tv = { (time_t)estimatedEpochTime, 0 };
  settimeofday(&tv, nullptr);
  Serial.printf(" Applied estimated time: %lu (from snapshot)\n", estimatedEpochTime);
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
      Serial.printf("âœ… Time synced to system clock: %lu (epoch)\n", lastEpoch);
      Serial.print("Formatted Time: ");
      Serial.println(timeClient.getFormattedTime());
      File f = LittleFS.open("/time_sync.dat", "w");
      if (f) {
        f.printf("%lu,%lu\n", (uint32_t)epochTime, millis());
        f.close();
        Serial.printf("ðŸ•’ Saved time snapshot: epoch=%lu, millis=%lu\n", epochTime, millis());
      } else {
        Serial.println(F("âŒ Failed to write time snapshot."));
      }
    } else {
      Serial.println("âŒ Failed to force update NTP time.");
    }
  } else {
    Serial.println("âš ï¸ NTP skipped, WiFi not connected.");
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
void factoryResetPreserveIndex() {
  Serial.println(F("!! Factory Reset: Preserving index.html.gz"));
  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    String filenameStr = dir.fileName();
    const char* filename = filenameStr.c_str();
    if (strstr(filename, "index.html.gz") != nullptr) {
      Serial.printf("Preserving: %s\n", filename);
      continue;
    }
    Serial.printf("Deleting: %s\n", filename);
    LittleFS.remove(filename);
  }
  applyDefaultSettings();
  if (saveSettings()) {
    Serial.println(F("Default settings saved."));
  } else {
    Serial.println(F("Failed to save default settings after factory reset."));
  }
  bootRecovery.bootCount = 0;
  saveBootRecovery();
  Serial.println(F("Boot counter reset."));
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
  Serial.println(F("Factory reset complete. Restarting ESP..."));
  pendingRestart = true;  
}
void checkRecovery() {
  Serial.println(F("--- checkRecovery() called ---"));
  bootRecovery.lastBootMillis = millis();
  bootRecovery.bootCount++; 
  Serial.printf("Boot count: %u\n", bootRecovery.bootCount);
  saveBootRecovery();
  String reason = ESP.getResetReason(); 
  Serial.print(F("Reset reason: "));
  Serial.println(reason);
  if ((bootRecovery.bootCount >= 3 && reason == "Exception") || reason.indexOf("Software Watchdog") != -1) {
    Serial.println(F("Critical error detected (3+ exceptions or software watchdog reset). Initiating factory reset..."));
    factoryResetPreserveIndex(); 
  }
   Serial.println(F("--- checkRecovery() finished ---"));
}
bool loadRTC() {
  ESP.rtcUserMemoryRead(RTC_SLOT, (uint32_t*)&rtcData, sizeof(rtcData));
  if (rtcData.magic != RTC_MAGIC) {
    return false;
  }
  if (rtcData.checksum != rtcChecksum(rtcData)) {
    return false;   
  }
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
void setup() {
  Serial.begin(115200);
  Serial.println("ESP8266 core detected!");
  Serial.println(ESP.getCoreVersion());
  while (!Serial && millis() < 5000);
  Serial.println(F("\nAquaMaster SSE 101 AI Starting..."));
  Serial.println(F("===== BOOT START ====="));
  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());
  secureClient.setInsecure();
secureClient.setBufferSizes(1024, 1024);
  bootStart = millis();
  bool rtcValid = loadRTC();
  if (rtcValid) {
    Serial.println(F("✅ RTC load SUCCESS"));
    Serial.printf("RTC remainingSleepSeconds = %lu\n", rtcData.remainingSleepSeconds);
    if (rtcData.remainingSleepSeconds > 86400UL) {
      Serial.println(F("⚠️ RTC corrupted. Clearing."));
      clearRTC();
      rtcValid = false;
    }
  } else {
    Serial.println(F("❌ RTC load FAILED / INVALID"));
    clearRTC();
  }
  if (rtcValid && rtcData.remainingSleepSeconds > 0) {
    Serial.printf("MID-CYCLE WAKE: %u sec remaining\n", rtcData.remainingSleepSeconds);
    isMidCycleWake = true;
  } else {
    Serial.println(F("✅ FINAL WAKE: Normal boot"));
    isMidCycleWake = false;
    clearRTC();
  }
  if (isMidCycleWake) {
    Serial.println(F("===== MID-CYCLE: Minimal setup ====="));
    return;
  }
  Serial.println(F("===== FULL INITIALIZATION ====="));
  if (!LittleFS.begin()) {
    Serial.println(F("❌ LittleFS mount failed."));
    factoryResetPreserveIndex();
  } else {
    Serial.println(F("✅ LittleFS mounted."));
    if (!LittleFS.check()) {
      Serial.println(F("⚠️ LittleFS check failed!"));
    }
  }
  applyDefaultSettings();
  if (!loadSettings()) {
    Serial.println(F("⚠️ Settings failed. Factory reset."));
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
  initWiFi();
  timeClient.begin();
  Serial.println(F("NTP Client initialized."));
  setupWatchdog();
  softwareWatchdogTicker.attach_ms(5000, checkLoopWatchdog);
  statusTicker.attach_ms(5000, broadcastStatus);
  applyEstimatedTimeFromSnapshot();
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
  autoDeepSleepScheduled = false;
  deepSleepRequested = false;
  sleepRequestTime = 0;
  telegramSendPending = false;
  telegramDelayActive = false;
  telegramSending = false;
  telegramTriggeredForThisConnection = false;  
  webServerStartPending = false;
  serverRunning = false;
  wifiConnected = false;  
  Serial.println(F("===== SETUP COMPLETE ====="));
}
bool hardcodedSSIDAvailable(const char* ssid) {
    int16_t numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        if (WiFi.SSID(i) == ssid) {
            return true; 
        }
    }
    return false; 
}
void manageWiFiConnection() {
    if (isMidCycleWake) {
        return;  
    }

    if (isApModeActive && WiFi.getMode() == WIFI_AP) {
        checkAPTimeout();
        return;
    }

    static unsigned long lastReconnectAttempt = 0;
    static int failedSavedStaAttempts = 0;
    const unsigned long reconnectInterval = 5000;

    if (WiFi.getMode() != WIFI_STA) return;

    wl_status_t status = WiFi.status();

    if (status != WL_CONNECTED) {
        wifiConnected = false;
        telegramTriggeredForThisConnection = false;

        if (status == WL_IDLE_STATUS || status == WL_DISCONNECTED) {
            if (!tryingHardcodedNetworks) {
                if (millis() - lastReconnectAttempt >= reconnectInterval) {
                    Serial.println(F("WiFi disconnected. Reconnecting to saved network..."));
                    WiFi.begin(settings.staSsid, settings.staPassword);
                    lastReconnectAttempt = millis();
                    failedSavedStaAttempts++;

                    if (failedSavedStaAttempts >= MAX_STA_CONNECT_ATTEMPTS) {
                        Serial.println(F("Saved STA failed. Checking hardcoded networks..."));
                        bool found = false;
                        for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                            if (hardcodedSSIDAvailable(hardcodedNetworks[i].ssid)) {
                                currentHardcodedNetworkIndex = i;
                                found = true;
                                break;
                            }
                        }
                        if (found) {
                            tryingHardcodedNetworks = true;
                            failedSavedStaAttempts = 0;
                            WiFi.begin(
                                hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                                hardcodedNetworks[currentHardcodedNetworkIndex].password
                            );
                            hardcodedConnectAttemptStart = millis();
                        } else {
                            failedSavedStaAttempts = 0;
                        }
                    }
                }
            } else {
                // Trying hardcoded networks
                if (millis() - hardcodedConnectAttemptStart >= HARDCODED_CONNECT_TIMEOUT_MS) {
                    currentHardcodedNetworkIndex++;
                    while (currentHardcodedNetworkIndex < (int)NUM_HARDCODED_NETWORKS &&
                           !hardcodedSSIDAvailable(hardcodedNetworks[currentHardcodedNetworkIndex].ssid)) {
                        currentHardcodedNetworkIndex++;
                    }

                    if (currentHardcodedNetworkIndex < (int)NUM_HARDCODED_NETWORKS) {
                        WiFi.begin(
                            hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                            hardcodedNetworks[currentHardcodedNetworkIndex].password
                        );
                        hardcodedConnectAttemptStart = millis();
                    } else {
                        // All hardcoded networks failed
                        if (telegramSendPending || telegramDelayActive || telegramSending) {
                            Serial.println(F("Telegram pending — delaying AP fallback"));
                            return;
                        }
                        Serial.println(F("All networks failed. Switching to AP mode."));
                        tryingHardcodedNetworks = false;
                        currentHardcodedNetworkIndex = 0;
                        setupAP();
                        isApModeActive = true;
                    }
                }
            }
        }
        return;
    }

    // ========== CONNECTED STATE ==========
    
    // Reset connection attempt counters
    failedSavedStaAttempts = 0;
    tryingHardcodedNetworks = false;
    currentHardcodedNetworkIndex = 0;
    hardcodedConnectAttemptStart = 0;

    // Handle first-time connection
    if (!wifiConnected) {
        wifiConnected = true;
        isApModeActive = false;
        
        Serial.println(F("✅ WiFi connected!"));
        Serial.print(F("IP: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("MAC: "));
        Serial.println(WiFi.macAddress());

        // ========== TRIGGER TELEGRAM NOTIFICATION ==========
        if (!telegramTriggeredForThisConnection) {
            Serial.println(F(" >>> TRIGGERING TELEGRAM NOW <<<"));
            telegramTriggeredForThisConnection = true;
            
            // Check heap before scheduling Telegram
            if (ESP.getFreeHeap() > 15000) {
                telegramSendPending = true;
                telegramDelayActive = false;
                telegramSending = false;
                telegramRetryCount = 0;
                Serial.println(F("Telegram scheduled successfully!"));
            } else {
                Serial.println(F("⚠️ Low heap - skipping Telegram, starting web server"));
                webServerStartPending = true;
            }
        }

        // ========== SAVE CONNECTED NETWORK CREDENTIALS ==========
        // Use local buffer to avoid dangling pointer from WiFi.SSID()
        char connectedSSID[33];
        strncpy(connectedSSID, WiFi.SSID().c_str(), sizeof(connectedSSID) - 1);
        connectedSSID[sizeof(connectedSSID) - 1] = '\0';
        
        bool needsSave = false;

        // Check if SSID changed
        if (strcmp(settings.staSsid, connectedSSID) != 0) {
            strncpy(settings.staSsid, connectedSSID, sizeof(settings.staSsid) - 1);
            settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';
            needsSave = true;
            Serial.printf("Connected to new network: %s\n", connectedSSID);
        }

        // Check if this is a hardcoded network and update password
        for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
            if (strcmp(hardcodedNetworks[i].ssid, connectedSSID) == 0) {
                // Check if password is different
                if (strcmp(settings.staPassword, hardcodedNetworks[i].password) != 0) {
                    strncpy(settings.staPassword, hardcodedNetworks[i].password,
                            sizeof(settings.staPassword) - 1);
                    settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
                    needsSave = true;
                    Serial.println(F(" Updated password from hardcoded networks"));
                }
                break;
            }
        }

        // Save settings if needed
        if (needsSave) {
            if (saveSettings()) {
                Serial.println(F("✅ Network credentials saved to flash"));
            } else {
                Serial.println(F("❌ Failed to save network credentials"));
            }
        }

        // ========== SYNC TIME FROM NTP ==========
        if (lastTimeSync == 0 || millis() - lastTimeSync > 300000UL) {
            syncTimeFromNTP();
            lastTimeSync = millis();
        }
    }

    // ========== PERIODIC UPDATES WHILE CONNECTED ==========
    if (WiFi.status() == WL_CONNECTED) {
        timeClient.update();
    }
}
void loop() {
  lastLoopMillis = millis();
  ESP.wdtFeed();
  if (isMidCycleWake && rtcData.remainingSleepSeconds > 0) {
    static unsigned long midCycleBootTime = millis();
    if (millis() - midCycleBootTime > 500) {  
      Serial.println(F(" Mid-cycle wake: Going back to sleep..."));
      uint32_t sleepNow = rtcData.remainingSleepSeconds;
      if (sleepNow > 3600) {
        rtcData.remainingSleepSeconds = sleepNow - 3600;
        sleepNow = 3600;
        saveRTC();
        Serial.printf("Sleeping %u sec, %u remaining\n", sleepNow, rtcData.remainingSleepSeconds);
      } else {
        rtcData.remainingSleepSeconds = 0;
        clearRTC();
        Serial.printf("Final chunk: %u sec\n", sleepNow);
      }
      WiFi.mode(WIFI_OFF);
      delay(100);
      ESP.deepSleep(sleepNow * 1000000UL);
    }
    return;  
  }
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 3000) {
    lastDebug = millis();
    Serial.println(F("===== LOOP STATUS ====="));
    Serial.printf("WiFi: %d | wifiConnected: %d\n", WiFi.status(), wifiConnected);
    Serial.printf("Telegram: pending=%d delay=%d sending=%d\n", 
                  telegramSendPending, telegramDelayActive, telegramSending);
    Serial.printf("WebServer: pending=%d running=%d\n", webServerStartPending, serverRunning);
    Serial.printf("DeepSleep: requested=%d\n", deepSleepRequested);
    Serial.printf("Heap: %u\n", ESP.getFreeHeap());
    Serial.println(F("======================="));
  }
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
  manageWiFiConnection();
  ESP.wdtFeed();
  if (pendingAPSetup) {
    pendingAPSetup = false;
    setupAP();
  }
  static unsigned long lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 5000) {
    lastHeapCheck = millis();
    Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
    
    if (ESP.getFreeHeap() < 4000) {
      Serial.println(F("⚠️ CRITICAL: Heap very low!"));
    }
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
  if (telegramSendPending && !telegramDelayActive && !telegramSending) {
    Serial.println(F(" Telegram: Starting 3-second delay..."));
    telegramDelayActive = true;
    telegramSendStartTime = millis();
  }
  if (telegramDelayActive && !telegramSending) {
    unsigned long elapsed = millis() - telegramSendStartTime;
    if (elapsed >= 3000) {
      Serial.println(F(" Telegram: Sending now..."));
      telegramDelayActive = false;
      telegramSending = true;
      if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        char ipMsg[100];
        snprintf(ipMsg, sizeof(ipMsg), " ESP8266 Online!\nIP: %d.%d.%d.%d\nMAC: %s", 
                 ip[0], ip[1], ip[2], ip[3], WiFi.macAddress().c_str());
        Serial.printf(" Message: %s\n", ipMsg);
        ESP.wdtFeed();
        bool success = bot.sendMessage(CHAT_ID, ipMsg, "");
        if (success) {
          Serial.println(F("✅ Telegram sent successfully!"));
        } else {
          Serial.println(F("❌ Telegram send failed!"));
        }
      } else {
        Serial.println(F("❌ WiFi disconnected, skipping Telegram"));
      }
      telegramSending = false;
      telegramSendPending = false;
      webServerStartPending = true;
    }
  }
  if (webServerStartPending && !serverRunning) {
  // Wait for Telegram to complete and heap to stabilize
  if (!telegramSendPending && !telegramDelayActive && !telegramSending) {
    uint32_t freeHeap = ESP.getFreeHeap();
    Serial.printf(" Ready to start web server. Free heap: %u\n", freeHeap);
    
    if (freeHeap > 12000) {
      Serial.println(F(" Starting web server..."));
      startWebServer();
      serverRunning = true;
      webServerStartPending = false;
      Serial.println(F("✅ Web server running!"));
    } else {
      Serial.println(F("⚠️ Waiting for more heap..."));
      // Try garbage collection
      ESP.wdtFeed();
      delay(100);
    }
  }
}
  if (deepSleepRequested) {
    if (telegramSendPending || telegramDelayActive || telegramSending) {
      return;
    }
    if (sleepRequestTime == 0) {
      sleepRequestTime = millis();
    }
    if (millis() - sleepRequestTime > 500) {
      Serial.println(F(" User-requested deep sleep starting..."));
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
        Serial.println(F("⚠️ Sleep duration is 0, aborting"));
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
        rtcData.remainingSleepSeconds = 0;
        clearRTC();
      }
      Serial.printf(" Sleeping for %u seconds...\n", sleepNow);
      ESP.deepSleep(sleepNow * 1000000UL);
    }
  }
  checkAPTimeout();
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
  }
  processScanResults();
  static bool scanDone = false;
  if (!scanDone && WiFi.status() == WL_CONNECTED && !scanInProgress) {
    Serial.println(F("Starting post-connection WiFi scan..."));
    startNonBlockingWiFiScan();
    scanDone = true;
  }
  if (WiFi.status() != WL_CONNECTED) {
    scanDone = false;
  }
  if (shouldSaveSettingsFlag) {
    shouldSaveSettingsFlag = false;
    saveSettings();
  }
  if (relayTimedOffPending) {
    relayTimedOffPending = false;
    if (relayState) {
      digitalWrite(settings.relayPin, HIGH);
      relayState = false;
      if (relayTicker.active()) relayTicker.detach();
      if (failsafeTicker.active()) failsafeTicker.detach();
      unsigned long endTime = millis();
      unsigned long actualDuration = (endTime - relayOnStartTime) / 1000UL;
      Serial.printf("Relay OFF after %lu seconds.\n", actualDuration);
      if (!isTemporaryRun) {
        logRelayRunTime();
        saveSettings();
      } else {
        isTemporaryRun = false;
      }
      broadcastStatus();
      if (settings.autoDeepSleepEnabled && !autoDeepSleepScheduled && 
          !failsafeActive && !emergencyStop) {
        const unsigned long DEEP_SLEEP_DELAY_MS = 10UL * 60UL * 1000UL;
        autoDeepSleepDelayTicker.once_ms(DEEP_SLEEP_DELAY_MS, initiateAutoDeepSleep);
        autoDeepSleepScheduled = true;
        Serial.printf("Auto deep sleep scheduled in %lu minutes\n", DEEP_SLEEP_DELAY_MS / 60000);
      }
    }
  }
}
void safeRestart() {
  Serial.println(F("Graceful restart initiated..."));
  if (serverRunning) {
    stopWebServer(); 
  } else {
    Serial.println(F("Web server was not running."));
  }
  failsafeTicker.detach();
  ESP.wdtFeed();
  watchdogTicker.detach();
  ESP.wdtFeed();
  relayTicker.detach();
  ESP.wdtFeed();
  logCleanupTicker.detach();
  ESP.wdtFeed();
  statusTicker.detach();
  ESP.wdtFeed();
  wifiScanTicker.detach();
  ESP.wdtFeed();
  autoDeepSleepDelayTicker.detach();
  ESP.wdtFeed();
  deferredServerDelete.detach();
  ESP.wdtFeed();
  dnsServer.stop();
  Serial.println(F("DNS server stopped."));
  WiFi.disconnect(true);
  Serial.println(F("WiFi STA disconnected."));
  WiFi.softAPdisconnect(true);
  Serial.println(F("SoftAP stopped."));
  WiFi.mode(WIFI_OFF);
  delay(100);
  Serial.println(F("WiFi interface turned off."));
  delay(500); 
  Serial.println(F("Restarting now..."));
  ESP.restart();
}
void setupWatchdog() {
  watchdogTicker.attach_ms(5000, feedWatchdog);
#ifdef DEBUG_MODE
  Serial.println(F("Hardware Watchdog initialized. Feeding every 5 seconds."));
#endif
}
void checkLoopWatchdog() {
  if (millis() - lastLoopMillis > 10000) { 
    Serial.println(F("Software watchdog triggered! Loop appears stuck. Rebooting..."));
    safeRestart();
  }
}
void failsafeTriggered() {
#ifdef DEBUG_MODE
  Serial.println(F("Failsafe Triggered: Max run time exceeded!"));
  Serial.printf_P(PSTR("DEBUG: Setting relay pin D%d to HIGH (OFF) from failsafeTriggered\n"), settings.relayPin);
#endif
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
#ifdef DEBUG_MODE
  Serial.println(F("Failsafe Reset."));
#endif
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
  Serial.println(F("Auto deep sleep initiated."));
  deepSleepRequested = true;
  sleepRequestTime = 0;
  autoDeepSleepScheduled = false;
  if (rtcData.remainingSleepSeconds == 0) {
    rtcData.remainingSleepSeconds = settings.deepSleepDurationSeconds;
    saveRTC();
    Serial.printf("Sleep initialized: %u seconds\n",
                  rtcData.remainingSleepSeconds);
  } else {
    Serial.printf("Sleep already in progress: %u seconds remaining\n",
                  rtcData.remainingSleepSeconds);
  }
}
void getMD5Hash_P(const __FlashStringHelper* flashStr, char* output, size_t outputSize) {
  if (outputSize < 33) return;
  constexpr size_t inputBufferSize = 64;
  char inputBuffer[inputBufferSize];
  strncpy_P(inputBuffer, (PGM_P)flashStr, inputBufferSize - 1);
  inputBuffer[inputBufferSize - 1] = '\0';
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
void generateSessionToken() {
  static const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  const size_t charsetLength = sizeof(charset) - 1; 
  for (uint8_t i = 0; i < 32; i++) {
    currentSessionToken[i] = charset[random(charsetLength)];
  }
  currentSessionToken[32] = '\0'; 
  sessionStart = millis(); 
#ifdef DEBUG_MODE
  Serial.print(F("New sessionToken: "));
  Serial.println(currentSessionToken);
#endif
}
bool authenticate(const char* user, const char* pass) {
  char hash[33];
  getMD5Hash(pass, hash, sizeof(hash));
#ifdef DEBUG_MODE
  Serial.printf_P(PSTR("Attempted login: User=%s, Pass (hashed)=%s\n"), user, hash);
  Serial.printf_P(PSTR("Stored credentials: User=%s, Pass (hashed)=%s\n"), settings.adminUsername, settings.adminPasswordHash);
#endif
  return (strcmp(user, settings.adminUsername) == 0 && strcmp(hash, settings.adminPasswordHash) == 0);
}
bool isLoggedIn(AsyncWebServerRequest *request) {
  if (!request->hasHeader("Cookie")) {
#ifdef DEBUG_MODE
    Serial.println(F("isLoggedIn: No Cookie header."));
#endif
    return false;
  }
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
  if (!tokenStart) {
#ifdef DEBUG_MODE
    Serial.println(F("isLoggedIn: No authToken cookie found."));
#endif
    return false;
  }
  tokenStart += strlen("authToken=");
  char receivedToken[33] = {0}; 
  size_t i = 0;
  while (*tokenStart && *tokenStart != ';' && i < sizeof(receivedToken) - 1) {
    receivedToken[i++] = *tokenStart++;
  }
  receivedToken[i] = '\0';
  if (strcmp(receivedToken, currentSessionToken) != 0) {
#ifdef DEBUG_MODE
    Serial.printf("isLoggedIn: Token mismatch. Received: %s, Expected: %s\n", receivedToken, currentSessionToken);
#endif
    return false;
  }
  if (millis() - sessionStart > sessionTimeout) {
    currentSessionToken[0] = '\0';
#ifdef DEBUG_MODE
    Serial.println(F("isLoggedIn: Session expired."));
#endif
    return false;
  }
  sessionStart = millis();
#ifdef DEBUG_MODE
  Serial.println(F("isLoggedIn: Session valid."));
#endif
  return true;
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
  currentSessionToken[0] = '\0';
  loggedIn = false; 
  settings.deepSleepDurationSeconds = 15 * 3600; 
  settings.autoDeepSleepEnabled = true; 
}
void setupLittleFS() {
  if (!LittleFS.begin()) {
#ifdef DEBUG_MODE
    Serial.println(F("âŒ LittleFS mount failed. Formatting..."));
#endif
    factoryResetPreserveIndex();
#ifdef DEBUG_MODE
    Serial.println(F("LittleFS formatted."));
#endif
  }
}
void initWiFi() {
  Serial.println(F("Initializing WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.hostname(settings.hostname);
  WiFi.disconnect();
  if (strlen(settings.staSsid) > 0) {
    if (settings.useDHCP) {
      WiFi.begin(settings.staSsid, settings.staPassword);
      Serial.printf("Attempting to connect to WiFi (DHCP): %s\n", settings.staSsid);
    } else {
      WiFi.config(settings.staticIp, settings.staticGateway, settings.staticSubnet, settings.staticDns1, settings.staticDns2);
      WiFi.begin(settings.staSsid, settings.staPassword);
      Serial.printf("Attempting to connect to WiFi (Static IP): %s\n", settings.staSsid);
    }
    lastWifiConnectAttemptMillis = millis(); 
    wifiConnected = false; 
  } else {
    Serial.println(F("No STA SSID configured. Starting in AP mode."));
    setupAP(); 
    isApModeActive = true; 
  }
}
void startWebServer() {
  if (serverRunning){
#ifdef DEBUG_MODE
    Serial.println(F("Web server handlers already attached. Skipping start."));
#endif
    return; 
  }
  // Add heap check
  if (ESP.getFreeHeap() < 12000) {
    Serial.println(F("⚠️ Insufficient heap for web server!"));
    Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
    return;
  }
  
  Serial.printf("Starting web server. Free heap: %u\n", ESP.getFreeHeap());
#ifdef DEBUG_MODE
  Serial.println(F("ElegantOTA initialized."));
#endif
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
    if (!isLoggedIn(request)) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }
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
server.onFileUpload(handleFileUpload);
server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(200, "text/html",
    "<form method='POST' action='/upload' enctype='multipart/form-data'>"
    "<input type='file' name='upload'>"
    "<input type='submit' value='Upload'>"
    "</form>");
});
server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){}, handleFileUpload);
server.on("/files", HTTP_GET, [](AsyncWebServerRequest *request) {
  Dir dir = LittleFS.openDir("/");
  String html = "<h3>Files in LittleFS:</h3><ul>";
  while (dir.next()) {
    html += "<li>" + dir.fileName() + " (" + String(dir.fileSize()) + " bytes)</li>";
  }
  html += "</ul>";
  request->send(200, "text/html", html);
});
  server.on("/format", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!isLoggedIn(request)) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }
    handleFormatFS(request);
  });
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/settings", HTTP_GET, handleGetSettings);
  server.addHandler(new AsyncCallbackJsonWebHandler("/saveSettings", [](AsyncWebServerRequest *request, JsonVariant &json) {
    if (!isLoggedIn(request)) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }
    StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
    doc.set(json); 
    Serial.println(F("ðŸ“¥ Received settings JSON:"));
    serializeJsonPretty(doc, Serial);
    Serial.println();
    const char* newHost = doc["hostname"] | "aquamaster";
    strncpy(settings.hostname, newHost, sizeof(settings.hostname) - 1);
    settings.hostname[sizeof(settings.hostname) - 1] = '\0';
    const char* staSsid = doc["staSsid"] | "";
    const char* staPass = doc["staPassword"] | "";
    strncpy(settings.staSsid, staSsid, sizeof(settings.staSsid) - 1);
    settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';
    strncpy(settings.staPassword, staPass, sizeof(settings.staPassword) - 1);
    settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
    settings.useDHCP = doc["useDHCP"] | true;
    IPAddress ip;
    if (ip.fromString(doc["staticIp"] | "192.168.1.100")) settings.staticIp = ip;
    if (ip.fromString(doc["staticGateway"] | "192.168.1.1")) settings.staticGateway = ip;
    if (ip.fromString(doc["staticDns1"] | "8.8.8.8")) settings.staticDns1 = ip;
    if (ip.fromString(doc["staticDns2"] | "8.8.4.4")) settings.staticDns2 = ip;
    settings.staticSubnet = IPAddress(255, 255, 255, 0);
    settings.wifiAPModeEnabled = doc["wifiAPModeEnabled"] | false;
    const char* apSsid = doc["apSsid"] | "AquaMaster_AP";
    const char* apPass = doc["apPassword"] | "aquapass";
    strncpy(settings.apSsid, apSsid, sizeof(settings.apSsid) - 1);
    settings.apSsid[sizeof(settings.apSsid) - 1] = '\0';
    strncpy(settings.apPassword, apPass, sizeof(settings.apPassword) - 1);
    settings.apPassword[sizeof(settings.apPassword) - 1] = '\0';
    settings.calibrationFactor = doc["calibrationFactor"] | 1.0;
    settings.failsafeDurationHours = doc["failsafeDurationHours"] | 12;
    settings.failsafeEnabled = doc["failsafeEnabled"] | true;
    settings.failsafePin = doc["failsafePin"] | D5;
    settings.relayPin = doc["relayPin"] | D1;
    if (doc.containsKey("defaultRelayDuration")) {
      if (doc["defaultRelayDuration"].is<int>()) {
        settings.defaultRelayDuration = doc["defaultRelayDuration"];
      } else if (doc["defaultRelayDuration"].is<const char*>()) {
        settings.defaultRelayDuration = atoi(doc["defaultRelayDuration"]);
      } else {
        Serial.println(F("âš ï¸ Invalid type for defaultRelayDuration"));
      }
      Serial.printf("âœ” defaultRelayDuration = %d\n", settings.defaultRelayDuration);
    }
    if (doc.containsKey("maxRelayRuntime")) {
      if (doc["maxRelayRuntime"].is<int>()) {
        settings.maxRelayRuntime = doc["maxRelayRuntime"];
      } else if (doc["maxRelayRuntime"].is<const char*>()) {
        settings.maxRelayRuntime = atoi(doc["maxRelayRuntime"]);
      } else {
        Serial.println(F("âš ï¸ Invalid type for maxRelayRuntime"));
      }
      Serial.printf("âœ” maxRelayRuntime = %d\n", settings.maxRelayRuntime);
    }
    if (doc.containsKey("deepSleepDurationSeconds")) {
      if (doc["deepSleepDurationSeconds"].is<int>()) {
        settings.deepSleepDurationSeconds = doc["deepSleepDurationSeconds"];
      } else if (doc["deepSleepDurationSeconds"].is<const char*>()) {
        settings.deepSleepDurationSeconds = atoi(doc["deepSleepDurationSeconds"]);
      } else {
        Serial.println(F("âš ï¸ Invalid type for deepSleepDurationSeconds"));
      }
      Serial.printf("âœ” deepSleepDurationSeconds = %d\n", settings.deepSleepDurationSeconds);
    }
    settings.autoDeepSleepEnabled = doc["autoDeepSleepEnabled"] | true;
    Serial.println(F("ðŸ’¾ Attempting to save settings..."));
    if (saveSettings()) {
      Serial.println(F("âœ… Settings saved successfully."));
      request->send(200, "text/plain", "Settings saved successfully.");
      broadcastStatus();
      pendingRestart = true;
    } else {
      Serial.println(F("âŒ Failed to save settings to flash."));
      request->send(500, "text/plain", "Failed to save settings.");
    }
  }));
  server.on("/clearFailsafe", HTTP_POST, handleClearFailsafe);
  server.on("/fs-check", HTTP_GET, handleFsCheck);
  server.on("/autoDeepSleepToggle", HTTP_POST, handleAutoDeepSleepToggle);
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
#ifdef DEBUG_MODE
    Serial.println(F("SSE Client Connected!"));
#endif
  });
  server.addHandler(&events);
server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest *request) {
  request->send(204);
});
server.on("/gen_204", HTTP_ANY, [](AsyncWebServerRequest *request) {
  request->send(204);
});
server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
server.on("/library/test/success.html", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(LittleFS, "/index.html.gz", "text/html");
});
server.onNotFound([](AsyncWebServerRequest *request) {
  if (!WiFi.isConnected()) {
    AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/index.html.gz", "text/html");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  } else {
    request->send(404, "text/plain", "Not found");
  }
});
  server.begin();
  serverRunning = true;
#ifdef DEBUG_MODE
  Serial.println(F("Web server started."));
#endif
}
void stopWebServer() {
  if (!serverRunning) {
#ifdef DEBUG_MODE
    Serial.println(F("Web server not running. Skipping stop."));
#endif
    return;
  }
#ifdef DEBUG_MODE
  Serial.println(F("Stopping web server..."));
#endif
  events.close();                
  events.onConnect(nullptr);    
#ifdef DEBUG_MODE
  Serial.println(F("SSE connections closed."));
#endif
#ifdef DEBUG_MODE
  Serial.println(F("ElegantOTA ended."));
#endif
  server.end();  
#ifdef DEBUG_MODE
  Serial.println(F("AsyncWebServer end() called."));
#endif
  serverRunning = false;
#ifdef DEBUG_MODE
  Serial.println(F("Web server shutdown sequence completed."));
#endif
}
 void handleRelayLogs(AsyncWebServerRequest *request) {
  Serial.println(F("ðŸ“¥ Received /logs request"));
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->print("[");
  FSLockGuard lock;
  if (!lock.isLocked()) {
    Serial.println(F("âŒ ERROR: Failed to acquire FS lock"));
    response->print("]");
    request->send(response);
    return;
  }
  if (!LittleFS.exists(RELAY_LOG_FILE)) {
    Serial.println(F("âš ï¸ Log file does not exist"));
    response->print("]");
    request->send(response);
    return;
  }
  File logFile = LittleFS.open(RELAY_LOG_FILE, "r");
  if (!logFile) {
    Serial.println(F("âŒ ERROR: Failed to open log file"));
    response->print("]");
    request->send(response);
    return;
  }
  Serial.println(F("ðŸ“– Reading logs from file..."));
  bool first = true;
  char buffer[LOG_ENTRY_LENGTH + 1] = {0};
  int validLogs = 0;
  for (uint8_t i = 0; i < MAX_LOGS; i++) {
    uint8_t idx = (relayLogIndex + i) % MAX_LOGS;
    size_t seekPos = idx * LOG_ENTRY_LENGTH;
    logFile.seek(seekPos, SeekSet);
    memset(buffer, 0, sizeof(buffer));
    size_t len = logFile.readBytes(buffer, LOG_ENTRY_LENGTH);
    buffer[len] = '\0';
    if (len == 0 || buffer[0] == '\0') {
      Serial.printf("ðŸ“› Empty log at index %d (seekPos=%u)\n", i, seekPos);
      continue;
    }
    char* newline = strchr(buffer, '\n');
    if (newline) *newline = '\0';
    char* endSpace = strchr(buffer, '\r');
    if (endSpace) *endSpace = '\0';
    Serial.printf("ðŸ” Raw entry at index %d: '%s'\n", i, buffer);
    char* comma = strchr(buffer, ',');
    if (!comma) {
      Serial.printf("âš ï¸ Skipping invalid entry at index %d: '%s'\n", i, buffer);
      continue;
    }
    *comma = '\0';
    const char* epochStr = buffer;
    const char* durationStr = comma + 1;
    uint32_t epoch = strtoul(epochStr, nullptr, 10);
    uint32_t duration = strtoul(durationStr, nullptr, 10);
    if (epoch < 1600000000UL || duration == 0 || duration > 86400UL) {
      Serial.printf("â­ï¸ Skipping out-of-range log at index %d: epoch=%lu, duration=%lu\n", i, epoch, duration);
      continue;
    }
    if (!first) response->print(",");
    response->printf("{\"epoch\":%lu,\"duration\":%lu}", epoch, duration);
    first = false;
    validLogs++;
    Serial.printf("âœ… Log %d => epoch: %lu, duration: %lu\n", i, epoch, duration);
  }
  logFile.close();
  response->print("]");
  request->send(response);
  Serial.printf("ðŸ“¤ Sent %d valid log(s)\n", validLogs);
}
void handleFailsafeLogs(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->print("[");  
  FSLockGuard lock;
  if (!lock.isLocked()) return;
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
    char *epochStr  = strtok(buffer, ",");  
    char *category  = strtok(NULL, ",");    
    char *reasonStr = strtok(NULL, ",");    
    if (reasonStr) {
      reasonStr[strcspn(reasonStr, "\r\n")] = '\0';  
    }
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
  LittleFS.remove("/relay.bin");
  request->send(200, "text/plain", "Relay log cleared.");
}
void handleClearFailsafeLogs(AsyncWebServerRequest *request) {
 FSLockGuard lock;
if (!lock.isLocked()) return;
  LittleFS.remove(FAILSAFE_LOG_FILE);
  failsafeLogIndex = 0;
  LittleFS.remove("/failsafe.bin");
  request->send(200, "text/plain", "Failsafe log cleared.");
}
void setupAP() {
  if (isApModeActive) return; 
  Serial.println(F("Stopping existing services before starting AP mode..."));
if (serverRunning) {
stopWebServer();    
    Serial.println("Web server stopped.");
}
  WiFi.disconnect(true); 
  delay(100); 
  Serial.println(F("Starting AP mode configuration..."));
  settings.apSsid[sizeof(settings.apSsid) - 1] = '\0';
  settings.apPassword[sizeof(settings.apPassword) - 1] = '\0';
  if (strlen(settings.apSsid) == 0 || strlen(settings.apSsid) > 31) {
    Serial.println(F("Invalid SSID. Using fallback."));
    strncpy(settings.apSsid, "AquaMaster_AP", sizeof(settings.apSsid) - 1);
  }
  if (strlen(settings.apPassword) < 8 || strlen(settings.apPassword) > 63) {
    Serial.println(F("Invalid password. Using fallback."));
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
    Serial.println("âœ… DNS server started.");
} else {
    Serial.println("âŒ DNS server failed to start!");
}
  isApModeActive = true;
  apStartTime = millis(); 
  startWebServer();
}
void checkAPTimeout() {
  if (!isApModeActive) return;
  const unsigned long AP_NO_CLIENT_TIMEOUT_MS = 5UL * 60UL * 1000UL;  
  const unsigned long AP_CLIENT_TIMEOUT_MS    = 14UL * 60UL * 1000UL; 
  int stationNum = WiFi.softAPgetStationNum();
  unsigned long currentTimeout = (stationNum > 0) ? AP_CLIENT_TIMEOUT_MS : AP_NO_CLIENT_TIMEOUT_MS;
  if (millis() - apStartTime >= currentTimeout) {
    if (relayState) {
      Serial.println(F("â³ AP timeout reached but relay is ON â€” delaying AP shutdown."));
      return; 
    }
    Serial.println(F("AP mode timeout. Restarting to exit AP mode..."));
    WiFi.disconnect(true);
    delay(100);
    pendingRestart = true;
  }
}
void stopAP() {
  if (dnsServerActive) {
    dnsServer.stop();
    dnsServerActive = false;
    Serial.println(F("DNS server stopped."));
  }
  if (isApModeActive) {
    WiFi.softAPdisconnect(true);
    isApModeActive = false;
    Serial.println(F("Soft AP stopped."));
  }
  stopWebServer(); 
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
          Serial.println(F("Switch HIGH → Relay OFF"));
          toggleRelayInternal();
        } 
        else if (currentReading == LOW && !relayState) {
          Serial.println(F("Switch LOW → Relay ON"));
          toggleRelayInternal();
        } 
        else {
          Serial.println(F("Switch stable but relay already correct"));
        }
        lastSwitchActionTime = millis();
      } else {
        Serial.println(F("Switch stable but cooldown active"));
      }
    }
  }
  lastSwitchReading = currentReading; 
}
const char* getContentType(const char* filename) {
  if (strstr(filename, ".html")) return "text/html";
  if (strstr(filename, ".css")) return "text/css";
  if (strstr(filename, ".js")) return "application/javascript";
  if (strstr(filename, ".json")) return "application/json";
  if (strstr(filename, ".png")) return "image/png";
  if (strstr(filename, ".gif")) return "image/gif";
  if (strstr(filename, ".jpg")) return "image/jpeg";
  if (strstr(filename, ".ico")) return "image/x-icon";
  if (strstr(filename, ".xml")) return "text/xml";
  if (strstr(filename, ".pdf")) return "application/pdf";
  return "text/plain";
}
bool loadSettings() {
   FSLockGuard lock;
if (!lock.isLocked()) return false;
  File configFile = LittleFS.open(SETTINGS_FILE, "r");
  if (!configFile) {
#ifdef DEBUG_MODE
    Serial.println(F("âŒ Failed to open settings file. Trying backup..."));
#endif
    configFile = LittleFS.open(SETTINGS_BACKUP_FILE, "r");
    if (!configFile) {
#ifdef DEBUG_MODE
      Serial.println(F("âŒ Failed to open backup settings file."));
#endif
      return false;
    }
  }
  StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();
  if (error) {
#ifdef DEBUG_MODE
    Serial.printf_P(PSTR("âŒ Failed to parse settings file: %s\n"), error.c_str());
#endif
    return false;
  }
  strncpy(settings.staSsid, doc["staSsid"] | "", sizeof(settings.staSsid) - 1);
  settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';
  strncpy(settings.staPassword, doc["staPassword"] | "", sizeof(settings.staPassword) - 1);
  settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
  strncpy(settings.hostname, doc["hostname"] | "aquamaster", sizeof(settings.hostname) - 1);
  settings.hostname[sizeof(settings.hostname) - 1] = '\0';
  settings.useDHCP = doc["useDHCP"] | true;
  settings.staticIp.fromString(doc["staticIp"] | "192.168.1.100");
  settings.staticGateway.fromString(doc["staticGateway"] | "192.168.1.1");
  settings.staticSubnet.fromString("255.255.255.0");
  settings.staticDns1.fromString(doc["staticDns1"] | "8.8.8.8");
  settings.staticDns2.fromString(doc["staticDns2"] | "8.8.4.4");
  settings.wifiAPModeEnabled = doc["wifiAPModeEnabled"] | false;
  strncpy(settings.apSsid, doc["apSsid"] | "AquaMaster_AP", sizeof(settings.apSsid) - 1);
  settings.apSsid[sizeof(settings.apSsid) - 1] = '\0';
  strncpy(settings.apPassword, doc["apPassword"] | "aquapass", sizeof(settings.apPassword) - 1);
  settings.apPassword[sizeof(settings.apPassword) - 1] = '\0';
  strncpy(settings.adminUsername, doc["adminUsername"] | "admin", sizeof(settings.adminUsername) - 1);
  settings.adminUsername[sizeof(settings.adminUsername) - 1] = '\0';
  strncpy(settings.adminPasswordHash, doc["adminPasswordHash"] | "", sizeof(settings.adminPasswordHash) - 1);
  settings.adminPasswordHash[sizeof(settings.adminPasswordHash) - 1] = '\0';
  if (strlen(settings.adminPasswordHash) == 0) {
    char hash[33];
    getMD5Hash_P(F("admin"), hash, sizeof(hash));
    strncpy(settings.adminPasswordHash, hash, sizeof(settings.adminPasswordHash));
    settings.adminPasswordHash[sizeof(settings.adminPasswordHash) - 1] = '\0';
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
#ifdef DEBUG_MODE
  Serial.println(F("âœ… Settings loaded successfully."));
#endif
  return true;
}
bool saveSettings() {
   FSLockGuard lock;
if (!lock.isLocked()) return false;
  bool success = false;  
  File configFile = LittleFS.open("/settings.tmp", "w");
  if (!configFile) {
#ifdef DEBUG_MODE
    Serial.println(F("âŒ Failed to open temp file for writing."));
#endif
    return false;
  }
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
  ESP.wdtFeed();
  if (serializeJson(doc, configFile) == 0) {
#ifdef DEBUG_MODE
    Serial.println(F("âŒ Failed to serialize settings JSON."));
#endif
    configFile.close();
    return false;
  }
  configFile.close();
  if (LittleFS.exists(SETTINGS_BACKUP_FILE)) {
    LittleFS.remove(SETTINGS_BACKUP_FILE);
  }
  if (LittleFS.exists(SETTINGS_FILE)) {
    LittleFS.rename(SETTINGS_FILE, SETTINGS_BACKUP_FILE);
  }
  success = LittleFS.rename("/settings.tmp", SETTINGS_FILE);
  ESP.wdtFeed();
#ifdef DEBUG_MODE
  if (success) {
    Serial.println(F("âœ… Settings saved successfully with backup rotation."));
  } else {
    Serial.println(F("âŒ Failed to rename /settings.tmp to settings file."));
  }
#endif
  return success;
}
bool loadBootRecovery() {
   FSLockGuard lock;
if (!lock.isLocked()) return false;
  bool success = false;
  File file = LittleFS.open(BOOT_RECOVERY_FILE, "r");
  if (!file) {
#ifdef DEBUG_MODE
    Serial.println(F("Boot recovery file missing. Creating new."));
#endif
    bootRecovery.lastBootMillis = 0;
    bootRecovery.bootCount = 0;
    saveBootRecovery();  
    success = true;      
  } else {
    StaticJsonDocument<128> doc; 
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
#ifdef DEBUG_MODE
      Serial.printf_P(PSTR("Failed to parse boot recovery file: %s\n"), error.c_str());
#endif
      success = false;
    } else {
      bootRecovery.lastBootMillis = doc["lastBootMillis"] | 0;
      bootRecovery.bootCount = doc["bootCount"] | 0;
#ifdef DEBUG_MODE
      Serial.println(F("Boot recovery data loaded."));
#endif
      success = true;
    }
  }
  return success;
}
void saveBootRecovery() {
 FSLockGuard lock;
if (!lock.isLocked()) return;
  File file = LittleFS.open(BOOT_RECOVERY_FILE, "w");
  if (!file) {
#ifdef DEBUG_MODE
    Serial.println(F("Failed to open boot recovery file for writing."));
#endif
    return;
  }
  StaticJsonDocument<128> doc;
  doc["lastBootMillis"] = bootRecovery.lastBootMillis;
  doc["bootCount"] = bootRecovery.bootCount;
  if (serializeJson(doc, file) == 0) {
#ifdef DEBUG_MODE
    Serial.println(F("Failed to write to boot recovery file."));
#endif
  }
  file.close();
#ifdef DEBUG_MODE
  Serial.println(F("Boot recovery data saved."));
#endif
}
void toggleRelayInternal() {
  if (failsafeActive || emergencyStop) {
    Serial.println(F("Cannot toggle relay: Failsafe or Emergency Stop is active."));
    return;
  }
  if (relayState) { 
    digitalWrite(settings.relayPin, HIGH); 
    relayState = false;
    Serial.println(F("Relay OFF."));
    if (relayTicker.active()) {
      relayTicker.detach();
     unsigned long endTime = millis();
      settings.wasTimedRunInterrupted = false;
      settings.interruptedRunDuration = 0;
      settings.interruptedRunStartTime = 0;
      saveSettings();
    }
    autoDeepSleepDelayTicker.detach();
    autoDeepSleepScheduled = false;
  } else { 
    relayOnEpochTime = time(nullptr);
    digitalWrite(settings.relayPin, LOW); 
    relayState = true;
    relayOnStartTime = millis();
    Serial.println(F("Relay ON."));
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
  Serial.println(F("â€¼ï¸ EMERGENCY STOP ACTIVATED!"));
  emergencyStop = true;
  failsafeActive = true;
  digitalWrite(settings.relayPin, HIGH);
  relayState = false;
  if (relayTicker.active()) {
    relayTicker.detach();
    settings.wasTimedRunInterrupted = true;
    settings.interruptedRunDuration = (millis() - relayOnStartTime) / 1000;
    settings.interruptedRunStartTime = relayOnStartTime;
    ESP.wdtFeed();
    saveSettings();  
    ESP.wdtFeed();
  }
if (failsafeTicker.active()) {
 failsafeTicker.detach();
}
  autoDeepSleepDelayTicker.detach();
  autoDeepSleepScheduled = false;
  ESP.wdtFeed();
strncpy(pendingFailsafeReason, "Emergency Stop", sizeof(pendingFailsafeReason) - 1);
  broadcastStatus();  
  ESP.wdtFeed();
}
void turnOffRelayTimed() {
  if (relayState) {
    relayTimedOffPending = true;
  }
}
void clearEmergencyStopAndFailsafe() {
  if (emergencyStop || failsafeActive) {
    Serial.println(F("EMERGENCY STOP/FAILSAFE CLEARED. Resuming normal operation."));
    emergencyStop = false;
    failsafeActive = false;
    shouldBroadcastAfterEmergencyClear = true;
    shouldLogEmergencyClear = true;
  } else {
    Serial.println(F("Emergency Stop/Failsafe not active. No action needed."));
  }
}
void sendIPToTelegram() {
  String ipMsg = "ESP8266 Connected!\nIP Address: " + WiFi.localIP().toString();
  if (bot.sendMessage(CHAT_ID, ipMsg, "")) {
    Serial.println(F("âœ… Telegram message sent successfully"));
  } else {
    Serial.println(F("âŒ Failed to send Telegram message"));
  }
}
void handleLogin(AsyncWebServerRequest *request) {
  if (failedLoginAttempts >= MAX_FAILED_LOGIN_ATTEMPTS && (millis() - lastLoginAttemptMillis < LOGIN_COOLDOWN_TIME)) {
    request->send(429, "text/plain", "Too many failed login attempts. Please wait.");
    return;
  }
  if (request->hasArg("username") && request->hasArg("password")) {
    char usernameBuffer[32];
    char passwordBuffer[64];
    strncpy(usernameBuffer, request->arg("username").c_str(), sizeof(usernameBuffer) - 1);
    usernameBuffer[sizeof(usernameBuffer) - 1] = '\0';
    strncpy(passwordBuffer, request->arg("password").c_str(), sizeof(passwordBuffer) - 1);
    passwordBuffer[sizeof(passwordBuffer) - 1] = '\0';
    if (authenticate(usernameBuffer, passwordBuffer)) {
      generateSessionToken();
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Login successful");
      response->addHeader("Set-Cookie", (String("authToken=") + currentSessionToken + "; Path=/; Max-Age=" + (sessionTimeout / 1000)));
      request->send(response);
      loggedIn = true;
      failedLoginAttempts = 0;
      lastLoginAttemptMillis = 0;
      broadcastStatus();
    } else {
      failedLoginAttempts++;
      lastLoginAttemptMillis = millis();
      request->send(401, "text/plain", "Unauthorized");
    }
  } else {
    request->send(400, "text/plain", "Missing username or password");
  }
}
void handleUpdateLogin(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!request->hasArg("newUsername") || !request->hasArg("newPassword") || !request->hasArg("oldPassword")) {
    request->send(400, "text/plain", "Missing newUsername, newPassword, or oldPassword");
    return;
  }
  char oldPasswordBuffer[64];
  char newUsernameBuffer[32];
  char newPasswordBuffer[64];
  strncpy(oldPasswordBuffer, request->arg("oldPassword").c_str(), sizeof(oldPasswordBuffer) - 1);
  oldPasswordBuffer[sizeof(oldPasswordBuffer) - 1] = '\0';
  strncpy(newUsernameBuffer, request->arg("newUsername").c_str(), sizeof(newUsernameBuffer) - 1);
  newUsernameBuffer[sizeof(newUsernameBuffer) - 1] = '\0';
  strncpy(newPasswordBuffer, request->arg("newPassword").c_str(), sizeof(newPasswordBuffer) - 1);
  newPasswordBuffer[sizeof(newPasswordBuffer) - 1] = '\0';
  if (authenticate(settings.adminUsername, oldPasswordBuffer)) {
    strncpy(settings.adminUsername, newUsernameBuffer, sizeof(settings.adminUsername) - 1);
    settings.adminUsername[sizeof(settings.adminUsername) - 1] = '\0';
    char newPasswordHash[33];
    getMD5Hash(newPasswordBuffer, newPasswordHash, sizeof(newPasswordHash));
    strncpy(settings.adminPasswordHash, newPasswordHash, sizeof(settings.adminPasswordHash));
    settings.adminPasswordHash[sizeof(settings.adminPasswordHash) - 1] = '\0';
    if (saveSettings()) {
      request->send(200, "text/plain", "Login credentials updated successfully.");
      currentSessionToken[0] = '\0'; 
      loggedIn = false;
      broadcastStatus();
    } else {
      request->send(500, "text/plain", "Failed to save new credentials.");
    }
  } else {
    request->send(401, "text/plain", "Unauthorized: Old password incorrect.");
  }
}
void handleClearFailsafe(AsyncWebServerRequest *request) {
  clearEmergencyStopAndFailsafe();
  resetFailsafe ();
  request->send(200, "text/plain", "Failsafe cleared.");
}
void circularLogWrite(const char* filename, const char* logMsg, uint8_t& logIndex) {
  FSLockGuard lock;
  if (!lock.isLocked()) {
    Serial.println(F("ERROR: Failed to acquire FSLockGuard!"));
    return;
  }
  ESP.wdtFeed();
  if (!filename || !logMsg || strlen(logMsg) == 0) {
    Serial.println(F("WARN: Invalid input to circularLogWrite(): filename or logMsg empty."));
    return;
  }
  if (logIndex >= MAX_LOGS) {
    Serial.printf("WARN: logIndex out of bounds (%u >= %u). Resetting.\n", logIndex, MAX_LOGS);
    logIndex = 0;
  }
  File logFile = LittleFS.open(filename, "r+");
  if (!logFile) {
    Serial.println(F("INFO: Log file not found. Creating new one..."));
    logFile = LittleFS.open(filename, "w+");
    if (!logFile) {
      Serial.println(F("ERROR: Cannot create log file."));
      return;
    }
    char blank[LOG_ENTRY_LENGTH] = {0};
    for (uint8_t i = 0; i < MAX_LOGS; i++) {
      if (logFile.write((const uint8_t*)blank, LOG_ENTRY_LENGTH) != LOG_ENTRY_LENGTH) {
        Serial.printf("ERROR: Failed to initialize blank log at %u\n", i);
        logFile.close();
        return;
      }
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
  padded[len + 1] = '\0';
  size_t offset = static_cast<size_t>(logIndex) * LOG_ENTRY_LENGTH;
  if (!logFile.seek(offset, SeekSet)) {
    Serial.printf("ERROR: Seek failed at offset %u\n", offset);
    logFile.close();
    return;
  }
  yield();
  size_t written = logFile.write((const uint8_t*)padded, LOG_ENTRY_LENGTH);
  if (written != LOG_ENTRY_LENGTH) {
    Serial.printf("ERROR: Write failed at index %u (%u bytes written)\n", logIndex, written);
    logFile.close();
    return;
  }
  logFile.flush();
  logFile.close();
  Serial.printf("[DEBUG] Log written at index %u (offset %u): '%s'\n", logIndex, offset, padded);
  yield();  
  logIndex = (logIndex + 1) % MAX_LOGS;
  ESP.wdtFeed();
  yield();
}
void handleEmergencyStop(AsyncWebServerRequest *request) {
  request->send(200, "application/json", "{\"status\":\"emergency_triggered\"}");
  emergencyStopDelay.once_ms(50, emergencyStopInternal);
}
void handleScanNetworks(AsyncWebServerRequest *request) {
  if (scanInProgress) {
    request->send(409, "text/plain", "WiFi scan already in progress.");
    return;
  }
  
  // Add heap check
  if (ESP.getFreeHeap() < 10000) {
    request->send(503, "text/plain", "Insufficient memory for scan.");
    return;
  }
  
  WiFi.mode(WIFI_STA); 
  Serial.println(F("Starting WiFi scan..."));
  scanInProgress = true;
  WiFi.scanNetworks(true, true); 
  request->send(202, "text/plain", "WiFi scan started.");
  wifiScanTicker.attach_ms(1000, broadcastScanResults); 
}
void broadcastScanResults() {
  if (events.count() == 0) return;
  
  int n = WiFi.scanComplete();
  if (n >= 0) {
    wifiScanTicker.detach();
    scanInProgress = false;
    
    // Use smaller buffer and limit networks
    const int maxNetworks = min(n, 10);  // Limit to 10 networks
    StaticJsonDocument<1024> doc;  // Reduced from 4096
    JsonArray networks = doc.to<JsonArray>();
    
    for (int i = 0; i < maxNetworks; ++i) {
      JsonObject network = networks.add<JsonObject>();
      network["ssid"] = WiFi.SSID(i);
      network["rssi"] = WiFi.RSSI(i);
      network["encryption"] = (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted";
    }
    
    WiFi.scanDelete();
    
    // Use smaller buffer
    char responseBuffer[1024];
    size_t jsonSize = serializeJson(doc, responseBuffer, sizeof(responseBuffer));
    if (jsonSize > 0) {
      events.send(responseBuffer, "scanResults");
    }
  }
}
void handleKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  
  // Use smaller static buffer
  static char responseBuffer[512];
  int offset = 0;
  offset += snprintf(responseBuffer + offset, sizeof(responseBuffer) - offset, "[");
  
  bool first = true;
  for (const auto& kn : knownNetworks) {
    if (!first) {
      offset += snprintf(responseBuffer + offset, sizeof(responseBuffer) - offset, ",");
    }
    offset += snprintf(responseBuffer + offset, sizeof(responseBuffer) - offset,
      "{\"ssid\":\"%s\"}", kn.ssid);
    first = false;
    
    if (offset >= (int)sizeof(responseBuffer) - 50) break; // Safety margin
  }
  offset += snprintf(responseBuffer + offset, sizeof(responseBuffer) - offset, "]");
  
  request->send(200, "application/json", responseBuffer);
}
void handleSaveKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!request->hasArg("plain")) {
    request->send(400, "text/plain", "Missing JSON body.");
    return;
  }
  
  // Use smaller document - limit networks
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, request->arg("plain"));
  if (error) {
    request->send(400, "text/plain", "Failed to parse JSON");
    return;
  }
  
  JsonArray networksArray = doc.as<JsonArray>();
  if (networksArray.isNull()) {
    request->send(400, "text/plain", "JSON is not an array.");
    return;
  }
  
  knownNetworks.clear();
  int count = 0;
  const int MAX_KNOWN_NETWORKS = 5;  // Limit stored networks
  
  for (JsonObject network : networksArray) {
    if (count >= MAX_KNOWN_NETWORKS) break;
    
    KnownNetwork kn;
    strncpy(kn.ssid, network["ssid"] | "", sizeof(kn.ssid) - 1);
    kn.ssid[sizeof(kn.ssid) - 1] = '\0';
    strncpy(kn.password, network["password"] | "", sizeof(kn.password) - 1);
    kn.password[sizeof(kn.password) - 1] = '\0';
    knownNetworks.push_back(kn);
    count++;
  }
  
  request->send(200, "text/plain", "Known networks updated.");
}
void handleClearKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  knownNetworks.clear();
  request->send(200, "text/plain", "Known networks cleared.");
}
void handleAPModeToggle(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (request->hasArg("enable")) {
    const char* argVal = request->arg("enable").c_str();
    bool enable = strcasecmp(argVal, "true") == 0;
    settings.wifiAPModeEnabled = enable;
    if (saveSettings()) {
      if (enable) {
        pendingAPSetup = true;
        request->send(200, "text/plain", "AP mode enabled. Switching modes...");
      } else {
        if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
          Serial.println(F("Disabling AP mode. Restarting in STA mode."));
          request->send(200, "text/plain", "AP mode disabled. Restarting...");
          delay(100);
          pendingRestart = true; 
        } else {
          request->send(200, "text/plain", "AP mode disabled.");
        }
      }
    } else {
      request->send(500, "text/plain", "Failed to save AP mode setting.");
    }
  } else {
    request->send(400, "text/plain", "Missing 'enable' parameter (true/false).");
  }
}
void handleRestart(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", "Restart scheduled.");
  pendingRestart = true;  
}
void handleRestoreFactorySettings(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  request->send(200, "text/plain", "Restoring factory settings...");
  factoryResetPreserveIndex(); 
}
void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not Found");
}
void handleUpdateWifi(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!request->hasArg("ssid") || !request->hasArg("password")) {
    request->send(400, "text/plain", "Missing SSID or Password.");
    return;
  }
  char newSsid[32];
  char newPassword[64];
  strncpy(newSsid, request->arg("ssid").c_str(), sizeof(newSsid) - 1);
  newSsid[sizeof(newSsid) - 1] = '\0';
  strncpy(newPassword, request->arg("password").c_str(), sizeof(newPassword) - 1);
  newPassword[sizeof(newPassword) - 1] = '\0';
  strncpy(settings.staSsid, newSsid, sizeof(settings.staSsid));
  strncpy(settings.staPassword, newPassword, sizeof(settings.staPassword));
  if (saveSettings()) {
    request->send(200, "text/plain", "WiFi credentials updated. Restarting to connect.");
    pendingRestart = true;
  } else {
    request->send(500, "text/plain", "Failed to save WiFi credentials.");
  }
}
void handleDisconnectWifi(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  WiFi.disconnect(true); 
  strncpy(settings.staSsid, "", sizeof(settings.staSsid));
  strncpy(settings.staPassword, "", sizeof(settings.staPassword));
  saveSettings();
  request->send(200, "text/plain", "Disconnected from WiFi and credentials cleared.");
  broadcastStatus();
}
void handleSetDeepSleepDuration(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (request->hasArg("duration")) {
    unsigned long duration = strtoul(request->arg("duration").c_str(), NULL, 10);
    settings.deepSleepDurationSeconds = duration;
    if (saveSettings()) {
      request->send(200, "text/plain", "Deep sleep duration updated.");
      broadcastStatus();
    } else {
      request->send(500, "text/plain", "Failed to save deep sleep duration.");
    }
  } else {
    request->send(400, "text/plain", "Missing 'duration' parameter.");
  }
}
void handleDeepSleep(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (settings.deepSleepDurationSeconds == 0) {
    request->send(400, "text/plain",
                  "Deep sleep duration is 0. Please set a duration first.");
    return;
  }
  initiateAutoDeepSleep();   
  request->send(200, "text/plain", "Initiating deep sleep.");
}
void handleFormatFS(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  request->send(200, "text/plain", "Formatting file system. ESP will restart.");
  factoryResetPreserveIndex();
  pendingRestart = true;
}
void handleFileUpload(AsyncWebServerRequest *request,
                      String filename,
                      size_t index,
                      uint8_t *data,
                      size_t len,
                      bool final) {
  String filename_str = filename; 
  FSLockGuard lock;
  if (!lock.isLocked()) return;
  const char *targetFile = "/index.html.gz";
  if (index == 0) {
    Serial.println("\n=== File Upload Start ===");
    Serial.printf("Original filename: %s\n", filename_str.c_str());
    Serial.printf("Target file: %s\n", targetFile);
    if (LittleFS.exists(targetFile)) {
      Serial.println("Removing existing /index.html.gz...");
      LittleFS.remove(targetFile);
      delay(50);
    }
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
  const char* wifiStatusStr = "Unknown";
  switch (WiFi.status()) {
    case WL_IDLE_STATUS:      wifiStatusStr = "Idle"; break;
    case WL_NO_SSID_AVAIL:    wifiStatusStr = "No SSID Available"; break;
    case WL_SCAN_COMPLETED:   wifiStatusStr = "Scan Completed"; break;
    case WL_CONNECTED:        wifiStatusStr = "Connected"; break;
    case WL_CONNECT_FAILED:   wifiStatusStr = "Connect Failed"; break;
    case WL_CONNECTION_LOST:  wifiStatusStr = "Connection Lost"; break;
    case WL_DISCONNECTED:     wifiStatusStr = "Disconnected"; break;
    default:                  wifiStatusStr = "Unknown"; break;
  }
  doc["wifiStatusText"] = wifiStatusStr;
  const char* apModeText = "Disabled";
  if (WiFi.getMode() == WIFI_AP) {
    apModeText = "Enabled (AP only)";
  } else if (WiFi.getMode() == WIFI_AP_STA) {
    apModeText = "Enabled (AP + STA)";
  }
  doc["apModeStatusText"] = apModeText;
  char responseBuffer[512];
  size_t jsonSize = serializeJson(doc, responseBuffer, sizeof(responseBuffer));
  if (jsonSize > 0) {
    request->send(200, "application/json", responseBuffer);
  } else {
    request->send(500, "text/plain", "Failed to serialize status JSON.");
  }
}
void handleGetSettings(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
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
  char responseBuffer[SETTINGS_DOC_CAPACITY]; 
  size_t jsonSize = serializeJson(doc, responseBuffer, sizeof(responseBuffer));
  if (jsonSize > 0) {
    request->send(200, "application/json", responseBuffer);
  } else {
    request->send(500, "text/plain", "Failed to serialize settings JSON.");
   Serial.print(F("ERROR: serializeJson() failed in handleGetSettings (buffer too small or data issue)."));
  }
}

void handleFsCheck(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
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
  doc["maxOpenFiles"] = fs_info.maxOpenFiles;
  char responseBuffer[256];
  size_t jsonSize = serializeJson(doc, responseBuffer, sizeof(responseBuffer));
  if (jsonSize > 0) {
    request->send(200, "application/json", responseBuffer);
  } else {
    request->send(500, "text/plain", "Failed to serialize FS info.");
  }
}
void handleAutoDeepSleepToggle(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!request->hasArg("enable")) {
    request->send(400, "text/plain", "Missing 'enable' parameter (true/false).");
    return;
  }
  bool enable = request->arg("enable").equalsIgnoreCase("true");
  settings.autoDeepSleepEnabled = enable;
  if (saveSettings()) {
    request->send(200, "text/plain", "Auto deep sleep setting updated.");
    broadcastStatus();
  } else {
    request->send(500, "text/plain", "Failed to save auto deep sleep setting.");
  }
}
void broadcastStatus() {
  if (events.count() == 0) return;
  
  // Use static buffer to avoid heap allocation
  static char jsonBuffer[512];
  
  // Build JSON manually to avoid JsonDocument overhead
  int len = snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"relayState\":%s,"
    "\"failsafeActive\":%s,"
    "\"emergencyStop\":%s,"
    "\"loggedIn\":%s,"
    "\"scanInProgress\":%s,"
    "\"relayOnStartTime\":%lu,"
    "\"currentDeviceMillis\":%lu,"
    "\"currentRelayDuration\":%lu,"
    "\"wifiStatus\":%d,"
    "\"ipAddress\":\"%s\","
    "\"ssid\":\"%s\","
    "\"hostname\":\"%s\","
    "\"apModeActive\":%s,"
    "\"deepSleepRequested\":%s,"
    "\"deepSleepDurationSeconds\":%lu,"
    "\"autoDeepSleepScheduled\":%s,"
    "\"autoDeepSleepEnabled\":%s,"
    "\"defaultRelayDuration\":%lu,"
    "\"maxRelayRuntime\":%lu}",
    relayState ? "true" : "false",
    failsafeActive ? "true" : "false",
    emergencyStop ? "true" : "false",
    loggedIn ? "true" : "false",
    scanInProgress ? "true" : "false",
    relayOnStartTime,
    millis(),
    relayDuration,
    WiFi.status(),
    WiFi.localIP().toString().c_str(),
    WiFi.SSID().c_str(),
    settings.hostname,
    isApModeActive ? "true" : "false",
    deepSleepRequested ? "true" : "false",
    settings.deepSleepDurationSeconds,
    autoDeepSleepScheduled ? "true" : "false",
    settings.autoDeepSleepEnabled ? "true" : "false",
    settings.defaultRelayDuration,
    settings.maxRelayRuntime
  );
  
  if (len > 0 && len < (int)sizeof(jsonBuffer)) {
    events.send(jsonBuffer, "status");
  }
}