#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h> // Ensure you have the ArduinoJson library installed (e.g., v6)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h> // For LittleFS
#include <Ticker.h>
//#include <ElegantOTA.h>
//#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <MD5Builder.h>
#include <vector>
#include <algorithm>
#include <time.h> // For time(nullptr) in setLastModified
#include <functional> // Required for std::function, which ArRequestHandlerFunction uses
#include <DNSServer.h>
#include <pgmspace.h>
#include <TimeLib.h> 
#include <AsyncJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <core_version.h>

#define BOT_TOKEN "8069292937:AAGleYiuXQjYCr0K24k6tEazagmqfCXlud8"  // Your BotFather token
#define CHAT_ID   "5348635862"                            // Your chat ID

WiFiClientSecure secureClient;
UniversalTelegramBot bot(BOT_TOKEN, secureClient);

bool telegramSendPending = false;
bool pendingAPSetup = false;

bool telegramDelayActive = false;   // Global flag
bool telegramIpSent = false;
unsigned long telegramSendStartTime = 0; // Global timestamp
/*
static const char TELEGRAM_CERT_PEM[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgISA0bnR2t8m6fYVxMRv3HgPCnpMA0GCSqGSIb3DQEBCwUA
MEoxCzAJBgNVBAYTAlVTMRYwFAYDVQQKDA1MZXQncyBFbmNyeXB0MRMwEQYDVQQD
DApMZXQncyBFbmNyeXB0IFJvb3QgQ0EgQjIwHhcNMjAwNjA0MDIwMDAwWhcNMzUw
NjA0MDIwMDAwWjBKMQswCQYDVQQGEwJVUzEWMBQGA1UECgwNTGV0J3MgRW5jcnlw
dDETMBEGA1UEAwwKTGV0J3MgRW5jcnlwdCBSb290IENBIEIyMIICIjANBgkqhkiG
9w0BAQEFAAOCAg8AMIICCgKCAgEA7bXyDFxD8AmKBYSm2suEMBdNUl6sqeBtzZQ/
a/3ZsSd52qZ6oPtPxo0nqUe5dzDfsHqH5N/EUxlCqH4jjTV8u2R3evlLrMzVqH3x
prwIfkPK1k9pUzRnbzB3TxkqkAwXKZ84o6+U6ZVRN9CJ6NP+m/sa8dTn2yfu5g9V
6dfWZtA1WOS7McydKpoDumUz/f0H4guSL33KQrfghO5dHVfMyZ0Sx1K6hdHhN8e+
O2oZtKObh+swRzrwF1UBXXi3HX5M2Mo/hCz0C8cw1qlJcJdU8jM+v7PfGiCmSuSU
csKawQQgiwRDTcmNEpA0RyBL+zqj8O0ZABSWX6kLZPgaUJHcHe6qKqIATxZXyxsw
+YyHybsoDID+u86mCgmbF0e2AZhMpD8EMlFIB/QLVQ9oYhnIImakFpCsmvL0kpdr
qkZxif5Dh8hX+iHePgVuDkoVdmmPQnrQdr2zdYhVAmP4mtRGU9KQm3evZ7UpnD33
qD1QzKZ4G9aDi4Uo1mV1qvA3Yjxn84pN8aoPZrDGHcT8KfLDlf4Ds+SxLhH9z2br
B98zn2QufFGQJZ0RwPaHbdV4YbL+kOZc2cLhZtlfQbU2R0EoG8M6zqLz1rM31mso
+qO7sQwUolsy8+d5XPPZ6Sjq8gzWvJZHi5vDtyckfGqEACW6iTb+a+FqROEZ5U7D
iMkbB5sCAwEAAaOCAXcwggFzMA4GA1UdDwEB/wQEAwIBBjAdBgNVHQ4EFgQUK+3X
ZgJ4g5KclFoyrU9snYwX6WcwHwYDVR0jBBgwFoAUK+3XZgJ4g5KclFoyrU9snYwX
6WcwDgYDVR0PAQH/BAQDAgEGMA8GA1UdEQQIMAaHBH8AAAEwTAYDVR0gBEUwQzBB
BgpghkgBhv1sAgEOMDYwNAYIKwYBBQUHAgEWKGh0dHBzOi8vY2VydC5sZXRzZW5j
cnlwdC5vcmcvcmVwb3NpdG9yeTCCAQUGA1UdEQQKMAiCC2xldHNlbmNyeXB0MB0G
A1UdDgQWBBSrLEfMiXbtmRp6w5V3qKqgZg+hUTAfBgNVHSMEGDAWgBSrLEfMiXbt
mRp6w5V3qKqgZg+hUTALBgNVHQ8EBAMCAQYwDQYJKoZIhvcNAQELBQADggIBAF6E
K66pcBYsGv3o/Gu4mr7fBwh90LbK6KxF4kFBNKqD84W0OprlEq2x2d3UtDcx80vI
6SR8rE+6xi9o3PaxCrueDJ6tZ9mOJWpRauWzfwqIVD2omP4aBRkJ1/zGuBQk7SYt
oZVfMTKRssSTbUBWjKfqOVK0mUJ6rczpCJxnxC1R8p+xOO1B7QIGbmlVC+1Rqs2+
DCE3K2coQZ9RQvA6H+hASG3DeXcKROHdLCwAaC1DCC1yYtVmbHYZqZUrP+U7n5XL
3jrxWPP8mbKCeEXDEudZ7apQpuM1oZJztNa7z2AhHB0yWTrl0OkbSkPg2fqPS55b
+5F9FTwsrVtulov59JtOaGlpx5MFZtkzCcfpToHj4B5xLf0P8ynRVwEhrXoVGflM
mCtRCQeFAs0MCxKi0C4TY64ofeNqkMHi/3w60IuDLcnnxT1kqA69t9rHMXDGMkfr
4VxPCbAcBT8Zyxm+POXyJvQKFDreWurL1mXIvglDh6iDsZChbfP4SxjAQKrJZ1RG
N7u4Gr9mEyD4Ebh1HdL6+x0R8D7v24rLpQJ7q45qNaPo1Q9m1MoUrT4/JyC5DWgx
eR8pE5j4WSugY3L7YwCr8bySCdzq3WTzBYecR5Z7v2nFv5ylADcCqvG0SqaJ1n6q
LaN3neN+AVXWw/hU3D04yFr6+05rO0GB53XhzX4X
-----END CERTIFICATE-----
)EOF";

*/

/*
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// -------- Wi-Fi credentials --------
const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASS";

// -------- Telegram credentials --------
#define BOT_TOKEN "YOUR_BOT_TOKEN"
#define CHAT_ID "YOUR_CHAT_ID"

// -------- Root CA for api.telegram.org --------
// (ISRG Root X1 – works for Telegram as of Oct 2025)
static const char TELEGRAM_CERT_PEM[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgISA0bnR2t8m6fYVxMRv3HgPCnpMA0GCSqGSIb3DQEBCwUA
MEoxCzAJBgNVBAYTAlVTMRYwFAYDVQQKDA1MZXQncyBFbmNyeXB0MRMwEQYDVQQD
DApMZXQncyBFbmNyeXB0IFJvb3QgQ0EgQjIwHhcNMjAwNjA0MDIwMDAwWhcNMzUw
NjA0MDIwMDAwWjBKMQswCQYDVQQGEwJVUzEWMBQGA1UECgwNTGV0J3MgRW5jcnlw
dDETMBEGA1UEAwwKTGV0J3MgRW5jcnlwdCBSb290IENBIEIyMIICIjANBgkqhkiG
9w0BAQEFAAOCAg8AMIICCgKCAgEA7bXyDFxD8AmKBYSm2suEMBdNUl6sqeBtzZQ/
a/3ZsSd52qZ6oPtPxo0nqUe5dzDfsHqH5N/EUxlCqH4jjTV8u2R3evlLrMzVqH3x
prwIfkPK1k9pUzRnbzB3TxkqkAwXKZ84o6+U6ZVRN9CJ6NP+m/sa8dTn2yfu5g9V
6dfWZtA1WOS7McydKpoDumUz/f0H4guSL33KQrfghO5dHVfMyZ0Sx1K6hdHhN8e+
O2oZtKObh+swRzrwF1UBXXi3HX5M2Mo/hCz0C8cw1qlJcJdU8jM+v7PfGiCmSuSU
csKawQQgiwRDTcmNEpA0RyBL+zqj8O0ZABSWX6kLZPgaUJHcHe6qKqIATxZXyxsw
+YyHybsoDID+u86mCgmbF0e2AZhMpD8EMlFIB/QLVQ9oYhnIImakFpCsmvL0kpdr
qkZxif5Dh8hX+iHePgVuDkoVdmmPQnrQdr2zdYhVAmP4mtRGU9KQm3evZ7UpnD33
qD1QzKZ4G9aDi4Uo1mV1qvA3Yjxn84pN8aoPZrDGHcT8KfLDlf4Ds+SxLhH9z2br
B98zn2QufFGQJZ0RwPaHbdV4YbL+kOZc2cLhZtlfQbU2R0EoG8M6zqLz1rM31mso
+qO7sQwUolsy8+d5XPPZ6Sjq8gzWvJZHi5vDtyckfGqEACW6iTb+a+FqROEZ5U7D
iMkbB5sCAwEAAaOCAXcwggFzMA4GA1UdDwEB/wQEAwIBBjAdBgNVHQ4EFgQUK+3X
ZgJ4g5KclFoyrU9snYwX6WcwHwYDVR0jBBgwFoAUK+3XZgJ4g5KclFoyrU9snYwX
6WcwDgYDVR0PAQH/BAQDAgEGMA8GA1UdEQQIMAaHBH8AAAEwTAYDVR0gBEUwQzBB
BgpghkgBhv1sAgEOMDYwNAYIKwYBBQUHAgEWKGh0dHBzOi8vY2VydC5sZXRzZW5j
cnlwdC5vcmcvcmVwb3NpdG9yeTCCAQUGA1UdEQQKMAiCC2xldHNlbmNyeXB0MB0G
A1UdDgQWBBSrLEfMiXbtmRp6w5V3qKqgZg+hUTAfBgNVHSMEGDAWgBSrLEfMiXbt
mRp6w5V3qKqgZg+hUTALBgNVHQ8EBAMCAQYwDQYJKoZIhvcNAQELBQADggIBAF6E
K66pcBYsGv3o/Gu4mr7fBwh90LbK6KxF4kFBNKqD84W0OprlEq2x2d3UtDcx80vI
6SR8rE+6xi9o3PaxCrueDJ6tZ9mOJWpRauWzfwqIVD2omP4aBRkJ1/zGuBQk7SYt
oZVfMTKRssSTbUBWjKfqOVK0mUJ6rczpCJxnxC1R8p+xOO1B7QIGbmlVC+1Rqs2+
DCE3K2coQZ9RQvA6H+hASG3DeXcKROHdLCwAaC1DCC1yYtVmbHYZqZUrP+U7n5XL
3jrxWPP8mbKCeEXDEudZ7apQpuM1oZJztNa7z2AhHB0yWTrl0OkbSkPg2fqPS55b
+5F9FTwsrVtulov59JtOaGlpx5MFZtkzCcfpToHj4B5xLf0P8ynRVwEhrXoVGflM
mCtRCQeFAs0MCxKi0C4TY64ofeNqkMHi/3w60IuDLcnnxT1kqA69t9rHMXDGMkfr
4VxPCbAcBT8Zyxm+POXyJvQKFDreWurL1mXIvglDh6iDsZChbfP4SxjAQKrJZ1RG
N7u4Gr9mEyD4Ebh1HdL6+x0R8D7v24rLpQJ7q45qNaPo1Q9m1MoUrT4/JyC5DWgx
eR8pE5j4WSugY3L7YwCr8bySCdzq3WTzBYecR5Z7v2nFv5ylADcCqvG0SqaJ1n6q
LaN3neN+AVXWw/hU3D04yFr6+05rO0GB53XhzX4X
-----END CERTIFICATE-----
)EOF";

// -------- Globals --------
WiFiClientSecure secureClient;
UniversalTelegramBot bot(BOT_TOKEN, secureClient);
bool telegramSendPending = true;  // send once on boot
bool telegramDelayActive = false;
unsigned long telegramSendStartTime = 0;
bool serverRunning = false;

// Dummy server start (replace with your AsyncWebServer setup)
void startWebServer() {
  serverRunning = true;
  Serial.println("Server started (dummy)");
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected: " + WiFi.localIP().toString());

  // Install the root certificate (trust anchor)
  BearSSL::X509List* cert = new BearSSL::X509List(TELEGRAM_CERT_PEM);
  secureClient.setTrustAnchors(cert);
}

void loop() {
  if (telegramSendPending && !telegramDelayActive) {
    telegramSendPending = false;
    telegramDelayActive = true;
    telegramSendStartTime = millis();
    Serial.println("⏳ Telegram send delay started...");
  }

  if (telegramDelayActive && millis() - telegramSendStartTime >= 3000) {
    telegramDelayActive = false;

    String msg = "ESP8266 Connected!\nIP: " + WiFi.localIP().toString();
    if (bot.sendMessage(CHAT_ID, msg, "")) {
      Serial.println("✅ Telegram message sent successfully");
    } else {
      Serial.println("❌ Failed to send Telegram message");
    }

    if (!serverRunning) {
      startWebServer();
    }
  }
}

*/


extern time_t relayOnEpochTime;  // Declare this globally in your main file
time_t relayOnEpochTime = 0;
// ðŸ•’ Timezone configuration (example for IST - India Standard Time)
const long gmtOffset_sec = 19800;        // GMT+5:30 â†’ 5.5*3600 = 19800 seconds
const int daylightOffset_sec = 0;        // No DST in India
char pendingFailsafeReason[48] = {0};
time_t estimatedEpochTime = 0;












WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // +5:30 (India), update every 60s

unsigned long lastEpoch = 0;
unsigned long lastSyncMillis = 0;
unsigned long lastTimeSync = 0;

volatile bool fsLock = false;


//volatile bool shouldLogRelayTime = false;
//unsigned long relayLogStart = 0;
//unsigned long relayLogEnd = 0;

volatile bool shouldSaveSettingsFlag = false;

volatile bool relayTimedOffPending = false;
uint32_t relayStartMillis = 0;








// --- Global WiFi State Variables ---
WiFiClient espClient; // For NTPClient, if it needs a client
bool wifiConnected = false;
unsigned long lastWifiConnectAttemptMillis = 0;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 30000; // Try reconnecting every 30 seconds if disconnected

// For Non-Blocking WiFi Scan
unsigned long lastScanStartTime = 0;
const unsigned long WIFI_SCAN_INTERVAL_MS = 60000; // Scan every 1 minute
bool scanInProgress = false; // Already declared, but ensure it's used for async scan control
bool isTemporaryRun = false;


// --- New Function Prototypes ---
void startNonBlockingWiFiScan();
void processScanResults();



#define ARDUINOJSON_USE_DEPRECATED 0
#define FAILSAFE_LOG_FILE "/failsafe.bin"
#define RELAY_LOG_FILE    "/relay.bin"
#define MAX_LOGS          10
#define LOG_ENTRY_LENGTH  128


// --- DEBUG MODE TOGGLE ---
// Uncomment the line below to enable extensive Serial debugging output and debug endpoints.
// Comment it out for a smaller, faster, and more production-ready firmware.
#define DEBUG_MODE 1

// User requested change: Enable async mode for ElegantOTA
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1

// User requested change: Optional Cleanup for DynamicJsonDocument deprecation
// Note: While ARDUINOJSON_USE_DEPRECATED 0 attempts to suppress warnings for older API usage,
// the StaticJsonDocument<N> template itself might still trigger deprecation warnings
// in newer ArduinoJson versions (v7+) because the library encourages using
// the non-templated JsonDocument (which defaults to DynamicJsonDocument) or
// JsonDocument::to<StaticJsonDocument<N>>() for static allocation.
// For embedded systems, StaticJsonDocument provides predictable memory usage,
// so we'll keep it as is, acknowledging the warning.






// --- Debounce setup for buttons ---
// DEBOUNCE_DELAY: Prevents electrical noise from being registered as multiple presses.
// This applies to both the emergency ISR and the regular switch check.
const unsigned long DEBOUNCE_DELAY = 100;
// Adjusted debounce delay to 100ms for responsiveness vs noise
unsigned long apStartTime = 0;
    static unsigned long sleepRequestTime = millis();



// Emergency Pin (D0/GPIO16) Debounce variables
volatile unsigned long lastEmergencyPressTime = 0;

extern AsyncCallbackJsonWebHandler* saveSettingsHandler;




const byte DNS_PORT = 53;
DNSServer dnsServer;
bool dnsServerActive = false;
//bool webServerStarted = false;

uint8_t relayLogIndex = 0;
uint8_t failsafeLogIndex = 0;


// Toggle Switch (D2) Debounce variables
#define SWITCH_PIN D2
//bool lastSwitchReading = HIGH;
// Last raw reading from the switch pin
//bool lastSwitchStableState = HIGH;
// Last confirmed stable state of the switch
unsigned long lastSwitchStateChangeTime = 0;
// Time when the switch state last changed
//const unsigned long SWITCH_TOGGLE_COOLDOWN = 10000;
// 10 seconds functional cooldown for D2 switch
//unsigned long lastSwitchToggleActionTime = 0;
// Time when the relay was last toggled by D2 switch
//bool lastSwitchState = HIGH; // last stable state
//unsigned long lastSwitchDebounceTime = 0;
//unsigned long lastSwitchActionTime = 0;  // time of last accepted change
const unsigned long SWITCH_CHANGE_COOLDOWN = 10000; // 10 seconds
// --- Switch debounce & cooldown variables ---
bool lastSwitchReading = HIGH;          // last raw read from pin
bool lastSwitchStableState = HIGH;      // last debounced stable state
unsigned long lastSwitchDebounceTime = 0;
unsigned long lastSwitchActionTime = 0;




//const int RELAY_PIN = D1;
const int EMERGENCY_PIN = D5; // Changed to D5
// Assuming this is the failsafe pin

bool deepSleepPending = false;
unsigned long deepSleepDurationSec = 0;
//uint32_t remainingSleepSeconds __attribute__((section(".noinit")));
struct RTCData {
  uint32_t magic;
  uint32_t remainingSleepSeconds;
};

RTCData rtcData;

#define RTC_MAGIC 0xA5A55A5A
uint32_t remainingSleepSeconds = 0;
// NEW: Hardcoded Wi-Fi Credentials (CHANGE THESE TO YOUR NETWORK!)
// Structure for hardcoded WiFi credentials
// --- Global WiFi State Variables (continued) ---
bool tryingHardcodedNetworks = false;
int currentHardcodedNetworkIndex = 0;
unsigned long hardcodedConnectAttemptStart = 0;
const unsigned long HARDCODED_CONNECT_TIMEOUT_MS = 15000; // 15 seconds per hardcoded network attempt
const int MAX_STA_CONNECT_ATTEMPTS = 5; // Max attempts for saved STA network before trying hardcoded ones

// --- Hardcoded Networks Definition (User-provided) ---
struct PredefinedNetwork {
    const char ssid[32];
    const char password[64];
};

// Array of hardcoded networks. Add up to three or more as needed.
const PredefinedNetwork hardcodedNetworks[] = {
    {"realme 5g", "bhavil23!"},
    {"ishwar", "12345689"}, // Example 2
    {"note 5pro", "bhavil23!"},    // Example 3
    {"lenovo", "12345678"}      // New hardcoded network added
};
const size_t NUM_HARDCODED_NETWORKS = sizeof(hardcodedNetworks) / sizeof(hardcodedNetworks[0]);




// --- State Variables ---
bool relayState = false;
unsigned long relayOnStartTime = 0;
bool failsafeActive = false; // Indicates if failsafe conditions are met
bool emergencyStop = false;
// Indicates if emergency stop is active
unsigned long relayDuration = 0; // Duration for timed relay runs
bool loggedIn = false;
// Re-added for login
bool isApModeActive = false; // Tracks if AP mode is currently active
unsigned long bootCount = 0;
// System boot count - This will now be managed by bootRecovery.bootCount
unsigned long lastLoginTime = 0;
// Re-added for login
unsigned long sessionStart = 0;
// Re-added for login
const unsigned long sessionTimeout = 300000;
// 5 mins for session timeout - Re-added for login

// Flag to track if WiFi scan is ongoing
//bool softShutdownPending = false;
// NEW: Flag to indicate soft shutdown is pending
bool deepSleepRequested = false;
bool serverRunning = false;
// --- UI Command Cooldown ---
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_COOLDOWN_MS = 2000;
// 2 seconds cooldown for UI commands

volatile bool pendingRestart = false; // Set this instead of calling handleRestart() directly


// --- Auto Deep Sleep after Relay Run ---
// The trigger for auto deep sleep is now simply "after a relay run ends"
const unsigned long AUTO_DEEP_SLEEP_DELAY_MS = 8 * 60 * 1000;   // 5 minutes in milliseconds
bool autoDeepSleepScheduled = false; // Flag to indicate if auto deep sleep is scheduled
Ticker autoDeepSleepDelayTicker; // New ticker for the 10-minute delay
Ticker deferredServerDelete;

// --- Web Server Objects ---
AsyncWebServer server(80);
AsyncEventSource events("/events");

// Server-Sent Events instance


char currentSessionToken[33] = {0}; // Re-added for login

// --- Consolidated Settings Structure ---
// Defined a capacity constant for ArduinoJson to allocate the document dynamically,
// or use StaticJsonDocument with this capacity.
// SETTINGS_DOC_CAPACITY should be adjusted if settings grow significantly.
// An estimate of 2KB (2048 bytes) should be sufficient for the current settings struct.
const size_t SETTINGS_DOC_CAPACITY = 2048; // Bytes

struct Settings {
  // WiFi STA settings
  char staSsid[32];
  char staPassword[64];
  char hostname[32]; // For mDNS
  bool useDHCP;
  IPAddress staticIp;
  IPAddress staticGateway;
  IPAddress staticSubnet;
  IPAddress staticDns1;
  IPAddress staticDns2;
// WiFi AP settings
  bool wifiAPModeEnabled; // New field to enable/disable AP mode
  char apSsid[32];
  char apPassword[64];
// Admin credentials
  char adminUsername[32];
  char adminPasswordHash[33]; // MD5 hash is 32 chars + null terminator
  // NOTE: For stronger security, consider using SHA256 (64 chars + null terminator)
  // or a more robust KDF like Argon2 if ESP8266 resources allow.
  // This would require a separate library and more complex integration.

  // Relay and Failsafe settings
  float calibrationFactor;
  unsigned long failsafeDurationHours;
  bool failsafeEnabled; // Renamed from failsafeActive for clarity
  int failsafePin;
// Digital pin for failsafe
  int relayPin;    // Digital pin for relay control
  unsigned long defaultRelayDuration;
// Default duration for timed relay runs (seconds)
  unsigned long maxRelayRuntime;
// Max allowed relay runtime (seconds)

  // Run time interruption tracking
  bool wasTimedRunInterrupted;
  unsigned int interruptedRunDuration;
  unsigned long interruptedRunStartTime;

  // System status/session
  unsigned long bootUnixTime = 0;
// Last known NTP time at boot

  // NEW: Deep Sleep Settings
  unsigned long deepSleepDurationSeconds;
// User configurable deep sleep duration in seconds
  bool autoDeepSleepEnabled; // NEW: Flag to enable/disable auto deep sleep
  bool autoDeepSleepDelayAfterRun;
};

// Global instance of the consolidated settings
Settings settings;
// Define a defaultSettings structure for factory reset
Settings defaultSettings;



// Boot Recovery Data - used to persist boot count across resets
struct BootRecoveryData {
  unsigned long lastBootMillis;
  uint8_t bootCount;
};
BootRecoveryData bootRecovery;


// --- Known Networks Structure and Vector ---
struct KnownNetwork {
  char ssid[33];
  char password[65];
};
std::vector<KnownNetwork> knownNetworks;

// Ticker for periodic tasks
Ticker failsafeTicker;    // For feeding the failsafe duration
Ticker watchdogTicker;
// For feeding the watchdog
Ticker relayTicker;       // For timed relay turn off
Ticker logCleanupTicker;  // For hourly log cleanup
Ticker statusTicker;
// For periodic status broadcast
Ticker wifiScanTicker;    // NEW: For checking WiFi scan completion

Ticker emergencyStopDelay;




// Advanced Watchdog Implementation (Software Watchdog) variables
volatile unsigned long lastLoopMillis = 0;
Ticker softwareWatchdogTicker; // New Ticker for software watchdog

// Rate limiting for login
unsigned long lastLoginAttemptMillis = 0;
int failedLoginAttempts = 0;
const int MAX_FAILED_LOGIN_ATTEMPTS = 5;
const unsigned long LOGIN_COOLDOWN_TIME = 60000;
// 60 seconds cooldown after max failures


bool otaEnabled = false;
// Flag for ElegantOTA status
bool serverStarted = false;
// Flag to track if the web server is currently running

// --- File system constants ---
const char* SETTINGS_FILE = "/settings.json";
const char* SETTINGS_BACKUP_FILE = "/settings.bak";

// Changed to .txt for append-only JSON lines

// Changed to .txt for append-only JSON lines
const char* BOOT_COUNT_FILE = "/boot_count.json";
// Original file for boot count
const char* BOOT_RECOVERY_FILE = "/boot_recovery.json";
// Specific file for boot recovery data




volatile bool watchdogFeedRequested = false;
volatile bool shouldBroadcastAfterEmergencyClear = false;
volatile bool shouldLogEmergencyClear = false;






// --- Function Prototypes ---
// Core System Functions
void setupLittleFS();
void initWiFi();
void manageWiFiConnection();
bool isWiFiStable(AsyncWebServerRequest *request);
void startWebServer();
void stopWebServer(); // Not implemented in original, but good to have a prototype
void setupAP();
// Prototype for the AP setup function
void checkAPTimeout();
// Not fully implemented in original context, but generally for captive portal
void checkSwitchPress();
// Updated for debounce
void generateSessionToken();
bool authenticate(const char* user, const char* pass);
bool isLoggedIn(AsyncWebServerRequest *request);
const char* getContentType(const char* filename);
// Modified return type
//bool isSSIDInRange(const char* ssid);

void clearEmergencyStopAndFailsafe();  // Declare the function so loop() knows it exists



// Configuration and Persistence
void applyDefaultSettings();
bool loadSettings(); // Modified to return bool
bool saveSettings();
// Modified to return bool
bool loadBootRecovery(); // Modified to return bool
void saveBootRecovery();
void checkRecovery();
// Declaration for the new recovery function
void factoryResetPreserveIndex(); // Declaration for the factory reset function


// Relay and Failsafe Control
void toggleRelayInternal();
// New internal function
void emergencyStopInternal(); // New internal function
void turnOffRelayTimed();

void failsafeTriggered();
// Attached to failsafeTicker
void resetFailsafe(); // MODIFIED
void initiateAutoDeepSleep(); // New function for auto deep sleep


// Logging (Memory optimized for large logs)
//void cleanupOldLogs();
// Placeholder for cleanup logic (e.g., keeping only X latest entries or Y days)

// Web Server Handlers (prototypes for functions used in server.on)
void handleLogin(AsyncWebServerRequest *request);
void handleUpdateLogin(AsyncWebServerRequest *request);
void handleClearFailsafe(AsyncWebServerRequest *request); // NEW: Clear Failsafe
void handleRelayControl(AsyncWebServerRequest *request); // MODIFIED - NO LOGIN
void handleEmergencyStop(AsyncWebServerRequest *request);
// MODIFIED - NO LOGIN
void handleScanNetworks(AsyncWebServerRequest *request); // NO LOGIN
void handleKnownNetworks(AsyncWebServerRequest *request);
void handleSaveKnownNetworks(AsyncWebServerRequest *request);
void handleClearKnownNetworks(AsyncWebServerRequest *request);
void handleAPModeToggle(AsyncWebServerRequest *request);
void handleRestart(AsyncWebServerRequest *request);
void handleRestoreFactorySettings(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);

void handleUpdateWifi(AsyncWebServerRequest *request);
// NEW: Update WiFi STA
void handleDisconnectWifi(AsyncWebServerRequest *request); // NEW: Disconnect WiFi STA
//void handleSoftShutdown(AsyncWebServerRequest *request);     // NEW: Soft Shutdown
void handleSetDeepSleepDuration(AsyncWebServerRequest *request);
// NEW: Set Deep Sleep Duration
void handleDeepSleep(AsyncWebServerRequest *request);            // NEW: Initiate Deep Sleep
void handleFormatFS(AsyncWebServerRequest *request);
// NO LOGIN
void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final); // NO LOGIN
void handleStatus(AsyncWebServerRequest *request);
// New handler for /status
void handleGetSettings(AsyncWebServerRequest *request); // New handler for /settings

// New handler for /saveSettings
void handleFsCheck(AsyncWebServerRequest *request); // NEW: /fs-check endpoint
void handleAutoDeepSleepToggle(AsyncWebServerRequest *request); // NEW: Toggle Auto Deep Sleep

// SSE Broadcast
void broadcastStatus();
// Now broadcasts to SSE clients
void broadcastScanResults();
// NEW: Broadcast WiFi scan results

// Watchdog
void IRAM_ATTR feedWatchdog();
void setupWatchdog();
void checkLoopWatchdog();
// Software watchdog check
// Helper function to initiate a non-blocking scan
void startNonBlockingWiFiScan() {
  if (scanInProgress) {
    // Serial.println(F("WiFi scan already in progress.")); // Uncomment for verbose debug
    return;
  }
  Serial.println(F("Starting non-blocking WiFi scan..."));
 // WiFi.setOutputPower(20.5);
 WiFi.setOutputPower(17.0);
  WiFi.scanNetworks(true); // true for asynchronous scan
  scanInProgress = true;
  lastScanStartTime = millis();
}

bool acquireFSLock(unsigned long timeout = 3000) { // Increased timeout from 1000ms to 3000ms
  unsigned long start = millis();

  while (fsLock) {
    if (millis() - start > timeout) {
      Serial.println(F("âš ï¸ FS lock wait timed out"));
      return false;
    }
    delay(1);  // This is essential to allow the watchdog to be fed and other tasks to run
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

  // Optional: check if lock was successful
  bool isLocked() const {
    return locked;
  }

private:
  bool locked;
};



void IRAM_ATTR feedWatchdog() {
  watchdogFeedRequested = true; // Set flag instead
}

// Call this from loop() to check scan completion and process results
void processScanResults() {
  if (!scanInProgress) return; // No scan in progress

  int n = WiFi.scanComplete();
  if (n >= 0) { // Scan complete (n is number of networks found, or 0 if none)
    scanInProgress = false; // Reset flag

    Serial.printf_P(PSTR("%d networks found:\n"), n);
    JsonDocument doc;
    JsonArray networks = doc.to<JsonArray>();

    for (int i = 0; i < n; ++i) {
      JsonObject network = networks.add<JsonObject>();
      network["ssid"] = WiFi.SSID(i);
      network["rssi"] = WiFi.RSSI(i);
      network["encryption"] = (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted";
      Serial.printf_P(PSTR("  %d: %s (%d dBm) %s\n"),
                      i + 1,
                      WiFi.SSID(i).c_str(),
                      WiFi.RSSI(i),
                      (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted");
    }

    // Broadcast results to web clients (assuming 'events' is your AsyncEventSource)
    String jsonString;
    serializeJson(doc, jsonString);
  
   events.send(jsonString.c_str(), "wifiScanResults");


   
    Serial.println(F("WiFi scan complete and results broadcasted."));

    WiFi.scanDelete(); // Clear scan results to free memory
    lastScanStartTime = millis(); // Update last scan time to prevent immediate re-scan
  } else if (n == WIFI_SCAN_RUNNING) {
    // Scan is still running, do nothing, just wait for next check
    #ifdef DEBUG_MODE
      // Serial.print("."); // Uncomment for feedback during scan
    #endif
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

  // ---- ON / TOGGLE ----
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

  // ---- OFF ----
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

  // ---- TIMED ----
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

    // ---- Auto Deep Sleep scheduling (for timed runs) ----
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

  // ---- TEMPORARY ----
  } else if (strcmp(state, "temporary") == 0) {
    unsigned long duration = 600;  // Default 10 minutes

    if (request->hasArg("duration")) {
      duration = strtoul(request->arg("duration").c_str(), nullptr, 10);
      if (duration == 0 || duration > 3600UL) {
        request->send(400, "text/plain", "Invalid duration (max 3600 sec).");
        return;
      }
    }

    // Stop any active relay runs
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

    isTemporaryRun = true;  // mark this run as temporary (no flash writes)

    relayTicker.once(duration, turnOffRelayTimed);
    Serial.printf_P(PSTR("Relay ON (temporary for %lu sec, no flash writes).\n"), duration);

    // ---- Auto Deep Sleep for temporary runs ≥ 10 min ----
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

  // ---- Finalize response ----
  if (success) {
    request->send(200, "text/plain", "Command processed.");
    lastCommandTime = millis();
    broadcastStatus();
  } else {
    request->send(500, "text/plain", "Failed to process command.");
  }
}

/*
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

  // ---- ON / TOGGLE ----
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

        autoDeepSleepDelayTicker.detach();
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

      autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
      success = true;
    }

  // ---- OFF ----
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

      autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }
    success = true;

  // ---- TIMED ----
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

    // ---- Auto Deep Sleep scheduling (for timed runs) ----
    if (settings.autoDeepSleepEnabled) {
      unsigned long sleepDelay = settings.autoDeepSleepDelayAfterRun;
      if (sleepDelay == 0) sleepDelay = 10;
      autoDeepSleepDelayTicker.once(duration + sleepDelay, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
      Serial.printf_P(PSTR("Auto deep sleep scheduled after timed run (delay %lu sec).\n"), sleepDelay);
    } else {
      autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }

    success = true;

  // ---- TEMPORARY ----
  } else if (strcmp(state, "temporary") == 0) {
    unsigned long duration = 600;  // Default 10 minutes

    if (request->hasArg("duration")) {
      duration = strtoul(request->arg("duration").c_str(), nullptr, 10);
      if (duration == 0 || duration > 3600UL) {
        request->send(400, "text/plain", "Invalid duration (max 3600 sec).");
        return;
      }
    }

    // Stop any active relay runs
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

    isTemporaryRun = true;  // mark this run as temporary (no flash writes)

    relayTicker.once(duration, turnOffRelayTimed);
    Serial.printf_P(PSTR("Relay ON (temporary for %lu sec, no flash writes).\n"), duration);

    // ---- Auto Deep Sleep for temporary runs > 10 min ----
    if (duration >= 600 && settings.autoDeepSleepEnabled) {
      unsigned long sleepDelay = settings.autoDeepSleepDelayAfterRun;
      if (sleepDelay == 0) sleepDelay = 10;

      autoDeepSleepDelayTicker.once(duration + sleepDelay, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
      Serial.printf_P(
        PSTR("Auto deep sleep scheduled after temporary run (duration %lu sec, delay %lu sec).\n"),
        duration, sleepDelay
      );
    } else {
      autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }

    success = true;

  } else {
    request->send(400, "text/plain", "Invalid state parameter.");
    return;
  }

  // ---- Finalize response ----
  if (success) {
    request->send(200, "text/plain", "Command processed.");
    lastCommandTime = millis();
    broadcastStatus();
  } else {
    request->send(500, "text/plain", "Failed to process command.");
  }
}
*/


/*

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

  // ---- ON / TOGGLE ----
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

        autoDeepSleepDelayTicker.detach();
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

      autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
      success = true;
    }

  // ---- OFF ----
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

      autoDeepSleepDelayTicker.detach();
      autoDeepSleepScheduled = false;
    }
    success = true;

  // ---- TIMED ----
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

    autoDeepSleepDelayTicker.detach();
    autoDeepSleepScheduled = false;

    success = true;

  // ---- TEMPORARY ----
  } else if (strcmp(state, "temporary") == 0) {
    unsigned long duration = 600;  // Default 10 minutes

    if (request->hasArg("duration")) {
      duration = strtoul(request->arg("duration").c_str(), nullptr, 10);
      if (duration == 0 || duration > 3600UL) {
        request->send(400, "text/plain", "Invalid duration (max 3600 sec).");
        return;
      }
    }

    // Stop any active relay runs
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

    isTemporaryRun = true;  // <<< mark this run as temporary (global flag)

    relayTicker.once(duration, turnOffRelayTimed);
    Serial.printf_P(PSTR("Relay ON (temporary for %lu sec, no flash writes).\n"), duration);

    autoDeepSleepDelayTicker.detach();
    autoDeepSleepScheduled = false;
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
*/





// Ensure time.h is included at the top of your .ino file
// #include <time.h>
/*
void logRelayRunTime(uint32_t startMillis, uint32_t endMillis) {
  uint32_t duration = (endMillis - startMillis) / 1000UL;

  if (duration == 0 || duration > 86400UL) {
    Serial.printf("âš ï¸ Invalid relay log. start=%lu, end=%lu, duration=%lu s\n", startMillis, endMillis, duration);
    return;
  }

  time_t currentEpoch = time(nullptr);
  if (currentEpoch <= 1600000000UL) {
    Serial.println(F("âš ï¸ Time not synced. Skipping relay log."));
    return;
  }

  char entry[LOG_ENTRY_LENGTH] = {0};
  snprintf(entry, sizeof(entry), "%lu,%lu\n", currentEpoch, duration);

  ESP.wdtFeed();
  circularLogWrite(RELAY_LOG_FILE, entry, relayLogIndex);
  ESP.wdtFeed();

  Serial.printf("âœ… Logged Relay Run: Epoch %lu, Duration %lu s\n", currentEpoch, duration);
}
*/

/*
void logRelayRunTime() {
  time_t now = time(nullptr);
  if (now <= 1600000000UL || relayOnEpochTime == 0) {
    Serial.println(F("âš ï¸ Time not synced or relayOnEpochTime is 0. Skipping relay log."));
    return;
  }
  uint32_t currentEpoch = time(nullptr);  // Get current epoch time


  uint32_t duration = now - relayOnEpochTime;
  if (duration == 0) duration = 1;  // Prevent zero-second logs
  
  uint32_t duration = (endMillis - startMillis + 999) / 1000UL; // Round up
if (duration == 0 || duration > 86400UL) {
  Serial.printf("âš ï¸ Invalid relay log. start=%lu, end=%lu, duration=%lu s\n", startMillis, endMillis, duration);
  return;
}


  char entry[LOG_ENTRY_LENGTH] = {0};
  snprintf(entry, sizeof(entry), "%lu,%lu\n", relayOnEpochTime, duration);
  Serial.printf("Log Entry: %s", entry);


 
snprintf(entry, sizeof(entry), "%lu,%lu\n", currentEpoch, duration);
entry[LOG_ENTRY_LENGTH - 1] = '\0';  // Safety null termination
Serial.printf("Log Entry: '%s'\n", entry);



  ESP.wdtFeed();
  circularLogWrite(RELAY_LOG_FILE, entry, relayLogIndex);
  ESP.wdtFeed();

  Serial.printf("âœ… Logged Relay Run: Epoch %lu, Duration %lu s\n", relayOnEpochTime, duration);
  Serial.printf("âœ… Logged Relay Run: Epoch %lu, Duration %lu s\n", currentEpoch, duration);
  #ifdef DEBUG_MODE

  File file = LittleFS.open(RELAY_LOG_FILE, "r");
Serial.println("ðŸ”Ž Validating log content:");
while (file.available()) {
  String line = file.readStringUntil('\n');
  Serial.println(line);
}
file.close();
#endif


}
*/
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








// Ensure time.h is included at the top of your .ino file
// #include <time.h>

// Make sure LOG_ENTRY_LENGTH is defined and large enough, e.g.,
// #define LOG_ENTRY_LENGTH 128 // Or more, depending on max log message size

// Add this to your global declarations (if not already there)
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // +5:30 (India), update every 60s
// unsigned long lastTimeSync = 0; // If you manage sync manually
/*

void logFailsafeEvent(const char* reason) {
  ESP.wdtFeed();

  char safeReason[48] = {0};
  if (reason && reason[0]) {
    strncpy(safeReason, reason, sizeof(safeReason) - 1);
    for (char* p = safeReason; *p; p++) {
      if (*p == ',' || *p == '\n' || *p == '\r') *p = '_';
    }
  } else {
    strncpy(safeReason, "Unknown", sizeof(safeReason) - 1);
  }

  time_t currentEpoch = time(nullptr);
  bool validTime = currentEpoch > 1600000000UL;

  char msg[LOG_ENTRY_LENGTH] = {0};
  if (validTime) {
    snprintf(msg, sizeof(msg), "%lu,%s\n", currentEpoch, safeReason);
  } else {
    snprintf(msg, sizeof(msg), "0,%s\n", safeReason);
  }

  ESP.wdtFeed();
  Serial.printf("[DEBUG] Logging Failsafe Event: Index=%u, Reason='%s'\n", failsafeLogIndex, safeReason);
  yield();  // ✅ Yield after serial print

  circularLogWrite(FAILSAFE_LOG_FILE, msg, failsafeLogIndex);

  ESP.wdtFeed();

  // ✅ SAFE ALTERNATIVE to String().c_str()
  if (validTime) {
    Serial.printf("[Failsafe Log] %lu, Reason: %s\n", currentEpoch, safeReason);
  } else {
    Serial.printf("[Failsafe Log] unknown_time, Reason: %s\n", safeReason);
  }
  yield();  // ✅ Yield again to avoid WDT reset
}

*/
/*
void logFailsafeEvent(const char* reason) {
  ESP.wdtFeed();

  char safeReason[48] = {0};
  if (reason && reason[0]) {
    strncpy(safeReason, reason, sizeof(safeReason) - 1);
    for (char* p = safeReason; *p; p++) {
      if (*p == ',' || *p == '\n' || *p == '\r') *p = '_';
    }
  } else {
    strncpy(safeReason, "Unknown", sizeof(safeReason) - 1);
  }

  time_t currentEpoch = time(nullptr);
  bool validTime = currentEpoch > 1600000000UL;

  char msg[LOG_ENTRY_LENGTH] = {0};
  if (validTime) {
    snprintf(msg, sizeof(msg), "%lu,%s\n", currentEpoch, safeReason);
  } else {
    snprintf(msg, sizeof(msg), "0,%s\n", safeReason);
  }

  ESP.wdtFeed();
  Serial.printf("[DEBUG] Logging Failsafe Event: Index=%u, Reason='%s'\n", failsafeLogIndex, safeReason);
  yield();  // ✅ Yield after serial print

  circularLogWrite(FAILSAFE_LOG_FILE, msg, failsafeLogIndex);

  ESP.wdtFeed();

  // ✅ SAFE ALTERNATIVE to String().c_str()
  if (validTime) {
    Serial.printf("[Failsafe Log] %lu, Reason: %s\n", currentEpoch, safeReason);
  } else {
    Serial.printf("[Failsafe Log] unknown_time, Reason: %s\n", safeReason);
  }
  yield();  // ✅ Yield again to avoid WDT reset
}

*/

void logFailsafeEvent(const char* reason) {
  ESP.wdtFeed();  // Start by feeding watchdog

  char logLine[128];
  time_t now = time(nullptr);

  // If NTP not synced, fallback to snapshot
  if (now < 1600000000UL) {
    now = estimatedEpochTime; // you already maintain this
  }

  // Sanitize reason
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

  // Build log line in RAM
  int len = snprintf(logLine, sizeof(logLine),
                     "%lu,Failsafe,%s\n", (unsigned long)now, cleanReason);

  ESP.wdtFeed();  // Yield before FS

  {
    FSLockGuard guard;

    File f = LittleFS.open(FAILSAFE_LOG_FILE, "a");
    if (!f) {
      Serial.println(F("❌ Could not open failsafe log for writing!"));
      return;
    }
    f.write((const uint8_t*)logLine, len);  // Raw write, avoids printf heap
    f.flush(); // ensure it's actually written
    f.close();
  }

  ESP.wdtFeed();  // Done
}



/*

void printCircularLog(const char* filename, uint8_t currentIndex) {
 FSLockGuard lock;
if (!lock.isLocked()) return;

// do file stuff
// no need to call releaseFSLock() â€” automatically released

  File logFile = LittleFS.open(filename, "r");
  if (!logFile) return;

  char buffer[LOG_ENTRY_LENGTH + 1] = {0};  // +1 for null-terminator
  for (uint8_t i = 0; i < MAX_LOGS; i++) {
    uint8_t idx = (currentIndex + i) % MAX_LOGS;
    logFile.seek(idx * LOG_ENTRY_LENGTH, SeekSet);
    logFile.readBytes(buffer, LOG_ENTRY_LENGTH);
    buffer[LOG_ENTRY_LENGTH] = 0;

    if (buffer[0] != 0 && buffer[0] != '\n') {  // skip empty or newline-only lines
      Serial.printf("[%d] %s", idx, buffer);
    }
  }
  logFile.close();
 
}
*/

/*
// Update your syncTimeFromNTP function:
void syncTimeFromNTP() {
  if (WiFi.status() == WL_CONNECTED) {
    // timeClient.begin() is now called in setup()
    if (timeClient.forceUpdate()) { // Force an immediate update
      time_t epochTime = timeClient.getEpochTime();

      // ** CRITICAL: Set the ESP8266's internal system time **
      struct timeval tv;
     tv.tv_sec = epochTime;
      tv.tv_usec = 0; // NTPClient typically doesn't provide microseconds
      settimeofday(&tv, nullptr); // Update the system's time

      lastEpoch = epochTime; // Update your global variable
      lastSyncMillis = millis(); // Update millis at sync point

      Serial.printf("âœ… Time synced to system clock: %lu (epoch)\n", lastEpoch);     Serial.print("Formatted Time: ");
      Serial.println(timeClient.getFormattedTime()); // Use NTPClient's formatted time for display
    } else {
      Serial.println("âŒ Failed to force update NTP time.");
    }
    // timeClient.end() is NOT called here
  } else {
    Serial.println("âš ï¸ NTP skipped, WiFi not connected.");
  }
}
*/void applyEstimatedTimeFromSnapshot() {
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
  estimatedEpochTime = savedEpoch + ((millis() - savedMillis) / 1000);  // <-- Store globally

  struct timeval tv = { (time_t)estimatedEpochTime, 0 };
  settimeofday(&tv, nullptr);

  Serial.printf(" Applied estimated time: %lu (from snapshot)\n", estimatedEpochTime);
}


void syncTimeFromNTP() {
  if (WiFi.status() == WL_CONNECTED) {
    if (timeClient.forceUpdate()) {
      time_t epochTime = timeClient.getEpochTime();
    //time_t epochTime = time(nullptr);  // ✅ system time (set by NTP or estimate)


      struct timeval tv;
      tv.tv_sec = epochTime;
      tv.tv_usec = 0;
      settimeofday(&tv, nullptr);

      lastEpoch = epochTime;
      lastSyncMillis = millis();

      Serial.printf("âœ… Time synced to system clock: %lu (epoch)\n", lastEpoch);
      Serial.print("Formatted Time: ");
      Serial.println(timeClient.getFormattedTime());

      // âœ… Save snapshot for future boots
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





// Helper to get MD5 hash for password verification (already exists in original sketch)
void getMD5Hash(const char* input, char* output, size_t outputSize) {
  if (outputSize < 33) return;
// MD5 hex string is 32 chars + null
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

// Function to list files in LittleFS (useful for debugging factory reset)
//void listFiles() {
//  if (!acquireFSLock()) {
  //  Serial.println(F("ERROR: Failed to acquire FS lock"));
  //  return;
//  }
 // Serial.println(F("Listing LittleFS files:"));
 // Dir dir = LittleFS.openDir("/");
//  while (dir.next()) {
 //   Serial.print(F("  "));
 //   Serial.print(dir.fileName());
 //   Serial.print(F(" - "));
 //   File f = dir.openFile("r");
 //   Serial.print(f.size());
 //   Serial.println(F(" bytes"));
 //   f.close();
//  }
 // Serial.println(F("--------------------"));
// 
//}


// Recovery Function 1: Factory Reset preserving index.html
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

  // Reset runtime state flags
  relayState = false;
  failsafeActive = false;
  emergencyStop = false;
  loggedIn = false;
  isApModeActive = false;
  deepSleepPending = false;
  deepSleepRequested = false;
  serverRunning = false;
  scanInProgress = false;
 // softShutdownPending = false;
  autoDeepSleepScheduled = false;

  Serial.println(F("Factory reset complete. Restarting ESP..."));
  pendingRestart = true;  // Defer restart to loop()
}




// Recovery Function 2: Check for repeated crashes
void checkRecovery() {
  Serial.println(F("--- checkRecovery() called ---"));
// No need to load here, setup() will call loadBootRecovery() with its own checks.
// This function is primarily for acting on the boot count.

  bootRecovery.lastBootMillis = millis();
// Update last boot time
  bootRecovery.bootCount++; // Increment boot count

  Serial.printf("Boot count: %u\n", bootRecovery.bootCount);

  saveBootRecovery();
// Save updated boot recovery data

  String reason = ESP.getResetReason(); // String is used here from ESP core library, minimal impact.
  Serial.print(F("Reset reason: "));
  Serial.println(reason);
// Factory reset after 3 fast crashes (Exception) or Software Watchdog
  // Note: "Exception" usually indicates a crash.
// "Software Watchdog" is from our custom watchdog.
  if ((bootRecovery.bootCount >= 3 && reason == "Exception") || reason.indexOf("Software Watchdog") != -1) {
    Serial.println(F("Critical error detected (3+ exceptions or software watchdog reset). Initiating factory reset..."));
    factoryResetPreserveIndex(); // Perform the factory reset
    // ESP.restart() is called inside factoryResetPreserveIndex(), so no need here.
  }
   Serial.println(F("--- checkRecovery() finished ---"));
}
bool loadRTC() {
  ESP.rtcUserMemoryRead(0, (uint32_t*)&rtcData, sizeof(rtcData));
  return rtcData.magic == RTC_MAGIC;
}

void saveRTC() {
  rtcData.magic = RTC_MAGIC;
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(rtcData));
}

void clearRTC() {
  rtcData.magic = 0;
  rtcData.remainingSleepSeconds = 0;
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(rtcData));
}

void setup() {
  Serial.begin(115200);
 // Serial.printf("ESP8266 Core Version: %s\n", ESP8266_CORE_VERSION);
 Serial.println("ESP8266 core detected!");
  Serial.println(ESP.getCoreVersion());
  while (!Serial && millis() < 5000);
  Serial.println(F("\nAquaMaster SSE 101 AI Starting..."));
  Serial.println(F("===== BOOT START ====="));
Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());

bool rtcValid = loadRTC();

if (rtcValid) {
  Serial.println(F("✅ RTC load SUCCESS"));
  Serial.printf("RTC.magic = 0x%08X\n", rtcData.magic);
  Serial.printf("RTC.remainingSleepSeconds = %u\n",
                rtcData.remainingSleepSeconds);
} else {
  Serial.println(F("❌ RTC load FAILED / INVALID"));
  rtcData.remainingSleepSeconds = 0;
}

/* 🔑 MID-CYCLE WAKE */
if (rtcData.remainingSleepSeconds > 0) {
  Serial.println(F("🔁 MID-CYCLE WAKE DETECTED"));
  Serial.printf("Remaining sleep = %u seconds\n",
                rtcData.remainingSleepSeconds);

  deepSleepRequested = true;
  sleepRequestTime = 0;
} else {
  Serial.println(F("🏁 FINAL WAKE (no remaining sleep)"));
}

Serial.println(F("===== SETUP END ====="));

/* 🚫 DO NOT CLEAR RTC HERE */
/*
// ---------- LOAD RTC ----------
if (loadRTC()) {
  Serial.printf("RTC loaded: %u seconds remaining\n",
                rtcData.remainingSleepSeconds);
} else {
  rtcData.remainingSleepSeconds = 0;
  Serial.println("RTC empty or invalid");
}

// ---------- MID-CYCLE WAKE ----------
if (rtcData.remainingSleepSeconds > 0) {
  Serial.println("Mid-cycle wake → scheduling next sleep");

  remainingSleepSeconds = rtcData.remainingSleepSeconds;  // 🔑 COPY TO RAM
  deepSleepRequested = true;
  sleepRequestTime = 0;

  // ⚠️ DO NOT return
  // ⚠️ DO NOT clear RTC
}

// ---------- FINAL WAKE ----------
else {  // 🔑 MUST be else
  remainingSleepSeconds = 0;
  clearRTC();
  Serial.println("Sleep cycle complete — normal boot");
}
*/
/*  if (loadRTC()) {
  remainingSleepSeconds = rtcData.remainingSleepSeconds;
  Serial.printf("RTC loaded: %u seconds remaining\n", remainingSleepSeconds);
} else {
  remainingSleepSeconds = 0;
  Serial.println("RTC empty or invalid");
}

// 🔑 THIS BLOCK GOES HERE
if (remainingSleepSeconds > 0) {
  Serial.println("Mid-cycle wake → going back to sleep");
  deepSleepRequested = true;
  sleepRequestTime = 0;
  return;   // ⬅️ EXACT PLACE
}

// Final wake only
if (remainingSleepSeconds == 0) {
  clearRTC();
  Serial.println("Sleep cycle complete");
}
*/
/*  if (loadRTC()) {
  remainingSleepSeconds = rtcData.remainingSleepSeconds;
  Serial.printf("RTC loaded: %u seconds remaining\n", remainingSleepSeconds);
} else {
  remainingSleepSeconds = 0;
  Serial.println("RTC empty or invalid");
}
if (remainingSleepSeconds == 0) {
  clearRTC();
  Serial.println("Sleep cycle complete");
}*/
//  BearSSL::X509List* cert = new BearSSL::X509List(TELEGRAM_CERT_PEM);
//  secureClient.setTrustAnchors(cert);
  Serial.printf("RTC remainingSleepSeconds = %lu\n", rtcData.remainingSleepSeconds);
  
  

  if (!LittleFS.begin()) {
    Serial.println(F("âŒ LittleFS mount failed. Attempting recovery..."));
    factoryResetPreserveIndex();
  } else { // Only if LittleFS.begin() was successful
    Serial.println(F("âœ… LittleFS mounted successfully."));
    // New: Perform LittleFS consistency check
    if (!LittleFS.check()) {
      Serial.println(F("âš ï¸ LittleFS consistency check failed! Consider formatting if data is not critical."));
      // You might attempt a format here if data is not critical for recovery
      // Or simply log and continue, as formatting might erase user settings
    } else {
      Serial.println(F("âœ… LittleFS consistency check passed."));
    }
#ifdef DEBUG_MODE
    FSInfo fs_info;
    LittleFS.info(fs_info);
    Serial.printf("LittleFS Total: %u KB, Used: %u KB\n", fs_info.totalBytes / 1024, fs_info.usedBytes / 1024);
 //   listFiles();
#endif
  }
  


  applyDefaultSettings();
  bool settingsOK = loadSettings();
  if (!settingsOK) {
    Serial.println(F("âš ï¸ Settings failed to load or were corrupted. Triggering factory reset."));
    factoryResetPreserveIndex();
  } else {
#ifdef DEBUG_MODE
    Serial.println(F("âœ… Settings loaded successfully."));
#endif
  }

  bool bootRecoveryOK = loadBootRecovery();
  if (!bootRecoveryOK) {
    Serial.println(F("âš ï¸ Boot recovery file corrupted or missing. Triggering factory reset."));
    factoryResetPreserveIndex();
  }

  checkRecovery();

  pinMode(settings.relayPin, OUTPUT);
  digitalWrite(settings.relayPin, HIGH); // relay OFF at boot
  relayState = (digitalRead(settings.relayPin) == LOW); // sync state from hardware

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastSwitchReading = digitalRead(SWITCH_PIN);     // store initial raw reading
  lastSwitchStableState = lastSwitchReading;       // store as initial stable state



  // --- REMOVED DEBUG_MODE: WIFI SCAN TEST START/END ---
  // The blocking WiFi scan test has been removed from setup().
  // It will now be handled asynchronously via handleScanNetworks and loop.

  // --- BEGIN NON-BLOCKING WIFI CONNECTION INITIATION ---
  initWiFi(); // Initiate non-blocking WiFi connection
#ifdef DEBUG_MODE
  Serial.printf_P(PSTR("Attempting to connect to WiFi: %s\n"), settings.staSsid);
#endif
  // The blocking 'while' loop for WiFi connection (connectTimeout) is removed from here.
  // Connection status and server start will be managed by manageWiFiConnection() in loop().
  // --- END NON-BLOCKING WIFI CONNECTION INITIATION ---

  timeClient.begin(); // Initialize NTPClient once (This was already present and correct)
  Serial.println(F("NTP Client initialized."));

  // Initial NTP sync will be handled by manageWiFiConnection() after successful connection
  // syncTimeFromNTP(); // REMOVED: Will be called by manageWiFiConnection when connected
  // lastTimeSync = millis(); // REMOVED: Managed by manageWiFiConnection

 // ElegantOTA.setAuth("admin", "AQUASTORM_OTA_PASS");

  setupWatchdog();
  softwareWatchdogTicker.attach_ms(5000, checkLoopWatchdog);
#ifdef DEBUG_MODE
  Serial.println(F("Software watchdog initialized. Checking every 5 seconds."));
#endif

  statusTicker.attach_ms(2000, broadcastStatus);
/*
  if (settings.failsafeEnabled && settings.maxRelayRuntime > 0) {
    failsafeTicker.attach(settings.maxRelayRuntime, failsafeTriggered);
#ifdef DEBUG_MODE
    Serial.printf_P(PSTR("Failsafe attached for %lu hours.\n"), settings.failsafeDurationHours);
#endif
  }
  */
 
  applyEstimatedTimeFromSnapshot();  // â³ Estimate based on last saved epoch
configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");  // Set timezone config
autoDeepSleepScheduled = false;
deepSleepRequested = false;
sleepRequestTime = 0;



  //logCleanupTicker.attach(3600 * 24, cleanupOldLogs); // Run hourly
}

// Called in loop() to manage WiFi
// Helper: Check if an SSID is available in scan
bool hardcodedSSIDAvailable(const char* ssid) {
    int16_t numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        if (WiFi.SSID(i) == ssid) {
            return true; // Found matching SSID
        }
    }
    return false; // Not found
}
void manageWiFiConnection() {

    // 🚫 AP MODE: no STA reconnects or scans
    if (isApModeActive && WiFi.getMode() == WIFI_AP) {
        if (!serverRunning) {
            startWebServer();
            Serial.println(F("HTTP server started in AP mode."));
        }
        checkAPTimeout();
        return;
    }

    static unsigned long lastReconnectAttempt = 0;
    static int failedSavedStaAttempts = 0;
    const unsigned long reconnectInterval = 5000;

    if (WiFi.getMode() != WIFI_STA) return;

    // ===================== NOT CONNECTED =====================
    if (WiFi.status() != WL_CONNECTED) {

        wifiConnected = false;   // 🔴 IMPORTANT for reconnect handling

        if (!tryingHardcodedNetworks) {

            if (millis() - lastReconnectAttempt > reconnectInterval) {

                Serial.println(F("WiFi disconnected. Reconnecting to saved network..."));
                WiFi.begin(settings.staSsid, settings.staPassword);
                lastReconnectAttempt = millis();
                failedSavedStaAttempts++;

                if (failedSavedStaAttempts >= MAX_STA_CONNECT_ATTEMPTS) {

                    Serial.println(F("Saved STA failed. Checking hardcoded networks..."));
                    bool foundAnyHardcoded = false;

                    for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                        if (hardcodedSSIDAvailable(hardcodedNetworks[i].ssid)) {
                            currentHardcodedNetworkIndex = i;
                            foundAnyHardcoded = true;
                            break;
                        }
                    }

                    if (foundAnyHardcoded) {
                        Serial.printf_P(PSTR("Trying hardcoded SSID: %s\n"),
                                        hardcodedNetworks[currentHardcodedNetworkIndex].ssid);
                        tryingHardcodedNetworks = true;
                        failedSavedStaAttempts = 0;
                        WiFi.begin(hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                                   hardcodedNetworks[currentHardcodedNetworkIndex].password);
                        hardcodedConnectAttemptStart = millis();
                    } else {
                        failedSavedStaAttempts = 0;
                    }
                }
            }

        } else {

            if (millis() - hardcodedConnectAttemptStart > HARDCODED_CONNECT_TIMEOUT_MS) {

                currentHardcodedNetworkIndex++;

                while (currentHardcodedNetworkIndex < NUM_HARDCODED_NETWORKS &&
                       !hardcodedSSIDAvailable(hardcodedNetworks[currentHardcodedNetworkIndex].ssid)) {
                    currentHardcodedNetworkIndex++;
                }

                if (currentHardcodedNetworkIndex < NUM_HARDCODED_NETWORKS) {

                    Serial.printf_P(PSTR("Trying hardcoded SSID: %s\n"),
                                    hardcodedNetworks[currentHardcodedNetworkIndex].ssid);
                    WiFi.begin(hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                               hardcodedNetworks[currentHardcodedNetworkIndex].password);
                    hardcodedConnectAttemptStart = millis();

                } else {

                    Serial.println(F("All networks failed. Switching to AP mode."));
                    tryingHardcodedNetworks = false;
                    currentHardcodedNetworkIndex = 0;
                    setupAP();
                    isApModeActive = true;

                    if (serverRunning) stopWebServer();
                }
            }
        }

        return;
    }

    // ===================== CONNECTED =====================
    failedSavedStaAttempts = 0;
    tryingHardcodedNetworks = false;
    currentHardcodedNetworkIndex = 0;
    hardcodedConnectAttemptStart = 0;

    if (!wifiConnected) {

        Serial.println(F("WiFi connected!"));
        Serial.print(F("IP: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("MAC: "));
        Serial.println(WiFi.macAddress());

        wifiConnected = true;
        isApModeActive = false;

        // 🔐 BearSSL (ESP8266 core 3.x safe)
        ESP.wdtFeed();
        secureClient.setInsecure();
        ESP.wdtFeed();

        // 📢 Telegram trigger
        telegramSendPending = true;
        telegramDelayActive = false;

        // 💾 Save credentials if changed
        if (strcmp(settings.staSsid, WiFi.SSID().c_str()) != 0) {

            strncpy(settings.staSsid, WiFi.SSID().c_str(),
                    sizeof(settings.staSsid) - 1);
            settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';

            for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                if (strcmp(hardcodedNetworks[i].ssid,
                           WiFi.SSID().c_str()) == 0) {

                    strncpy(settings.staPassword,
                            hardcodedNetworks[i].password,
                            sizeof(settings.staPassword) - 1);
                    settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
                    break;
                }
            }
            saveSettings();
        }

        if (lastTimeSync == 0 || millis() - lastTimeSync > 300000) {
            syncTimeFromNTP();
            lastTimeSync = millis();
        }
    }

    timeClient.update();
}
/*
void manageWiFiConnection() {

    // 🚫 If AP mode is active, do NOT attempt STA reconnect or WiFi scans
    if (isApModeActive && WiFi.getMode() == WIFI_AP) {
        if (!serverRunning) {
            startWebServer();
            Serial.println(F("HTTP server started in AP mode."));
        }
        checkAPTimeout();
        return;  // ⛔ Prevent STA logic & non-blocking scans
    }

    static unsigned long lastReconnectAttempt = 0;
    static int failedSavedStaAttempts = 0;
    const unsigned long reconnectInterval = 5000;

    if (WiFi.getMode() == WIFI_STA) {

        if (WiFi.status() != WL_CONNECTED) {  // Not connected

            if (!tryingHardcodedNetworks) {  // Trying saved STA network

                if (millis() - lastReconnectAttempt > reconnectInterval) {

                    Serial.println(F("WiFi disconnected. Attempting reconnection to saved network..."));
                    WiFi.begin(settings.staSsid, settings.staPassword);
                    lastReconnectAttempt = millis();
                    failedSavedStaAttempts++;

                    if (failedSavedStaAttempts >= MAX_STA_CONNECT_ATTEMPTS) {

                        Serial.println(F("Saved STA failed too many times. Checking hardcoded networks..."));
                        bool foundAnyHardcoded = false;

                        for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                            if (hardcodedSSIDAvailable(hardcodedNetworks[i].ssid)) {
                                foundAnyHardcoded = true;
                                currentHardcodedNetworkIndex = i;
                                break;
                            }
                        }

                        if (foundAnyHardcoded) {
                            Serial.printf_P(PSTR("Found hardcoded SSID: %s\n"),
                                            hardcodedNetworks[currentHardcodedNetworkIndex].ssid);
                            tryingHardcodedNetworks = true;
                            failedSavedStaAttempts = 0;
                            WiFi.begin(hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                                       hardcodedNetworks[currentHardcodedNetworkIndex].password);
                            hardcodedConnectAttemptStart = millis();
                        } else {
                            Serial.println(F("No hardcoded SSIDs found in scan. Continuing STA retry loop."));
                            failedSavedStaAttempts = 0;
                        }
                    }
                }

            } else {  // Trying hardcoded networks

                if (millis() - hardcodedConnectAttemptStart > HARDCODED_CONNECT_TIMEOUT_MS) {

                    Serial.printf_P(PSTR("Failed to connect to %s. Moving to next hardcoded network.\n"),
                                    hardcodedNetworks[currentHardcodedNetworkIndex].ssid);

                    currentHardcodedNetworkIndex++;

                    while (currentHardcodedNetworkIndex < NUM_HARDCODED_NETWORKS &&
                           !hardcodedSSIDAvailable(hardcodedNetworks[currentHardcodedNetworkIndex].ssid)) {
                        currentHardcodedNetworkIndex++;
                    }

                    if (currentHardcodedNetworkIndex < NUM_HARDCODED_NETWORKS) {

                        Serial.printf_P(PSTR("Trying hardcoded SSID: %s\n"),
                                        hardcodedNetworks[currentHardcodedNetworkIndex].ssid);

                        WiFi.begin(hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                                   hardcodedNetworks[currentHardcodedNetworkIndex].password);
                        hardcodedConnectAttemptStart = millis();

                    } else {

                        Serial.println(F("All available hardcoded networks failed. Starting AP mode."));
                        tryingHardcodedNetworks = false;
                        currentHardcodedNetworkIndex = 0;
                        setupAP();
                        isApModeActive = true;

                        if (serverRunning) stopWebServer();
                    }
                }
            }

        } else {  // ✅ Connected

            failedSavedStaAttempts = 0;
            tryingHardcodedNetworks = false;
            currentHardcodedNetworkIndex = 0;
            hardcodedConnectAttemptStart = 0;

            if (!wifiConnected) {

                Serial.println(F("WiFi connected!"));
                Serial.print(F("IP: "));
                Serial.println(WiFi.localIP());
                Serial.print(F("MAC: "));
                Serial.println(WiFi.macAddress());

                wifiConnected = true;
                isApModeActive = false;

                // 📢 Request Telegram send before starting HTTP server
                secureClient.setInsecure();
                telegramSendPending = true;
                telegramDelayActive = false;

                // Save new credentials if connected via hardcoded
                if (strcmp(settings.staSsid, WiFi.SSID().c_str()) != 0) {

                    Serial.println(F("Saving new connected SSID..."));
                    strncpy(settings.staSsid, WiFi.SSID().c_str(),
                            sizeof(settings.staSsid) - 1);
                    settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';

                    for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                        if (strcmp(hardcodedNetworks[i].ssid,
                                   WiFi.SSID().c_str()) == 0) {

                            strncpy(settings.staPassword,
                                    hardcodedNetworks[i].password,
                                    sizeof(settings.staPassword) - 1);
                            settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
                            break;
                        }
                    }
                    saveSettings();
                }

                if (lastTimeSync == 0 || (millis() - lastTimeSync > 300000)) {
                    syncTimeFromNTP();
                    lastTimeSync = millis();
                }
            }

            timeClient.update();
        }
    }
}
*/
/*
void manageWiFiConnection() {
    static unsigned long lastReconnectAttempt = 0;
    static int failedSavedStaAttempts = 0;
    const unsigned long reconnectInterval = 5000;

    if (WiFi.getMode() == WIFI_STA) {
        if (WiFi.status() != WL_CONNECTED) {  // Not connected
            if (!tryingHardcodedNetworks) {  // Trying saved STA network
                if (millis() - lastReconnectAttempt > reconnectInterval) {
                    Serial.println(F("WiFi disconnected. Attempting reconnection to saved network..."));
                    WiFi.begin(settings.staSsid, settings.staPassword);
                    lastReconnectAttempt = millis();
                    failedSavedStaAttempts++;

                    if (failedSavedStaAttempts >= MAX_STA_CONNECT_ATTEMPTS) {
                        Serial.println(F("Saved STA failed too many times. Checking hardcoded networks..."));
                        bool foundAnyHardcoded = false;
                        for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
                            if (hardcodedSSIDAvailable(hardcodedNetworks[i].ssid)) {
                                foundAnyHardcoded = true;
                                currentHardcodedNetworkIndex = i;
                                break;
                            }
                        }
                        if (foundAnyHardcoded) {
                            Serial.printf_P(PSTR("Found hardcoded SSID: %s\n"), hardcodedNetworks[currentHardcodedNetworkIndex].ssid);
                            tryingHardcodedNetworks = true;
                            failedSavedStaAttempts = 0;
                            WiFi.begin(hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                                       hardcodedNetworks[currentHardcodedNetworkIndex].password);
                            hardcodedConnectAttemptStart = millis();
                        } else {
                            Serial.println(F("No hardcoded SSIDs found in scan. Continuing STA retry loop."));
                            failedSavedStaAttempts = 0;
                        }
                    }
                }
            } else { // Trying hardcoded networks
                if (millis() - hardcodedConnectAttemptStart > HARDCODED_CONNECT_TIMEOUT_MS) {
                    Serial.printf_P(PSTR("Failed to connect to %s. Moving to next hardcoded network.\n"),
                                    hardcodedNetworks[currentHardcodedNetworkIndex].ssid);
                    currentHardcodedNetworkIndex++;

                    while (currentHardcodedNetworkIndex < NUM_HARDCODED_NETWORKS &&
                           !hardcodedSSIDAvailable(hardcodedNetworks[currentHardcodedNetworkIndex].ssid)) {
                        currentHardcodedNetworkIndex++;
                    }

                    if (currentHardcodedNetworkIndex < NUM_HARDCODED_NETWORKS) {
                        Serial.printf_P(PSTR("Trying hardcoded SSID: %s\n"), hardcodedNetworks[currentHardcodedNetworkIndex].ssid);
                        WiFi.begin(hardcodedNetworks[currentHardcodedNetworkIndex].ssid,
                                   hardcodedNetworks[currentHardcodedNetworkIndex].password);
                        hardcodedConnectAttemptStart = millis();
                    } else {
                        Serial.println(F("All available hardcoded networks failed. Starting AP mode."));
                        tryingHardcodedNetworks = false;
                        currentHardcodedNetworkIndex = 0;
                        setupAP();
                        isApModeActive = true;
                        if (serverRunning) stopWebServer();
                    }
                }
            }
        } else { // Connected
            failedSavedStaAttempts = 0;
            tryingHardcodedNetworks = false;
            currentHardcodedNetworkIndex = 0;
            hardcodedConnectAttemptStart = 0;

           if (!wifiConnected) {
    Serial.println(F("WiFi connected!"));
    Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
    Serial.print(F("MAC: ")); Serial.println(WiFi.macAddress());
    wifiConnected = true;
    isApModeActive = false;

    // ðŸ“¢ Request Telegram send before starting HTTP server
    telegramSendPending = true;  // Delay flag for loop()
    telegramDelayActive = false; // Reset in case of previous run

    // Save new credentials if connected via hardcoded
    if (strcmp(settings.staSsid, WiFi.SSID().c_str()) != 0) {
        Serial.println(F("Saving new connected SSID..."));
        strncpy(settings.staSsid, WiFi.SSID().c_str(), sizeof(settings.staSsid) - 1);
        settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';
        for (size_t i = 0; i < NUM_HARDCODED_NETWORKS; i++) {
            if (strcmp(hardcodedNetworks[i].ssid, WiFi.SSID().c_str()) == 0) {
                strncpy(settings.staPassword, hardcodedNetworks[i].password, sizeof(settings.staPassword) - 1);
                settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';
                break;
            }
        }
        saveSettings();
    }

    // Skip starting the server here â€” wait until Telegram send completes
// MDNS.end();

if (lastTimeSync == 0 || (millis() - lastTimeSync > 300000)) {
    syncTimeFromNTP();
    lastTimeSync = millis();
}


}

            timeClient.update();
        }
    }
    else if (WiFi.getMode() == WIFI_AP) {
        if (!serverRunning) {
            startWebServer();
            Serial.println(F("HTTP server started in AP mode."));
        }
        checkAPTimeout();
    }
}
*/


// Your loop and other functions are generally fine for this problem.
// manageWiFiConnection will continue to handle reconnections if STA drops.
// --- Loop Function ---
void loop() {
  lastLoopMillis = millis();
  ESP.wdtFeed();

  // --- Emergency Button Edge Detection (no ISR, fully WDT-safe) ---
  static uint32_t lastEmergencyCheck = 0;
  if (millis() - lastEmergencyCheck > 10) {  // Debounce every 10ms
    lastEmergencyCheck = millis();

    static bool lastEmergencyState = HIGH;
    bool currentEmergencyState = digitalRead(EMERGENCY_PIN);

    if (lastEmergencyState == HIGH && currentEmergencyState == LOW) {
      Serial.println(F("Main Loop: Emergency Button Pressed detected. Activating Emergency Stop."));
      logFailsafeEvent("Button:EmergencyStop");
      emergencyStopInternal();
    }

    if (lastEmergencyState == LOW && currentEmergencyState == HIGH) {
      Serial.println(F("Main Loop: Emergency Button Released detected. Clearing Failsafe/Emergency Stop."));
      clearEmergencyStopAndFailsafe();
    }

    lastEmergencyState = currentEmergencyState;
  }
  if (pendingFailsafeReason[0]) {
  ESP.wdtFeed();  // Extra safety
  logFailsafeEvent(pendingFailsafeReason);
  pendingFailsafeReason[0] = 0;
  ESP.wdtFeed();
}


  ESP.wdtFeed();
  checkSwitchPress();           // Debounced toggle switch handler
  ESP.wdtFeed();
  manageWiFiConnection();       // Handles WiFi + AP fallback
  ESP.wdtFeed();

   if (pendingAPSetup) {
    pendingAPSetup = false;
    setupAP(); // Now called safely from the main loop
    Serial.println("AP mode setup completed from main loop.");
  }


  // --- Handle safe restart ---
  if (pendingRestart) {
    pendingRestart = false;
    ESP.wdtFeed();
    safeRestart();              // Safe function to reboot the device
  }
  if (shouldBroadcastAfterEmergencyClear) {
  shouldBroadcastAfterEmergencyClear = false;
  Serial.println(F("ðŸ” Broadcasting status after emergency clear..."));
  broadcastStatus();  // Now it's safe
  ESP.wdtFeed();
}

if (shouldLogEmergencyClear) {
  shouldLogEmergencyClear = false;
  Serial.println(F("ðŸ“ Logging emergency clear event..."));
  logFailsafeEvent("Emergency Stop Cleared");
  ESP.wdtFeed();
}


  if (watchdogFeedRequested) {
    watchdogFeedRequested = false;
    ESP.wdtFeed();
  }
  
  if (dnsServerActive) {
    dnsServer.processNextRequest();
  }

 // --- Telegram send delay handling ---
 
 /*

if (telegramDelayActive && millis() - telegramSendStartTime >= 3000) {
  telegramDelayActive = false;
 // secureClient.setInsecure();

  Serial.println("Connecting to Telegram...");
  if (!secureClient.connect("api.telegram.org", 443)) {
    Serial.println("❌ Failed to connect to Telegram server");
    return;
  }

  String ipMsg = "ESP8266 Connected!\nIP Address: " + WiFi.localIP().toString();
  Serial.println(" Sending message to Telegram...");

  bool success = bot.sendMessage(CHAT_ID, ipMsg, "");
  if (success) {
    Serial.println("✅ Telegram message sent successfully");
  } else {
    Serial.println("❌ Failed to send Telegram message");
  }



    // Only start the server after Telegram is sent
    if (!serverRunning) {
        startWebServer();
        Serial.println(F("HTTP server started after Telegram send."));
    }
    
}

*/
/*
if (telegramSendPending && !telegramDelayActive) {
    telegramSendPending = false;
    telegramDelayActive = true;
    telegramSendStartTime = millis();
    Serial.println(F("â�� Telegram send delay started..."));
    if (!secureClient.connect("api.telegram.org", 443)) {
    Serial.println("❌ Failed to connect to Telegram server");
    return;
  }
}

if (telegramDelayActive && millis() - telegramSendStartTime >= 3000) {
    telegramDelayActive = false;

   secureClient.setInsecure();

    String ipMsg = "ESP8266 Connected!\nIP Address: " + WiFi.localIP().toString();
    if (bot.sendMessage(CHAT_ID, ipMsg, "")) {
        Serial.println(F("âœ… Telegram message sent successfully"));
    } else {
        Serial.println(F("âŒ Failed to send Telegram message"));
    }

    // Only start the server after Telegram is sent
    if (!serverRunning) {
        startWebServer();
        Serial.println(F("HTTP server started after Telegram send."));
    }
}
*/

/*
if (telegramSendPending && !telegramDelayActive) {
    telegramSendPending = false;
    telegramDelayActive = true;
    telegramSendStartTime = millis();
    Serial.println(F("⏳ Telegram send delay started..."));
}

if (telegramDelayActive && millis() - telegramSendStartTime >= 3000) {
    telegramDelayActive = false;

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("❌ WiFi lost before Telegram send"));
        telegramSendPending = true;   // retry later
        return;
    }

    String ipMsg = "ESP8266 Connected!\nIP Address: " + WiFi.localIP().toString();

    if (bot.sendMessage(CHAT_ID, ipMsg, "")) {
        Serial.println(F("✅ Telegram message sent successfully"));

        // Start server ONLY after success
        if (!serverRunning) {
            startWebServer();
            Serial.println(F("HTTP server started after Telegram send."));
        }

    } else {
        Serial.println(F("❌ Telegram failed — will retry"));
        telegramSendPending = true;   // retry
    }
}
//}

*/

/*
if (telegramSendPending && !telegramDelayActive) {
    telegramSendPending = false;
    telegramDelayActive = true;
    telegramSendStartTime = millis();
    Serial.println("⏳ Telegram send delay started...");
  }

  if (telegramDelayActive && millis() - telegramSendStartTime >= 3000) {
    telegramDelayActive = false;

    String msg = "ESP8266 Connected!\nIP: " + WiFi.localIP().toString();
    if (bot.sendMessage(CHAT_ID, msg, "")) {
      Serial.println("✅ Telegram message sent successfully");
    } else {
      Serial.println("❌ Failed to send Telegram message");
    }

    if (!serverRunning) {
      startWebServer();
    }
  }

*/
/*
if (telegramSendPending && !telegramDelayActive) {
    telegramSendPending = false;
    telegramDelayActive = true;
    telegramSendStartTime = millis();
}

if (telegramDelayActive && millis() - telegramSendStartTime >= 4000) {
    telegramDelayActive = false;

    if (WiFi.status() != WL_CONNECTED) {
        telegramSendPending = true;
        return;
    }

    String ipMsg = "ESP8266 Connected!\nIP: " + WiFi.localIP().toString();

    ESP.wdtFeed();
    bool ok = bot.sendMessage(CHAT_ID, ipMsg, "");
    ESP.wdtFeed();

    if (ok) {
        if (!serverRunning) startWebServer();
    } else {
        telegramSendPending = true;  // retry
    }
}
*/
if (telegramSendPending && !telegramDelayActive && !telegramIpSent) {
    telegramSendPending = false;
    telegramDelayActive = true;
    telegramSendStartTime = millis();
}

if (telegramDelayActive && millis() - telegramSendStartTime >= 4000) {
    telegramDelayActive = false;

    if (WiFi.status() != WL_CONNECTED) {
        telegramSendPending = true;
        return;
    }

    String ipMsg = "ESP8266 Connected!\nIP: " + WiFi.localIP().toString();

    ESP.wdtFeed();
    bool ok = bot.sendMessage(CHAT_ID, ipMsg, "");
    ESP.wdtFeed();

    if (ok) {
        telegramIpSent = true;   // ✅ BLOCK FUTURE SENDS
        if (!serverRunning) startWebServer();
    } else {
        telegramSendPending = true;  // retry
    }
}


  // --- Handle deep sleep request ---
  static bool once = false;
if (!once) {
  once = true;
  Serial.printf("Loop start: deepSleepRequested=%d, remaining=%u\n",
                deepSleepRequested,
                rtcData.remainingSleepSeconds);
}
  if (deepSleepRequested) {
  if (sleepRequestTime == 0) sleepRequestTime = millis();

  if (millis() - sleepRequestTime > 300) {  // Wait 300ms before sleep
    Serial.println(F("Preparing for deep sleep..."));
    ESP.wdtFeed();

    // Detach tickers
    failsafeTicker.detach();        ESP.wdtFeed();
    watchdogTicker.detach();        ESP.wdtFeed();
    relayTicker.detach();           ESP.wdtFeed();
    logCleanupTicker.detach();      ESP.wdtFeed();
    statusTicker.detach();          ESP.wdtFeed();
    wifiScanTicker.detach();        ESP.wdtFeed();
    autoDeepSleepDelayTicker.detach(); ESP.wdtFeed();
    deferredServerDelete.detach();  ESP.wdtFeed();

    stopWebServer();
    yield(); ESP.wdtFeed();

    WiFi.disconnect(true);
    delay(100);
    ESP.wdtFeed();
    Serial.printf("settings.deepSleepDurationSeconds = %u\n",
              settings.deepSleepDurationSeconds);

    /* ===============================
       🔑 CHUNKED SLEEP LOGIC (FIXED)
       =============================== */

    uint32_t sleepNow = min(rtcData.remainingSleepSeconds, (uint32_t)3600); // 1 hour

    // 🚨 CRITICAL GUARD
    if (sleepNow == 0) {
      Serial.println(F("Sleep cycle complete — not entering deep sleep"));
      clearRTC();
      deepSleepRequested = false;
      return;   // ✅ YES — return is CORRECT here
    }

    rtcData.remainingSleepSeconds -= sleepNow;
    saveRTC();

    Serial.printf("Sleeping %u seconds, remaining %u\n",
                  sleepNow, rtcData.remainingSleepSeconds);
                  Serial.println(F("😴 ENTERING DEEP SLEEP"));
Serial.printf("SleepNow = %u seconds\n", sleepNow);
Serial.printf("RTC.remainingSleepSeconds (saved) = %u\n",
              rtcData.remainingSleepSeconds);

    ESP.deepSleep(sleepNow * 1000000UL);
  }
}


  // --- Ongoing maintenance tasks ---
  //ElegantOTA.loop();   // Non-blocking OTA handler
  ESP.wdtFeed();

 // MDNS.update();       // mDNS responder update
  checkAPTimeout();    // AP timeout check
  ESP.wdtFeed();

  // Captive portal DNS request handler
  if (WiFi.getMode() == WIFI_AP && dnsServerActive) {
    dnsServer.processNextRequest();
    ESP.wdtFeed();
  }
  

  // Periodic NTP sync
  if (WiFi.status() == WL_CONNECTED && (millis() - lastTimeSync > 6UL * 60UL * 60UL * 1000UL)) {
    syncTimeFromNTP();
    lastTimeSync = millis();
    ESP.wdtFeed();
  }

  // Handle WiFi scanning logic (only once after connection)
  processScanResults();  // Always check if scan finished
  ESP.wdtFeed();

  static bool scanAfterConnectionDone = false;
  if (!scanAfterConnectionDone && WiFi.status() == WL_CONNECTED) {
    if (!scanInProgress) {
      Serial.println(F("ðŸ” Starting post-connection WiFi scan (once)..."));
      startNonBlockingWiFiScan();
      scanAfterConnectionDone = true;
    }
  }

  // Optionally reset flag if WiFi disconnects
  if (WiFi.status() != WL_CONNECTED) {
    scanAfterConnectionDone = false;  // Allow future scan on reconnect
  }
    // Deferred LittleFS operations
//  if (shouldLogRelayTime) {
//    shouldLogRelayTime = false;
//    logRelayRunTime(relayLogStart, relayLogEnd);
//    ESP.wdtFeed();
//  }

  if (shouldSaveSettingsFlag) {
    shouldSaveSettingsFlag = false;
    saveSettings();
    ESP.wdtFeed();
  }

// --- Handle pending timed relay OFF ---
if (relayTimedOffPending) {
  relayTimedOffPending = false;

  if (relayState) {
    digitalWrite(settings.relayPin, HIGH); // Active-low OFF
    relayState = false;

    if (relayTicker.active()) relayTicker.detach();
    if (failsafeTicker.active()) failsafeTicker.detach();

    unsigned long endTime = millis();
    unsigned long actualDuration = (endTime - relayOnStartTime) / 1000UL;
    Serial.printf_P(PSTR("Timed Relay OFF after %lu seconds.\n"), actualDuration);

    // ✅ Skip flash write for temporary runs
    if (!isTemporaryRun) {
      logRelayRunTime();
      saveSettings();
    } else {
      Serial.println(F("Skipping flash write (temporary run)."));
      isTemporaryRun = false;
    }

    broadcastStatus();

    // ---- Auto Deep Sleep scheduling after run completion ----
    if (settings.autoDeepSleepEnabled && !autoDeepSleepScheduled && !failsafeActive && !emergencyStop) {
      // If temporary run was long enough (≥600s) OR any normal timed run
      if (!isTemporaryRun || actualDuration >= 600) {

        // ⏳ Set delay before entering deep sleep (10 minutes)
        const unsigned long DEEP_SLEEP_DELAY_MS = 10UL * 60UL * 1000UL; // 10 minutes

        Serial.printf_P(PSTR("Run completed. Scheduling auto deep sleep in %lu minutes.\n"),
                        DEEP_SLEEP_DELAY_MS / (60 * 1000));

        if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();

        // Schedule deep sleep after 10 minutes
        autoDeepSleepDelayTicker.once_ms(DEEP_SLEEP_DELAY_MS, initiateAutoDeepSleep);
        autoDeepSleepScheduled = true;
      } else {
        Serial.println(F("Temporary run <600s — skipping deep sleep."));
      }
    } else {
      Serial.println(F("Auto deep sleep not scheduled (disabled, already active, or failsafe/emergency)."));
    }
  }
}

/*
if (relayTimedOffPending) {
  relayTimedOffPending = false;

  if (relayState) {
    digitalWrite(settings.relayPin, HIGH); // Active-low OFF
    relayState = false;

    if (relayTicker.active()) relayTicker.detach();
    if (failsafeTicker.active()) failsafeTicker.detach();

    unsigned long endTime = millis();
    unsigned long actualDuration = (endTime - relayOnStartTime) / 1000UL;
    Serial.printf_P(PSTR("Timed Relay OFF after %lu seconds.\n"), actualDuration);

    // ✅ Skip flash write for temporary runs
    if (!isTemporaryRun) {
      logRelayRunTime();
      saveSettings();
    } else {
      Serial.println(F("Skipping flash write (temporary run)."));
      isTemporaryRun = false;
    }

    broadcastStatus();

    // ---- Auto Deep Sleep scheduling after run completion ----
    if (settings.autoDeepSleepEnabled && !autoDeepSleepScheduled && !failsafeActive && !emergencyStop) {
      // If temporary run was long enough (≥600s) OR any normal timed run
      if (!isTemporaryRun || actualDuration >= 600) {
        Serial.printf_P(PSTR("Run completed. Scheduling auto deep sleep in %lu minutes.\n"),
                        AUTO_DEEP_SLEEP_DELAY_MS / (60 * 1000));

        if (autoDeepSleepDelayTicker.active()) autoDeepSleepDelayTicker.detach();

        autoDeepSleepDelayTicker.once_ms(AUTO_DEEP_SLEEP_DELAY_MS, initiateAutoDeepSleep);
        autoDeepSleepScheduled = true;
      } else {
        Serial.println(F("Temporary run <600s — skipping deep sleep."));
      }
    } else {
      Serial.println(F("Auto deep sleep not scheduled (disabled, already active, or failsafe/emergency)."));
    }
  }
}
*/

/*
if (relayTimedOffPending) {
  relayTimedOffPending = false;

  if (relayState) {
    digitalWrite(settings.relayPin, HIGH); // Active-low OFF
    relayState = false;
    relayTicker.detach();
    if (failsafeTicker.active()) failsafeTicker.detach();

    unsigned long endTime = millis();
    unsigned long actualDuration = (endTime - relayOnStartTime) / 1000UL;
    Serial.printf_P(PSTR("Timed Relay OFF after %lu seconds.\n"), actualDuration);

    // ✅ Skip flash logging/saving for temporary runs
    if (!isTemporaryRun) {
      logRelayRunTime();
      saveSettings();
    } else {
      Serial.println(F("Skipping flash write (temporary run)."));
      isTemporaryRun = false;
    }

    broadcastStatus();

    if (settings.autoDeepSleepEnabled && !autoDeepSleepScheduled && !failsafeActive && !emergencyStop) {
      Serial.printf_P(PSTR("Timed relay run completed. Scheduling auto deep sleep in %lu minutes.\n"),
                      AUTO_DEEP_SLEEP_DELAY_MS / (60 * 1000));
      autoDeepSleepDelayTicker.once_ms(AUTO_DEEP_SLEEP_DELAY_MS, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
    } else {
      Serial.println(F("Auto deep sleep not scheduled (disabled, already scheduled, or failsafe/emergency active)."));
    }
  }
}

*/


  /*
  if (relayTimedOffPending) {
  relayTimedOffPending = false;

  if (relayState) {
    digitalWrite(settings.relayPin, HIGH); // Active-low
    relayState = false;
    relayTicker.detach();

    unsigned long endTime = millis();
    Serial.printf_P(PSTR("Timed Relay OFF after %lu seconds.\n"), relayDuration);
    logRelayRunTime(relayOnStartTime, endTime);
    Serial.printf("Debug: relayOnStartTime = %lu, now = %lu\n", relayOnStartTime, millis());


    settings.wasTimedRunInterrupted = false;
    settings.interruptedRunDuration = 0;
    settings.interruptedRunStartTime = 0;
    saveSettings(); // Safe here
    broadcastStatus();

    

    if (settings.autoDeepSleepEnabled && !autoDeepSleepScheduled && !failsafeActive && !emergencyStop) {
      Serial.printf_P(PSTR("Timed relay run completed. Scheduling auto deep sleep in %lu minutes.\n"), AUTO_DEEP_SLEEP_DELAY_MS / (60 * 1000));
      autoDeepSleepDelayTicker.once_ms(AUTO_DEEP_SLEEP_DELAY_MS, initiateAutoDeepSleep);
      autoDeepSleepScheduled = true;
    } else {
      Serial.println(F("Auto deep sleep not scheduled (disabled, already scheduled, or failsafe/emergency active)."));
    }
  }
}
*/

}




void safeRestart() {
  Serial.println(F("Graceful restart initiated..."));

  if (serverRunning) {
    stopWebServer(); // Call the refined stopWebServer to handle dynamic allocation
  } else {
    Serial.println(F("Web server was not running."));
  }

  // Detach all Tickers to prevent them from firing during the restart
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
  
//  if (MDNS.isRunning()) {
//    MDNS.end();
//    Serial.println(F("MDNS stopped."));
//  }
  
  // Stop the DNS server if it was active
  dnsServer.stop();
  Serial.println(F("DNS server stopped."));

  // Disconnect from WiFi
  WiFi.disconnect(true);
  Serial.println(F("WiFi STA disconnected."));

  // Stop the SoftAP
  WiFi.softAPdisconnect(true);
  Serial.println(F("SoftAP stopped."));

  // Turn off WiFi completely
  WiFi.mode(WIFI_OFF);
  delay(100);
  Serial.println(F("WiFi interface turned off."));

  // Final wait before the restart
  delay(500); // A longer delay to ensure all cleanup is done
  Serial.println(F("Restarting now..."));
  ESP.restart();
}




void setupWatchdog() {
  // Attach feedWatchdog function to the ticker to run every 5 seconds
  // This will feed the hardware watchdog to prevent system resets
  watchdogTicker.attach_ms(5000, feedWatchdog);
#ifdef DEBUG_MODE
  Serial.println(F("Hardware Watchdog initialized. Feeding every 5 seconds."));
// Use F() macro
#endif
}

// New function to implement the software watchdog check
void checkLoopWatchdog() {
  if (millis() - lastLoopMillis > 10000) { // If loop hasn't updated for 10 seconds
    Serial.println(F("Software watchdog triggered! Loop appears stuck. Rebooting..."));
// Use F() macro
    safeRestart();
// Force a restart
  }
}

// --- Failsafe Implementation ---
void failsafeTriggered() {
#ifdef DEBUG_MODE
  Serial.println(F("Failsafe Triggered: Max run time exceeded!"));
// Use F() macro
  Serial.printf_P(PSTR("DEBUG: Setting relay pin D%d to HIGH (OFF) from failsafeTriggered\n"), settings.relayPin);
#endif
  failsafeActive = true;
  digitalWrite(settings.relayPin, HIGH);
  // Inverted: HIGH to turn OFF active-low relay
  relayState = false;
  logFailsafeEvent("Duration exceeded");
  failsafeTicker.detach(); // Stop the failsafe ticker to prevent re-triggering
  settings.wasTimedRunInterrupted = false;
// Failsafe is an intentional shutdown due to condition
  saveSettings();
// Save immediately to clear the recovery flag
  broadcastStatus(); // Update clients
  
  // USER CHANGE: Removed auto deep sleep scheduling from failsafeTriggered
  // if (settings.autoDeepSleepEnabled && !autoDeepSleepScheduled) {
  //   Serial.printf_P(PSTR("Failsafe triggered. Scheduling auto deep sleep in %lu minutes.\n"), AUTO_DEEP_SLEEP_DELAY_MS / (60 * 1000));
  //   autoDeepSleepDelayTicker.once_ms(AUTO_DEEP_SLEEP_DELAY_MS, initiateAutoDeepSleep);
  //   autoDeepSleepScheduled = true;
  // }
}

void resetFailsafe() {
#ifdef DEBUG_MODE
  Serial.println(F("Failsafe Reset."));
#endif

  // Clear all failsafe-related flags and actions
  failsafeActive = false;
  emergencyStop = false;
  failsafeTicker.detach();
// --- Ensure relay is physically OFF (failsafe-safe guarantee) ---
  if (relayState) {
    digitalWrite(settings.relayPin, HIGH);
// Turn OFF relay (active-low)
    relayState = false;
  }
/*
  // If failsafe is enabled and has a duration set, re-arm it
  if (settings.failsafeEnabled && settings.maxRelayRuntime > 0) {
    failsafeTicker.attach(settings.failsafeDurationHours * 3600, failsafeTriggered);
#ifdef DEBUG_MODE
    Serial.printf_P(PSTR("Failsafe re-attached for %lu hours.\n"), settings.failsafeDurationHours);
#endif
  }
  */

  broadcastStatus();
// Update any connected clients
}


// New function to initiate auto deep sleep
/*
void initiateAutoDeepSleep() {
  Serial.println(F("Auto deep sleep initiated."));
  deepSleepRequested = true; // Set the flag to trigger deep sleep in loop()
  autoDeepSleepScheduled = false; // Reset the schedule flag
}
*/
/*void initiateAutoDeepSleep() {
  Serial.println(F("Auto deep sleep initiated."));
  Serial.println(F("Setting deepSleepRequested = true"));
  deepSleepRequested = true;
  sleepRequestTime = 0;          // 🔴 THIS IS IMPORTANT
  autoDeepSleepScheduled = false;
}
*/
/*
void initiateAutoDeepSleep() {
  Serial.println(F("Auto deep sleep initiated."));
  Serial.println(F("Setting deepSleepRequested = true"));

  if (remainingSleepSeconds == 0) {
    remainingSleepSeconds = settings.deepSleepDurationSeconds;
  }

  deepSleepRequested = true;
  sleepRequestTime = 0;
  autoDeepSleepScheduled = false;
}
*/
/*void initiateAutoDeepSleep() {
  Serial.println(F("Auto deep sleep initiated."));
  deepSleepRequested = true;
  sleepRequestTime = 0;
  autoDeepSleepScheduled = false;

  // 🔑 Initialize RTC-backed sleep only once
  if (rtcData.remainingSleepSeconds == 0) {
    rtcData.remainingSleepSeconds = settings.deepSleepDurationSeconds;
    saveRTC();  // ✅ CRITICAL
    Serial.printf("Total sleep requested: %u seconds\n",
                  rtcData.remainingSleepSeconds);
  }
}*/
/*void initiateAutoDeepSleep() {
  Serial.println(F("Auto deep sleep initiated."));

  deepSleepRequested = true;
  sleepRequestTime = 0;
  autoDeepSleepScheduled = false;

  // 🔑 ALWAYS initialize RTC when starting sleep
  rtcData.remainingSleepSeconds = settings.deepSleepDurationSeconds;
  saveRTC();

  Serial.printf("Sleep initialized: %u seconds\n",
                rtcData.remainingSleepSeconds);
}
*/
void initiateAutoDeepSleep() {
  Serial.println(F("Auto deep sleep initiated."));

  deepSleepRequested = true;
  sleepRequestTime = 0;
  autoDeepSleepScheduled = false;

  // 🔑 Initialize ONLY if not already sleeping
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

// --- Authentication ---
void getMD5Hash_P(const __FlashStringHelper* flashStr, char* output, size_t outputSize) {
  if (outputSize < 33) return;
// MD5 hex string = 32 chars + null

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
// Use getBytes to avoid String

  for (uint8_t i = 0; i < 16; i++) {
    sprintf(output + (i * 2), "%02x", digest[i]);
  }
  output[32] = '\0';
}




void generateSessionToken() {
  static const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  const size_t charsetLength = sizeof(charset) - 1; // Exclude null terminator
  for (uint8_t i = 0; i < 32; i++) {
    currentSessionToken[i] = charset[random(charsetLength)];
  }
  currentSessionToken[32] = '\0'; // Null-terminate
  sessionStart = millis(); // Start session timer
#ifdef DEBUG_MODE
  Serial.print(F("New sessionToken: "));
  Serial.println(currentSessionToken);
#endif
}

bool authenticate(const char* user, const char* pass) {
  char hash[33];
// 32 for MD5 + null terminator
  // Hash the provided password
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

  // Avoid String and c_str(): use raw buffer
  char cookieBuffer[128] = {0}; // Adjust size based on expected cookie length
  size_t len = cookieHdr->value().length();
  if (len >= sizeof(cookieBuffer)) len = sizeof(cookieBuffer) - 1;

  // Copy raw cookie header into buffer without String methods
  for (size_t i = 0; i < len; ++i) {
    cookieBuffer[i] = cookieHdr->value()[i];
  }
  cookieBuffer[len] = '\0'; // Null-terminate

  // Parse manually to find authToken
  const char* tokenStart = strstr(cookieBuffer, "authToken=");
  if (!tokenStart) {
#ifdef DEBUG_MODE
    Serial.println(F("isLoggedIn: No authToken cookie found."));
#endif
    return false;
  }
  tokenStart += strlen("authToken=");

  char receivedToken[33] = {0}; // 32-char token max + null
  size_t i = 0;
  while (*tokenStart && *tokenStart != ';' && i < sizeof(receivedToken) - 1) {
    receivedToken[i++] = *tokenStart++;
  }
  receivedToken[i] = '\0';

  // Validate token
  if (strcmp(receivedToken, currentSessionToken) != 0) {
#ifdef DEBUG_MODE
    Serial.printf("isLoggedIn: Token mismatch. Received: %s, Expected: %s\n", receivedToken, currentSessionToken);
#endif
    return false;
  }

  // Check timeout
  if (millis() - sessionStart > sessionTimeout) {
    currentSessionToken[0] = '\0';
// Invalidate
#ifdef DEBUG_MODE
    Serial.println(F("isLoggedIn: Session expired."));
#endif
    return false;
  }

  sessionStart = millis();
// Refresh session
#ifdef DEBUG_MODE
  Serial.println(F("isLoggedIn: Session valid."));
#endif
  return true;
}

// --- Setup Functions ---
void applyDefaultSettings() {
  // WiFi STA settings
  strncpy(settings.staSsid, hardcodedNetworks[0].ssid, sizeof(settings.staSsid) - 1); // Use first hardcoded
  settings.staSsid[sizeof(settings.staSsid) - 1] = '\0';
  strncpy(settings.staPassword, hardcodedNetworks[0].password, sizeof(settings.staPassword) - 1); // Use first hardcoded
  settings.staPassword[sizeof(settings.staPassword) - 1] = '\0';

  strncpy(settings.hostname, "aquamaster", sizeof(settings.hostname) - 1);
  settings.hostname[sizeof(settings.hostname) - 1] = '\0';
  settings.useDHCP = true;
  settings.staticIp.fromString("192.168.1.100");
  settings.staticGateway.fromString("192.168.1.1");
  settings.staticSubnet.fromString("255.255.255.0");
  settings.staticDns1.fromString("8.8.8.8");
  settings.staticDns2.fromString("8.8.4.4");
// WiFi AP settings
  settings.wifiAPModeEnabled = false;
  strncpy(settings.apSsid, "AquaMaster_AP", sizeof(settings.apSsid) - 1);
  settings.apSsid[sizeof(settings.apSsid) - 1] = '\0';
  strncpy(settings.apPassword, "aquapass", sizeof(settings.apPassword) - 1);
  settings.apPassword[sizeof(settings.apPassword) - 1] = '\0';
// Admin credentials
  strncpy(settings.adminUsername, "admin", sizeof(settings.adminUsername) - 1);
  settings.adminUsername[sizeof(settings.adminUsername) - 1] = '\0';
  char hash[33];
  getMD5Hash_P(F("admin"), hash, sizeof(hash));
  strncpy(settings.adminPasswordHash, hash, sizeof(settings.adminPasswordHash));
  settings.adminPasswordHash[sizeof(settings.adminPasswordHash) - 1] = '\0';
// Relay and Failsafe settings
  settings.calibrationFactor = 1.0;
  settings.failsafeDurationHours = 12;
  settings.failsafeEnabled = true;
  settings.failsafePin = D5; // Changed to D0 as requested
  settings.relayPin = D1; // Ensure this matches your hardware
  settings.defaultRelayDuration = 720;
// 15 minutes in seconds
  settings.maxRelayRuntime = 960; // 16 minutes in seconds
// Run time interruption tracking
  settings.wasTimedRunInterrupted = false;
  settings.interruptedRunDuration = 0;
  settings.interruptedRunStartTime = 0;
// System status/session
  settings.bootUnixTime = 0;
  currentSessionToken[0] = '\0';
// Clear current session token - Re-added
  loggedIn = false; // Re-added
// NEW: Deep Sleep Settings
  settings.deepSleepDurationSeconds = 15 * 3600; // Default to 3 hours (10800 seconds) for deep sleep
  settings.autoDeepSleepEnabled = true; // Default to enabled 
}

void setupLittleFS() {
  // This function is now mostly superseded by the LittleFS.begin() check in setup()
  // and the subsequent actions in case of failure.
  // Keeping it for clarity, but the main logic is in setup().
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
// Function to initialize WiFi connection (STA mode or AP mode)
void initWiFi() {
  Serial.println(F("Initializing WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.hostname(settings.hostname);

  // Clear previous connection attempts (optional, but good for clean start)
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
    lastWifiConnectAttemptMillis = millis(); // Record when connection attempt started
    wifiConnected = false; // Assume not connected until confirmed by manageWiFiConnection
  } else {
    Serial.println(F("No STA SSID configured. Starting in AP mode."));
    setupAP(); // Fallback to AP immediately if no STA SSID configured
    isApModeActive = true; // Set flag to indicate AP mode
  }
  // No blocking loops or delays here. Return immediately.
}




void startWebServer() {
  if (serverRunning){
#ifdef DEBUG_MODE
    Serial.println(F("Web server handlers already attached. Skipping start."));
#endif
    return; // Already running
  }
  
  // NOTE: The server object itself is already created globally, we only
  // need to attach the handlers and start accepting connections.
  
  // Start ElegantOTA
 // ElegantOTA.begin(&server); // Pass the address of the global object
#ifdef DEBUG_MODE
  Serial.println(F("ElegantOTA initialized."));
#endif

  // Attach handlers (All 'server->' become 'server.')
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
    
  // Handlers with inline login check
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
  //server.on("/softShutdown", HTTP_POST, handleSoftShutdown);
  server.on("/setDeepSleepDuration", HTTP_POST, handleSetDeepSleepDuration);
  server.on("/deepSleep", HTTP_POST, handleDeepSleep);
  // (A) Register the upload callback
server.onFileUpload(handleFileUpload);

// (B) Define the GET route for the upload form (optional, for browser UI)
server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(200, "text/html",
    "<form method='POST' action='/upload' enctype='multipart/form-data'>"
    "<input type='file' name='upload'>"
    "<input type='submit' value='Upload'>"
    "</form>");
});

// (C) Define the POST route (this is what triggers the upload handler)
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
    
  // AsyncCallbackJsonWebHandler
  server.addHandler(new AsyncCallbackJsonWebHandler("/saveSettings", [](AsyncWebServerRequest *request, JsonVariant &json) {
    if (!isLoggedIn(request)) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }
    StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
    doc.set(json); // Copy the received JSON

    Serial.println(F("ðŸ“¥ Received settings JSON:"));
    serializeJsonPretty(doc, Serial);
    Serial.println();

    // Parse fields safely
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

    // Optional fields with type checks
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

  // Serve compressed index.html.gz manually
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/index.html.gz")) {
      AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/index.html.gz", "text/html");
      response->addHeader("Content-Encoding", "gzip");
      request->send(response);
    } else {
      request->send(500, "text/plain", "index.html.gz not found");
    }
  });
    
  // Serve file uploads
 // server.onFileUpload(handleFileUpload);

  // SSE client connections
  events.onConnect([](AsyncEventSourceClient *client) {
    client->send("connected", "event");
#ifdef DEBUG_MODE
    Serial.println(F("SSE Client Connected!"));
#endif
  });
  server.addHandler(&events);
  // Android captive portal URLs
// Android
/*
server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
server.on("/gen_204", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
*/
server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest *request) {
  request->send(204);
});
server.on("/gen_204", HTTP_ANY, [](AsyncWebServerRequest *request) {
  request->send(204);
});

// iOS/macOS
server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
server.on("/library/test/success.html", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});

// Windows
server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});
server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->redirect("/index.html");
});


server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(LittleFS, "/index.html.gz", "text/html");
});


  
  // Captive portal catch-all handler (must be last!)
server.onNotFound([](AsyncWebServerRequest *request) {
  // Check if we are in AP mode and not connected to a network.
  // The 'isLoggedIn' check has been removed.
  if (!WiFi.isConnected()) {
    // Correctly serve the gzipped file with the proper header
    AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/index.html.gz", "text/html");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  } else {
    // If in STA mode and not found, return 404
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

  // Step 1: Stop SSE cleanly
  events.close();                // Close all SSE client connections
  events.onConnect(nullptr);    // Detach SSE connect callback
#ifdef DEBUG_MODE
  Serial.println(F("SSE connections closed."));
#endif

  // Step 2: Stop ElegantOTA
  //ElegantOTA.end();  // This detaches OTA handlers
#ifdef DEBUG_MODE
  Serial.println(F("ElegantOTA ended."));
#endif

  // Step 3: Gracefully stop the web server
  server.end();  // Stop new requests
#ifdef DEBUG_MODE
  Serial.println(F("AsyncWebServer end() called."));
#endif
  
  // Note: We don't need to delete the server object because it's global.
  // We can't actually detach the other handlers without restarting the ESP.
  // This is a known limitation of AsyncWebServer.

  serverRunning = false;

#ifdef DEBUG_MODE
  Serial.println(F("Web server shutdown sequence completed."));
#endif
}

/*

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
    size_t len = logFile.readBytes(buffer, LOG_ENTRY_LENGTH);
    buffer[len] = '\0';  // Ensure null-termination

    if (len == 0) {
      Serial.printf("ðŸ“› Empty log at index %d (seekPos=%u)\n", i, seekPos);
      continue;
    }

    char* epochStr = strtok(buffer, ",");
    char* durationStr = strtok(NULL, ",");

    if (!epochStr || !durationStr) {
      Serial.printf("âš ï¸ Skipping invalid entry at index %d: '%s'\n", i, buffer);
      continue;
    }

    uint32_t epoch = strtoul(epochStr, nullptr, 10);
    uint32_t duration = strtoul(durationStr, nullptr, 10);

    if (epoch == 0 || duration == 0 || duration > 86400UL) {
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
*/void handleRelayLogs(AsyncWebServerRequest *request) {
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

    // Trim newline and whitespace
    char* newline = strchr(buffer, '\n');
    if (newline) *newline = '\0';
    char* endSpace = strchr(buffer, '\r');
    if (endSpace) *endSpace = '\0';

    Serial.printf("ðŸ” Raw entry at index %d: '%s'\n", i, buffer);

    // Find comma
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
  response->print("[");  // Start JSON array

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

    // Parse CSV: timestamp, category, reason
    char *epochStr  = strtok(buffer, ",");  // timestamp
    char *category  = strtok(NULL, ",");    // "Failsafe"
    char *reasonStr = strtok(NULL, ",");    // actual reason

    if (reasonStr) {
      reasonStr[strcspn(reasonStr, "\r\n")] = '\0';  // strip \r\n
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

// do file stuff
// no need to call releaseFSLock() â€” automatically released

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

// Set up Access Point
// Assuming 'server' is your global AsyncWebServer instance pointer (e.g., AsyncWebServer *server;)
// Assuming MDNS (ESP8266mDNS) is globally available
// Assuming 'dnsServer' is your global DNSServer instance
// Assuming 'dnsServerActive' is a global boolean tracking its state

void setupAP() {
  if (isApModeActive) return; // Prevent re-setup if already active

  Serial.println(F("Stopping existing services before starting AP mode..."));

  // 1. STOP ALL DEPENDENT NETWORK SERVICES FIRST
  // Stop the Web Server
if (serverRunning) {
stopWebServer();    
    Serial.println("Web server stopped.");
}


  // Stop mDNS
  //if (MDNS.isRunning()) {
   // MDNS.end();
   // Serial.println("mDNS stopped.");
  //}

  // Stop DNS Server (if it was active, e.g., from a previous AP mode session)
 
  // Disconnect from current WiFi (crucial if coming from STA mode)
  WiFi.disconnect(true); // 'true' also removes saved credentials from flash
  delay(100); // Give a small delay for the network stack to settle

  Serial.println(F("Starting AP mode configuration..."));

  // --- (Keep your existing SSID/Password clamping/fallback logic here) ---
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

  // 2. CHANGE WIFI MODE AND CONFIGURE AP
  WiFi.mode(WIFI_AP); // Now it's safe to change the mode
  WiFi.softAP(settings.apSsid, settings.apPassword);

  IPAddress apIP = WiFi.softAPIP();
  Serial.print(F("AP IP: "));
  Serial.println(apIP);

  // 3. RESTART NETWORK SERVICES FOR THE NEW MODE
  // Restart DNS Server (if used for AP functionality like captive portal)
  // This check ensures it starts only once for the new AP session
// Always stop any existing DNS instance
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
  apStartTime = millis(); // Reset AP timeout

  // Restart Web Server and mDNS (your startWebServer() function should handle these)
  startWebServer();
  // Ensure startWebServer() re-initializes and starts the server (server->begin())
  // and re-initializes and starts mDNS (MDNS.begin(), MDNS.addService()) for the new mode.
  // For example, within startWebServer() it might look like:
  // if (server) server->begin();
  // if (MDNS.begin(settings.hostname)) { MDNS.addService("http", "tcp", 80); }
}

// AP Timeout Checker (call this from loop)
// AP Timeout Checker (call this from loop)
void checkAPTimeout() {
  if (!isApModeActive) return;

  // Timeout durations
  const unsigned long AP_NO_CLIENT_TIMEOUT_MS = 5UL * 60UL * 1000UL;  // 5 minutes
  const unsigned long AP_CLIENT_TIMEOUT_MS    = 10UL * 60UL * 1000UL; // 10 minutes

  // Get number of connected clients
  int stationNum = WiFi.softAPgetStationNum();

  // Select timeout based on connected clients
  unsigned long currentTimeout = (stationNum > 0) ? AP_CLIENT_TIMEOUT_MS : AP_NO_CLIENT_TIMEOUT_MS;

  // Check if timeout has elapsed
  if (millis() - apStartTime >= currentTimeout) {
    // Now check relay state before shutting down AP
    if (relayState) {
      Serial.println(F("â³ AP timeout reached but relay is ON â€” delaying AP shutdown."));
      return; // Wait until relay turns off
    }

    // Proceed with AP shutdown
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

  stopWebServer(); // If you're running a separate stopWebServer function
}


// --- Debounced Switch Check ---
void checkSwitchPress() {
  bool currentReading = digitalRead(SWITCH_PIN);

  // Debounce: if raw reading changed, reset timer
  if (currentReading != lastSwitchReading) {
    lastSwitchDebounceTime = millis();
  }

  // If state stable for debounce delay
  if ((millis() - lastSwitchDebounceTime) > DEBOUNCE_DELAY) {
    if (currentReading != lastSwitchStableState) {
      lastSwitchStableState = currentReading;

      // Cooldown check
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

  lastSwitchReading = currentReading; // update raw reading
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
/*
bool isSSIDInRange(const char* ssid) {
    int n = WiFi.scanNetworks(false, true); // Don't block, show hidden
    bool found = false;
    for (int i = 0; i < n; ++i) {
        if (strcmp(ssid, WiFi.SSID(i).c_str()) == 0) { // Compare char arrays
            found = true;
            break;
        }
    }
    WiFi.scanDelete(); // Clear results to free memory
    return found;
}
*/


// Load settings from LittleFS
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

  // WiFi STA
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

  // WiFi AP
  settings.wifiAPModeEnabled = doc["wifiAPModeEnabled"] | false;
  strncpy(settings.apSsid, doc["apSsid"] | "AquaMaster_AP", sizeof(settings.apSsid) - 1);
  settings.apSsid[sizeof(settings.apSsid) - 1] = '\0';
  strncpy(settings.apPassword, doc["apPassword"] | "aquapass", sizeof(settings.apPassword) - 1);
  settings.apPassword[sizeof(settings.apPassword) - 1] = '\0';

  // Admin login
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

  // Relay & Failsafe
  settings.calibrationFactor = doc["calibrationFactor"] | 1.0;
  settings.failsafeDurationHours = doc["failsafeDurationHours"] | 12;
  settings.failsafeEnabled = doc["failsafeEnabled"] | true;
  settings.failsafePin = doc["failsafePin"] | D5;
  settings.relayPin = doc["relayPin"] | D1;
  settings.defaultRelayDuration = doc["defaultRelayDuration"] | 720;
  settings.maxRelayRuntime = doc["maxRelayRuntime"] | 960;

  // Run interruption recovery
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


  bool success = false;  // Final return value

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

  // Safe rotation
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
    saveBootRecovery();  // This function should also manage locking internally
    success = true;      // Consider initialized
  } else {
    StaticJsonDocument<128> doc; // Small capacity needed
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



// do file stuff
// no need to call releaseFSLock() â€” automatically released

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

// Internal function to toggle relay (used by multiple handlers)
void toggleRelayInternal() {
  if (failsafeActive || emergencyStop) {
    Serial.println(F("Cannot toggle relay: Failsafe or Emergency Stop is active."));
    return;
  }

  if (relayState) { // Relay is ON, so turn OFF
    digitalWrite(settings.relayPin, HIGH); // HIGH = OFF for active-low
    relayState = false;
    Serial.println(F("Relay OFF."));

    if (relayTicker.active()) {
      relayTicker.detach();
     unsigned long endTime = millis();
//logRelayRunTime(relayOnStartTime, endTime);

      settings.wasTimedRunInterrupted = false;
      settings.interruptedRunDuration = 0;
      settings.interruptedRunStartTime = 0;
      saveSettings();
    }

    autoDeepSleepDelayTicker.detach();
    autoDeepSleepScheduled = false;

  } else { // Relay is OFF, so turn ON
    relayOnEpochTime = time(nullptr);

    digitalWrite(settings.relayPin, LOW); // LOW = ON for active-low
    relayState = true;
    relayOnStartTime = millis();
    Serial.println(F("Relay ON."));

    // âœ… Start timer to auto-turn-off relay after default duration
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

// Internal function for emergency stop logic
// Internal function for emergency stop logic
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
    saveSettings();  // Already safe, but keep WDT before & after
    ESP.wdtFeed();
  }
if (failsafeTicker.active()) {
 failsafeTicker.detach();
}
  autoDeepSleepDelayTicker.detach();
  autoDeepSleepScheduled = false;

  ESP.wdtFeed();
strncpy(pendingFailsafeReason, "Emergency Stop", sizeof(pendingFailsafeReason) - 1);


  broadcastStatus();  // Safe if it avoids String internally

  ESP.wdtFeed();
}

// Function to turn off relay after timed duration
void turnOffRelayTimed() {
  if (relayState) {
    relayTimedOffPending = true;
  }
}


// ISR for emergency pin (D0)
// Add a new global volatile boolean flag outside your functions:


void clearEmergencyStopAndFailsafe() {
  if (emergencyStop || failsafeActive) {
    Serial.println(F("EMERGENCY STOP/FAILSAFE CLEARED. Resuming normal operation."));

    emergencyStop = false;
    failsafeActive = false;

    // Re-attach failsafe ticker only if needed
  //  if (settings.failsafeEnabled && settings.failsafeDurationHours > 0) {
   //   failsafeTicker.attach(settings.failsafeDurationHours * 3600, failsafeTriggered);
//    }

    // Defer these actions for safety
    shouldBroadcastAfterEmergencyClear = true;
    shouldLogEmergencyClear = true;
  } else {
    Serial.println(F("Emergency Stop/Failsafe not active. No action needed."));
  }
}


void sendIPToTelegram() {
//  secureClient.setInsecure(); // Disable certificate verification for ESP8266
  String ipMsg = "ESP8266 Connected!\nIP Address: " + WiFi.localIP().toString();
  if (bot.sendMessage(CHAT_ID, ipMsg, "")) {
    Serial.println(F("âœ… Telegram message sent successfully"));
  } else {
    Serial.println(F("âŒ Failed to send Telegram message"));
  }
}

/*
// Placeholder for cleanup logic (e.g., keeping only X latest entries or Y days)
void cleanupOldLogs() {
  // This is a more complex function, might involve reading line by line,
  // parsing timestamps, and rewriting the file if entries are too old.
  // For simplicity, this is a placeholder.
#ifdef DEBUG_MODE
  Serial.println(F("Performing log cleanup... (Placeholder)"));
#endif
}
*/


// --- Web Server Handlers ---
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

      // âœ… Create a custom response with a cookie header
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
      currentSessionToken[0] = '\0'; // Invalidate current session
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
 // if (!isLoggedIn(request)) {
  //  request->send(401, "text/plain", "Unauthorized");
//    return;
//  }
  clearEmergencyStopAndFailsafe();
  resetFailsafe ();
  request->send(200, "text/plain", "Failsafe cleared.");
}
/*

void circularLogWrite(const char* filename, const char* logMsg, uint8_t& logIndex) {
 FSLockGuard lock;
if (!lock.isLocked()) return;

// do file stuff
// no need to call releaseFSLock() â€” automatically released

  ESP.wdtFeed();

  if (!filename || !logMsg || strlen(logMsg) == 0 || logIndex >= MAX_LOGS) {
    Serial.println(F("WARN: Invalid input to circularLogWrite(). Resetting index."));
    logIndex = 0;
    return;
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
        Serial.printf("ERROR: Failed to init blank log at %u\n", i);
        logFile.close();
        return;
      }
      yield();
    }
  }

  // Create padded buffer for write
  char padded[LOG_ENTRY_LENGTH] = {0};
  size_t len = strnlen(logMsg, LOG_ENTRY_LENGTH - 2);
  strncpy(padded, logMsg, len);

  // Replace any newline characters with spaces
  for (size_t i = 0; i < len; i++) {
    if (padded[i] == '\r' || padded[i] == '\n') padded[i] = ' ';
  }

  padded[len] = '\n';  // Properly terminated log entry
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

  logIndex = (logIndex + 1) % MAX_LOGS;

  ESP.wdtFeed();
  yield();
 
}
*/
/*
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

    // ⚠️ This may still be slow if MAX_LOGS is large
    char blank[LOG_ENTRY_LENGTH] = {0};
    for (uint8_t i = 0; i < MAX_LOGS; i++) {
      if (logFile.write((const uint8_t*)blank, LOG_ENTRY_LENGTH) != LOG_ENTRY_LENGTH) {
        Serial.printf("ERROR: Failed to initialize blank log at %u\n", i);
        logFile.close();
        return;
      }
      yield();  // ✅ Yield during large file init
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
  yield();  // ✅ Yield after writing

  logIndex = (logIndex + 1) % MAX_LOGS;

  ESP.wdtFeed();
  yield();
}
*/

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

    // ⚠️ This may still be slow if MAX_LOGS is large
    char blank[LOG_ENTRY_LENGTH] = {0};
    for (uint8_t i = 0; i < MAX_LOGS; i++) {
      if (logFile.write((const uint8_t*)blank, LOG_ENTRY_LENGTH) != LOG_ENTRY_LENGTH) {
        Serial.printf("ERROR: Failed to initialize blank log at %u\n", i);
        logFile.close();
        return;
      }
      yield();  // ✅ Yield during large file init
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
  yield();  // ✅ Yield after writing

  logIndex = (logIndex + 1) % MAX_LOGS;

  ESP.wdtFeed();
  yield();
}






void handleEmergencyStop(AsyncWebServerRequest *request) {
  request->send(200, "application/json", "{\"status\":\"emergency_triggered\"}");

  // Schedule emergency logic safely AFTER response is sent
  emergencyStopDelay.once_ms(50, emergencyStopInternal);
}

void handleScanNetworks(AsyncWebServerRequest *request) {
//  if (!isLoggedIn(request)) {
  //  request->send(401, "text/plain", "Unauthorized");
 //   return;
//  }
  if (scanInProgress) {
    request->send(409, "text/plain", "WiFi scan already in progress.");
    return;
  }

  WiFi.mode(WIFI_STA); // Ensure STA mode for scanning
  Serial.println(F("Starting WiFi scan..."));
  scanInProgress = true;
  WiFi.scanNetworks(true, true); // Async scan, show hidden
  request->send(202, "text/plain", "WiFi scan started.");

  // Attach a ticker to check scan completion periodically
  wifiScanTicker.attach_ms(1000, broadcastScanResults); // Check every second
}


void broadcastScanResults() {
 if (events.count() == 0) {
        return;
    }

    int n = WiFi.scanComplete();
    if (n >= 0) { // Scan is complete
        wifiScanTicker.detach(); // Stop checking
        scanInProgress = false;

        // Allocate a DynamicJsonDocument because the number of networks is unknown.
        // If memory is tight, send networks in chunks or limit total.
        // For typical home use, 20-30 networks is reasonable, keep capacity for ~25 networks
        // Each network entry might be ~100-150 bytes, so 25*150 = 3750 bytes.
        // Let's use 4KB as a safe estimate for JSON.
        const size_t JSON_SCAN_CAPACITY = JSON_ARRAY_SIZE(n) + n * (JSON_OBJECT_SIZE(3) + 32 + 32);
        StaticJsonDocument<4096> doc; // Adjusted capacity estimate

        JsonArray networks = doc.to<JsonArray>();
        for (int i = 0; i < n; ++i) {
            JsonObject network = networks.add<JsonObject>();
            network["ssid"] = WiFi.SSID(i); // String is created temporarily by WiFi.SSID, then copied
            network["rssi"] = WiFi.RSSI(i);
            network["encryption"] = (WiFi.encryptionType(i) == AUTH_OPEN) ? "Open" : "Encrypted";
        }
        WiFi.scanDelete(); // Clear results to free memory

        char responseBuffer[4096]; // Use a fixed-size buffer for the response
        size_t jsonSize = serializeJson(doc, responseBuffer, sizeof(responseBuffer));

        if (jsonSize > 0) {
          
   events.send(responseBuffer, "scanResults"); // Broadcast to all SSE clients

           
#ifdef DEBUG_MODE
            Serial.printf_P(PSTR("Broadcasted %d scan results.\n"), n);
#endif
        } else {
#ifdef DEBUG_MODE
            Serial.println(F("Failed to serialize scan results JSON for broadcast."));
#endif
        }
    } else if (n == -1) { // Scan not complete yet
#ifdef DEBUG_MODE
        Serial.print(F("Scan in progress... "));
#endif
    }
}


void handleKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }

  StaticJsonDocument<2048> doc; // Adjust capacity based on expected knownNetworks count
  JsonArray networks = doc.to<JsonArray>();

  for (const auto& kn : knownNetworks) {
    JsonObject network = networks.add<JsonObject>();
    network["ssid"] = kn.ssid;
    // Don't send password for security
  }

  char responseBuffer[2048]; // Use a fixed-size buffer for the response
  size_t jsonSize = serializeJson(doc, responseBuffer, sizeof(responseBuffer));

  if (jsonSize > 0) {
    request->send(200, "application/json", responseBuffer);
  } else {
    request->send(500, "text/plain", "Failed to serialize known networks JSON.");
  }
}

void handleSaveKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!request->hasArg("plain")) { // Expecting a JSON body
    request->send(400, "text/plain", "Missing JSON body.");
    return;
  }

  StaticJsonDocument<2048> doc; // Adjust capacity
  DeserializationError error = deserializeJson(doc, request->arg("plain"));

  if (error) {
    request->send(400, "text/plain", "Failed to parse JSON: " + String(error.c_str()));
    return;
  }

  JsonArray networksArray = doc.as<JsonArray>();
  if (networksArray.isNull()) {
    request->send(400, "text/plain", "JSON is not an array.");
    return;
  }

  knownNetworks.clear(); // Clear existing
  for (JsonObject network : networksArray) {
    KnownNetwork kn;
    strncpy(kn.ssid, network["ssid"] | "", sizeof(kn.ssid) - 1);
    kn.ssid[sizeof(kn.ssid) - 1] = '\0';
    strncpy(kn.password, network["password"] | "", sizeof(kn.password) - 1);
    kn.password[sizeof(kn.password) - 1] = '\0';
    knownNetworks.push_back(kn);
  }

  // Saving known networks to file (if implemented)
  // File knownNetworksFile = LittleFS.open("/known_networks.json", "w");
  // if (knownNetworksFile) {
  //    serializeJson(doc, knownNetworksFile);
  //    knownNetworksFile.close();
  // }

  request->send(200, "text/plain", "Known networks updated.");
}

void handleClearKnownNetworks(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  knownNetworks.clear();
  // Also remove from file if persisted
  // LittleFS.remove("/known_networks.json");
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
        // Set a flag to trigger the mode change in the main loop
        pendingAPSetup = true;
        request->send(200, "text/plain", "AP mode enabled. Switching modes...");
      } else {
        if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
          Serial.println(F("Disabling AP mode. Restarting in STA mode."));
          request->send(200, "text/plain", "AP mode disabled. Restarting...");
          delay(100);
          pendingRestart = true; // Use existing restart logic
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
  pendingRestart = true;  // Safe delayed restart
}



void handleRestoreFactorySettings(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  request->send(200, "text/plain", "Restoring factory settings...");
  factoryResetPreserveIndex(); // This will restart the ESP
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
  WiFi.disconnect(true); // Disconnect and erase STA creds
  strncpy(settings.staSsid, "", sizeof(settings.staSsid));
  strncpy(settings.staPassword, "", sizeof(settings.staPassword));
  saveSettings();
  request->send(200, "text/plain", "Disconnected from WiFi and credentials cleared.");
  broadcastStatus();
}

/*

void handleSoftShutdown(AsyncWebServerRequest *request) {
  if (!isLoggedIn(request)) {
    request->send(401, "text/plain", "Unauthorized");
    return;
  }
  softShutdownPending = true;
  request->send(200, "text/plain", "Soft shutdown sequence initiated.");
  // In a real scenario, this would trigger a controlled shutdown
  // (e.g., turn off relays, save state, then deep sleep or power off).
  // For now, it just sets a flag.
  // You might want to add a `delay(some_time)` here to allow the response to be sent.
  // Or handle the actual shutdown in loop() based on the flag.
}
*/

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
      request->send(400, "text/plain", "Deep sleep duration is 0. Please set a duration first.");
      return;
  }
  deepSleepRequested = true;
  request->send(200, "text/plain", "Initiating deep sleep.");
  // Actual deep sleep will happen in loop() after a small delay to send response
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

/*
void handleFileUpload(AsyncWebServerRequest *request, String filename_str, size_t index, uint8_t *data, size_t len, bool final) {
 FSLockGuard lock;
if (!lock.isLocked()) return;

// do file stuff
// no need to call releaseFSLock() â€” automatically released

 // if (!isLoggedIn(request)) {
    // If not logged in, reject upload. AsyncWebServer might still call this, but we deny.
   // return;
//  }

  // Convert String filename to char array for safer file operations
  char filename[128]; // Max filename length
  strncpy(filename, filename_str.c_str(), sizeof(filename) - 1);
  filename[sizeof(filename) - 1] = '\0';

  if (index == 0) {
    Serial.printf("Upload Start: %s\n", filename);
    // Open file in write mode, overwriting if exists
    request->_tempFile = LittleFS.open(filename, "w");
  }
  if (len) {
    request->_tempFile.write(data, len);
    Serial.printf("Uploading: %s, Bytes: %u\n", filename, len);
  }
  if (final) {
    request->_tempFile.close();
    Serial.printf("Upload End: %s, Size: %u\n", filename, index + len);
    request->send(200, "text/plain", "File uploaded successfully.");
  }
 
}
*/

void handleFileUpload(AsyncWebServerRequest *request, String filename_str,
                      size_t index, uint8_t *data, size_t len, bool final) {
  FSLockGuard lock;
  if (!lock.isLocked()) return;

 // if (!isLoggedIn(request)) {
//    request->send(403, "text/plain", "Not authorized");
//    return;
  //}

  // We always want to overwrite /index.html.gz, regardless of upload filename
  const char *targetFile = "/index.html.gz";

  if (index == 0) {
    Serial.println("\n=== File Upload Start ===");
    Serial.printf("Original filename_str: %s\n", filename_str.c_str());
    Serial.printf("Target file: %s\n", targetFile);

    // Remove old version before writing
    if (LittleFS.exists(targetFile)) {
      Serial.println("Removing existing /index.html.gz...");
      if (LittleFS.remove(targetFile)) {
        Serial.println("Old file removed successfully.");
      } else {
        Serial.println("⚠️ Failed to remove old file!");
      }
      delay(50); // ensure flash erase completes
    }

    // Open file for writing (overwrites if somehow still present)
    request->_tempFile = LittleFS.open(targetFile, "w");
    if (!request->_tempFile) {
      Serial.println("❌ Failed to open /index.html.gz for writing!");
      request->send(500, "text/plain", "Failed to open file for writing");
      return;
    }

    Serial.println("File opened for writing.");
  }

  // Write data chunks
  if (len && request->_tempFile) {
    size_t written = request->_tempFile.write(data, len);
    Serial.printf("Writing chunk: %u bytes (written: %u)\n", len, written);
  }

  // Finalize
  if (final) {
    if (request->_tempFile) {
      request->_tempFile.close();
      Serial.printf("Upload completed. Total size: %u bytes\n", index + len);
    }
    Serial.println("=== File Upload End ===\n");

    request->send(200, "text/plain", "index.html.gz uploaded successfully and replaced.");
  }
}



void handleStatus(AsyncWebServerRequest *request) {
 // if (!isLoggedIn(request)) {
    // request->send(401, "text/plain", "Unauthorized");
   // return;
//  }

  // Immediately broadcast status to all SSE clients
  broadcastStatus();

  StaticJsonDocument<512> doc;
  doc["relayState"] = relayState;
  doc["failsafeActive"] = failsafeActive;
  doc["emergencyStop"] = emergencyStop;
  doc["loggedIn"] = isLoggedIn(request); // Per-request check
  doc["wifiStatus"] = WiFi.status();
  doc["ipAddress"] = WiFi.localIP().toString(); // temporary String OK
  doc["ssid"] = WiFi.SSID();
  doc["hostname"] = settings.hostname;
  doc["apModeActive"] = isApModeActive;
  doc["scanInProgress"] = scanInProgress;
 // doc["softShutdownPending"] = softShutdownPending;
  doc["deepSleepRequested"] = deepSleepRequested;
  doc["deepSleepDurationSeconds"] = settings.deepSleepDurationSeconds;
  doc["autoDeepSleepScheduled"] = autoDeepSleepScheduled;
  doc["autoDeepSleepEnabled"] = settings.autoDeepSleepEnabled;

  // Human-readable WiFi connection status
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

  // Human-readable AP mode status
  const char* apModeText = "Disabled";
  if (WiFi.getMode() == WIFI_AP) {
    apModeText = "Enabled (AP only)";
  } else if (WiFi.getMode() == WIFI_AP_STA) {
    apModeText = "Enabled (AP + STA)";
  }
  doc["apModeStatusText"] = apModeText;

  // Serialize JSON and send it
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
  doc["autoDeepSleepEnabled"] = settings.autoDeepSleepEnabled; // New setting field

  // IMPORTANT: Do NOT send passwords or hashes for security reasons.
  // The UI should handle password changes separately.

  char responseBuffer[SETTINGS_DOC_CAPACITY]; // Ensure SETTINGS_DOC_CAPACITY is large enough for the JSON
  size_t jsonSize = serializeJson(doc, responseBuffer, sizeof(responseBuffer));
  
  if (jsonSize > 0) {
    request->send(200, "application/json", responseBuffer);
  } else {
    request->send(500, "text/plain", "Failed to serialize settings JSON.");
   Serial.print(F("ERROR: serializeJson() failed in handleGetSettings (buffer too small or data issue)."));
  
 //Serial.println(error.f_str());
      
  }
}
AsyncCallbackJsonWebHandler* saveSettingsHandler = new AsyncCallbackJsonWebHandler("/saveSettings", [](AsyncWebServerRequest *request, JsonVariant &json) {
  StaticJsonDocument<SETTINGS_DOC_CAPACITY> doc;
  doc.set(json); // Copy the received JSON

  Serial.println(F("ðŸ“¥ Received settings JSON:"));
  serializeJsonPretty(doc, Serial);
  Serial.println();

  // Parse fields safely
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

  // Optional fields with type checks
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
});






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
 // doc["fileAlign"] = fs_info.fileAlign;

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

// SSE Broadcast
void broadcastStatus() {

  if (events.count() == 0) return;

  // Use heap, not stack (ESP8266 core 3.1.2 safe)
  DynamicJsonDocument doc(768);

  // -------- CORE STATUS --------
  doc["relayState"]             = relayState;
  doc["failsafeActive"]         = failsafeActive;
  doc["emergencyStop"]          = emergencyStop;
  doc["loggedIn"]               = loggedIn;
  doc["scanInProgress"]         = scanInProgress;
  doc["relayOnStartTime"]       = relayOnStartTime;
  doc["currentDeviceMillis"]    = millis();
  doc["currentRelayDuration"]   = relayDuration;

  // -------- WIFI STATUS --------
  wl_status_t ws = WiFi.status();
  doc["wifiStatus"] = ws;
  doc["ipAddress"]  = WiFi.localIP().toString();
  doc["ssid"]       = WiFi.SSID();
  doc["hostname"]   = settings.hostname;
  doc["apModeActive"] = isApModeActive;

  const char* wifiStatusStr = "Unknown";
  switch (ws) {
    case WL_IDLE_STATUS:      wifiStatusStr = "Idle"; break;
    case WL_NO_SSID_AVAIL:    wifiStatusStr = "No SSID Available"; break;
    case WL_SCAN_COMPLETED:   wifiStatusStr = "Scan Completed"; break;
    case WL_CONNECTED:        wifiStatusStr = "Connected"; break;
    case WL_CONNECT_FAILED:   wifiStatusStr = "Connect Failed"; break;
    case WL_CONNECTION_LOST:  wifiStatusStr = "Connection Lost"; break;
    case WL_DISCONNECTED:     wifiStatusStr = "Disconnected"; break;
  }
  doc["wifiStatusText"] = wifiStatusStr;

  // -------- AP INFO --------
  if (WiFi.getMode() & WIFI_AP) {
    doc["apIP"]   = WiFi.softAPIP().toString();
    doc["apSSID"] = WiFi.softAPSSID();
  }

  // -------- POWER / SLEEP --------
  doc["deepSleepRequested"]        = deepSleepRequested;
  doc["deepSleepDurationSeconds"] = settings.deepSleepDurationSeconds;
  doc["autoDeepSleepScheduled"]   = autoDeepSleepScheduled;
  doc["autoDeepSleepEnabled"]     = settings.autoDeepSleepEnabled;
  doc["defaultRelayDuration"]     = settings.defaultRelayDuration;
  doc["maxRelayRuntime"]          = settings.maxRelayRuntime;

  // Serialize directly to Async buffer (NO big stack buffer)
  String out;
  serializeJson(doc, out);

  events.send(out.c_str(), "status");
}
/*
void broadcastStatus() {
 if (events.count() == 0) {
        return;
    }

  StaticJsonDocument<512> doc;
  doc["relayState"] = relayState;
  doc["failsafeActive"] = failsafeActive;
  doc["emergencyStop"] = emergencyStop;
  doc["loggedIn"] = loggedIn;
  doc["wifiStatus"] = WiFi.status();
  doc["ipAddress"] = WiFi.localIP().toString();
  doc["ssid"] = WiFi.SSID();
  doc["hostname"] = settings.hostname;
  doc["apModeActive"] = isApModeActive;
  doc["scanInProgress"] = scanInProgress;
  //doc["softShutdownPending"] = softShutdownPending;
  doc["deepSleepRequested"] = deepSleepRequested;
  doc["deepSleepDurationSeconds"] = settings.deepSleepDurationSeconds;
  doc["autoDeepSleepScheduled"] = autoDeepSleepScheduled;
  doc["autoDeepSleepEnabled"] = settings.autoDeepSleepEnabled;
  doc["defaultRelayDuration"] = settings.defaultRelayDuration;
  doc["maxRelayRuntime"] = settings.maxRelayRuntime;
  doc["relayOnStartTime"] = relayOnStartTime;
  doc["currentDeviceMillis"] = millis();
  doc["currentRelayDuration"] = relayDuration; // For temporary run countdown 
  

  // Add human-readable WiFi status
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

  // Add AP details if AP mode is active
  if (WiFi.getMode() & WIFI_AP) {
    doc["apIP"] = WiFi.softAPIP().toString();
    doc["apSSID"] = WiFi.softAPSSID();
  }

  // Send the JSON via SSE
  char jsonBuffer[512];
  size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
  if (len > 0) {
             
   events.send(jsonBuffer, "status");

    
  }
}
*/
