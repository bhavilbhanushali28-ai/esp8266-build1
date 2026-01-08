#pragma once
#include <Arduino.h>

// ================= RTC DEFINES =================
#define RTC_MAGIC 0xA5A55A5A
#define RTC_SLOT  65   // Avoid slot 0 (used by ESP core)

// ================= RTC STRUCT ==================
struct RTCData {
  uint32_t magic;
  uint32_t remainingSleepSeconds;
  uint32_t checksum;
};

// ================= RTC INSTANCE (DECLARATION) =================
// IMPORTANT: This is NOT a definition
extern RTCData rtcData;

// ================= RTC API =================
uint32_t rtcChecksum(const RTCData &d);
bool loadRTC();
void saveRTC();
void clearRTC();