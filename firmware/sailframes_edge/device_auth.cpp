// Device identity/claim/auth glue — see device_auth.h.
#include "device_auth.h"
#include "config.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SD.h>

static char s_externalId[24] = "";

const char* externalId() {
  if (s_externalId[0] == '\0') {
    String mac = WiFi.macAddress();  // already "AA:BB:CC:DD:EE:FF"
    mac.toCharArray(s_externalId, sizeof(s_externalId));
  }
  return s_externalId;
}

// Device-owned persisted credentials (/device.txt) — separate from the
// user-edited config.txt, same pattern as /boot.log, /imu_cal.txt.
static bool   s_credsLoaded = false;
static bool   s_claimed = false;
static char   s_deviceId[40] = "";
static char   s_deviceApiKey[80] = "";

static void loadDeviceCredentials() {
  s_credsLoaded = true;
  File f = SD.open("/device.txt", FILE_READ);
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String k = line.substring(0, eq);
    String v = line.substring(eq + 1);
    if (k == "device_id") v.toCharArray(s_deviceId, sizeof(s_deviceId));
    else if (k == "device_api_key") v.toCharArray(s_deviceApiKey, sizeof(s_deviceApiKey));
  }
  f.close();
  // device_id is display metadata only — "claimed" means "has a usable
  // key", since that's the only thing apiRequest()'s Authorization header
  // actually needs. A BLE-relayed claim (persistDeviceApiKey()) may leave
  // device_id empty.
  s_claimed = strlen(s_deviceApiKey) > 0;
}

// newDeviceId may be "" (unknown) — see persistDeviceApiKey().
static bool saveDeviceCredentials(const char* newDeviceId, const char* newApiKey) {
  File f = SD.open("/device.txt", FILE_WRITE);
  if (!f) return false;
  if (newDeviceId && newDeviceId[0]) f.printf("device_id=%s\n", newDeviceId);
  f.printf("device_api_key=%s\n", newApiKey);
  f.close();
  if (newDeviceId) strncpy(s_deviceId, newDeviceId, sizeof(s_deviceId) - 1);
  strncpy(s_deviceApiKey, newApiKey, sizeof(s_deviceApiKey) - 1);
  s_claimed = strlen(s_deviceApiKey) > 0;
  return true;
}

bool persistDeviceApiKey(const char* apiKey) {
  if (apiKey == nullptr || strlen(apiKey) == 0) return false;
  if (!s_credsLoaded) loadDeviceCredentials();  // don't clobber a known device_id
  return saveDeviceCredentials(s_deviceId, apiKey);
}

bool isClaimed() {
  if (!s_credsLoaded) loadDeviceCredentials();
  return s_claimed;
}

const char* deviceApiKey() {
  if (!s_credsLoaded) loadDeviceCredentials();
  return s_deviceApiKey;
}

const char* deviceId() {
  if (!s_credsLoaded) loadDeviceCredentials();
  return s_deviceId;
}

int apiRequest(const char* method, const char* path, const String& jsonBody,
               String& responseBody, bool authenticated) {
  responseBody = "";
  if (strlen(config.api_base_url) == 0) {
    Serial.println("[API] api_base_url not set");
    return -1;
  }
  if (authenticated && !isClaimed()) {
    Serial.println("[API] not claimed — refusing authenticated call");
    return -1;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[API] WiFi not connected");
    return -1;
  }

  String url = String(config.api_base_url) + path;
  bool isHttps = url.startsWith("https://");

  HTTPClient http;
  http.setTimeout(30000);
  http.setReuse(false);

  bool began;
  WiFiClientSecure secureClient;
  WiFiClient plainClient;
  if (isHttps) {
    secureClient.setInsecure();  // ESP32 Arduino Core 3.3.7 TLS is unreliable — see firmware/README.md
    began = http.begin(secureClient, url);
  } else {
    began = http.begin(plainClient, url);
  }
  if (!began) {
    Serial.printf("[API] http.begin failed: %s\n", url.c_str());
    return -1;
  }

  if (jsonBody.length() > 0) {
    http.addHeader("Content-Type", "application/json");
  }
  if (authenticated) {
    http.addHeader("Authorization", String("DeviceKey ") + deviceApiKey());
  }

  int httpCode = http.sendRequest(method, (uint8_t*)jsonBody.c_str(), jsonBody.length());
  if (httpCode > 0) {
    responseBody = http.getString();
  } else {
    Serial.printf("[API] %s %s failed: %s\n", method, path, http.errorToString(httpCode).c_str());
  }
  http.end();
  return httpCode;
}

bool claimDevice(const char* claimCode) {
  if (claimCode == nullptr || strlen(claimCode) == 0) {
    Serial.println("[CLAIM] No claim code given");
    return false;
  }

  JsonDocument body;
  body["external_id"] = externalId();
  body["claim_code"] = claimCode;
  String bodyStr;
  serializeJson(body, bodyStr);

  Serial.printf("[CLAIM] Confirming claim_code=%s external_id=%s\n", claimCode, externalId());
  String response;
  int code = apiRequest("POST", "/api/devices/claim/confirm", bodyStr, response, false);

  if (code == 200) {
    JsonDocument resp;
    DeserializationError err = deserializeJson(resp, response);
    if (err) {
      Serial.printf("[CLAIM] Malformed response: %s\n", err.c_str());
      return false;
    }
    const char* newDeviceId = resp["device_id"] | "";
    const char* newApiKey = resp["device_api_key"] | "";
    if (strlen(newDeviceId) == 0 || strlen(newApiKey) == 0) {
      Serial.println("[CLAIM] Response missing device_id/device_api_key");
      return false;
    }
    if (!saveDeviceCredentials(newDeviceId, newApiKey)) {
      Serial.println("[CLAIM] Failed to persist /device.txt — claim not saved!");
      return false;
    }
    Serial.printf("[CLAIM] Claimed OK — device_id=%s\n", newDeviceId);
    return true;
  }

  // Per docs/device-protocol.md: 400/404/409 are not retryable (bad
  // input, code not found, code expired or external_id already
  // claimed elsewhere) — the user must issue a fresh claim. 429 means
  // back off; the caller decides whether/when to call claimDevice again.
  switch (code) {
    case 400: Serial.println("[CLAIM] 400 Bad Request — check external_id/claim_code (not retryable)"); break;
    case 404: Serial.println("[CLAIM] 404 claim_code not found — get a fresh code (not retryable)"); break;
    case 409: Serial.println("[CLAIM] 409 code expired or device already claimed — get a fresh code (not retryable)"); break;
    case 429: Serial.println("[CLAIM] 429 rate-limited — back off and retry later"); break;
    default:  Serial.printf("[CLAIM] Failed: HTTP %d\n", code); break;
  }
  return false;
}
