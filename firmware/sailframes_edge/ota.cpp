// OTA firmware update (docs/ota.md).
//
// Fetches <config.ota_base_url>/manifest.json -> { version, url, checksum }
// (same shape as xgsail's ota-service; public, no auth), and if it advertises
// a build newer than FW_VERSION, streams the image straight into the inactive
// OTA slot via Update.h, verifies its SHA256 against the manifest BEFORE
// committing, then reboots into it. The dual-app partition table
// (partitions.csv) is what makes this possible without a USB reflash.
//
// Integrity is SHA256-only by design — no secure boot (docs/ota.md explains
// why). RECORDING GATE: no download is started while `logging` is true, and
// an in-progress download is SUSPENDED (aborted, retried on a later pass) the
// moment recording starts — a firmware flash must never contend with an
// active session for SD/CPU/WiFi.
#include "ota.h"
#include "config.h"
#include "storage.h"    // logging
#include "upload.h"     // wifiConnected — owned by the upload task
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <esp_task_wdt.h>
#include "mbedtls/sha256.h"

// Coarse OTA state, surfaced over BLE `status` (ble_relay.cpp) so the app can
// show progress/result. g_otaProgress is a percentage, or -1 when unknown/idle.
volatile const char* g_otaState = "idle";
volatile int         g_otaProgress = -1;
static char          s_otaMessage[48] = "";
// Set by ble_relay.cpp's control `ota-update` command; serviced by the upload
// task, which brings WiFi up and runs a check even if ota_auto_update is off.
volatile bool        otaManualRequested = false;

const char* otaMessage() { return s_otaMessage; }

static void otaSetState(const char* state, const char* msg) {
  g_otaState = state;
  strncpy(s_otaMessage, msg ? msg : "", sizeof(s_otaMessage) - 1);
  s_otaMessage[sizeof(s_otaMessage) - 1] = '\0';
  if (msg && msg[0]) Serial.printf("[OTA] %s: %s\n", state, msg);
  else               Serial.printf("[OTA] %s\n", state);
}

// True if `candidate` (YYYY.MM.DD.N) is strictly newer than FW_VERSION. Same
// date-tuple scheme display.cpp's fwShortTag() parses; a malformed version on
// either side is treated as "not newer" (fail safe — don't flash).
static bool otaVersionIsNewer(const char* candidate) {
  int cy = 0, cm = 0, cd = 0, cn = 0, fy = 0, fm = 0, fd = 0, fn = 0;
  if (sscanf(candidate, "%d.%d.%d.%d", &cy, &cm, &cd, &cn) != 4) return false;
  if (sscanf(FW_VERSION, "%d.%d.%d.%d", &fy, &fm, &fd, &fn) != 4) return false;
  if (cy != fy) return cy > fy;
  if (cm != fm) return cm > fm;
  if (cd != fd) return cd > fd;
  return cn > fn;
}

// Selects a WiFiClientSecure (https, setInsecure — see device_auth.cpp for why
// TLS validation is off) or a plain WiFiClient by URL scheme. The caller owns
// nothing; the returned pointer is one of the two passed-in stack objects.
static WiFiClient* otaPickClient(const String& url, WiFiClientSecure& secure, WiFiClient& plain) {
  if (url.startsWith("https://")) { secure.setInsecure(); return &secure; }
  return &plain;
}

// GET <ota_base_url>/manifest.json. On success fills version/url/checksum and
// returns true; an empty body ("{}") yields version="" meaning "nothing
// published / up to date". Returns false only on a transport/HTTP error.
static bool otaFetchManifest(String& version, String& url, String& checksum) {
  String base = config.ota_base_url;
  if (base.length() == 0) return false;
  if (!base.endsWith("/")) base += "/";
  String manifestUrl = base + "manifest.json";

  WiFiClientSecure secure;
  WiFiClient plain;
  WiFiClient* client = otaPickClient(manifestUrl, secure, plain);

  HTTPClient http;
  http.setConnectTimeout(15000);
  http.setTimeout(15000);
  if (!http.begin(*client, manifestUrl)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] manifest GET %s -> HTTP %d\n", manifestUrl.c_str(), code);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, body)) return false;
  version = doc["version"] | "";
  url = doc["url"] | "";
  checksum = doc["checksum"] | "";
  return true;
}

// Streams `url` into the inactive OTA slot, verifying its SHA256 against
// `expectedSha` before committing. Reboots into the new image on success (does
// not return). Returns false on any failure or a recording-triggered suspend.
static bool otaDownloadAndApply(const String& url, const String& expectedSha) {
  WiFiClientSecure secure;
  WiFiClient plain;
  WiFiClient* client = otaPickClient(url, secure, plain);

  HTTPClient http;
  http.setConnectTimeout(20000);
  http.setTimeout(20000);
  if (!http.begin(*client, url)) { otaSetState("error", "image begin failed"); return false; }
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] image GET -> HTTP %d\n", code);
    otaSetState("error", "image http error");
    http.end();
    return false;
  }

  int contentLen = http.getSize();  // -1 if the server didn't send it
  size_t total = contentLen > 0 ? (size_t)contentLen : 0;
  if (!Update.begin(total > 0 ? total : UPDATE_SIZE_UNKNOWN)) {
    otaSetState("error", Update.errorString());
    http.end();
    return false;
  }

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);  // 0 = SHA-256 (not 224)

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[1024];
  size_t written = 0;
  unsigned long lastProgress = millis();
  unsigned long startTime = millis();
  bool ok = true;
  const char* failMsg = "";

  while (http.connected() && (contentLen < 0 || written < total)) {
    // Recording gate: a session must never contend with an OTA flash. Abort
    // the transfer and let it be retried on a later WiFi window.
    if (logging) { ok = false; failMsg = "suspended: recording started"; break; }

    size_t avail = stream->available();
    if (avail) {
      size_t toRead = avail > sizeof(buf) ? sizeof(buf) : avail;
      int r = stream->readBytes(buf, toRead);
      if (r > 0) {
        mbedtls_sha256_update(&sha, buf, r);
        if (Update.write(buf, r) != (size_t)r) { ok = false; failMsg = "flash write failed"; break; }
        written += r;
        lastProgress = millis();
        g_otaProgress = total > 0 ? (int)((written * 100) / total) : -1;
      }
    } else {
      if (millis() - lastProgress > 30000) { ok = false; failMsg = "stalled 30s"; break; }
      delay(2);
    }
    if (millis() - startTime > 600000) { ok = false; failMsg = "ceiling 10min"; break; }
    esp_task_wdt_reset();
  }
  http.end();

  uint8_t digest[32];
  mbedtls_sha256_finish(&sha, digest);
  mbedtls_sha256_free(&sha);

  if (!ok) {
    Update.abort();
    // A recording-triggered stop is a suspend, not a hard error.
    otaSetState(logging ? "suspended" : "error", failMsg);
    return false;
  }

  // Verify integrity BEFORE committing the image.
  char hex[65];
  for (int i = 0; i < 32; i++) snprintf(hex + i * 2, 3, "%02x", digest[i]);
  if (expectedSha.length() > 0 && !expectedSha.equalsIgnoreCase(hex)) {
    Update.abort();
    Serial.printf("[OTA] checksum mismatch: got %s want %s\n", hex, expectedSha.c_str());
    otaSetState("error", "checksum mismatch");
    return false;
  }

  if (!Update.end(true)) {   // true = set the new slot as boot partition
    otaSetState("error", Update.errorString());
    return false;
  }

  otaSetState("applying", "rebooting into new firmware");
  delay(500);      // let the BLE status notify / serial flush
  ESP.restart();   // boots the freshly written slot
  return true;     // not reached
}

void runOtaCheck(bool manual) {
  if (logging) { if (manual) otaSetState("error", "recording in progress"); return; }
  if (!wifiConnected) { if (manual) otaSetState("error", "wifi not connected"); return; }

  otaSetState("checking", "");
  g_otaProgress = -1;

  String version, url, checksum;
  if (!otaFetchManifest(version, url, checksum)) { otaSetState("error", "manifest fetch failed"); return; }
  if (version.length() == 0) { otaSetState("up_to_date", ""); return; }
  if (!otaVersionIsNewer(version.c_str())) {
    Serial.printf("[OTA] running %s, latest %s — up to date\n", FW_VERSION, version.c_str());
    otaSetState("up_to_date", "");
    return;
  }
  if (url.length() == 0) { otaSetState("error", "manifest missing url"); return; }

  Serial.printf("[OTA] newer firmware %s available — downloading\n", version.c_str());
  otaSetState("downloading", version.c_str());
  otaDownloadAndApply(url, checksum);   // reboots on success; sets error/suspended otherwise
}

// Automatic OTA check — the once-per-boot/periodic path from the upload task's
// WiFi window. Gated on the owner's opt-in and the recording state; the manual
// BLE trigger uses runOtaCheck(true) directly (see upload.cpp's task loop).
void checkForFirmwareUpdate() {
  if (!config.ota_auto_update) return;   // owner hasn't opted in
  if (logging) return;                   // never during a recording
  if (!wifiConnected) return;
  runOtaCheck(false);
}
