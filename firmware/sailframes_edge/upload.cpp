// WiFi + XGSail device-protocol upload pipeline glue — see upload.h.
// Wire-level contract: xgsail's docs/device-protocol.md §4 (session
// uploads) and §4.4 (health snapshot). device_auth.h owns identity/claim/
// the authenticated-HTTP helper; this module owns WiFi connection
// management, the SD-file-driven upload loop, and the Core-0 task that
// runs it.
#include "upload.h"
#include "config.h"
#include "device_auth.h"
#include "gnss.h"
#include "imu.h"
#include "wind_sensor.h"
#include "battery.h"
#include "pressure.h"
#include "mesh.h"
#include "rtk_relay.h"
#include "v2_types.h"
#include "ocs.h"
#include "recording.h"
#include "storage.h"
#include "display.h"
#include "telnet.h"
#include "shared_state.h"
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

bool uploading = false;
int pendingUploads = 0;  // N: sessions with files still to upload
bool wifiConnected = false;
char connectedSSID[64] = "";
int uploadCount = 0, uploadTotal = 0;
int uploadSuccess = 0, uploadFailed = 0;
char uploadCurrentFile[32] = "";  // Short name of file being uploaded

volatile const char* g_uploadSection = "idle";

unsigned long lastUploadCheck = 0;
const unsigned long UPLOAD_CHECK_INTERVAL_MS = 30000;  // Check every 30 seconds
int uploadRetryCount = 0;
const int MAX_UPLOAD_RETRIES = 5;  // More attempts before 25-min backoff
unsigned long lastUploadAttempt = 0;
const unsigned long UPLOAD_RETRY_DELAY_MS = 30000;  // Wait 30 seconds between retries after failure

bool g_bootSessionLogged = false;
unsigned long g_lastAliveLog = 0;

bool isUploaded(const char* filepath) {
  char marker[128];
  snprintf(marker, sizeof(marker), "%s.uploaded", filepath);
  return SD.exists(marker);
}

// Delete files that have been uploaded (have .uploaded marker)
int deleteUploadedFiles(const char* dirname) {
  int count = 0;
  File root = SD.open(dirname);
  if (!root || !root.isDirectory()) return 0;

  // First pass: collect files to delete (can't delete while iterating)
  String filesToDelete[50];
  int fileCount = 0;

  File file = root.openNextFile();
  while (file && fileCount < 50) {
    char filepath[128];
    snprintf(filepath, sizeof(filepath), "%s/%s", dirname, file.name());
    String name = String(file.name());

    if (file.isDirectory()) {
      file.close();
      count += deleteUploadedFiles(filepath);  // Recurse
    } else if (name.endsWith(".uploaded")) {
      // This is a marker file - get the original filename
      String original = String(filepath);
      original = original.substring(0, original.length() - 9);  // Remove ".uploaded"
      filesToDelete[fileCount++] = original;
      filesToDelete[fileCount++] = String(filepath);  // Also delete marker
    }
    file.close();
    file = root.openNextFile();
    yield();
  }
  root.close();

  // Second pass: delete collected files
  for (int i = 0; i < fileCount; i++) {
    if (SD.remove(filesToDelete[i].c_str())) {
      Serial.printf("[CLEANUP] Deleted: %s\n", filesToDelete[i].c_str());
      count++;
    }
    yield();
  }

  return count;
}

// Mark file as uploaded
void markUploaded(const char* filepath) {
  char marker[128];
  snprintf(marker, sizeof(marker), "%s.uploaded", filepath);
  File f = SD.open(marker, FILE_WRITE);
  if (f) {
    f.printf("uploaded:%lu\n", millis());
    f.close();
  }
}

// If config.auto_cleanup_uploads is set, deletes filepath and its
// .uploaded marker right away instead of leaving them for the manual
// `cleanup`/`delup` console command. Called after markUploaded() from
// both upload paths — this WiFi one (uploadDirectory(), below) and the
// BLE relay's ack-uploaded (ble_relay.cpp) — so the two never diverge on
// what "uploaded" ends up doing to the SD card. Caller holds sdMutex.
void cleanupIfAutoDelete(const char* filepath) {
  if (!config.auto_cleanup_uploads) return;
  char marker[128];
  snprintf(marker, sizeof(marker), "%s.uploaded", filepath);
  if (SD.remove(filepath)) {
    Serial.printf("[UPLOAD] Auto-cleanup: deleted %s\n", filepath);
  }
  SD.remove(marker);
}

// Splits a URL into scheme/host/port/path — needed because session-uploads
// (§4.1) hands back an arbitrary presigned upload_url (S3, MinIO, whatever
// the deployment's object store is), not a fixed known host like the old
// hardcoded S3 bucket.
static bool parseUrl(const String& url, bool& isHttps, String& host, int& port, String& path) {
  String rest;
  if (url.startsWith("https://")) { isHttps = true; port = 443; rest = url.substring(8); }
  else if (url.startsWith("http://")) { isHttps = false; port = 80; rest = url.substring(7); }
  else return false;

  int slash = rest.indexOf('/');
  String hostPort = (slash >= 0) ? rest.substring(0, slash) : rest;
  path = (slash >= 0) ? rest.substring(slash) : "/";

  int colon = hostPort.indexOf(':');
  if (colon >= 0) {
    host = hostPort.substring(0, colon);
    port = hostPort.substring(colon + 1).toInt();
  } else {
    host = hostPort;
  }
  return host.length() > 0;
}

// Direct PUT of raw file bytes to a presigned upload_url (docs/
// device-protocol.md §4.1) — no Authorization header, the signed URL is
// its own credential. Manual chunked send (not HTTPClient::sendRequest)
// so we can feed the task wdt every chunk and run our own stall/ceiling
// watchdogs — HTTPClient's setTimeout is per-read, not total, and blocks
// with no esp_task_wdt_reset() calls inside for the whole body write.
// Multi-MB files at typical link speeds (50-100 KB/s) can take 3+ minutes;
// the 300s task wdt fired mid-upload before this was chunked (2026-05-25).
static bool putFileBytes(File& file, size_t fileSize, const String& uploadUrl, const String& filename) {
  bool isHttps; String host; int port; String path;
  if (!parseUrl(uploadUrl, isHttps, host, port, path)) {
    Serial.printf("[UPLOAD] Malformed upload_url: %s\n", uploadUrl.c_str());
    return false;
  }

  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  if (isHttps) secureClient.setInsecure();  // ESP32 Arduino Core 3.3.7 TLS is unreliable — see firmware/README.md
  Client* client = isHttps ? (Client*)&secureClient : (Client*)&plainClient;

  g_uploadSection = "uploadFile.connect";
  if (!client->connect(host.c_str(), port)) {
    Serial.printf("[UPLOAD] TCP connect failed: %s:%d\n", host.c_str(), port);
    return false;
  }

  String contentType = filename.endsWith(".csv") ? "text/csv" : "application/octet-stream";

  g_uploadSection = "uploadFile.headers";
  client->printf("PUT %s HTTP/1.1\r\n", path.c_str());
  client->printf("Host: %s\r\n", host.c_str());
  client->printf("Content-Type: %s\r\n", contentType.c_str());
  client->printf("Content-Length: %u\r\n", (unsigned)fileSize);
  client->print("Connection: close\r\n\r\n");

  yield();
  esp_task_wdt_reset();

  // Body — chunked send with per-chunk wdt feed. Static buf keeps 4 KB
  // off the upload task's stack.
  g_uploadSection = "uploadFile.body";
  const size_t CHUNK = 4096;
  static uint8_t buf[CHUNK];
  unsigned long startTime = millis();
  unsigned long lastProgress = startTime;
  size_t sent = 0;
  bool aborted = false;
  const char* abortReason = "";

  while (sent < fileSize) {
    esp_task_wdt_reset();
    yield();

    unsigned long now = millis();
    if (now - lastProgress > 30000) { aborted = true; abortReason = "STALL_30S"; break; }
    if (now - startTime > 600000)   { aborted = true; abortReason = "CEILING_10MIN"; break; }
    if (!client->connected())       { aborted = true; abortReason = "PEER_CLOSED"; break; }

    size_t want = (fileSize - sent < CHUNK) ? (fileSize - sent) : CHUNK;
    int r = file.read(buf, want);
    if (r <= 0) { aborted = true; abortReason = "SD_READ_FAILED"; break; }

    size_t w = client->write(buf, (size_t)r);
    if (w == 0) { aborted = true; abortReason = "SOCKET_WRITE_0"; break; }
    sent += w;
    lastProgress = millis();
  }

  unsigned long elapsed = (millis() - startTime) / 1000;

  if (aborted) {
    Serial.printf("[UPLOAD] Aborted: %s at %u/%u bytes after %lus\n",
                  abortReason, (unsigned)sent, (unsigned)fileSize, elapsed);
    client->stop();
    return false;
  }

  esp_task_wdt_reset();

  // Response — wait up to 60s for the object store to start replying,
  // then parse the status line and drain the rest.
  g_uploadSection = "uploadFile.response";
  int httpCode = -1;
  String response;
  unsigned long respDeadline = millis() + 60000;
  while (client->connected() && !client->available() && millis() < respDeadline) {
    esp_task_wdt_reset();
    yield();
    delay(10);
  }

  if (client->available()) {
    String statusLine = client->readStringUntil('\n');
    int sp1 = statusLine.indexOf(' ');
    int sp2 = statusLine.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > sp1) {
      httpCode = statusLine.substring(sp1 + 1, sp2).toInt();
    }
    unsigned long drainDeadline = millis() + 5000;
    while (client->connected() && millis() < drainDeadline) {
      if (client->available()) {
        char c = client->read();
        if (response.length() < 500) response += c;
      } else {
        esp_task_wdt_reset();
        yield();
        delay(1);
      }
    }
  } else {
    Serial.println("[UPLOAD] No response from object store within 60s after PUT");
  }

  client->stop();
  yield();
  delay(50);

  if (httpCode >= 200 && httpCode < 300) {
    Serial.printf("[UPLOAD] PUT OK: %s (HTTP %d, %lus, %u bytes)\n",
                  filename.c_str(), httpCode, elapsed, (unsigned)fileSize);
    return true;
  }
  Serial.printf("[UPLOAD] PUT failed: %s (HTTP %d, %lus)\n", filename.c_str(), httpCode, elapsed);
  if (response.length() > 0) Serial.printf("[UPLOAD] Response: %s\n", response.c_str());
  return false;
}

// Uploads one file via the XGSail device protocol: open a session_upload
// (with is_final=true — this is always a single-shot upload, never the
// incremental/chunked case in §4.2), PUT the bytes, and — only on
// failure — PATCH the upload as failed (§4.3). No PATCH is sent on
// success: is_final was already true on the opening POST, which is the
// documented "standard case"; the object-storage PUT completing is what
// actually finalizes the data on the backend side.
bool uploadFile(const char* filepath) {
  g_uploadSection = "uploadFile.open";
  uploadCount++;

  const char* lastSlash = strrchr(filepath, '/');
  const char* shortName = lastSlash ? lastSlash + 1 : filepath;
  strncpy(uploadCurrentFile, shortName, sizeof(uploadCurrentFile) - 1);
  uploadCurrentFile[sizeof(uploadCurrentFile) - 1] = '\0';
  String filename = String(shortName);

  // Feed watchdog before file operations
  yield();
  delay(10);

  if (!isClaimed()) {
    Serial.printf("[UPLOAD] Device not claimed, skipping: %s\n", filepath);
    uploadFailed++;
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[UPLOAD] WiFi disconnected, skipping: %s\n", filepath);
    uploadFailed++;
    return false;
  }

  File file = SD.open(filepath, FILE_READ);
  if (!file) {
    Serial.printf("[UPLOAD] Cannot open: %s\n", filepath);
    return false;
  }

  size_t fileSize = file.size();
  if (fileSize == 0) {
    Serial.printf("[UPLOAD] Skipping empty file: %s\n", filepath);
    file.close();
    return true;  // Mark as success so it gets .uploaded marker
  }

  String startedAt = sessionStartedAtIso(filepath);
  if (startedAt.length() == 0) {
    Serial.printf("[UPLOAD] Cannot determine started_at (no session-folder GPS timestamp, no live GPS fix), skipping: %s\n", filepath);
    file.close();
    uploadFailed++;
    return false;
  }

  Serial.printf("[UPLOAD] %s (%u bytes) heap:%u rssi:%d\n",
    filepath, (unsigned)fileSize, ESP.getFreeHeap(), WiFi.RSSI());

  yield();
  delay(10);

  JsonDocument openBody;
  openBody["started_at"] = startedAt;
  openBody["sequence_number"] = 0;
  openBody["is_final"] = true;
  openBody["filename"] = filename;
  // Per-session overrides picked from the app at recording start (see
  // storage.h's startLogging()) — omitted entirely when absent, so the
  // backend applies its own defaults (device.owner_boat_id / a fresh
  // solo activity) exactly as if this device had no opinion.
  String boatId = sessionBoatId(filepath);
  if (boatId.length() > 0) openBody["boat_id"] = boatId;
  String activityId = sessionActivityId(filepath);
  if (activityId.length() > 0) openBody["activity_id"] = activityId;
  String openBodyStr;
  serializeJson(openBody, openBodyStr);

  String openResponse;
  g_uploadSection = "uploadFile.open-session";
  int openCode = apiRequest("POST", "/api/devices/me/session-uploads", openBodyStr, openResponse, true);
  if (openCode != 201) {
    Serial.printf("[UPLOAD] session-uploads POST failed: HTTP %d\n", openCode);
    file.close();
    uploadFailed++;
    return false;
  }

  JsonDocument openResp;
  if (deserializeJson(openResp, openResponse)) {
    Serial.println("[UPLOAD] Malformed session-uploads response");
    file.close();
    uploadFailed++;
    return false;
  }
  const char* sessionUploadId = openResp["session_upload_id"] | "";
  const char* uploadUrl = openResp["upload_url"] | "";
  if (strlen(sessionUploadId) == 0 || strlen(uploadUrl) == 0) {
    Serial.println("[UPLOAD] Response missing session_upload_id/upload_url");
    file.close();
    uploadFailed++;
    return false;
  }

  bool putOk = putFileBytes(file, fileSize, String(uploadUrl), filename);
  file.close();

  if (!putOk) {
    JsonDocument failBody;
    failBody["status"] = "failed";
    String failBodyStr;
    serializeJson(failBody, failBodyStr);
    String path = String("/api/devices/me/session-uploads/") + sessionUploadId;
    String failResponse;
    apiRequest("PATCH", path.c_str(), failBodyStr, failResponse, true);
    uploadFailed++;
    return false;
  }

  uploadSuccess++;
  return true;
}

// Count files we will actually try to upload.
int countFilesToUpload(const char* dirname) {
  int count = 0;
  File root = SD.open(dirname);
  if (!root || !root.isDirectory()) return 0;

  File file = root.openNextFile();
  while (file) {
    char filepath[128];
    snprintf(filepath, sizeof(filepath), "%s/%s", dirname, file.name());

    if (file.isDirectory()) {
      count += countFilesToUpload(filepath);
    } else {
      String name = String(file.name());
      if (!name.endsWith(".uploaded") && !isUploaded(filepath)) {
        count++;
      }
    }
    file = root.openNextFile();
    yield();  // Feed watchdog
  }
  return count;
}

// Cheap DNS+TCP reachability check against config.api_base_url.
bool testApiConnectivity() {
  size_t freeHeap = ESP.getFreeHeap();
  Serial.printf("[UPLOAD] Testing API connectivity (heap: %u, RSSI: %d)...\n",
                freeHeap, WiFi.RSSI());

  bool isHttps; String host; int port; String path;
  if (strlen(config.api_base_url) == 0 || !parseUrl(String(config.api_base_url), isHttps, host, port, path)) {
    Serial.println("[UPLOAD] api_base_url not set or malformed");
    return false;
  }

  int rssi = WiFi.RSSI();
  if (rssi >= 0) {
    Serial.printf("[UPLOAD] WiFi not ready (RSSI: %d, expected negative)\n", rssi);
    return false;
  }

  IPAddress ip;
  if (!WiFi.hostByName(host.c_str(), ip)) {
    Serial.printf("[UPLOAD] DNS FAILED for %s\n", host.c_str());
    return false;
  }
  if (ip == IPAddress(0, 0, 0, 0)) {
    Serial.println("[UPLOAD] DNS returned 0.0.0.0 - network not ready");
    return false;
  }
  Serial.printf("[UPLOAD] DNS OK: %s -> %s\n", host.c_str(), ip.toString().c_str());
  yield();

  WiFiClient testClient;
  testClient.setTimeout(10);
  if (!testClient.connect(ip, port)) {
    Serial.printf("[UPLOAD] TCP port %d FAILED\n", port);
    return false;
  }
  testClient.stop();
  Serial.printf("[UPLOAD] TCP OK (port %d ready)\n", port);

  yield();
  delay(50);
  return true;
}

void uploadDirectory(const char* dirname) {
  // Feed watchdog before directory operations
  yield();
  delay(10);

  Serial.printf("[UPLOAD] Opening dir: %s\n", dirname);
  Serial.printf("[UPLOAD] Heap: %u, Stack: %u\n", ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(NULL));

  File root = SD.open(dirname);
  if (!root) {
    Serial.printf("[UPLOAD] Failed to open dir: %s\n", dirname);
    return;
  }
  if (!root.isDirectory()) {
    Serial.printf("[UPLOAD] Not a directory: %s\n", dirname);
    root.close();
    return;
  }

  File file = root.openNextFile();
  while (file) {
    // Feed watchdog on each iteration
    yield();

    char filepath[128];
    snprintf(filepath, sizeof(filepath), "%s/%s", dirname, file.name());

    if (file.isDirectory()) {
      // Recurse into subdirectories
      file.close();  // Close before recursing
      uploadDirectory(filepath);
    } else {
      // Skip marker files and already uploaded files
      String name = String(file.name());
      file.close();  // Close file handle before upload

      if (!name.endsWith(".uploaded") && !isUploaded(filepath)) {
        // Recording is button-triggered now, but still bail out of an
        // upload cycle once the boat is actually moving or already
        // recording, so a slow PUT doesn't contend with SD/WiFi the
        // operator needs mid-sail.
        if (logging || gps.speed_kts >= config.start_speed_knots) {
          Serial.println("[UPLOAD] Boat moving/recording, aborting upload");
          root.close();
          return;  // Exit uploadDirectory immediately
        }

        // Feed watchdog before upload
        esp_task_wdt_reset();
        yield();
        delay(100);

        if (uploadFile(filepath)) {
          markUploaded(filepath);
          cleanupIfAutoDelete(filepath);
        }
        esp_task_wdt_reset();  // and after — single PUT can be 8s+

        // Longer pause between uploads to prevent crashes. Use vTaskDelay
        // to properly yield to other tasks on this core.
        vTaskDelay(pdMS_TO_TICKS(200));

        // Do NOT call handleTelnet() here — that runs on Core 1 in the
        // main loop. WiFi stack and telnet globals are not thread-safe;
        // calling from Core 0 corrupts heap and crashes after upload
        // finishes (see firmware 2026.05.01.4 fleet crashes).
      }
    }

    // Feed watchdog before getting next file
    yield();
    file = root.openNextFile();
  }
  root.close();
}

// Try to connect to any configured WiFi network
// Returns true if connected, stores SSID in connectedSSID
bool connectWiFi() {
  connectedSSID[0] = '\0';

  if (config.wifi_count == 0) {
    Serial.println("[WIFI] No networks configured");
    return false;
  }

#if ENABLE_WIND
  // Pause BLE before any WiFi operation — shared radio. The first WiFi
  // call below is WiFi.disconnect(true) which reconfigures the radio;
  // doing that with a NimBLE scan in flight corrupts NimBLE state and
  // hangs Core 1 the next time it touches BLE.
  pauseBLEForWiFi();
#endif

  // Scan for networks first
  Serial.println("[WIFI] Scanning...");
  g_uploadSection = "wifi.scan";
  int n = WiFi.scanNetworks();
  Serial.printf("[WIFI] Found %d networks:\n", n);
  for (int i = 0; i < n && i < 10; i++) {
    wifi_auth_mode_t auth = WiFi.encryptionType(i);
    const char* authStr =
      auth == WIFI_AUTH_OPEN ? "OPEN" :
      auth == WIFI_AUTH_WEP ? "WEP" :
      auth == WIFI_AUTH_WPA_PSK ? "WPA" :
      auth == WIFI_AUTH_WPA2_PSK ? "WPA2" :
      auth == WIFI_AUTH_WPA_WPA2_PSK ? "WPA/WPA2" :
      auth == WIFI_AUTH_WPA3_PSK ? "WPA3" :
      auth == WIFI_AUTH_WPA2_WPA3_PSK ? "WPA2/WPA3" : "OTHER";
    Serial.printf("[WIFI]   %d: %s (%d dBm) %s ch%d\n",
      i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), authStr, WiFi.channel(i));
  }

  // BSSID-aware AP selection. The ESP32 Arduino stack's default
  // WiFi.begin(ssid, pass) picks "first BSSID that auth-completes",
  // not "strongest BSSID for that SSID". On a multi-AP mesh that
  // means a boat can latch onto a far AP at -90 dBm even when an
  // identical SSID is broadcasting at -50 dBm next to it. Observed
  // 2026-05-21 with E5: stuck on Family room AP at -90 dBm with
  // ~2.5 KB/s OTA throughput while Office AP (same room as boat)
  // was available at -45 dBm.
  //
  // Walk the scan once, find the strongest BSSID for each configured
  // SSID, copy the BSSID + channel into local storage (scan results
  // get freed below), then pass them to WiFi.begin to pin association.
  struct BestAp {
    bool   seen;
    int    rssi;
    int32_t channel;
    uint8_t bssid[6];
  };
  BestAp best[MAX_WIFI_NETWORKS];
  for (int s = 0; s < MAX_WIFI_NETWORKS; s++) {
    best[s].seen = false;
    best[s].rssi = -200;
    best[s].channel = 0;
  }
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);
    for (int s = 0; s < config.wifi_count; s++) {
      if (strlen(config.wifi[s].ssid) == 0) continue;
      if (ssid == config.wifi[s].ssid && rssi > best[s].rssi) {
        best[s].seen = true;
        best[s].rssi = rssi;
        best[s].channel = WiFi.channel(i);
        memcpy(best[s].bssid, WiFi.BSSID(i), 6);
      }
    }
  }
  for (int s = 0; s < config.wifi_count; s++) {
    if (strlen(config.wifi[s].ssid) == 0) continue;
    if (best[s].seen) {
      Serial.printf("[WIFI] Best AP for %s: %02X:%02X:%02X:%02X:%02X:%02X ch%d %d dBm\n",
        config.wifi[s].ssid,
        best[s].bssid[0], best[s].bssid[1], best[s].bssid[2],
        best[s].bssid[3], best[s].bssid[4], best[s].bssid[5],
        (int)best[s].channel, best[s].rssi);
    } else {
      Serial.printf("[WIFI] %s not visible in scan — will skip\n",
        config.wifi[s].ssid);
    }
  }
  WiFi.scanDelete();

  // Try each configured network, in config order, but skip ones not
  // visible in the scan (saves the 20 s per-network timeout when the
  // iPhone hotspot isn't around).
  for (int i = 0; i < config.wifi_count; i++) {
    if (strlen(config.wifi[i].ssid) == 0) continue;
    if (!best[i].seen) continue;

    Serial.printf("[WIFI] Trying %s (%d/%d) — pinned to strongest BSSID at %d dBm...\n",
      config.wifi[i].ssid, i + 1, config.wifi_count, best[i].rssi);
    g_uploadSection = "wifi.associate";

    // Ensure clean state before connecting
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    // Max TX power. The .04 era reduced this to 15 dBm to save battery,
    // but slow uploads at marginal signal are now the dominant operational
    // problem (suspected cause of the 2026-05-03 simultaneous reboot:
    // slow PUTs > previous 120s wdt budget). +4.5 dB of link margin
    // halves typical upload time when at the edge of AP range. WiFi only
    // runs during the post-sail upload window, so the average-current
    // cost across a day is ~1-2% of LiPo capacity. Watch /boot.log for
    // BROWNOUT entries — if low-SoC devices start tripping that, dial
    // back to 17 dBm or add an SoC-conditional setting.
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setSleep(false);    // keep RX live for the always-on ESP-NOW mesh once the upload window ends
    WiFi.persistent(false);  // Don't save to flash
    WiFi.setAutoReconnect(false);
    // Pin to the strongest-BSSID + channel from the scan above. This
    // bypasses the ESP32 stack's "first-respond-wins" AP picker.
    WiFi.begin(config.wifi[i].ssid, config.wifi[i].pass,
               best[i].channel, best[i].bssid);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts++ < 40) {  // 40 attempts = 20 sec
      delay(500);
      Serial.print(".");
      if (attempts % 10 == 0) {
        // Print WiFi status for debugging
        int status = WiFi.status();
        const char* statusStr =
          status == WL_IDLE_STATUS ? "IDLE" :
          status == WL_NO_SSID_AVAIL ? "NO_SSID" :
          status == WL_SCAN_COMPLETED ? "SCAN_DONE" :
          status == WL_CONNECTED ? "CONNECTED" :
          status == WL_CONNECT_FAILED ? "FAILED" :
          status == WL_CONNECTION_LOST ? "LOST" :
          status == WL_DISCONNECTED ? "DISCONNECTED" : "UNKNOWN";
        Serial.printf(" [%s] ", statusStr);
      }
      yield();  // Feed watchdog
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      strncpy(connectedSSID, config.wifi[i].ssid, sizeof(connectedSSID) - 1);
      Serial.printf("[WIFI] Connected to %s! IP: %s\n",
        connectedSSID, WiFi.localIP().toString().c_str());

      // Allow network stack to fully stabilize (DNS, routes, etc.)
      Serial.println("[WIFI] Waiting for network stack to stabilize...");
      delay(1000);
      yield();

      wifiConnected = true;

      // Telnet listener stays off by default — its WiFiServer/WiFiClient
      // calls into LWIP deadlock Core 1 when Core 0 is doing concurrent
      // HTTP uploads (firmware 2026.05.03.04 fleet hang). Enable at
      // runtime with serial command 'telneton'.
      if (telnetEnabled) {
        startTelnetServer();
      } else {
        Serial.println("[TELNET] Listener disabled (use 'telneton' to enable)");
      }

      // Trigger upload check on WiFi connect (reset timer so task checks immediately)
      lastUploadCheck = 0;
      uploadRetryCount = 0;  // Reset retries on new connection
      Serial.println("[WIFI] Upload check triggered on connect");

      return true;
    }

    WiFi.disconnect(true);
    delay(100);
    yield();  // Feed watchdog
  }

  Serial.println("[WIFI] All networks failed");
  return false;
}

// checkWiFiUpload() REMOVED — was racing with uploadTaskFunc() on Core 0.
// All upload logic now lives in uploadTaskFunc() (single owner, uses sdMutex).

// ============================================================
// Health snapshot (docs/device-protocol.md §4.4)
// ============================================================
// POST /api/devices/me/health with the 5 documented fields. Each call
// replaces the previous snapshot server-side (latest-wins) — no need to
// track a version or diff against the last push. Called once per boot
// from the upload task, piggybacking on whatever WiFi window is already
// open (upload cycle, or the periodic no-pending-uploads wake below).
bool uploadHealthSnapshot() {
  if (!wifiConnected || !isClaimed()) return false;

  JsonDocument body;
  body["battery_pct"] = battery.percent;
  body["battery_v"] = battery.voltage;
  body["heap_free"] = (uint32_t)ESP.getFreeHeap();
  body["firmware_version"] = FW_VERSION;
  body["uptime_s"] = (uint32_t)(millis() / 1000UL);

  String bodyStr;
  serializeJson(body, bodyStr);

  String response;
  int code = apiRequest("POST", "/api/devices/me/health", bodyStr, response, true);
  if (code == 200) {
    Serial.println("[HEALTH] Snapshot uploaded");
    return true;
  }
  Serial.printf("[HEALTH] Upload failed: HTTP %d\n", code);
  return false;
}

// ============================================================

void countPendingUploads() {
  // IMPORTANT: Skip counting while logging to avoid SD card conflicts
  // The logging task on Core 1 owns the SD card during recording
  if (!sdOK || logging) {
    // Don't change pendingUploads - keep last known value
    return;
  }

  // Try to get mutex, but don't block - skip if busy
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;  // SD busy, try again later
  }

  int navCount = 0;   // N: sessions with files still pending
  File root = SD.open("/sf");
  if (!root) {
    xSemaphoreGive(sdMutex);
    pendingUploads = 0;
    return;
  }

  // Walk each session, classifying unuploaded files. Stops at the first
  // pending file per session (the per-file flag flips and we move on).
  File session = root.openNextFile();
  while (session) {
    yield();  // Prevent watchdog timeout
    if (session.isDirectory()) {
      String sessName = session.name();
      if (!sessName.startsWith(".")) {
        bool hasPending = false;
        File f = session.openNextFile();
        while (f) {
          if (hasPending) {
            f.close();
            f = session.openNextFile();
            continue;
          }
          String fname = f.name();
          if (!fname.endsWith(".uploaded") && !fname.startsWith(".")) {
            String markerPath = String("/sf/") + sessName + "/" + fname + ".uploaded";
            if (!SD.exists(markerPath.c_str())) {
              hasPending = true;
            }
          }
          f.close();
          f = session.openNextFile();
        }
        if (hasPending) navCount++;
      }
    }
    session.close();
    session = root.openNextFile();
  }
  root.close();
  xSemaphoreGive(sdMutex);
  pendingUploads = navCount;
}

// ============================================================
// UPLOAD TASK (RUNS ON CORE 0)
// ============================================================
// Independent FreeRTOS task that prints Core 1's last-known section every 5s.
// If loopTask hangs, this task keeps running and the last printed [DIAG] line
// names the section Core 1 was inside. Replaces guesswork with evidence.
void diagnosticsTask(void* param) {
  esp_task_wdt_add(NULL);
  Serial.println("[DIAG] task started");
  uint32_t lastIter = 0;
  // Core 1 loop watchdog state: we let g_loopIter run for one diag tick
  // before we start the timer, so a fresh boot's first slow setup() pass
  // doesn't trip the watchdog.
  uint32_t loopWdLastIter = 0;
  unsigned long loopWdLastChangeMs = millis();

  while (true) {
    esp_task_wdt_reset();
    unsigned long now = millis();
    uint32_t iter = g_loopIter;
    long delta = (long)(iter - lastIter);
    // Suppress the heartbeat print while the live fleet dashboard is up, so
    // its 5 s lines don't corrupt the ANSI table. Watchdog logic below still runs.
    if (!g_fleetWatch)
      Serial.printf("[DIAG] uptime=%lus heap=%u sect=%s up=%s iter=%lu (+%ld)\n",
                    now / 1000, ESP.getFreeHeap(),
                    (const char*)g_loopSection,
                    (const char*)g_uploadSection,
                    (unsigned long)iter, delta);
    lastIter = iter;

    // ---------- Core 1 loop watchdog ----------
    // If g_loopIter hasn't moved in LOOP_HANG_MS, Core 1 is wedged
    // somewhere. Log the last-known section and force restart so the
    // hang becomes a recoverable `reset=SW` next boot instead of a
    // permanent black-screen brick.
    if (iter != loopWdLastIter) {
      loopWdLastIter = iter;
      loopWdLastChangeMs = now;
    } else if (now - loopWdLastChangeMs > LOOP_HANG_MS) {
      char line[128];
      snprintf(line, sizeof(line),
               "loop watchdog: Core 1 stuck at sect=%s for %lums — restart",
               (const char*)g_loopSection,
               (unsigned long)(now - loopWdLastChangeMs));
      appendBootLog(line);
      Serial.println(line);
      Serial.flush();
      delay(50);
      esp_restart();
    }

    // Every 5 minutes, append an "alive" line to /boot.log with wall-clock
    // + battery + heap. The last such line before the next boot is the
    // device's last known good moment — that gap is how we tell crash /
    // battery-died / clean-power-off apart.
    if (g_bootSessionLogged && now - g_lastAliveLog >= 5UL * 60UL * 1000UL) {
      char iso[24];
      if (formatGpsIso(iso, sizeof(iso))) {
        char line[96];
        snprintf(line, sizeof(line), "alive t=%s batt=%.2fV %d%% heap=%u",
                 iso, battery.voltage, battery.percent, ESP.getFreeHeap());
        appendBootLog(line);
        g_lastAliveLog = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Attempts the WiFi claim flow (docs/device-protocol.md §2) once per boot,
// using config.txt's claim_code. A no-op if already claimed or no code is
// configured. Device-side failures (400/404/409) are logged by
// claimDevice() itself and are not retried automatically — the operator
// gets a fresh code and either edits config.txt + reboots, or uses the
// `claim <CODE>` console command.
static bool g_claimAttemptedThisBoot = false;
static void attemptClaimIfNeeded() {
  if (isClaimed() || g_claimAttemptedThisBoot) return;
  g_claimAttemptedThisBoot = true;
  if (strlen(config.claim_code) == 0) return;
  Serial.println("[UPLOAD] Unclaimed device with config.txt claim_code — attempting claim...");
  claimDevice(config.claim_code);
}

void uploadTaskFunc(void* param) {
  Serial.println("[UPLOAD] Task started on Core 0");

  // Subscribe to the task watchdog so a hang here produces a backtrace
  // instead of a silent freeze. Reset at the top of every iteration.
  esp_err_t wdt_err = esp_task_wdt_add(NULL);
  if (wdt_err != ESP_OK) {
    Serial.printf("[WDT] Failed to subscribe uploadTask: %d\n", wdt_err);
  } else {
    Serial.println("[WDT] uploadTask subscribed");
  }

  static bool healthCheckedThisBoot = false;
  unsigned long stationaryStart = 0;  // track how long boat has been still
  unsigned long lastPendingCount = 0;  // Last time we counted pending uploads

  // Count pending uploads immediately on boot (don't wait 30 seconds)
  g_uploadSection = "count-pending-initial";
  countPendingUploads();
  Serial.printf("[UPLOAD] Initial pending: N=%d\n", pendingUploads);

  while (true) {
    g_uploadSection = "idle";
    esp_task_wdt_reset();
    unsigned long now = millis();
    bool shouldUpload = false;

    // Count pending uploads every 30 seconds (for display)
    if (now - lastPendingCount >= 30000) {
      g_uploadSection = "count-pending-periodic";
      lastPendingCount = now;
      countPendingUploads();
    }

    // Check various upload triggers
    if (triggerUpload && !logging) {
      // Recording just stopped - attempt upload (but respect recent WiFi failures)
      triggerUpload = false;

      // Force recount now that logging stopped (count was skipped during recording)
      countPendingUploads();
      Serial.printf("[UPLOAD] Recording stopped: N=%d pending\n", pendingUploads);

      if (pendingUploads == 0) {
        Serial.println("[UPLOAD] Nothing to upload");
      } else if (uploadRetryCount >= MAX_UPLOAD_RETRIES && now - lastUploadAttempt < UPLOAD_RETRY_DELAY_MS) {
        Serial.println("[UPLOAD] Recording stopped but WiFi backing off — will retry later");
      } else {
        shouldUpload = true;
        uploadRetryCount = 0;  // Reset retry counter for new session
        Serial.println("[UPLOAD] Triggered: recording stopped");
      }
    }
    else if (!logging && !uploading) {
      // Skip if no WiFi configured
      if (config.wifi_count == 0) {
        // No WiFi configured, skip
      }
      // Nothing pending to upload — but we still wake WiFi periodically
      // to push a health snapshot (and attempt a pending claim), so a
      // boat that's fully caught up doesn't go dark to the app forever.
      else if (pendingUploads == 0) {
        if (!healthCheckedThisBoot || (!isClaimed() && strlen(config.claim_code) > 0)) {
          if (gps.valid && gps.speed_kts >= 0.5) {
            stationaryStart = 0;  // boat moving — skip
          } else {
            if (stationaryStart == 0) stationaryStart = now;
            // Wait 30 s of stationary uptime before competing for the
            // radio (lets GPS get a fix, BNO settle, BLE wind connect).
            if (now - stationaryStart >= 30000 &&
                now - lastUploadCheck >= UPLOAD_CHECK_INTERVAL_MS) {
              lastUploadCheck = now;
              Serial.println("[UPLOAD] No pending uploads — waking WiFi for claim/health check");
              wifiBusy = true;
              if (!wifiConnected) { g_uploadSection = "wifi-connect.health-only"; connectWiFi(); }
              if (wifiConnected) {
                g_uploadSection = "claim.health-only";
                attemptClaimIfNeeded();
                if (!healthCheckedThisBoot) {
                  g_uploadSection = "health-upload.health-only";
                  if (uploadHealthSnapshot()) healthCheckedThisBoot = true;
                }
                // Release the radio either way.
                wifiTeardownRequested = true;
                wifiBusy = false;
              } else {
                Serial.println("[UPLOAD] WiFi connect failed for health-only check");
                wifiBusy = false;
              }
            }
          }
        }
      }
      // Only upload when stationary (speed < 0.5 kt) or no GPS fix
      // If no GPS fix, assume stationary (allow upload)
      else if (gps.valid && gps.speed_kts >= 0.5) {
        stationaryStart = 0;  // reset — boat is moving
      }
      else {
        if (stationaryStart == 0) stationaryStart = now;
        // No delay - connect immediately when stationary
        if (true) {
          // Stationary long enough — check periodic interval
          if (now - lastUploadCheck >= UPLOAD_CHECK_INTERVAL_MS) {
            lastUploadCheck = now;

            // Check retry backoff
            if (uploadRetryCount > 0 && now - lastUploadAttempt < UPLOAD_RETRY_DELAY_MS) {
              // Still in retry backoff period - skip
            } else if (uploadRetryCount >= MAX_UPLOAD_RETRIES) {
              // Max retries reached - wait longer before next attempt
              if (now - lastUploadAttempt >= UPLOAD_RETRY_DELAY_MS * 5) {
                uploadRetryCount = 0;  // Reset after extended wait
                shouldUpload = true;
                Serial.println("[UPLOAD] Triggered: retry reset");
              }
            } else {
              shouldUpload = true;
              Serial.printf("[UPLOAD] Triggered: periodic check (N=%d)\n",
                            pendingUploads);
            }
          }
        }
      }
    }

    // Perform upload if triggered
    if (shouldUpload) {
      lastUploadAttempt = now;

      // Mark WiFi as busy for the entire connect-+-upload window so Core 1
      // skips any LWIP-touching code paths (handleTelnet calls). Without
      // this guard, Core 1 deadlocks inside handleTelnet on LWIP mutex
      // contention during heavy uploads (see 2026.05.03.03 diag log: iter
      // frozen at handleTelnet for entire upload phase).
      wifiBusy = true;

      // Try to connect to WiFi if not connected
      if (!wifiConnected) {
        g_uploadSection = "wifi-connect";
        connectWiFi();
      }

      if (wifiConnected) {
        g_uploadSection = "claim";
        attemptClaimIfNeeded();

        // Set uploading=true to show UPLOADING screen
        uploading = true;
        uploadCount = 0;
        uploadSuccess = 0;
        uploadFailed = 0;
        uploadCurrentFile[0] = '\0';

        vTaskDelay(pdMS_TO_TICKS(100));

        Serial.printf("[UPLOAD] Starting (heap: %u, maxBlock: %u)\n",
                      ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

        if (!isClaimed()) {
          Serial.println("[UPLOAD] Device not claimed — skipping upload cycle (see 'claim <CODE>' or config.txt)");
          uploading = false;
          wifiTeardownRequested = true;
        } else {
          // Test connectivity before starting uploads (skip after repeated failures)
          bool connOK = true;
          if (uploadRetryCount < 2) {
            g_uploadSection = "api-conn-test";
            connOK = testApiConnectivity();
            if (!connOK) {
              Serial.println("[UPLOAD] Connectivity test failed");
            }
          } else {
            Serial.printf("[UPLOAD] Skipping conn test (retry %d), trying upload directly\n", uploadRetryCount);
          }

          if (!connOK && uploadRetryCount < 2) {
            uploading = false;
            uploadRetryCount++;
          } else {
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000))) {
              // Count files for progress display
              g_uploadSection = "count-files";
              uploadTotal = countFilesToUpload("/sf");
              Serial.printf("[UPLOAD] Found %d files to upload\n", uploadTotal);

              if (uploadTotal > 0) {
                g_uploadSection = "upload-dir";
                uploadDirectory("/sf");
              }
              xSemaphoreGive(sdMutex);
            }

            uploading = false;
            uploadRetryCount = 0;  // Reset on successful cycle
            Serial.println("[UPLOAD] Cycle complete");

            // After upload cycle, recount pending to get accurate number.
            // MUST hold sdMutex — countFilesToUpload walks the SD tree and races
            // with Core 1's logging/recording start otherwise.
            int remaining = -1;
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) {
              g_uploadSection = "count-files.post";
              remaining = countFilesToUpload("/sf");
              xSemaphoreGive(sdMutex);
            } else {
              Serial.println("[UPLOAD] Could not lock SD for recount — assuming 0");
              remaining = 0;
            }
            pendingUploads = (remaining > 0) ? remaining : 0;

            if (remaining <= 0) {
              // Health snapshot — once per boot, piggybacking on the same
              // WiFi window used for session uploads.
              if (!healthCheckedThisBoot) {
                g_uploadSection = "health-upload";
                if (uploadHealthSnapshot()) healthCheckedThisBoot = true;
              }

              // All done — request WiFi teardown. We do NOT tear down here on
              // Core 0: handleTelnet() runs on Core 1 in the main loop; tearing
              // down WiFi from Core 0 races against it (caused the 2026.05.01.4
              // post-upload crashes). The main loop sees this flag, gates on
              // !uploading && !triggerUpload, and performs the teardown safely
              // on Core 1.
              // Releasing WiFi is also required so the iPhone hotspot frees
              // a client slot for any boat that hasn't uploaded yet (only ~5
              // simultaneous clients allowed).
              Serial.println("[UPLOAD] All files uploaded — requesting WiFi teardown on Core 1");
              wifiTeardownRequested = true;
            } else {
              Serial.printf("[UPLOAD] %d files remaining — will retry\n", remaining);
            }
          }
        }
      } else {
        Serial.println("[UPLOAD] WiFi connect failed");
        uploadRetryCount++;
        if (uploadRetryCount >= MAX_UPLOAD_RETRIES) {
          Serial.println("[UPLOAD] Max WiFi retries — backing off 25 min");
        }
      }

      // Reset stationary timer to avoid rapid retries
      stationaryStart = 0;
      // After WiFi failure, force backoff by updating lastUploadAttempt
      lastUploadAttempt = now;

      // WiFi work for this trigger is done — let Core 1 service telnet again.
      wifiBusy = false;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
  }
}
