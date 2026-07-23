// WiFi + S3 upload pipeline glue — see upload.h.
#include "upload.h"
#include "config.h"
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
#include "ota.h"
#include "cloud_config.h"
#include "shared_state.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
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

// Threshold for using presigned URL - larger files bypass API Gateway timeout
// ESP32 uploads at ~20-50KB/s, API Gateway times out at 29s, so keep threshold low
#define PRESIGN_THRESHOLD 200000  // 200KB

// Extract presigned URL from JSON response
// Returns empty string if not found
String extractPresignedUrl(const String& json) {
  // Try with space: "url": "
  int urlStart = json.indexOf("\"url\": \"");
  if (urlStart >= 0) {
    urlStart += 8;  // Skip past "url": "
  } else {
    // Try without space: "url":"
    urlStart = json.indexOf("\"url\":\"");
    if (urlStart >= 0) {
      urlStart += 7;  // Skip past "url":"
    }
  }
  if (urlStart < 0) return "";
  int urlEnd = json.indexOf("\"", urlStart);
  if (urlEnd < 0) return "";
  return json.substring(urlStart, urlEnd);
}

// Upload directly to S3 using presigned URL (for large files)
bool uploadToS3Presigned(const char* filepath, File& file, size_t fileSize, const String& presignedUrl) {
  Serial.println("[UPLOAD] Using presigned S3 URL (direct upload)");
  Serial.printf("[UPLOAD] File size: %u bytes, heap: %u\n", fileSize, ESP.getFreeHeap());

  // Convert HTTPS to HTTP - S3 supports both, HTTP is faster (no TLS overhead)
  String httpUrl = presignedUrl;
  if (httpUrl.startsWith("https://")) {
    httpUrl = "http://" + httpUrl.substring(8);
    Serial.println("[UPLOAD] Using HTTP (no TLS) for faster upload");
  }

  WiFiClient s3Client;  // Plain HTTP, no TLS
  s3Client.setTimeout(300);  // 5 minute timeout for large files

  HTTPClient s3Http;
  s3Http.setTimeout(300000);  // 5 minute timeout
  s3Http.setReuse(false);

  // Connect to S3
  Serial.println("[UPLOAD] Connecting to S3...");
  if (!s3Http.begin(s3Client, httpUrl)) {
    Serial.println("[UPLOAD] Failed to begin S3 HTTP");
    return false;
  }

  // Determine content type
  String contentType = "application/octet-stream";
  if (String(filepath).endsWith(".csv")) {
    contentType = "text/csv";
  }
  s3Http.addHeader("Content-Type", contentType);
  s3Http.addHeader("Content-Length", String(fileSize));

  yield();

  Serial.printf("[UPLOAD] Starting PUT (%u bytes)...\n", fileSize);
  unsigned long startTime = millis();

  // Upload file directly to S3
  int httpCode = s3Http.sendRequest("PUT", &file, fileSize);

  unsigned long elapsed = (millis() - startTime) / 1000;
  Serial.printf("[UPLOAD] Request completed in %lu sec, heap: %u\n", elapsed, ESP.getFreeHeap());

  String response = s3Http.getString();
  s3Http.end();

  if (httpCode == 200 || httpCode == 201 || httpCode == 204) {
    Serial.printf("[UPLOAD] S3 Success: %s (HTTP %d)\n", filepath, httpCode);
    return true;
  } else {
    Serial.printf("[UPLOAD] S3 Failed: %s (HTTP %d)\n", filepath, httpCode);
    if (response.length() > 0 && response.length() < 500) {
      Serial.printf("[UPLOAD] S3 Response: %s\n", response.c_str());
    }
    return false;
  }
}

// Request presigned URL from API Gateway
String requestPresignedUrl(const char* filepath, size_t fileSize) {
  // Check WiFi before attempting request
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[UPLOAD] WiFi not connected, skipping presign request");
    return "";
  }

  Serial.printf("[UPLOAD] Requesting presigned URL (heap: %u)...\n", ESP.getFreeHeap());

  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(30);
  client.setTimeout(60);

  HTTPClient http;
  http.setTimeout(60000);  // 60 second timeout
  http.setReuse(false);

  // Build presign request URL
  String url = String(config.upload_url);
  url += "?boat=";
  url += config.boat_id;
  url += "&file=";
  url += filepath;
  url += "&presign=1&size=";
  url += String(fileSize);

  if (!http.begin(client, url)) {
    Serial.println("[UPLOAD] Failed to begin presign request");
    return "";
  }

  // Use POST with empty body (API Gateway only accepts POST/PUT)
  http.addHeader("Content-Type", "application/octet-stream");
  http.addHeader("Content-Length", "0");
  int httpCode = http.POST("");

  if (httpCode != 200) {
    Serial.printf("[UPLOAD] Presign request failed: HTTP %d\n", httpCode);
    http.end();
    return "";
  }

  String response = http.getString();
  http.end();

  Serial.printf("[UPLOAD] Response length: %d\n", response.length());
  if (response.length() < 500) {
    Serial.printf("[UPLOAD] Response: %s\n", response.c_str());
  } else {
    Serial.printf("[UPLOAD] Response (first 200): %.200s\n", response.c_str());
  }

  String presignedUrl = extractPresignedUrl(response);
  if (presignedUrl.length() == 0) {
    Serial.println("[UPLOAD] Failed to parse presigned URL from response");
    return "";
  }

  Serial.printf("[UPLOAD] Got presigned URL (%d chars)\n", presignedUrl.length());
  return presignedUrl;
}

// Extract date from filepath for S3 key
// Filepath format: /sf/20260405_225030/E1_nav.csv -> 2026-04-05
String extractDateFromPath(const char* filepath) {
  String path = String(filepath);

  // Find the session folder (e.g., "20260405_225030")
  int sfIdx = path.indexOf("/sf/");
  if (sfIdx >= 0) {
    int dateStart = sfIdx + 4;  // Skip "/sf/"
    if (path.length() > dateStart + 8) {
      String dateStr = path.substring(dateStart, dateStart + 8);
      // Convert YYYYMMDD to YYYY-MM-DD
      if (dateStr.length() == 8) {
        return dateStr.substring(0, 4) + "-" + dateStr.substring(4, 6) + "-" + dateStr.substring(6, 8);
      }
    }
  }

  // Fallback: use GPS date if available (format: DDMMYY)
  if (strlen(gps.date) >= 6 && (gps.date[4] != '0' || gps.date[5] != '0')) {
    // gps.date is DDMMYY, convert to YYYY-MM-DD
    char dateBuf[12];
    snprintf(dateBuf, sizeof(dateBuf), "20%c%c-%c%c-%c%c",
             gps.date[4], gps.date[5],  // YY
             gps.date[2], gps.date[3],  // MM
             gps.date[0], gps.date[1]); // DD
    return String(dateBuf);
  }

  // Last resort: use a placeholder
  return "unknown-date";
}

// Upload a single file directly to S3 via HTTP (no TLS)
// Bypasses API Gateway entirely - bucket policy allows unauthenticated PUT to raw/E1/*
bool uploadFile(const char* filepath) {
  g_uploadSection = "uploadFile.open";
  uploadCount++;

  // Extract short filename for display (main loop will update display)
  const char* lastSlash = strrchr(filepath, '/');
  const char* shortName = lastSlash ? lastSlash + 1 : filepath;
  strncpy(uploadCurrentFile, shortName, sizeof(uploadCurrentFile) - 1);
  uploadCurrentFile[sizeof(uploadCurrentFile) - 1] = '\0';

  // Don't call updateDisplay() here - it runs on Core 1, we're on Core 0
  // The main loop will pick up uploadCount/uploadCurrentFile changes

  // Feed watchdog before file operations
  yield();
  delay(10);

  // Verify WiFi is still connected
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

  // Skip empty files
  if (fileSize == 0) {
    Serial.printf("[UPLOAD] Skipping empty file: %s\n", filepath);
    file.close();
    return true;  // Mark as success so it gets .uploaded marker
  }

  Serial.printf("[UPLOAD] %s (%u bytes) heap:%u rssi:%d\n",
    filepath, fileSize, ESP.getFreeHeap(), WiFi.RSSI());

  // Feed watchdog
  yield();
  delay(10);

  // Extract filename from path
  String pathStr = String(filepath);
  int slashIdx = pathStr.lastIndexOf('/');
  String filename = (slashIdx >= 0) ? pathStr.substring(slashIdx + 1) : pathStr;

  // Extract date for S3 path organization
  String dateFolder = extractDateFromPath(filepath);

  // Build S3 URL: http://{bucket}.s3.{region}.amazonaws.com/raw/{boat_id}/{date}/{filename}
  String s3Url = "http://";
  s3Url += config.s3_bucket;
  s3Url += ".s3.";
  s3Url += config.s3_region;
  s3Url += ".amazonaws.com/raw/";
  s3Url += config.boat_id;
  s3Url += "/";
  s3Url += dateFolder;
  s3Url += "/";
  s3Url += filename;

  Serial.printf("[UPLOAD] S3 HTTP PUT: %s\n", s3Url.c_str());

  // Determine content type
  String contentType = "application/octet-stream";
  if (filename.endsWith(".csv")) {
    contentType = "text/csv";
  } else if (filename.endsWith(".rtcm3")) {
    contentType = "application/octet-stream";
  }

  // Manual chunked PUT (replaces HTTPClient::sendRequest). HTTPClient
  // blocks the entire body transmission with no esp_task_wdt_reset()
  // calls inside. For multi-MB files at typical link speeds (50-100
  // KB/s) a single PUT can take 3+ minutes; the 300 s task wdt fires
  // mid-upload (2026-05-25 event: 19.8 MB IMU CSV repeatedly tripped
  // wdt at ~300 s into the PUT, even though bytes were flowing).
  // setTimeout on HTTPClient is per-read, not total — useless against
  // genuinely-slow-but-progressing transfers.
  //
  // Doing the write loop ourselves lets us:
  //   - feed the task wdt every chunk (~4 KB)
  //   - run a no-progress stall watchdog independent of total elapsed
  //   - enforce a hard ceiling (10 min/file) so a truly stuck PUT
  //     bails without burning the task wdt
  String s3Host = String(config.s3_bucket) + ".s3." + String(config.s3_region) + ".amazonaws.com";
  String s3Path = "/raw/" + String(config.boat_id) + "/" + dateFolder + "/" + filename;

  WiFiClient client;
  g_uploadSection = "uploadFile.connect";
  if (!client.connect(s3Host.c_str(), 80, 10000)) {
    Serial.printf("[UPLOAD] TCP connect failed: %s\n", s3Host.c_str());
    file.close();
    uploadFailed++;
    return false;
  }

  // Headers
  g_uploadSection = "uploadFile.headers";
  client.printf("PUT %s HTTP/1.1\r\n", s3Path.c_str());
  client.printf("Host: %s\r\n", s3Host.c_str());
  client.printf("Content-Type: %s\r\n", contentType.c_str());
  client.printf("Content-Length: %u\r\n", (unsigned)fileSize);
  client.printf("x-amz-meta-boat-id: %s\r\n", config.boat_id);
  client.printf("x-amz-meta-original-path: %s\r\n", filepath);
  client.print("Connection: close\r\n\r\n");

  yield();
  esp_task_wdt_reset();

  // Body — chunked send with per-chunk wdt feed.
  // Static buf keeps 4 KB off the upload task's stack (only ~9 KB).
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
    if (now - lastProgress > 30000) {
      aborted = true; abortReason = "STALL_30S"; break;
    }
    if (now - startTime > 600000) {
      aborted = true; abortReason = "CEILING_10MIN"; break;
    }
    if (!client.connected()) {
      aborted = true; abortReason = "PEER_CLOSED"; break;
    }

    size_t want = (fileSize - sent < CHUNK) ? (fileSize - sent) : CHUNK;
    int r = file.read(buf, want);
    if (r <= 0) {
      aborted = true; abortReason = "SD_READ_FAILED"; break;
    }

    size_t w = client.write(buf, (size_t)r);
    if (w == 0) {
      aborted = true; abortReason = "SOCKET_WRITE_0"; break;
    }
    sent += w;
    lastProgress = millis();
  }

  unsigned long elapsed = (millis() - startTime) / 1000;
  file.close();

  if (aborted) {
    Serial.printf("[UPLOAD] Aborted: %s (%s) at %u/%u bytes after %lus\n",
                  filepath, abortReason, (unsigned)sent, (unsigned)fileSize, elapsed);
    client.stop();
    uploadFailed++;
    return false;
  }

  esp_task_wdt_reset();

  // Response — wait up to 60 s for S3 to start replying, then parse
  // status line and drain. 60 s is generous because S3 can take a
  // few seconds to acknowledge a large PUT.
  g_uploadSection = "uploadFile.response";
  int httpCode = -1;
  String response;
  unsigned long respDeadline = millis() + 60000;
  while (client.connected() && !client.available() && millis() < respDeadline) {
    esp_task_wdt_reset();
    yield();
    delay(10);
  }

  if (client.available()) {
    String statusLine = client.readStringUntil('\n');
    int sp1 = statusLine.indexOf(' ');
    int sp2 = statusLine.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > sp1) {
      httpCode = statusLine.substring(sp1 + 1, sp2).toInt();
    }
    // Drain remainder so connection closes cleanly + body is logged on errors.
    unsigned long drainDeadline = millis() + 5000;
    while (client.connected() && millis() < drainDeadline) {
      if (client.available()) {
        char c = client.read();
        if (response.length() < 500) response += c;
      } else {
        esp_task_wdt_reset();
        yield();
        delay(1);
      }
    }
  } else {
    Serial.println("[UPLOAD] No response from S3 within 60 s after upload");
  }

  client.stop();
  yield();
  delay(50);

  if (httpCode == 200 || httpCode == 201 || httpCode == 204) {
    Serial.printf("[UPLOAD] Success: %s (HTTP %d, %lus, %u bytes)\n",
                  filepath, httpCode, elapsed, (unsigned)fileSize);
    uploadSuccess++;
    return true;
  } else {
    const char* errMsg =
      (httpCode == -1)  ? "NO_RESPONSE" :
      (httpCode == 403) ? "FORBIDDEN (check bucket policy)" :
      (httpCode == 400) ? "BAD_REQUEST" : "";
    Serial.printf("[UPLOAD] Failed: %s (HTTP %d %s, %lus)\n",
                  filepath, httpCode, errMsg, elapsed);
    if (response.length() > 0 && response.length() < 500) {
      Serial.printf("[UPLOAD] Response: %s\n", response.c_str());
    }
    uploadFailed++;
    return false;
  }
}

// Returns true if this filename should NOT be uploaded on the current
// connected SSID. RTCM3 PPK files are large and only uploaded on the
// home network — on hotspots they're deferred until back at base.
static bool isSkippedForCurrentNetwork(const String& filename) {
  if (!filename.endsWith(".rtcm3")) return false;
  return strcmp(connectedSSID, HOME_WIFI_SSID) != 0;
}

// Count files we will actually try to upload on the current SSID.
// Files that would be skipped (e.g. RTCM3 on hotspot) are NOT counted —
// otherwise the post-upload "remaining" check sees them, never reaches 0,
// and we never request WiFi teardown.
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
      if (!name.endsWith(".uploaded") &&
          !isUploaded(filepath) &&
          !isSkippedForCurrentNetwork(name)) {
        count++;
      }
    }
    file = root.openNextFile();
    yield();  // Feed watchdog
  }
  return count;
}

// Test S3 connectivity via HTTP (no TLS needed)
bool testS3Connection() {
  size_t freeHeap = ESP.getFreeHeap();
  Serial.printf("[UPLOAD] Testing S3 connectivity (heap: %u, RSSI: %d)...\n",
                freeHeap, WiFi.RSSI());

  // Build S3 hostname
  String s3Host = String(config.s3_bucket) + ".s3." + String(config.s3_region) + ".amazonaws.com";

  // Check RSSI to verify WiFi is actually connected (should be negative)
  int rssi = WiFi.RSSI();
  if (rssi >= 0) {
    Serial.printf("[UPLOAD] WiFi not ready (RSSI: %d, expected negative)\n", rssi);
    return false;
  }

  // Test DNS
  IPAddress ip;
  if (!WiFi.hostByName(s3Host.c_str(), ip)) {
    Serial.printf("[UPLOAD] DNS FAILED for %s\n", s3Host.c_str());
    return false;
  }

  // Validate DNS returned a real IP (ESP32 can return 0.0.0.0 when network not ready)
  if (ip == IPAddress(0, 0, 0, 0)) {
    Serial.println("[UPLOAD] DNS returned 0.0.0.0 - network not ready");
    return false;
  }
  Serial.printf("[UPLOAD] DNS OK: %s -> %s\n", s3Host.c_str(), ip.toString().c_str());
  yield();

  // Test TCP connection to port 80 (HTTP)
  WiFiClient testClient;
  testClient.setTimeout(10);
  if (!testClient.connect(ip, 80)) {
    Serial.println("[UPLOAD] TCP port 80 FAILED");
    return false;
  }
  testClient.stop();
  Serial.println("[UPLOAD] TCP OK (HTTP ready)");

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

  Serial.println("[UPLOAD] Dir opened OK");
  yield();
  delay(50);

  Serial.println("[UPLOAD] Getting first file...");
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
        // Defer RTCM3 PPK files until back on home WiFi — too large for
        // mobile hotspots and not needed for in-event analytics.
        if (isSkippedForCurrentNetwork(name)) {
          Serial.printf("[UPLOAD] Skipping RTCM3 on %s (%s only): %s\n",
                        connectedSSID, HOME_WIFI_SSID, name.c_str());
          // Don't mark as uploaded — will upload when on home WiFi.
        } else {
          // Check if boat started moving - abort upload to allow recording to start
          if (gps.speed_kts >= config.start_speed_knots || recState == REC_ARMED) {
            Serial.println("[UPLOAD] Boat moving, aborting upload to allow recording");
            root.close();
            return;  // Exit uploadDirectory immediately
          }

          // Feed watchdog before upload
          esp_task_wdt_reset();
          yield();
          delay(100);

          if (uploadFile(filepath)) {
            markUploaded(filepath);
          }
          esp_task_wdt_reset();  // and after — single PUT can be 8s+
        }

        // Longer pause between uploads to prevent crashes
        // Use vTaskDelay to properly yield to other tasks on this core
        vTaskDelay(pdMS_TO_TICKS(200));

        // Do NOT call ArduinoOTA.handle()/handleTelnet() here — those run on
        // Core 1 in the main loop. WiFi stack and telnet globals are not
        // thread-safe; calling from Core 0 corrupts heap and crashes after
        // upload finishes (see firmware 2026.05.01.4 fleet crashes).
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

    // No display update - WiFi connects silently in background

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

      // Start OTA. Telnet listener stays off by default — its WiFiServer/
      // WiFiClient calls into LWIP deadlock Core 1 when Core 0 is doing
      // concurrent HTTP uploads (firmware 2026.05.03.04 fleet hang).
      // Enable at runtime with serial command 'telneton'.
      setupOTA();
      if (telnetEnabled) {
        startTelnetServer();
      } else {
        Serial.println("[TELNET] Listener disabled (use 'telneton' to enable)");
      }

      // No display update - connection is silent, status shown in status bar

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

// v2.0.0 Stage 3 — fleet health snapshot upload
// ============================================================
// Uploads a small JSON blob to S3 with current device state. Lets a
// cloud admin UI (TBD) get a fleet-wide health view without touching
// individual devices. Spec target: status/<boat_id>/latest.json.
// MVP target (this commit): raw/<boat_id>/_health.json so we stay
// under the existing FleetDirectHTTPUpload bucket policy (which
// covers raw/* only). Promote to status/<boat_id>/latest.json in a
// follow-up that also bumps the bucket policy.
//
// Called from the upload task after each successful WiFi acquisition.
// Cheap (sub-1 KB JSON, plain HTTP PUT). Failure is non-fatal — we
// just skip and try again next cycle.
static bool g_statusCheckedThisBoot = false;

// Scan /sf/ for the lexically largest folder name. The session folder
// naming convention is YYYYMMDD_HHMMSS so lex-max == chronological-max.
// Falls back to session_NNN fallback names if no GPS-timed folders
// exist. Returns "" if /sf/ is empty.
static void findLatestSessionFolder(char* out, size_t outlen) {
  out[0] = '\0';
  if (outlen == 0) return;
  File root = SD.open("/sf");
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    return;
  }
  File f = root.openNextFile();
  while (f) {
    if (f.isDirectory()) {
      const char* name = f.name();
      // Skip the SD root walker's leading slash if present.
      const char* base = strrchr(name, '/');
      base = base ? base + 1 : name;
      if (base[0] != '\0' && base[0] != '.' && strcmp(base, out) > 0) {
        strncpy(out, base, outlen - 1);
        out[outlen - 1] = '\0';
      }
    }
    f = root.openNextFile();
    yield();
  }
  root.close();
}

bool uploadStatusSnapshot() {
  if (!wifiConnected) return false;

  // Build JSON in a stack-local buffer. Keep under 1 KB.
  char body[1024];
  char ts[24] = "";
  formatGpsIso(ts, sizeof(ts));   // empty string if GPS time not yet valid

  const char* fixStr =
    gps.fix_quality == 2 ? "dgps" :
    gps.fix_quality == 1 ? "gps"  :
    "none";

  // SD free space — totalBytes/usedBytes return uint64_t, convert to MB.
  uint64_t sdTotal = SD.totalBytes();
  uint64_t sdUsed  = SD.usedBytes();
  uint32_t sdFreeMb = (sdTotal > sdUsed) ? (uint32_t)((sdTotal - sdUsed) / (1024ULL * 1024ULL)) : 0;

  // Latest /sf/<session>/ folder name. Empty if no sessions yet.
  char lastSail[32] = "";
  findLatestSessionFolder(lastSail, sizeof(lastSail));

  int written = snprintf(body, sizeof(body),
    "{"
    "\"version\":\"%s\","
    "\"boat_id\":\"%s\","
    "\"ts_iso\":\"%s\","
    "\"gps_fix\":\"%s\","
    "\"sats\":%d,"
    "\"hdop\":%.1f,"
    "\"last_position\":{\"lat\":%.7f,\"lon\":%.7f},"
    "\"battery_pct\":%d,"
    "\"battery_v\":%.2f,"
    "\"uptime_s\":%lu,"
    "\"free_heap\":%u,"
    "\"min_heap\":%u,"
    "\"espnow_peers\":%d,"
    "\"espnow_tx\":%lu,"
    "\"espnow_rx\":%lu,"
    "\"config_version\":%d,"
    "\"wifi_ssid\":\"%s\","
    "\"wifi_rssi\":%d,"
    "\"hardware_platform\":\"%s\","
    "\"unit_role\":\"%s\","
    "\"imu_ok\":%s,"
    "\"sd_ok\":%s,"
    "\"pending_uploads\":%d,"
    "\"sd_free_mb\":%lu,"
    "\"last_sail_folder\":\"%s\""
    "}",
    FW_VERSION,
    config.boat_id,
    ts,
    fixStr,
    gps.satellites,
    gps.hdop,
    gps.lat, gps.lon,
    battery.percent,
    battery.voltage,
    millis() / 1000UL,
    (unsigned)ESP.getFreeHeap(),
    (unsigned)esp_get_minimum_free_heap_size(),
    g_mesh_peer_count,
    (unsigned long)g_mesh_tx_count,
    (unsigned long)g_mesh_rx_count,
    config.config_version,
    connectedSSID,
    (int)WiFi.RSSI(),
    hwName(g_hw),
    roleName(g_role),
    imuOK ? "true" : "false",
    sdOK ? "true" : "false",
    pendingUploads,
    (unsigned long)sdFreeMb,
    lastSail);
  if (written < 0 || written >= (int)sizeof(body)) {
    Serial.println("[STATUS] JSON truncated, skip");
    return false;
  }

  String host = String(config.s3_bucket) + ".s3." + String(config.s3_region) + ".amazonaws.com";
  String url = "http://" + host + "/raw/" + String(config.boat_id) + "/_health.json";

  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(10000);
  http.setReuse(false);
  if (!http.begin(client, url)) {
    Serial.println("[STATUS] http.begin failed");
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT((uint8_t*)body, written);
  http.end();

  if (code == 200) {
    Serial.printf("[STATUS] uploaded (%d bytes) -> %s\n", written, url.c_str());
    return true;
  }
  Serial.printf("[STATUS] upload failed: HTTP %d\n", code);
  return false;
}

// ============================================================
// /boot.log → S3 upload (post-status, once per boot)
// ============================================================
// The diag/wdt boot log lives at /boot.log (SD root), so the
// recursive /sf walker in uploadDirectory never picks it up.
// This function PUTs the entire current /boot.log to
//   raw/<boat>/_boot.log
// once per boot, after the status snapshot. The file is
// append-only on the boat side — every upload contains the
// complete history, so post-flash boats give us their whole
// life history (back to 2026-05-05 when the file was first
// written) on first upload, then incremental tails as new
// alive/boot lines accrue. Used by the web battery dashboard.
static bool g_bootLogUploadedThisBoot = false;

bool uploadBootLogSnapshot() {
  if (!wifiConnected) return false;
  g_uploadSection = "bootlog.open";

  File file = SD.open("/boot.log", FILE_READ);
  if (!file) {
    Serial.println("[BOOTLOG] /boot.log not on SD — nothing to upload");
    return true;  // not an error per se
  }
  size_t fileSize = file.size();
  if (fileSize == 0) {
    Serial.println("[BOOTLOG] /boot.log empty");
    file.close();
    return true;
  }

  Serial.printf("[BOOTLOG] Uploading /boot.log (%u bytes, heap %u, rssi %d)\n",
                (unsigned)fileSize, ESP.getFreeHeap(), WiFi.RSSI());

  String s3Host = String(config.s3_bucket) + ".s3." + String(config.s3_region) + ".amazonaws.com";
  String s3Path = "/raw/" + String(config.boat_id) + "/_boot.log";

  WiFiClient client;
  g_uploadSection = "bootlog.connect";
  if (!client.connect(s3Host.c_str(), 80, 10000)) {
    Serial.printf("[BOOTLOG] TCP connect failed: %s\n", s3Host.c_str());
    file.close();
    return false;
  }

  g_uploadSection = "bootlog.headers";
  client.printf("PUT %s HTTP/1.1\r\n", s3Path.c_str());
  client.printf("Host: %s\r\n", s3Host.c_str());
  client.print("Content-Type: text/plain\r\n");
  client.printf("Content-Length: %u\r\n", (unsigned)fileSize);
  client.print("Connection: close\r\n\r\n");

  yield();
  esp_task_wdt_reset();

  // Body — same chunked-PUT pattern as uploadFile (per-chunk wdt feed,
  // stall watchdog, hard ceiling). boot.log is small (~100 KB after
  // ~3 weeks) so the 2-min ceiling is plenty even on weak signal.
  g_uploadSection = "bootlog.body";
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
    if (now - startTime > 120000)   { aborted = true; abortReason = "CEILING_2MIN"; break; }
    if (!client.connected())        { aborted = true; abortReason = "PEER_CLOSED"; break; }

    size_t want = (fileSize - sent < CHUNK) ? (fileSize - sent) : CHUNK;
    int r = file.read(buf, want);
    if (r <= 0) { aborted = true; abortReason = "SD_READ_FAILED"; break; }
    size_t w = client.write(buf, (size_t)r);
    if (w == 0) { aborted = true; abortReason = "SOCKET_WRITE_0"; break; }
    sent += w;
    lastProgress = millis();
  }

  unsigned long elapsed = (millis() - startTime) / 1000;
  file.close();

  if (aborted) {
    Serial.printf("[BOOTLOG] Aborted: %s at %u/%u bytes after %lus\n",
                  abortReason, (unsigned)sent, (unsigned)fileSize, elapsed);
    client.stop();
    return false;
  }

  g_uploadSection = "bootlog.response";
  int httpCode = -1;
  unsigned long respDeadline = millis() + 30000;
  while (client.connected() && !client.available() && millis() < respDeadline) {
    esp_task_wdt_reset();
    yield();
    delay(10);
  }
  if (client.available()) {
    String statusLine = client.readStringUntil('\n');
    int sp1 = statusLine.indexOf(' ');
    int sp2 = statusLine.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > sp1) {
      httpCode = statusLine.substring(sp1 + 1, sp2).toInt();
    }
    unsigned long drainDeadline = millis() + 3000;
    while (client.connected() && millis() < drainDeadline) {
      if (client.available()) client.read();
      else { esp_task_wdt_reset(); yield(); delay(1); }
    }
  }
  client.stop();

  if (httpCode >= 200 && httpCode < 300) {
    Serial.printf("[BOOTLOG] Uploaded OK (%u bytes, %lus, HTTP %d)\n",
                  (unsigned)fileSize, elapsed, httpCode);
    return true;
  }
  Serial.printf("[BOOTLOG] Failed: HTTP %d (%lus)\n", httpCode, elapsed);
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

    // ---------- OTA hard deadline ----------
    // performOTAUpdate arms g_otaDeadlineMs at start and clears it at
    // every exit. If we see a non-zero deadline that's already past,
    // OTA is wedged (the 2026-05-05 16:10-EDT 3-of-6 hang signature).
    // Force a restart so the device doesn't sit indefinitely with
    // wifiBusy/uploading flags stuck.
    if (g_otaDeadlineMs && (long)(now - g_otaDeadlineMs) > 0) {
      char line[96];
      snprintf(line, sizeof(line),
               "ota watchdog: deadline exceeded at sect=%s — restart",
               (const char*)g_loopSection);
      appendBootLog(line);
      Serial.println(line);
      Serial.flush();
      delay(50);
      esp_restart();
    }

    // ---------- Core 1 loop watchdog ----------
    // If g_loopIter hasn't moved in LOOP_HANG_MS, Core 1 is wedged
    // somewhere. Log the last-known section and force restart so the
    // hang becomes a recoverable `reset=SW` next boot instead of a
    // permanent black-screen brick. Skipped while OTA is intentionally
    // running — flash writes can pause Core 1 for many seconds.
    if (iter != loopWdLastIter) {
      loopWdLastIter = iter;
      loopWdLastChangeMs = now;
    } else if (g_otaDeadlineMs == 0 && now - loopWdLastChangeMs > LOOP_HANG_MS) {
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
      if (config.wifi_count == 0 || !strlen(config.upload_url)) {
        // No WiFi configured, skip
      }
      // Nothing pending to upload — but we still need to wake WiFi
      // periodically to check for new firmware. Without this branch a
      // boat that's fully caught up never connects, never checks the
      // OTA manifest, and never updates. Diagnosed 2026-05-16 after
      // the fleet missed multiple firmware pushes despite booting at
      // home on Home-IOT. The check runs at most once per boot
      // (g_otaCheckedThisBoot, enforced inside performOTAUpdate); the
      // stationary + interval gates avoid waking the radio while the
      // boat is on the water about to record.
      else if (pendingUploads == 0) {
        if (!g_otaCheckedThisBoot) {
          // Track stationary time the same way the upload branch does.
          if (gps.valid && gps.speed_kts >= 0.5) {
            stationaryStart = 0;  // boat moving — skip
          } else {
            if (stationaryStart == 0) stationaryStart = now;
            // Wait 30 s of stationary uptime before competing for the
            // radio (lets GPS get a fix, BNO settle, BLE wind connect).
            if (now - stationaryStart >= 30000 &&
                now - lastUploadCheck >= UPLOAD_CHECK_INTERVAL_MS) {
              lastUploadCheck = now;
              Serial.println("[OTA] No pending uploads — running OTA-only check");
              wifiBusy = true;
              if (!wifiConnected) { g_uploadSection = "wifi-connect.ota-only"; connectWiFi(); }
              if (wifiConnected) {
                g_uploadSection = "ota-only";
                performOTAUpdate(false);   // version gate only (any-WiFi OTA)
                // Stage 3: piggyback fleet health snapshot on the same
                // WiFi window. Once per boot — boats that idle on
                // Home-IOT for hours don't need to spam status PUTs.
                if (!g_statusCheckedThisBoot) {
                  g_uploadSection = "status-upload.ota-only";
                  if (uploadStatusSnapshot()) g_statusCheckedThisBoot = true;
                }
                if (!g_bootLogUploadedThisBoot) {
                  g_uploadSection = "bootlog-upload.ota-only";
                  if (uploadBootLogSnapshot()) g_bootLogUploadedThisBoot = true;
                }
                // Stage 3.5: cloud config sync (observe-only MVP).
                if (!g_configSyncCheckedThisBoot) {
                  g_uploadSection = "cfgsync.ota-only";
                  if (performConfigSync()) g_configSyncCheckedThisBoot = true;
                }
                // Release the radio whether OTA happened or not.
                wifiTeardownRequested = true;
                // Clear wifiBusy so (a) the Core 1 teardown block can
                // proceed (it gates on !wifiBusy) and (b) meshTick can
                // resume broadcasting. Previously left stuck true after
                // "Already up to date" returns since OTA didn't restart
                // — observed on .14 as tx=N frozen on the canary boat.
                wifiBusy = false;
              } else {
                Serial.println("[OTA] WiFi connect failed for OTA-only check");
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
      // skips any LWIP-touching code paths (handleTelnet, telnetServer
      // calls). Without this guard, Core 1 deadlocks inside handleTelnet
      // on LWIP mutex contention during heavy uploads (see 2026.05.03.03
      // diag log: iter frozen at handleTelnet for entire upload phase).
      wifiBusy = true;

      // Try to connect to WiFi if not connected
      if (!wifiConnected) {
        g_uploadSection = "wifi-connect";
        connectWiFi();
      }

      if (wifiConnected) {
        // Set uploading=true to show UPLOADING screen
        uploading = true;
        uploadCount = 0;
        uploadSuccess = 0;
        uploadFailed = 0;
        uploadCurrentFile[0] = '\0';

        vTaskDelay(pdMS_TO_TICKS(100));

        Serial.printf("[UPLOAD] Starting (heap: %u, maxBlock: %u)\n",
                      ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

        // Test connectivity before starting uploads (skip after repeated failures)
        bool connOK = true;
        if (uploadRetryCount < 2) {
          g_uploadSection = "s3-conn-test";
          connOK = testS3Connection();
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
            // Auto-OTA: at most ONCE per boot. The first clean upload
            // cycle on Home-IOT WiFi triggers an OTA manifest check;
            // every subsequent clean cycle is a no-op (the function's
            // g_otaCheckedThisBoot flag flips on first entry). Keeps
            // the day-of-racing behaviour simple — the OTA either
            // happened at boot or it doesn't happen, no surprise
            // mid-day reboots if a new build gets published. Operator
            // can force a re-check via the serial 'update' command.
            Serial.println("[UPLOAD] Cycle clean — checking OTA manifest (one-shot per boot)");
            g_uploadSection = "ota-check";
            performOTAUpdate(false);

            // Stage 3 status snapshot upload — once per boot. Piggybacks
            // on the same WiFi window used for OTA + session uploads.
            if (!g_statusCheckedThisBoot) {
              g_uploadSection = "status-upload";
              if (uploadStatusSnapshot()) g_statusCheckedThisBoot = true;
            }
            // /boot.log upload — once per boot, after status. Surfaces
            // alive-heartbeat history to the web battery dashboard.
            if (!g_bootLogUploadedThisBoot) {
              g_uploadSection = "bootlog-upload";
              if (uploadBootLogSnapshot()) g_bootLogUploadedThisBoot = true;
            }
            // Stage 3.5 cloud config sync (observe-only MVP).
            if (!g_configSyncCheckedThisBoot) {
              g_uploadSection = "cfgsync";
              if (performConfigSync()) g_configSyncCheckedThisBoot = true;
            }

            // All done — request WiFi teardown. We do NOT tear down here on
            // Core 0: ArduinoOTA.handle()/handleTelnet() run on Core 1 in the
            // main loop; tearing down WiFi from Core 0 races against them
            // (caused the 2026.05.01.4 post-upload crashes). The main loop
            // sees this flag, gates on !uploading && !triggerUpload, and
            // performs the teardown safely on Core 1.
            // Releasing WiFi is also required so the iPhone hotspot frees
            // a client slot for any boat that hasn't uploaded yet (only ~5
            // simultaneous clients allowed).
            Serial.println("[UPLOAD] All files uploaded — requesting WiFi teardown on Core 1");
            wifiTeardownRequested = true;
          } else {
            Serial.printf("[UPLOAD] %d files remaining — will retry\n", remaining);
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

// ============================================================
