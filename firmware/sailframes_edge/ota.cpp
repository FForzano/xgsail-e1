// OTA + telnet console transport glue — see ota.h.
#include "ota.h"
#include "config.h"
#include "display.h"
#include "storage.h"
#include "shared_state.h"
#include "upload.h"
#include "console.h"
#include "recording.h"
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>
#include "mbedtls/sha256.h"

bool otaInProgress = false;

WiFiServer telnetServer(23);
WiFiClient telnetClient;
bool telnetEnabled = TELNET_ENABLED_DEFAULT;
bool telnetServerRunning = false;
String telnetBuffer = "";

volatile unsigned long g_otaDeadlineMs = 0;
volatile bool g_otaCheckedThisBoot = false;
const unsigned long OTA_MAX_MS   = 600UL * 1000UL;  // hard ceiling per OTA cycle
const unsigned long OTA_STALL_MS =  20UL * 1000UL;  // abort if no bytes received for 20 s
const unsigned long LOOP_HANG_MS =  90UL * 1000UL;  // Core 1 must tick at least every 90 s

void setupOTA() {
#if ENABLE_ARDUINO_OTA
  ArduinoOTA.setHostname(config.boat_id);  // Use boat ID as hostname
  ArduinoOTA.setPassword("sailframes");     // OTA password

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
    Serial.printf("[OTA] Start updating %s\n", type.c_str());

    // Show on display
    if (oledOK) {
      tft.fillScreen(COLOR_WARN);
      tft.setTextColor(TFT_BLACK, COLOR_WARN);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("OTA UPDATE", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 - 30, 4);
      tft.drawString("DO NOT POWER OFF", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 + 20, 2);
    }

    // Close log files before OTA
    if (logging) {
      navFile.close();
      if (imuFile) imuFile.close();
      logging = false;
    }
  });

  ArduinoOTA.onEnd([]() {
    otaInProgress = false;
    Serial.println("\n[OTA] Complete! Rebooting...");
    if (oledOK) {
      tft.fillScreen(COLOR_GOOD);
      tft.setTextColor(TFT_BLACK, COLOR_GOOD);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("REBOOTING...", SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 4);
    }
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int pct = progress / (total / 100);
    Serial.printf("[OTA] Progress: %u%%\r", pct);

    // Update progress bar on display
    if (oledOK) {
      tft.fillScreen(COLOR_WARN);
      tft.setTextColor(TFT_BLACK, COLOR_WARN);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("UPDATING FIRMWARE", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 - 60, 4);

      // Progress bar background (400px wide, 30px tall)
      int barX = 40, barY = SCREEN_HEIGHT/2 - 15;
      int barW = 400, barH = 30;
      tft.drawRect(barX, barY, barW, barH, TFT_BLACK);
      tft.fillRect(barX + 2, barY + 2, (barW - 4) * pct / 100, barH - 4, COLOR_GOOD);

      // Percentage text
      char buf[16];
      snprintf(buf, sizeof(buf), "%d%%", pct);
      tft.drawString(buf, SCREEN_WIDTH/2, SCREEN_HEIGHT/2 + 50, 4);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");

    if (oledOK) {
      tft.fillScreen(COLOR_ERROR);
      tft.setTextColor(TFT_WHITE, COLOR_ERROR);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("OTA ERROR!", SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 4);
    }
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");
#else
  Serial.println("[OTA] ArduinoOTA disabled in firmware (see ENABLE_ARDUINO_OTA)");
#endif
}


void startTelnetServer() {
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  telnetServerRunning = true;
  Serial.println("[TELNET] Server started on port 23");
}

void handleTelnet() {
  // Bail if the listener was never started OR if Core 0 is mid-upload.
  // telnetServer.hasClient() goes through LWIP and deadlocks under
  // sustained Core 0 traffic (firmware 2026.05.03.04 fleet hang).
  if (!telnetServerRunning || wifiBusy) return;

  // Check for new clients
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      telnetClient.println("\n=================================");
      telnetClient.printf("  SailFrames Edge %s\n", FW_VERSION);
      telnetClient.printf("  Boat: %s\n", config.boat_id);
      telnetClient.println("  Type 'help' for commands");
      telnetClient.println("=================================\n");
      telnetClient.print("> ");
      Serial.println("[TELNET] Client connected");
    } else {
      // Reject additional clients
      telnetServer.available().stop();
    }
  }

  // Handle client input
  if (telnetClient && telnetClient.connected()) {
    while (telnetClient.available()) {
      char c = telnetClient.read();
      if (c == '\n' || c == '\r') {
        if (telnetBuffer.length() > 0) {
          telnetClient.println();  // Echo newline
          processCommand(telnetBuffer, true);
          telnetBuffer = "";
          telnetClient.print("> ");
        }
      } else if (c == 127 || c == 8) {  // Backspace
        if (telnetBuffer.length() > 0) {
          telnetBuffer.remove(telnetBuffer.length() - 1);
          telnetClient.print("\b \b");  // Erase character
        }
      } else if (c >= 32 && c < 127) {  // Printable
        telnetBuffer += c;
        telnetClient.print(c);  // Echo
      }
    }
  }
}

// Print to both Serial and Telnet if connected

static String otaExtractJsonString(const String& json, const char* key) {
  String pattern = String("\"") + key + "\"";
  int k = json.indexOf(pattern);
  if (k < 0) return "";
  int colon = json.indexOf(':', k);
  if (colon < 0) return "";
  int q1 = json.indexOf('"', colon);
  if (q1 < 0) return "";
  int q2 = json.indexOf('"', q1 + 1);
  if (q2 < 0) return "";
  return json.substring(q1 + 1, q2);
}

static long otaExtractJsonNumber(const String& json, const char* key) {
  String pattern = String("\"") + key + "\"";
  int k = json.indexOf(pattern);
  if (k < 0) return -1;
  int colon = json.indexOf(':', k);
  if (colon < 0) return -1;
  int p = colon + 1;
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;
  int q = p;
  while (q < (int)json.length() && isdigit((unsigned char)json[q])) q++;
  if (q == p) return -1;
  return json.substring(p, q).toInt();
}

String otaHexDigest(const uint8_t* digest, size_t len) {
  static const char hex[] = "0123456789abcdef";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    out += hex[(digest[i] >> 4) & 0xF];
    out += hex[digest[i] & 0xF];
  }
  return out;
}

// OTA progress display — TFT was previously frozen on the last D2/D3
// frame during a 30-60 s firmware download, with no user-visible signal
// that anything was happening. Helper draws a one-time layout, then
// updates only the % number + progress bar on subsequent calls to keep
// SPI churn minimal during download. Pair the cadence to the existing
// 2-second serial log block so we never paint per-iteration.
static bool g_otaScreenDrawn = false;
static int  g_otaLastPctDrawn = -1;

static void drawOTAProgress(int percent, const char* targetVersion, const char* phase) {
  if (!oledOK) return;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  if (!g_otaScreenDrawn) {
    tft.fillScreen(COLOR_WARN);
    tft.setTextColor(TFT_BLACK, COLOR_WARN);
    tft.setTextDatum(MC_DATUM);
    // Header — font 4 is the largest text font (~26 px tall, full ASCII).
    tft.drawString("OTA UPDATE", SCREEN_WIDTH/2, 40, 4);
    // Target version — bumped from font 2 (~16 px) to font 4.
    if (targetVersion && targetVersion[0]) {
      tft.drawString(targetVersion, SCREEN_WIDTH/2, 90, 4);
    }
    // Progress bar frame — taller (40 px instead of 30).
    tft.drawRect(20, SCREEN_HEIGHT/2 + 60, SCREEN_WIDTH - 40, 40, TFT_BLACK);
    // Footer — also bumped to font 4 so it's legible from across the cockpit.
    tft.drawString("DO NOT POWER OFF", SCREEN_WIDTH/2, SCREEN_HEIGHT - 40, 4);
    g_otaScreenDrawn = true;
    g_otaLastPctDrawn = -1;
  }

  // Phase tag (e.g. "downloading", "verifying", "rebooting") shown above %.
  // Bumped to font 4 — the previous font 2 was unreadable across the cabin.
  if (phase && phase[0]) {
    tft.fillRect(0, 125, SCREEN_WIDTH, 34, COLOR_WARN);
    tft.setTextColor(TFT_BLACK, COLOR_WARN);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(phase, SCREEN_WIDTH/2, 142, 4);
  }

  if (percent != g_otaLastPctDrawn) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", percent);
    tft.fillRect(0, 175, SCREEN_WIDTH, 95, COLOR_WARN);
    tft.setTextColor(TFT_BLACK, COLOR_WARN);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buf, SCREEN_WIDTH/2, 220, 8);  // Font 8 = 75 px (digits)
    // Progress bar fill — matches the taller frame above.
    int barX = 21;
    int barY = SCREEN_HEIGHT/2 + 61;
    int barW = SCREEN_WIDTH - 42;
    int barH = 38;
    int fillW = (barW * percent) / 100;
    tft.fillRect(barX, barY, fillW, barH, TFT_BLACK);
    tft.fillRect(barX + fillW, barY, barW - fillW, barH, COLOR_WARN);
    g_otaLastPctDrawn = percent;
  }
}

// `manual` = true bypasses the one-shot per-boot guard. The serial
// `update` command sets it; auto-triggers from the upload task call
// with the default false, so they no-op after the first run.

bool performOTAUpdate(bool manual) {
  if (!manual && g_otaCheckedThisBoot) {
    Serial.println("[OTA] Already checked this boot — skipping. Use 'update' over serial to force a re-check.");
    return true;  // not an error; intended one-shot behaviour
  }
  // Mark BEFORE doing the work so a partial / failed run also counts
  // as "checked this boot". Prevents an upload-task retry loop from
  // hammering the manifest endpoint after every cycle.
  g_otaCheckedThisBoot = true;

  if (logging) {
    Serial.println("[OTA] Refusing: recording active. Stop recording first.");
    return false;
  }
  if (uploading || triggerUpload) {
    Serial.println("[OTA] Refusing: upload in flight.");
    return false;
  }

  if (!wifiConnected) {
    Serial.println("[OTA] WiFi not connected, attempting to connect...");
    if (!connectWiFi()) {
      Serial.println("[OTA] WiFi connect failed");
      return false;
    }
  }

  // OTA runs on ANY connected WiFi (SSID gate removed 2026-06-06 per request) —
  // the fleet should pick up firmware wherever it gets online (yacht club,
  // phone hotspot, home). Note: the ~1.5 MB pull will use hotspot data, and a
  // boat associating with an unfamiliar AP may update there. The stall + 180 s
  // deadline watchdogs (gotcha #22) still bound a bad download.
  Serial.printf("[OTA] on %s — proceeding (any-WiFi OTA)\n", connectedSSID);

  // Claim the radio for OTA. Same gates the upload task uses:
  //  - pauseBLEForWiFi() stops in-flight NimBLE scans / wind client.
  //  - uploading=true makes checkWindConnection() early-return.
  //  - wifiBusy=true blocks Core 1 LWIP-touching paths (telnet etc.).
  // Without this, BLE coexistence steals airtime mid-download and
  // throughput collapses to ~30 B/s.
  pauseBLEForWiFi();
  bool prevUploading = uploading;
  bool prevWifiBusy  = wifiBusy;
  uploading = true;
  wifiBusy  = true;

  // Park Core 1's display loop while we paint the OTA progress screen
  // from Core 0. TFT_eSPI is not thread-safe — concurrent calls from
  // both cores through the shared VSPI peripheral deadlock the bus
  // (observed on .15: 2 of 6 boats hung at "rebooting..." with
  // sect=display frozen, iter not advancing, while the diag task on
  // Core 0 kept printing). updateDisplay() early-returns when
  // otaInProgress is true, so only Core 0 touches the TFT during OTA.
  otaInProgress = true;

  // Arm the hang watchdog: diagnosticsTask will forcibly esp_restart()
  // if we don't return within OTA_MAX_MS. Clears at every exit point
  // below so a successful or cleanly-failing OTA never trips it.
  g_otaDeadlineMs = millis() + OTA_MAX_MS;
  appendBootLog("ota start");

  bool ok = performOTAUpdateBody();

  // On success the body calls ESP.restart() and never returns here;
  // on any failure path we land here and must release the radio AND
  // disarm the watchdog (otherwise diag would restart us shortly).
  g_otaDeadlineMs = 0;
  uploading = prevUploading;
  wifiBusy  = prevWifiBusy;
  otaInProgress = false;
  appendBootLog(ok ? "ota end ok" : "ota end fail");
  // After any OTA exit (other than the success path that restarts) we
  // may have painted the yellow OTA UPDATE screen. Force the next D2/D3
  // update to redraw fully so the user doesn't see the yellow framebuffer
  // bleed through with new numbers overlaid.
  d2LayoutDrawn = false;
  d3LayoutDrawn = false;
  g_otaScreenDrawn = false;
  return ok;
}

static bool performOTAUpdateBody() {
  Serial.printf("[OTA] WiFi RSSI: %d dBm, free heap: %u\n", WiFi.RSSI(), ESP.getFreeHeap());

  String host = String(config.s3_bucket) + ".s3." + String(config.s3_region) + ".amazonaws.com";
  String manifestUrl = "http://" + host + "/firmware/" + String(config.boat_id) + "/latest.json";

  Serial.printf("[OTA] Fetching manifest: %s\n", manifestUrl.c_str());

  WiFiClient mClient;
  HTTPClient mHttp;
  // Tighter than the previous 30 s — manifest is 200 bytes, so anything
  // beyond ~10 s means we're stuck in DNS/TCP, not waiting for data.
  // setConnectTimeout bounds the connect phase (HTTPClient honors it
  // separately from setTimeout which only covers receive).
  mHttp.setConnectTimeout(8000);
  mHttp.setTimeout(10000);
  mHttp.setReuse(false);
  mHttp.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!mHttp.begin(mClient, manifestUrl)) {
    Serial.println("[OTA] http.begin (manifest) failed");
    return false;
  }
  int code = mHttp.GET();
  if (code != 200) {
    Serial.printf("[OTA] Manifest GET failed: HTTP %d\n", code);
    mHttp.end();
    return false;
  }
  String manifest = mHttp.getString();
  mHttp.end();

  Serial.printf("[OTA] Manifest: %s\n", manifest.c_str());

  String version = otaExtractJsonString(manifest, "version");
  String binUrl  = otaExtractJsonString(manifest, "url");
  String sha256  = otaExtractJsonString(manifest, "sha256");
  long   size    = otaExtractJsonNumber(manifest, "size");

  if (version.isEmpty() || binUrl.isEmpty() || sha256.isEmpty() || size <= 0) {
    Serial.println("[OTA] Manifest missing required fields");
    return false;
  }

  Serial.printf("[OTA] Latest:  %s (%ld bytes)\n", version.c_str(), size);
  Serial.printf("[OTA] Current: %s\n", FW_VERSION);

  if (version == FW_VERSION) {
    Serial.println("[OTA] Already up to date.");
    return true;
  }

  if (binUrl.startsWith("https://")) {
    binUrl = "http://" + binUrl.substring(8);
  }
  Serial.printf("[OTA] Downloading: %s\n", binUrl.c_str());

  WiFiClient bClient;
  HTTPClient bHttp;
  // 300 s was much too generous and let HTTPClient's recv block for
  // five minutes on a stalled stream. The stall watchdog inside the
  // download loop now bounds true stalls at OTA_STALL_MS; this only
  // sets the per-call recv ceiling.
  bHttp.setConnectTimeout(8000);
  bHttp.setTimeout(20000);
  bHttp.setReuse(false);
  bHttp.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!bHttp.begin(bClient, binUrl)) {
    Serial.println("[OTA] http.begin (bin) failed");
    return false;
  }
  code = bHttp.GET();
  if (code != 200) {
    Serial.printf("[OTA] Binary GET failed: HTTP %d\n", code);
    bHttp.end();
    return false;
  }
  int contentLen = bHttp.getSize();
  if (contentLen > 0 && contentLen != (int)size) {
    Serial.printf("[OTA] Size mismatch: manifest=%ld, header=%d\n", size, contentLen);
    bHttp.end();
    return false;
  }

  if (!Update.begin((size_t)size, U_FLASH)) {
    Serial.printf("[OTA] Update.begin failed: %s\n", Update.errorString());
    bHttp.end();
    return false;
  }

  // Paint the OTA screen now that we're committed to writing flash.
  // The 2-second throttled block below updates the % from here on.
  g_otaScreenDrawn = false;
  drawOTAProgress(0, version.c_str(), "downloading...");

  mbedtls_sha256_context shaCtx;
  mbedtls_sha256_init(&shaCtx);
  mbedtls_sha256_starts(&shaCtx, 0);

  WiFiClient* stream = bHttp.getStreamPtr();
  uint8_t buf[4096];  // matches ESP32 flash sector size — Update.write batches per sector
  size_t total = 0;
  unsigned long lastLog = millis();
  unsigned long lastByteMs = millis();   // stall watchdog

  esp_task_wdt_reset();
  while (total < (size_t)size && (bHttp.connected() || stream->available())) {
    // Hard ceiling — gives up regardless of socket state. The diag
    // task would also catch this, but bailing here lets us clean up
    // (Update.abort, free SHA ctx) before the restart instead of
    // leaving the partition in a half-written state.
    if (g_otaDeadlineMs && (long)(millis() - g_otaDeadlineMs) > 0) {
      Serial.println("[OTA] Hard deadline hit — aborting download");
      Update.abort();
      mbedtls_sha256_free(&shaCtx);
      bHttp.end();
      return false;
    }
    size_t avail = stream->available();
    if (avail) {
      size_t toRead = avail > sizeof(buf) ? sizeof(buf) : avail;
      int n = stream->readBytes(buf, toRead);
      if (n <= 0) break;
      mbedtls_sha256_update(&shaCtx, buf, (size_t)n);
      size_t w = Update.write(buf, (size_t)n);
      if (w != (size_t)n) {
        Serial.printf("[OTA] Update.write short: %u/%d (%s)\n",
                      (unsigned)w, n, Update.errorString());
        Update.abort();
        mbedtls_sha256_free(&shaCtx);
        bHttp.end();
        return false;
      }
      total += n;
      lastByteMs = millis();
      if (millis() - lastLog > 2000) {
        int pct = (int)((100.0 * total) / size);
        Serial.printf("[OTA] %u / %ld bytes (%d%%)\n",
                      (unsigned)total, size, pct);
        drawOTAProgress(pct, version.c_str(), "downloading...");
        lastLog = millis();
        esp_task_wdt_reset();
      }
    } else {
      // No bytes ready. CRITICAL: the previous version called
      // delay(5) here without resetting the wdt and without bounding
      // how long a stall could last. With bHttp.connected() returning
      // true (TCP keep-alive) but the server stopped sending, the
      // loop would spin forever, never feed the wdt (esp_task_wdt_reset
      // was inside `if (avail)`), and the upload task would be wedged
      // — which is exactly what happened to E2/E4/E5 at 16:10 EDT.
      esp_task_wdt_reset();
      if (millis() - lastByteMs > OTA_STALL_MS) {
        Serial.printf("[OTA] Stall: no bytes for %lu ms — aborting\n",
                      (unsigned long)(millis() - lastByteMs));
        Update.abort();
        mbedtls_sha256_free(&shaCtx);
        bHttp.end();
        return false;
      }
      delay(5);
    }
    yield();
  }

  bHttp.end();

  if (total != (size_t)size) {
    Serial.printf("[OTA] Short download: %u/%ld\n", (unsigned)total, size);
    Update.abort();
    mbedtls_sha256_free(&shaCtx);
    return false;
  }

  uint8_t digest[32];
  mbedtls_sha256_finish(&shaCtx, digest);
  mbedtls_sha256_free(&shaCtx);

  String got = otaHexDigest(digest, 32);
  String want = sha256;
  want.toLowerCase();
  got.toLowerCase();
  if (got != want) {
    Serial.printf("[OTA] SHA256 mismatch:\n  got:  %s\n  want: %s\n",
                  got.c_str(), want.c_str());
    Update.abort();
    return false;
  }
  Serial.println("[OTA] SHA256 OK");
  drawOTAProgress(100, version.c_str(), "verifying...");

  if (!Update.end(true)) {
    Serial.printf("[OTA] Update.end failed: %s\n", Update.errorString());
    return false;
  }

  drawOTAProgress(100, version.c_str(), "rebooting...");
  Serial.printf("[OTA] Update OK. Rebooting into %s...\n", version.c_str());
  delay(1000);
  ESP.restart();
  return true;  // unreachable
}
