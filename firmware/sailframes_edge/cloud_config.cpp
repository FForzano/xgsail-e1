// Cloud config sync glue — see cloud_config.h.
#include "cloud_config.h"
#include "config.h"
#include "storage.h"
#include "ota.h"
#include "shared_state.h"
#include "upload.h"
#include <SD.h>
#include "mbedtls/sha256.h"

bool g_configSyncCheckedThisBoot = false;
int  g_cloud_config_version = -1;   // -1 = unknown / not fetched
bool     g_configRebootPending = false;
uint32_t g_configRebootAtMs = 0;

static const char* CLOUD_CONFIG_ALLOW_KEYS[] = {
    "wind_enabled",
    "wind_offset",
    "start_speed_knots",
    "stop_speed_knots",
    "start_delay_sec",
    "stop_delay_sec",
    "unit_role",
    "rtk_enabled",   // RTK operating mode — settable via cloud so the fleet's
                     // base/rover assignment is OTA-managed (not identity/conn,
                     // so consistent with the allow-list philosophy, gotcha #27).
                     // Applying it reconfigures the GNSS + reboots (gated on
                     // !armed && !logging by the cloud-apply path).
    nullptr
};

static bool isAllowedConfigKey(const String& key) {
    for (int i = 0; CLOUD_CONFIG_ALLOW_KEYS[i] != nullptr; i++) {
        if (key.equalsIgnoreCase(CLOUD_CONFIG_ALLOW_KEYS[i])) return true;
    }
    return false;
}


static String sha256OfString(const String& s) {
  uint8_t digest[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const uint8_t*)s.c_str(), s.length());
  mbedtls_sha256_finish(&ctx, digest);
  mbedtls_sha256_free(&ctx);
  return otaHexDigest(digest, 32);
}

// Rewrite /sf/config.txt with cloud key=value overrides merged in.
// Strategy: load current file into memory, replace allow-listed
// keys' lines with new values, append new keys not already present,
// force config_version=<cloudVersion>, then atomic rename through
// .tmp with .prev as one-deep backup.
static bool applyCloudConfigBody(const String& cloudBody, int cloudVersion) {
  // Load current config.txt into memory (line-by-line)
  String existing = "";
  {
    File f = SD.open("/config.txt", FILE_READ);
    if (f) {
      while (f.available()) existing += (char)f.read();
      f.close();
    }
  }
  if (existing.length() == 0) {
    Serial.println("[CFGSYNC] WARNING: local /config.txt empty/missing — bailing out for safety");
    return false;
  }

  // Parse cloud body into (key, val) pairs, allow-listed only
  struct KV { String k; String v; };
  static const int MAX_KV = 16;
  KV kv[MAX_KV];
  int kvCount = 0;
  int dropped = 0;
  int p = 0;
  while (p < (int)cloudBody.length() && kvCount < MAX_KV) {
    int nl = cloudBody.indexOf('\n', p);
    String line = (nl < 0) ? cloudBody.substring(p) : cloudBody.substring(p, nl);
    p = (nl < 0) ? cloudBody.length() : nl + 1;
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String k = line.substring(0, eq); k.trim();
    String v = line.substring(eq + 1); v.trim();
    if (k.equalsIgnoreCase("config_version")) continue;  // forced from manifest
    if (!isAllowedConfigKey(k)) { dropped++; continue; }
    kv[kvCount].k = k;
    kv[kvCount].v = v;
    kvCount++;
  }
  Serial.printf("[CFGSYNC] parsed cloud body: %d allowed, %d dropped\n", kvCount, dropped);

  // Build merged output: walk existing line-by-line, replace matching keys
  String out = "";
  bool kvUsed[MAX_KV] = {false};
  int existingP = 0;
  bool sawConfigVersion = false;
  while (existingP < (int)existing.length()) {
    int nl = existing.indexOf('\n', existingP);
    String line = (nl < 0) ? existing.substring(existingP) : existing.substring(existingP, nl);
    int origLen = (nl < 0) ? line.length() : nl - existingP + 1;
    existingP += origLen;

    String trimmed = line; trimmed.trim();
    if (trimmed.length() == 0 || trimmed.startsWith("#")) {
      out += line;
      if (nl >= 0) out += "\n";
      continue;
    }
    int eq = trimmed.indexOf('=');
    if (eq < 0) {
      out += line;
      if (nl >= 0) out += "\n";
      continue;
    }
    String k = trimmed.substring(0, eq); k.trim();

    if (k.equalsIgnoreCase("config_version")) {
      out += "config_version=" + String(cloudVersion) + "\n";
      sawConfigVersion = true;
      continue;
    }

    bool replaced = false;
    for (int i = 0; i < kvCount; i++) {
      if (k.equalsIgnoreCase(kv[i].k) && !kvUsed[i]) {
        out += kv[i].k + "=" + kv[i].v + "\n";
        kvUsed[i] = true;
        replaced = true;
        break;
      }
    }
    if (!replaced) {
      out += line;
      if (nl >= 0) out += "\n";
    }
  }
  if (!out.endsWith("\n")) out += "\n";
  // Append any cloud keys not already present
  for (int i = 0; i < kvCount; i++) {
    if (!kvUsed[i]) out += kv[i].k + "=" + kv[i].v + "\n";
  }
  if (!sawConfigVersion) out += "config_version=" + String(cloudVersion) + "\n";

  // Atomic rewrite: tmp → rename. Keep .prev as one-deep backup.
  if (SD.exists("/config.txt.tmp")) SD.remove("/config.txt.tmp");
  File tf = SD.open("/config.txt.tmp", FILE_WRITE);
  if (!tf) {
    Serial.println("[CFGSYNC] cannot open /config.txt.tmp");
    return false;
  }
  size_t wrote = tf.print(out);
  tf.flush();
  tf.close();
  if (wrote != out.length()) {
    Serial.printf("[CFGSYNC] short write: %u of %u\n",
                  (unsigned)wrote, (unsigned)out.length());
    SD.remove("/config.txt.tmp");
    return false;
  }
  // Verify the tmp file read back matches
  {
    File rf = SD.open("/config.txt.tmp", FILE_READ);
    if (!rf || (int)rf.size() != (int)out.length()) {
      Serial.println("[CFGSYNC] tmp read-back size mismatch");
      if (rf) rf.close();
      SD.remove("/config.txt.tmp");
      return false;
    }
    rf.close();
  }
  if (SD.exists("/config.txt.prev")) SD.remove("/config.txt.prev");
  if (!SD.rename("/config.txt", "/config.txt.prev")) {
    Serial.println("[CFGSYNC] rename .txt -> .prev failed");
    SD.remove("/config.txt.tmp");
    return false;
  }
  if (!SD.rename("/config.txt.tmp", "/config.txt")) {
    Serial.println("[CFGSYNC] rename .tmp -> .txt failed — restoring .prev");
    SD.rename("/config.txt.prev", "/config.txt");
    return false;
  }
  Serial.printf("[CFGSYNC] /config.txt rewritten (%u bytes, v%d)\n",
                (unsigned)out.length(), cloudVersion);
  return true;
}

bool performConfigSync() {
  if (!wifiConnected) return false;

  String host = String(config.s3_bucket) + ".s3." + String(config.s3_region) + ".amazonaws.com";
  String url = "http://" + host + "/config/" + String(config.boat_id) + "/latest.json";
  Serial.printf("[CFGSYNC] Fetching %s\n", url.c_str());

  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(10000);
  http.setReuse(false);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) {
    Serial.println("[CFGSYNC] http.begin failed");
    return false;
  }
  int code = http.GET();
  if (code == 404) {
    Serial.println("[CFGSYNC] No cloud config (404) — nothing to do");
    http.end();
    return true;  // not an error; expected default state
  }
  if (code != 200) {
    Serial.printf("[CFGSYNC] HTTP %d\n", code);
    http.end();
    return false;
  }
  String manifest = http.getString();
  http.end();

  long cloudVersion = otaExtractJsonNumber(manifest, "version");
  String bodyUrl    = otaExtractJsonString(manifest, "url");
  String sha256Hex  = otaExtractJsonString(manifest, "sha256");
  if (cloudVersion < 0) {
    Serial.println("[CFGSYNC] manifest missing 'version' — skipping");
    return false;
  }
  g_cloud_config_version = (int)cloudVersion;
  int localVersion = config.config_version;
  Serial.printf("[CFGSYNC] cloud v%d, local v%d\n",
                g_cloud_config_version, localVersion);

  if (g_cloud_config_version == localVersion) {
    Serial.printf("[CFGSYNC] up to date (v%d)\n", g_cloud_config_version);
    return true;
  }
  if (g_cloud_config_version < localVersion) {
    Serial.printf("[CFGSYNC] cloud older than local (v%d < v%d) — ignoring\n",
                  g_cloud_config_version, localVersion);
    return true;
  }

  // Stage 3.6 safety gates — defer apply, keep flag false so a later
  // post-race boot picks it up. We still mark checkedThisBoot=true
  // via the caller so we don't re-fetch the manifest 10x this boot,
  // but the defer path returns true to indicate "fetch OK, no apply".
  if (g_ocs.armed) {
    Serial.printf("[CFGSYNC] DEFER: OCS armed — won't rewrite config mid-race\n");
    appendBootLog("cfgsync defer=ocs-armed");
    return true;
  }
  if (logging) {
    Serial.printf("[CFGSYNC] DEFER: recording active — won't reboot mid-session\n");
    appendBootLog("cfgsync defer=logging");
    return true;
  }
  if (bodyUrl.length() == 0) {
    Serial.println("[CFGSYNC] manifest missing 'url' — cannot fetch body");
    return false;
  }

  Serial.printf("[CFGSYNC] cloud NEWER (v%d > v%d) — fetching body %s\n",
                g_cloud_config_version, localVersion, bodyUrl.c_str());

  WiFiClient client2;
  HTTPClient http2;
  http2.setConnectTimeout(8000);
  http2.setTimeout(10000);
  http2.setReuse(false);
  http2.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http2.begin(client2, bodyUrl)) {
    Serial.println("[CFGSYNC] body http.begin failed");
    return false;
  }
  int code2 = http2.GET();
  if (code2 != 200) {
    Serial.printf("[CFGSYNC] body HTTP %d\n", code2);
    http2.end();
    return false;
  }
  String body = http2.getString();
  http2.end();
  if (body.length() == 0 || body.length() > 4096) {
    Serial.printf("[CFGSYNC] body length %u out of bounds\n", (unsigned)body.length());
    return false;
  }

  if (sha256Hex.length() == 64) {
    String got = sha256OfString(body);
    if (!got.equalsIgnoreCase(sha256Hex)) {
      Serial.printf("[CFGSYNC] sha256 mismatch: got %s, want %s — aborting\n",
                    got.c_str(), sha256Hex.c_str());
      appendBootLog("cfgsync abort=sha256-mismatch");
      return false;
    }
    Serial.println("[CFGSYNC] sha256 OK");
  } else {
    Serial.println("[CFGSYNC] manifest has no sha256 — skipping integrity check");
  }

  Serial.printf("[CFGSYNC] cloud body (%u bytes):\n%s\n",
                (unsigned)body.length(), body.c_str());

  if (!applyCloudConfigBody(body, g_cloud_config_version)) {
    Serial.println("[CFGSYNC] apply failed");
    appendBootLog("cfgsync apply=failed");
    return false;
  }
  char line[80];
  snprintf(line, sizeof(line), "cfgsync applied cloud=v%d (was v%d) reboot=3s",
           g_cloud_config_version, localVersion);
  appendBootLog(line);
  Serial.println("[CFGSYNC] Apply OK. Rebooting in 3s.");

  // Schedule reboot — main loop drains diag + flushes any in-flight
  // serial before the actual restart. We don't restart here directly
  // because we're inside the upload task; an immediate esp_restart
  // would race with the diag heartbeat + watchdog deinit.
  g_configRebootPending = true;
  g_configRebootAtMs = millis() + 3000;
  return true;
}

