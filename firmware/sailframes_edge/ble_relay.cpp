// BLE GATT relay glue — see ble_relay.h. Wire-level contract for the
// identity/provisioning/session_manifest/session_data/control
// characteristics: xgsail's docs/device-protocol.md §8.2 (cross-check
// those against that doc before changing anything, not against this
// file's history). device_config and control's calibrate* commands are
// an E1-specific extension — see docs/ble-config.md instead.
#include "ble_relay.h"
#include "config.h"
#include "device_auth.h"
#include "upload.h"
#include "storage.h"
#include "recording.h"
#include "mesh.h"
#include "imu.h"
#include "gnss.h"
#include "battery.h"
#include "pressure.h"
#include "wind_sensor.h"  // bleInitialized — shared with the wind-sensor central role
#include "shared_state.h"  // sdMutex
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <SD.h>

static const char* SERVICE_UUID       = "24e6db2c-3c8a-4b5b-ba5a-23bc4c818046";
static const char* IDENTITY_UUID      = "985a1aae-858e-4727-9d5c-c8670bd6bd06";
static const char* PROVISIONING_UUID  = "db2c2e63-9e13-4fa9-867c-0b579ce2ae57";
static const char* MANIFEST_UUID      = "ed9efdc8-70d4-4ce5-a0a3-9fa6d88b9b9e";
static const char* SESSION_DATA_UUID  = "728d2815-0409-49ce-ad73-ecca6fc6d981";
static const char* CONTROL_UUID       = "ec88dd3e-2562-420c-aebe-30a4ae40bdf9";
// E1-specific extensions (not in xgsail's docs/device-protocol.md) — see
// docs/ble-config.md.
static const char* DEVICE_CONFIG_UUID = "042dfd7c-88f4-4ae8-af9a-eb1d7be7a3c6";
static const char* STATUS_UUID        = "bfef7865-f3f7-486c-93fe-bbae78cfdc43";

// Short timeout for sdMutex acquisition from BLE callbacks. These run on
// the NimBLE host's own FreeRTOS task — a third execution context beyond
// Core 1's loop() and Core 0's upload task — so every SD access reachable
// from a BLE callback must take sdMutex like Core 0's upload path does;
// skip-if-busy (not a hard block) matches the existing convention
// (storage.cpp's appendBootLog(), upload.cpp's countPendingUploads()).
static const TickType_t BLE_SD_MUTEX_TIMEOUT = pdMS_TO_TICKS(500);

static NimBLECharacteristic* pManifestChar = nullptr;
static NimBLECharacteristic* pSessionDataChar = nullptr;

// In-flight session_data transfer state (§8.2's session_data: notify,
// chunked, 4-byte BE sequence index + raw bytes, no end marker — the app
// knows it's done once it has the manifest entry's byte_size).
static bool   s_xferActive = false;
static File   s_xferFile;
static String s_xferPath;      // also the manifest's "session_id" token
static uint32_t s_xferSeq = 0;
static uint16_t s_connHandle = 0;
static bool s_haveConn = false;
static NimBLEServer* s_pServer = nullptr;

// Pairing window (config.h's BLE_BOND_WINDOW_MS), opened by button.cpp's
// long-press handler. 0 = closed. Gates writes to the two
// secret-carrying characteristics (provisioning, device_config) for a
// connection NimBLE doesn't already recognize as bonded — see
// bondWriteAllowed() and docs/ble-config.md.
static unsigned long s_bondWindowUntil = 0;

static bool bondWindowOpen() {
  return s_bondWindowUntil != 0 && (long)(millis() - s_bondWindowUntil) < 0;
}

void bleOpenBondWindow() {
  s_bondWindowUntil = millis() + BLE_BOND_WINDOW_MS;
  // Actually flip the SMP-level bonding flag, not just our app-side gate:
  // with bonding off (the default, set below in bleRelayInit()), the
  // WRITE_ENC permission on provisioning/device_config only demands an
  // *encrypted* link, which a transient (non-bonded) pairing still
  // satisfies — so a first-time bond that should persist for the owner's
  // phone needs bonding actually enabled while the window is open.
  // bleRelayTick() flips it back off once the window closes; an existing
  // stored bond keeps reconnecting fine regardless of this flag (bondable
  // mode only governs *new* pairing, not re-establishing an existing bond).
  NimBLEDevice::setSecurityAuth(true, true, true);
  Serial.printf("[BLE] Pairing window open for %lus\n", BLE_BOND_WINDOW_MS / 1000UL);
}

// A write is allowed either because the connection is already a
// recognized bond from an earlier pairing (the owner's phone reconnecting
// normally — no button press needed), or because the pairing window is
// currently open (first-time bonding, gated behind the long-press).
static bool bondWriteAllowed(NimBLEConnInfo& connInfo) {
  return connInfo.isBonded() || bondWindowOpen();
}

// Recursively collects every pending (not yet uploaded) file under /sf,
// same "file IS the upload unit" model as upload.cpp's countFilesToUpload/
// uploadDirectory — the backend's session_upload is per-file here (one
// CSV = one session-uploads call), so each pending file is its own
// manifest entry, keyed by its SD path (the "device-local token" the plan
// calls for — the device never learns the backend's real session_id).
static void collectPendingFiles(const char* dirname, JsonArray& out) {
  File root = SD.open(dirname);
  if (!root || !root.isDirectory()) return;

  File file = root.openNextFile();
  while (file) {
    char filepath[128];
    snprintf(filepath, sizeof(filepath), "%s/%s", dirname, file.name());

    if (file.isDirectory()) {
      file.close();
      collectPendingFiles(filepath, out);
    } else {
      String name = String(file.name());
      size_t size = file.size();
      file.close();
      if (!name.endsWith(".uploaded") && !isUploaded(filepath)) {
        JsonObject entry = out.add<JsonObject>();
        entry["session_id"] = filepath;
        entry["byte_size"] = (uint32_t)size;
        entry["started_at"] = sessionStartedAtIso(filepath);
        entry["ended_at"] = nullptr;
      }
    }
    file = root.openNextFile();
    yield();
  }
  root.close();
}

static String buildManifestJson() {
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  if (xSemaphoreTake(sdMutex, BLE_SD_MUTEX_TIMEOUT) == pdTRUE) {
    collectPendingFiles("/sf", arr);
    xSemaphoreGive(sdMutex);
  } else {
    Serial.println("[BLE] manifest: SD busy, returning empty list");
  }
  String out;
  serializeJson(doc, out);
  return out;
}

class ManifestCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    pChar->setValue(buildManifestJson());
  }
};

class ProvisioningCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    if (!bondWriteAllowed(connInfo)) {
      Serial.println("[BLE] provisioning: write rejected — outside pairing window");
      return;
    }
    NimBLEAttValue rx = pChar->getValue();
    String json((const char*)rx.data(), rx.length());

    JsonDocument doc;
    if (deserializeJson(doc, json)) {
      Serial.println("[BLE] provisioning: malformed JSON");
      return;
    }
    const char* apiKey = doc["device_api_key"] | "";
    if (strlen(apiKey) == 0) {
      Serial.println("[BLE] provisioning: missing device_api_key");
      return;
    }
    bool ok = false;
    if (xSemaphoreTake(sdMutex, BLE_SD_MUTEX_TIMEOUT) == pdTRUE) {
      ok = persistDeviceApiKey(apiKey);
      xSemaphoreGive(sdMutex);
    } else {
      Serial.println("[BLE] provisioning: SD busy, try again");
    }
    if (!ok) {
      Serial.println("[BLE] provisioning: failed to persist key to SD");
      return;
    }
    Serial.println("[BLE] provisioning: claimed via relay");
    pChar->setValue("{\"status\":\"claimed\"}");
    pChar->notify();
  }
};

// Opens the manifested file (session_id == its SD path) and resets the
// sequence counter — session_data notifications start flowing on the
// next bleRelayTick().
static void startTransfer(const String& sessionId) {
  if (s_xferActive) {
    s_xferFile.close();
    s_xferActive = false;
  }
  if (xSemaphoreTake(sdMutex, BLE_SD_MUTEX_TIMEOUT) != pdTRUE) {
    Serial.println("[BLE] control: SD busy, try start-transfer again");
    return;
  }
  File f = SD.open(sessionId.c_str(), FILE_READ);
  xSemaphoreGive(sdMutex);
  if (!f) {
    Serial.printf("[BLE] control: cannot open %s\n", sessionId.c_str());
    return;
  }
  s_xferFile = f;
  s_xferPath = sessionId;
  s_xferSeq = 0;
  s_xferActive = true;
  Serial.printf("[BLE] control: start-transfer %s (%u bytes)\n",
                sessionId.c_str(), (unsigned)s_xferFile.size());
}

// Frees the device's copy of a session the app has confirmed uploaded —
// same .uploaded marker upload.cpp's WiFi path uses, so a file relayed
// over BLE is never re-offered (by either path) once acknowledged.
static void ackUploaded(const String& sessionId) {
  if (s_xferActive && s_xferPath == sessionId) {
    s_xferFile.close();
    s_xferActive = false;
  }
  if (xSemaphoreTake(sdMutex, BLE_SD_MUTEX_TIMEOUT) == pdTRUE) {
    markUploaded(sessionId.c_str());
    xSemaphoreGive(sdMutex);
  } else {
    Serial.println("[BLE] control: SD busy, ack-uploaded not marked — app should retry");
  }
  Serial.printf("[BLE] control: ack-uploaded %s\n", sessionId.c_str());
}

// "calibrate"/"calibrate-reset" are an E1-specific extension beyond
// xgsail's §8.2 control vocabulary (start-transfer/ack-uploaded) — see
// docs/ble-config.md. Reuses the same calibrateIMU()/resetIMUCalibration()
// the console's `cal`/`calreset` commands call (imu.h) — no duplicated
// logic between the two entry points.
static void handleCalibrate(bool reset, JsonDocument& statusOut) {
  if (!imuOK) {
    statusOut["status"] = "error";
    statusOut["reason"] = "no_imu";
    return;
  }
  bool ok = false;
  if (xSemaphoreTake(sdMutex, BLE_SD_MUTEX_TIMEOUT) == pdTRUE) {
    if (reset) resetIMUCalibration();
    else       calibrateIMU();
    ok = true;
    xSemaphoreGive(sdMutex);
  }
  if (!ok) {
    statusOut["status"] = "error";
    statusOut["reason"] = "sd_busy";
    return;
  }
  statusOut["status"] = "ok";
  statusOut["heel_offset"] = imuHeelOffset;
  statusOut["pitch_offset"] = imuPitchOffset;
}

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue rx = pChar->getValue();
    String json((const char*)rx.data(), rx.length());

    JsonDocument doc;
    if (deserializeJson(doc, json)) {
      Serial.println("[BLE] control: malformed JSON");
      return;
    }
    const char* cmd = doc["cmd"] | "";
    const char* sessionId = doc["session_id"] | "";

    JsonDocument status;
    status["cmd"] = cmd;
    bool haveStatus = false;

    if (strcmp(cmd, "start-transfer") == 0) {
      startTransfer(String(sessionId));
    } else if (strcmp(cmd, "ack-uploaded") == 0) {
      ackUploaded(String(sessionId));
    } else if (strcmp(cmd, "calibrate") == 0) {
      handleCalibrate(false, status);
      haveStatus = true;
    } else if (strcmp(cmd, "calibrate-reset") == 0) {
      handleCalibrate(true, status);
      haveStatus = true;
    } else if (strcmp(cmd, "start-rec") == 0) {
      // E1-specific extension alongside calibrate/calibrate-reset: lets
      // the phone app start a session over BLE, same entry point as the
      // physical button's short press and the console's `rec` command.
      status["ok"] = startRecording();
      status["logging"] = logging;
      haveStatus = true;
    } else if (strcmp(cmd, "stop-rec") == 0) {
      status["ok"] = stopRecording();
      status["logging"] = logging;
      haveStatus = true;
    } else {
      Serial.printf("[BLE] control: unknown cmd '%s'\n", cmd);
    }

    // Only calibrate/calibrate-reset notify a status — start-transfer's
    // "status" is the session_data stream itself, and ack-uploaded has
    // nothing further to report; matches §8.2's control being write-only
    // for those two, while still letting this E1 extension use the same
    // characteristic's NOTIFY property.
    if (haveStatus) {
      String out;
      serializeJson(status, out);
      pChar->setValue(out);
      pChar->notify();
    }
  }
};

// device_config — E1-specific extension, not in xgsail's
// docs/device-protocol.md. See docs/ble-config.md for the full field
// list/semantics; this mirrors it exactly. wifi[].pass is write-only —
// never echoed back on read, regardless of bonding state.
static String buildDeviceConfigJson() {
  JsonDocument doc;
  doc["boat_id"] = config.boat_id;
  doc["unit_role"] = config.unit_role;
  doc["api_base_url"] = config.api_base_url;
  doc["wind_mac"] = config.wind_mac;
  doc["wind_offset"] = config.wind_offset;
  doc["start_speed_knots"] = config.start_speed_knots;
  doc["stop_speed_knots"] = config.stop_speed_knots;
  doc["start_delay_sec"] = config.start_delay_sec;
  doc["stop_delay_sec"] = config.stop_delay_sec;
  doc["rtk_enabled"] = config.rtk_enabled;
  JsonArray wifiArr = doc["wifi"].to<JsonArray>();
  for (int i = 0; i < config.wifi_count; i++) {
    JsonObject w = wifiArr.add<JsonObject>();
    w["ssid"] = config.wifi[i].ssid;
    w["pass"] = "";
  }
  String out;
  serializeJson(doc, out);
  return out;
}

// Partial update: only keys present in `doc` are changed. Each field is
// applied live (no reboot) via the small refresh helper its owning module
// already exposes — see docs/ble-config.md for why each one is safe to
// hot-apply. Caller holds sdMutex for the whole thing (this may call
// saveWindMAC()/saveConfig(), both of which touch SD without locking
// internally, by design — see their own comments).
static void applyDeviceConfigWrite(JsonDocument& doc) {
  if (doc["boat_id"].is<const char*>()) {
    const char* v = doc["boat_id"];
    strncpy(config.boat_id, v, sizeof(config.boat_id) - 1);
    config.boat_id[sizeof(config.boat_id) - 1] = '\0';
    // Mesh identity is a hash of boat_id, cached once at meshInit() —
    // recompute now so the mesh doesn't keep broadcasting under the old
    // identity until next reboot.
    g_mesh_local_sender_id = boatIdHash(config.boat_id);
  }

  if (doc["unit_role"].is<const char*>()) {
    const char* v = doc["unit_role"];
    strncpy(config.unit_role, v, sizeof(config.unit_role) - 1);
    config.unit_role[sizeof(config.unit_role) - 1] = '\0';
    applyUnitRole();
  }

  if (doc["api_base_url"].is<const char*>()) {
    const char* v = doc["api_base_url"];
    strncpy(config.api_base_url, v, sizeof(config.api_base_url) - 1);
    config.api_base_url[sizeof(config.api_base_url) - 1] = '\0';
  }

  if (doc["wind_offset"].is<int>()) {
    config.wind_offset = doc["wind_offset"];
  }

  if (doc["wind_mac"].is<const char*>()) {
    const char* mac = doc["wind_mac"];
    if (strlen(mac) == 0) {
      config.wind_mac[0] = '\0';
      config.wind_enabled = false;
      SD.remove("/wind_mac.txt");
    } else {
      saveWindMAC(mac);
      config.wind_enabled = true;
    }
    forceWindReconnect();
  }

  bool thresholdsChanged = false;
  if (doc["start_speed_knots"].is<float>()) { config.start_speed_knots = doc["start_speed_knots"]; thresholdsChanged = true; }
  if (doc["stop_speed_knots"].is<float>())  { config.stop_speed_knots  = doc["stop_speed_knots"];  thresholdsChanged = true; }
  if (doc["start_delay_sec"].is<int>())     { config.start_delay_sec   = doc["start_delay_sec"];   thresholdsChanged = true; }
  if (doc["stop_delay_sec"].is<int>())      { config.stop_delay_sec    = doc["stop_delay_sec"];    thresholdsChanged = true; }
  if (thresholdsChanged) applyRecordingThresholds();

  if (doc["rtk_enabled"].is<bool>()) {
    config.rtk_enabled = doc["rtk_enabled"];
    // Same live reconfiguration path the console's `gpscfg` command
    // already uses — proven safe without a reboot.
    gnssConfigure();
  }

  if (doc["wifi"].is<JsonArray>()) {
    JsonArray arr = doc["wifi"];
    int n = 0;
    for (JsonObject w : arr) {
      if (n >= MAX_WIFI_NETWORKS) break;
      const char* ssid = w["ssid"] | "";
      const char* pass = w["pass"] | "";
      strncpy(config.wifi[n].ssid, ssid, sizeof(config.wifi[n].ssid) - 1);
      config.wifi[n].ssid[sizeof(config.wifi[n].ssid) - 1] = '\0';
      strncpy(config.wifi[n].pass, pass, sizeof(config.wifi[n].pass) - 1);
      config.wifi[n].pass[sizeof(config.wifi[n].pass) - 1] = '\0';
      n++;
    }
    config.wifi_count = n;
    // connectWiFi() reads config.wifi[] fresh on every call — no
    // additional refresh needed for this to take effect.
  }

  saveConfig();
}

class DeviceConfigCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    pChar->setValue(buildDeviceConfigJson());
  }

  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue rx = pChar->getValue();
    String json((const char*)rx.data(), rx.length());

    JsonDocument doc;
    JsonDocument status;
    if (!bondWriteAllowed(connInfo)) {
      Serial.println("[BLE] device_config: write rejected — outside pairing window");
      status["status"] = "error";
      status["reason"] = "pairing_window_closed";
    } else if (deserializeJson(doc, json)) {
      status["status"] = "error";
      status["reason"] = "bad_json";
    } else if (xSemaphoreTake(sdMutex, BLE_SD_MUTEX_TIMEOUT) != pdTRUE) {
      status["status"] = "error";
      status["reason"] = "sd_busy";
    } else {
      applyDeviceConfigWrite(doc);
      xSemaphoreGive(sdMutex);
      status["status"] = "ok";
    }

    String out;
    serializeJson(status, out);
    pChar->setValue(out);
    pChar->notify();
  }
};

// status — E1-specific extension, read-only, no bonding required (nothing
// here is a secret: own-boat GPS position and WiFi SSID, not a password).
// Pure snapshot of already-maintained in-memory state, same fields the
// console's `status` command prints (console.cpp) — no SD access, so no
// sdMutex needed, unlike device_config/session_manifest. See
// docs/ble-config.md for the full field reference.
static String buildStatusJson() {
  JsonDocument doc;
  doc["claimed"] = isClaimed();
  doc["firmware_version"] = FW_VERSION;
  doc["uptime_s"] = (uint32_t)(millis() / 1000UL);
  doc["heap_free"] = (uint32_t)ESP.getFreeHeap();

  JsonObject batt = doc["battery"].to<JsonObject>();
  batt["pct"] = battery.percent;
  batt["v"] = battery.voltage;
  batt["critical"] = battery.critical;

  doc["sd_ok"] = sdOK;

  JsonObject wifiObj = doc["wifi"].to<JsonObject>();
  wifiObj["connected"] = wifiConnected;
  if (wifiConnected) {
    wifiObj["ssid"] = connectedSSID;
    wifiObj["ip"] = WiFi.localIP().toString();
  }

  JsonObject sensors = doc["sensors"].to<JsonObject>();
  sensors["imu"] = imuOK;
  sensors["pressure"] = presOK;
  sensors["wind"] = config.wind_enabled;

  JsonObject gpsObj = doc["gps"].to<JsonObject>();
  gpsObj["fix"] = gps.valid;
  gpsObj["satellites"] = gps.satellites;
  gpsObj["hdop"] = gps.hdop;
  gpsObj["lat"] = gps.lat;
  gpsObj["lon"] = gps.lon;
  gpsObj["speed_kts"] = gps.speed_kts;
  gpsObj["course"] = gps.course;

  JsonObject windObj = doc["wind"].to<JsonObject>();
  windObj["connected"] = wind.connected;
  if (wind.connected) {
    windObj["speed_kts"] = wind.speed_kts;
    windObj["angle_deg"] = wind.angle_deg;
    windObj["battery"] = wind.battery;
  }

  JsonObject rec = doc["recording"].to<JsonObject>();
  rec["logging"] = logging;
  rec["session_count"] = sessionCount;
  rec["pending_uploads"] = pendingUploads;

  String out;
  serializeJson(doc, out);
  return out;
}

class StatusCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    pChar->setValue(buildStatusJson());
  }
};

class RelayServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    s_connHandle = connInfo.getConnHandle();
    s_haveConn = true;
    Serial.println("[BLE] Relay client connected");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    s_haveConn = false;
    if (s_xferActive) {
      s_xferFile.close();
      s_xferActive = false;
    }
    Serial.println("[BLE] Relay client disconnected — resuming advertising");
    NimBLEDevice::getAdvertising()->start();
  }
};

void bleRelayInit() {
  // Peripheral role runs alongside the existing central role (Calypso
  // wind sensor, wind_sensor.cpp) — NimBLE-Arduino only needs one
  // NimBLEDevice::init() call for both. Wind sensor init is skipped
  // entirely when no wind sensor is configured, so this relay must be
  // able to init BLE itself: it's a first-class upload path, not
  // something that only exists when the wind sensor already brought
  // the radio up.
  if (!bleInitialized) {
    NimBLEDevice::init("SailFrames-E1");
    bleInitialized = true;
  }

  // `provisioning`/`device_config` carry secrets (device_api_key, WiFi
  // password) and are WRITE_ENC, which only demands an encrypted link —
  // bonding itself starts OFF so a first-time pairing is transient
  // (no persistent bond, no future auto-reconnect for that phone) unless
  // the operator's long-press opens the pairing window (bleOpenBondWindow()
  // flips this to bonding=true for the window's duration). An already-
  // bonded phone's reconnect is unaffected by this flag either way.
  NimBLEDevice::setSecurityAuth(false, true, true);  // MITM + secure connections, bonding gated by the pairing window

  s_pServer = NimBLEDevice::createServer();
  s_pServer->setCallbacks(new RelayServerCallbacks());

  NimBLEService* pService = s_pServer->createService(SERVICE_UUID);

  NimBLECharacteristic* pIdentityChar =
      pService->createCharacteristic(IDENTITY_UUID, NIMBLE_PROPERTY::READ);
  JsonDocument idDoc;
  idDoc["external_id"] = externalId();
  idDoc["firmware_version"] = FW_VERSION;
  String idJson;
  serializeJson(idDoc, idJson);
  pIdentityChar->setValue(idJson);

  NimBLECharacteristic* pProvisioningChar = pService->createCharacteristic(
      PROVISIONING_UUID, NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::NOTIFY);
  pProvisioningChar->setCallbacks(new ProvisioningCallbacks());

  pManifestChar = pService->createCharacteristic(
      MANIFEST_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pManifestChar->setCallbacks(new ManifestCallbacks());

  pSessionDataChar = pService->createCharacteristic(SESSION_DATA_UUID, NIMBLE_PROPERTY::NOTIFY);

  // NOTIFY added beyond §8.2's write-only control (E1 extension: the
  // calibrate/calibrate-reset commands notify a status back — see
  // ControlCallbacks::onWrite). Purely additive — a client that never
  // subscribes to notifications on this characteristic behaves exactly
  // as documented in docs/device-protocol.md §8.2.
  NimBLECharacteristic* pControlChar = pService->createCharacteristic(
      CONTROL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pControlChar->setCallbacks(new ControlCallbacks());

  // device_config — E1-specific extension (docs/ble-config.md), not part
  // of xgsail's docs/device-protocol.md. WRITE_ENC because it can carry a
  // WiFi password; READ is plain since nothing it returns is sensitive
  // (wifi[].pass is never echoed back — see buildDeviceConfigJson()).
  NimBLECharacteristic* pDeviceConfigChar = pService->createCharacteristic(
      DEVICE_CONFIG_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::NOTIFY);
  pDeviceConfigChar->setCallbacks(new DeviceConfigCallbacks());

  // status — E1-specific extension (docs/ble-config.md), read-only,
  // on-demand (no notify — the app reads when it wants a snapshot).
  NimBLECharacteristic* pStatusChar =
      pService->createCharacteristic(STATUS_UUID, NIMBLE_PROPERTY::READ);
  pStatusChar->setCallbacks(new StatusCallbacks());

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("[BLE] Device-protocol relay advertising");
}

void bleRelayTick() {
  if (s_bondWindowUntil != 0 && (long)(millis() - s_bondWindowUntil) >= 0) {
    s_bondWindowUntil = 0;
    NimBLEDevice::setSecurityAuth(false, true, true);
    Serial.println("[BLE] Pairing window closed");
  }

  if (!s_xferActive || !s_haveConn) return;

  // A handful of chunks per tick — enough throughput without hogging
  // Core 1 against the sensor/display/mesh work sharing this loop().
  for (int i = 0; i < 4 && s_xferActive; i++) {
    if (!s_xferFile || !s_xferFile.available()) {
      Serial.printf("[BLE] Transfer complete: %s (%lu chunks)\n",
                    s_xferPath.c_str(), (unsigned long)s_xferSeq);
      s_xferFile.close();
      s_xferActive = false;
      break;
    }

    // MTU-aware chunk size: stay under (this peer's negotiated MTU - 3
    // byte ATT header - 4 byte sequence index). getPeerMTU (not the
    // static NimBLEDevice::getMTU(), which is just the locally configured
    // preference) reflects what was actually negotiated with this
    // connection. Falls back to the pre-negotiation default (23) if
    // called before negotiation completes.
    uint16_t mtu = s_pServer ? s_pServer->getPeerMTU(s_connHandle) : 23;
    size_t cap = (mtu > 7) ? (size_t)(mtu - 7) : 16;
    if (cap > 240) cap = 240;

    uint8_t buf[244];
    size_t want = (cap < sizeof(buf) - 4) ? cap : sizeof(buf) - 4;

    // This runs on Core 1 (called from loop()), but s_xferFile stays open
    // across many ticks — every actual read still needs sdMutex, same as
    // any other cross-context SD access, since Core 0's upload task might
    // be walking/uploading the SD tree at the same instant. Skip this
    // tick (not a hard block) if busy; the file position is unchanged, so
    // the next tick just retries the same chunk.
    if (xSemaphoreTake(sdMutex, BLE_SD_MUTEX_TIMEOUT) != pdTRUE) break;
    int n = s_xferFile.read(buf + 4, want);
    xSemaphoreGive(sdMutex);
    if (n <= 0) {
      Serial.printf("[BLE] SD read failed mid-transfer: %s\n", s_xferPath.c_str());
      s_xferFile.close();
      s_xferActive = false;
      break;
    }

    buf[0] = (uint8_t)((s_xferSeq >> 24) & 0xFF);
    buf[1] = (uint8_t)((s_xferSeq >> 16) & 0xFF);
    buf[2] = (uint8_t)((s_xferSeq >> 8) & 0xFF);
    buf[3] = (uint8_t)(s_xferSeq & 0xFF);

    pSessionDataChar->setValue(buf, (size_t)n + 4);
    pSessionDataChar->notify();
    s_xferSeq++;
    yield();
  }
}
