// BLE GATT relay glue — see ble_relay.h. Wire-level contract: xgsail's
// docs/device-protocol.md §8.2 (exact UUIDs/properties/JSON shapes below
// are quoted from there — cross-check against that doc before changing
// anything here, not against this file's history).
#include "ble_relay.h"
#include "config.h"
#include "device_auth.h"
#include "upload.h"
#include "storage.h"
#include "wind_sensor.h"  // bleInitialized — shared with the wind-sensor central role
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <SD.h>

static const char* SERVICE_UUID       = "24e6db2c-3c8a-4b5b-ba5a-23bc4c818046";
static const char* IDENTITY_UUID      = "985a1aae-858e-4727-9d5c-c8670bd6bd06";
static const char* PROVISIONING_UUID  = "db2c2e63-9e13-4fa9-867c-0b579ce2ae57";
static const char* MANIFEST_UUID      = "ed9efdc8-70d4-4ce5-a0a3-9fa6d88b9b9e";
static const char* SESSION_DATA_UUID  = "728d2815-0409-49ce-ad73-ecca6fc6d981";
static const char* CONTROL_UUID       = "ec88dd3e-2562-420c-aebe-30a4ae40bdf9";

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
  collectPendingFiles("/sf", arr);
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
    if (!persistDeviceApiKey(apiKey)) {
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
  File f = SD.open(sessionId.c_str(), FILE_READ);
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
  markUploaded(sessionId.c_str());
  Serial.printf("[BLE] control: ack-uploaded %s\n", sessionId.c_str());
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
    if (strcmp(cmd, "start-transfer") == 0) {
      startTransfer(String(sessionId));
    } else if (strcmp(cmd, "ack-uploaded") == 0) {
      ackUploaded(String(sessionId));
    } else {
      Serial.printf("[BLE] control: unknown cmd '%s'\n", cmd);
    }
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

  // `provisioning` carries the device_api_key — bonded + encrypted only
  // (docs/device-protocol.md §8.2).
  NimBLEDevice::setSecurityAuth(true, true, true);  // bond, MITM, secure connections

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

  NimBLECharacteristic* pControlChar =
      pService->createCharacteristic(CONTROL_UUID, NIMBLE_PROPERTY::WRITE);
  pControlChar->setCallbacks(new ControlCallbacks());

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("[BLE] Device-protocol relay advertising");
}

void bleRelayTick() {
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
    int n = s_xferFile.read(buf + 4, want);
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
