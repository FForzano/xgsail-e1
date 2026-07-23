// Calypso Mini wind sensor (BLE) glue — see wind_sensor.h.
#include "wind_sensor.h"
#include "config.h"
#include "gnss.h"
#include "storage.h"
#include "display.h"

// Calypso BLE UUIDs (Environmental Sensing Service 0x181A)
static NimBLEUUID WIND_SERVICE_UUID("181A");
static NimBLEUUID WIND_SPEED_CHAR_UUID("2A72");      // Apparent Wind Speed (uint16, m/s * 100)
static NimBLEUUID WIND_DIR_CHAR_UUID("2A73");        // Apparent Wind Direction (uint16, degrees * 100)
static NimBLEUUID BATTERY_SERVICE_UUID("180F");
static NimBLEUUID BATTERY_CHAR_UUID("2A19");         // Battery Level (uint8, 0-100%)

// Calypso Data Service (0x180D) - combined wind+battery in single notification
// Format: [speed_lo, speed_hi, dir_lo, dir_hi, battery] where battery * 10 = %
static NimBLEUUID DATA_SERVICE_UUID("180D");
static NimBLEUUID DATA_CHAR_UUID("2A39");            // Combined wind+battery (5 bytes)

// Device Information Service (0x180A) - for reading firmware version
static NimBLEUUID DEVINFO_SERVICE_UUID("180A");
static NimBLEUUID FIRMWARE_CHAR_UUID("2A26");        // Firmware Revision String

WindData wind;
bool windScanning = false;
bool windOK = false;
bool bleInitialized = false;

// BLE client for Calypso wind sensor
static NimBLEClient* pWindClient = nullptr;
static NimBLERemoteCharacteristic* pWindSpeedChar = nullptr;
static NimBLERemoteCharacteristic* pWindDirChar = nullptr;
static NimBLERemoteCharacteristic* pBatteryChar = nullptr;
static NimBLERemoteCharacteristic* pDataChar = nullptr;      // Combined wind+battery (0x2A39)
static unsigned long lastWindScan = 0;

void windSpeedNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 2) {
    uint16_t raw = pData[0] | (pData[1] << 8);
    wind.speed_mps = raw / 100.0;
    wind.speed_kts = wind.speed_mps * 1.94384;
    wind.newData = true;
    wind.lastUpdate = millis();
  }
}

// BLE notification callback for wind direction
void windDirNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 2) {
    uint16_t raw = pData[0] | (pData[1] << 8);
    wind.angle_deg = raw / 100;  // 0.01 degree resolution
    wind.newData = true;
    wind.lastUpdate = millis();
  }
}

// BLE notification callback for battery (0x180F / 0x2A19)
void batteryNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 1) {
    wind.battery = pData[0];
  }
}

// BLE notification callback for combined Data Service (0x180D / 0x2A39)
// Format: [speed_lo, speed_hi, dir_lo, dir_hi, battery] where battery * 10 = %
void dataNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 5) {
    uint16_t speedRaw = pData[0] | (pData[1] << 8);
    uint16_t dirRaw = pData[2] | (pData[3] << 8);
    uint8_t battRaw = pData[4];

    wind.speed_mps = speedRaw / 100.0;
    wind.speed_kts = wind.speed_mps * 1.94384;
    wind.angle_deg = dirRaw;
    wind.battery = battRaw * 10;  // Manual says value * 10 = %
    wind.newData = true;
    wind.lastUpdate = millis();
  }
}

// BLE client callbacks
class WindClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) {
    Serial.println("[WIND] BLE connected");
    wind.connected = true;
  }

  void onDisconnect(NimBLEClient* pClient) {
    Serial.println("[WIND] BLE disconnected");
    wind.connected = false;
    windOK = false;
    pWindSpeedChar = nullptr;
    pWindDirChar = nullptr;
    pBatteryChar = nullptr;
    pDataChar = nullptr;
  }
};

static WindClientCallbacks windClientCallbacks;

// Save discovered MAC to config for faster reconnection
void saveWindMAC(const char* mac) {
  strncpy(config.wind_mac, mac, sizeof(config.wind_mac) - 1);

  // Save to SD card
  File f = SD.open("/wind_mac.txt", FILE_WRITE);
  if (f) {
    f.println(mac);
    f.close();
    Serial.printf("[WIND] Saved MAC %s for auto-reconnect\n", mac);
  }
}

// Load wind MAC from SD - if /wind_mac.txt exists, enable wind sensor
void loadWindMAC() {
  File f = SD.open("/wind_mac.txt", FILE_READ);
  if (f) {
    String mac = f.readStringUntil('\n');
    mac.trim();
    if (mac.length() >= 17) {  // Valid MAC is 17 chars (XX:XX:XX:XX:XX:XX)
      mac.toCharArray(config.wind_mac, sizeof(config.wind_mac));
      config.wind_enabled = true;
      Serial.printf("[WIND] Loaded MAC from SD: %s - wind ENABLED\n", config.wind_mac);
    } else {
      Serial.println("[WIND] /wind_mac.txt exists but invalid MAC format");
    }
    f.close();
  } else {
    Serial.println("[WIND] No /wind_mac.txt on SD - wind DISABLED");
  }
}

// Scan for Calypso wind sensor
bool scanForCalypso() {
  Serial.println("[WIND] Scanning for Calypso...");
  windScanning = true;

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->clearResults();

  // NimBLE 2.x: start() is non-blocking, must wait for completion
  if (!pScan->start(WIND_SCAN_TIMEOUT_MS / 1000, false)) {
    Serial.println("[WIND] Scan failed to start");
    windScanning = false;
    return false;
  }

  // Wait for scan to complete
  unsigned long scanStart = millis();
  while (pScan->isScanning() && millis() - scanStart < WIND_SCAN_TIMEOUT_MS + 2000) {
    delay(100);
  }

  NimBLEScanResults results = pScan->getResults();
  Serial.printf("[WIND] Scan found %d devices\n", results.getCount());

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* pDevice = results.getDevice(i);
    if (!pDevice) continue;

    String name = pDevice->getName().c_str();
    String nameLower = name;
    nameLower.toLowerCase();

    Serial.printf("[WIND]   %d: \"%s\" @ %s\n", i+1, name.c_str(),
      pDevice->getAddress().toString().c_str());

    // Look for Calypso devices
    if (nameLower.indexOf("calypso") >= 0 || nameLower.indexOf("ultrasonic") >= 0) {
      Serial.printf("[WIND] Found Calypso: %s at %s\n",
        pDevice->getName().c_str(), pDevice->getAddress().toString().c_str());

      strncpy(wind.deviceName, pDevice->getName().c_str(), sizeof(wind.deviceName) - 1);
      strncpy(wind.deviceAddr, pDevice->getAddress().toString().c_str(), sizeof(wind.deviceAddr) - 1);

      // Save MAC for faster reconnection
      saveWindMAC(wind.deviceAddr);

      windScanning = false;
      pScan->clearResults();
      return true;
    }
  }

  Serial.println("[WIND] Calypso not found");
  windScanning = false;
  pScan->clearResults();
  return false;
}

// Connect to Calypso wind sensor
bool connectToCalypso() {
  // Use saved MAC if available
  const char* targetAddr = strlen(config.wind_mac) > 0 ? config.wind_mac : wind.deviceAddr;

  if (strlen(targetAddr) == 0) {
    // Need to scan first
    if (!scanForCalypso()) {
      return false;
    }
    targetAddr = wind.deviceAddr;
  }

  Serial.printf("[WIND] Connecting to %s...\n", targetAddr);

  if (pWindClient == nullptr) {
    pWindClient = NimBLEDevice::createClient();
    pWindClient->setClientCallbacks(&windClientCallbacks);
  }

  // NimBLE 2.x: NimBLEAddress requires std::string and address type
  // Address type 1 = random (most BLE devices use random addresses)
  NimBLEAddress addr(std::string(targetAddr), 1);
  if (!pWindClient->connect(addr)) {
    Serial.println("[WIND] Connection failed");
    // Clear saved MAC if connection failed - might be wrong device
    if (strlen(config.wind_mac) > 0) {
      Serial.println("[WIND] Clearing saved MAC, will scan next time");
      config.wind_mac[0] = '\0';
    }
    return false;
  }

  // Get Wind Service
  NimBLERemoteService* pWindService = pWindClient->getService(WIND_SERVICE_UUID);
  if (pWindService == nullptr) {
    Serial.println("[WIND] Wind service not found");
    pWindClient->disconnect();
    return false;
  }

  // Subscribe to wind speed notifications
  pWindSpeedChar = pWindService->getCharacteristic(WIND_SPEED_CHAR_UUID);
  if (pWindSpeedChar && pWindSpeedChar->canNotify()) {
    pWindSpeedChar->subscribe(true, windSpeedNotifyCallback);
    Serial.println("[WIND] Subscribed to wind speed");
  }

  // Subscribe to wind direction notifications
  pWindDirChar = pWindService->getCharacteristic(WIND_DIR_CHAR_UUID);
  if (pWindDirChar && pWindDirChar->canNotify()) {
    pWindDirChar->subscribe(true, windDirNotifyCallback);
    Serial.println("[WIND] Subscribed to wind direction");
  }

  // Try to get battery from Battery Service (0x180F)
  NimBLERemoteService* pBattService = pWindClient->getService(BATTERY_SERVICE_UUID);
  if (pBattService) {
    pBatteryChar = pBattService->getCharacteristic(BATTERY_CHAR_UUID);
    if (pBatteryChar) {
      if (pBatteryChar->canNotify()) {
        pBatteryChar->subscribe(true, batteryNotifyCallback);
      }
      if (pBatteryChar->canRead()) {
        wind.battery = pBatteryChar->readValue<uint8_t>();
        Serial.printf("[WIND] Battery: %d%%\n", wind.battery);
      }
    }
  }

  // Also try Data Service (0x180D) for combined wind+battery notifications
  NimBLERemoteService* pDataService = pWindClient->getService(DATA_SERVICE_UUID);
  if (pDataService) {
    pDataChar = pDataService->getCharacteristic(DATA_CHAR_UUID);
    if (pDataChar && pDataChar->canNotify()) {
      pDataChar->subscribe(true, dataNotifyCallback);
    }
  }

  // Read firmware version from Device Information Service (0x180A)
  NimBLERemoteService* pDevInfoService = pWindClient->getService(DEVINFO_SERVICE_UUID);
  if (pDevInfoService) {
    NimBLERemoteCharacteristic* pFwChar = pDevInfoService->getCharacteristic(FIRMWARE_CHAR_UUID);
    if (pFwChar && pFwChar->canRead()) {
      std::string fw = pFwChar->readValue();
      if (fw.length() > 0) {
        strncpy(wind.firmware, fw.c_str(), sizeof(wind.firmware) - 1);
        wind.firmware[sizeof(wind.firmware) - 1] = '\0';
        Serial.printf("[WIND] Firmware: %s\n", wind.firmware);
      }
    }
  }

  wind.connected = true;
  windOK = true;
  Serial.println("[WIND] Connected and streaming");

  // Display wind sensor status on TFT at startup
  if (oledOK) {
    tft.fillScreen(COLOR_BG);
    tft.setTextColor(COLOR_GOOD, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("WIND SENSOR OK", SCREEN_WIDTH/2, 100, 4);

    char buf[32];
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    if (strlen(wind.deviceName) > 0) {
      snprintf(buf, sizeof(buf), "%s", wind.deviceName);
      tft.drawString(buf, SCREEN_WIDTH/2, 150, 2);
    }

    if (wind.battery >= 0) {
      snprintf(buf, sizeof(buf), "Battery: %d%%", wind.battery);
      tft.drawString(buf, SCREEN_WIDTH/2, 180, 2);
    } else {
      tft.drawString("Battery: --", SCREEN_WIDTH/2, 180, 2);
    }

    delay(2000);  // Show for 2 seconds
    d2LayoutDrawn = false;  // Force main display to redraw
  }

  return true;
}

// Initialize BLE for wind sensor
void initWindSensor() {
  // Check for /wind_mac.txt on SD first - this enables wind if file exists
  if (sdOK) {
    loadWindMAC();
  }

  if (!config.wind_enabled) {
    Serial.println("[WIND] No wind_mac.txt found - wind sensor disabled");
    return;
  }

  Serial.println("[WIND] Initializing BLE...");
  NimBLEDevice::init("SailFrames-E1");
  bleInitialized = true;
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // Max power for range

  // Try to connect using MAC from wind_mac.txt
  connectToCalypso();
}

// Check wind connection and reconnect if needed
// Stop any in-flight BLE operations before WiFi work begins. ESP32 has a
// single shared radio for BLE and WiFi; under sustained WiFi traffic
// (large uploads, e.g. 400KB+ RTCM3 PUTs) NimBLE's host task can stall
// without timing out, hanging whichever core is blocked inside
// NimBLEScan::start()/getResults(). Symptoms: hard hang, no panic, no
// reboot — display and serial freeze (firmware 2026.05.03.01 fleet test).
void pauseBLEForWiFi() {
  NimBLEScan* pScanLocal = NimBLEDevice::getScan();
  if (pScanLocal && pScanLocal->isScanning()) {
    Serial.println("[BLE] Stopping active scan before WiFi work");
    pScanLocal->stop();
  }
  if (pWindClient && pWindClient->isConnected()) {
    Serial.println("[BLE] Disconnecting wind client before WiFi work");
    pWindClient->disconnect();
  }
}

void checkWindConnection() {
  if (!config.wind_enabled) return;

  // Don't run BLE work while WiFi is in use — shared radio. New scans
  // started here under WiFi load are the documented hang trigger.
  if (wifiConnected || uploading || triggerUpload) return;

  unsigned long now = millis();

  // If connected, nothing to do
  if (wind.connected && windOK) {
    return;
  }

  // Throttle reconnection attempts
  if (now - lastWindScan < WIND_RECONNECT_MS) {
    return;
  }
  lastWindScan = now;

  Serial.println("[WIND] Attempting reconnect...");
  connectToCalypso();
}

// Log wind data to CSV
void logWind() {
  if (!windFile || !logging || !wind.newData) return;

  sdWriting = true;
  unsigned long e = millis() - logStart;
  // Apply user-configured wind_offset only (no 180° correction needed)
  int correctedAwa = (wind.angle_deg + config.wind_offset) % 360;
  if (correctedAwa < 0) correctedAwa += 360;
  windFile.printf("%lu,%s,%.2f,%.2f,%d,%d\n",
    e, gps.utc_time, wind.speed_kts, wind.speed_mps, correctedAwa, wind.battery);
  totalBytes += 60;
  wind.newData = false;
  sdWriting = false;
}

#endif // ENABLE_WIND
