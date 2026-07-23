/*
 * SailFrames E1 — Fleet Tracker Firmware
 *
 * Hardware:
 *   - ESP32 DevKit V1 (ELEGOO)
 *   - Waveshare LG290P GNSS (UART2: RX=GPIO16, TX=GPIO17, 460800 baud)
 *   - BNO085 IMU (I2C: 0x4A) — heel, pitch, heading
 *   - DPS310 Pressure/Temp (I2C: 0x77) — barometric pressure for gust detection
 *   - Hosyond 3.5" IPS ST7796U TFT 480x320 (SPI: CS=5, DC=2, RST=4, BL=25)
 *   - MicroSD standalone module (SPI shared: MOSI=23, MISO=19, CLK=18, CS=27)
 *   - Calypso Mini wind sensor (BLE) — apparent wind speed/direction
 *   - DWEII USB-C 5V Boost Converter + LiPo cell
 *   - 100K/100K voltage divider on GPIO34 for battery monitoring
 *
 * Behavior:
 *   Power on → init sensors → configure LG290P (Rover mode, 10 Hz NMEA)
 *   → scan for Calypso wind sensor (BLE) → wait for GPS fix
 *   → auto-log to SD (NMEA CSV + IMU CSV + Wind CSV + Pres CSV)
 *   → when yacht club Wi-Fi detected → auto-upload to AWS S3
 *   Power off → done
 *
 * NOTE: PPK / raw-RTCM3 capture was retired in firmware 2026.05.20.09.
 * See docs/gnss-rtk.md for the previous architecture.
 *
 * Log files per session:
 *   /sf/YYYYMMDD/E1_YYYYMMDD_HHMMSS_nav.csv  (10 Hz)
 *   /sf/YYYYMMDD/E1_YYYYMMDD_HHMMSS_imu.csv  (10 Hz)
 *   /sf/YYYYMMDD/E1_YYYYMMDD_HHMMSS_wind.csv (1 Hz when Calypso paired)
 *   /sf/YYYYMMDD/E1_YYYYMMDD_HHMMSS_pres.csv (0.1 Hz, DPS310 only)
 *
 * License: Apache 2.0
 *
 * This file is the composition root: global object wiring, setup(), and
 * loop(). Everything else lives in the per-responsibility modules listed
 * below (see CLAUDE.md "Repository Structure").
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DPS310.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
// NimBLE configuration. CENTRAL+OBSERVER: the existing Calypso wind-sensor
// client (wind_sensor.cpp). PERIPHERAL: the device-protocol GATT relay
// server (ble_relay.cpp) — connectable advertising + accepting the phone
// app's connection, run concurrently with the central role. 2 connections/
// bonds: one outbound (wind sensor, unbonded) + one inbound (phone relay,
// bonded for the `provisioning` characteristic).
#define CONFIG_BT_NIMBLE_ROLE_CENTRAL 1
#define CONFIG_BT_NIMBLE_ROLE_PERIPHERAL 1
#define CONFIG_BT_NIMBLE_ROLE_OBSERVER 1
#define CONFIG_BT_NIMBLE_ROLE_BROADCASTER 0
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 2
#define CONFIG_BT_NIMBLE_MAX_BONDS 2
#define CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME "SailFrames-E1"
#include <NimBLEDevice.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string>

#include "config.h"
#include "v2_types.h"
#include "mesh.h"
#include "rtk_relay.h"
#include "gnss.h"
#include "imu.h"
#include "wind_sensor.h"
#include "pressure.h"
#include "battery.h"
#include "recording.h"
#include "ocs.h"
#include "display.h"
#include "storage.h"
#include "telnet.h"
#include "console.h"
#include "device_auth.h"
#include "ble_relay.h"
#include "upload.h"
#include "shared_state.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  // Capture WHY we last rebooted before any other init runs. The fleet
  // had a simultaneous-reboot event on 2026-05-03 with no serial captured;
  // this prints the cause at the top of the next boot log AND appends it
  // to /boot.log on SD so we can read it later if no USB was connected.
  esp_reset_reason_t resetReason = esp_reset_reason();

  Serial.println("\n=================================");
  Serial.printf("  SailFrames Edge %s\n", FW_VERSION);
  Serial.println("  Hardware Power Switch Edition");
  Serial.printf("  Reset reason: %s (%d)\n", resetReasonStr(resetReason), (int)resetReason);
  Serial.printf("  Free heap: %u, min ever: %u\n",
                ESP.getFreeHeap(),
                (unsigned)esp_get_minimum_free_heap_size());
  Serial.println("=================================");

  // Create SD mutex for dual-core safety
  sdMutex = xSemaphoreCreateMutex();
  if (sdMutex == NULL) {
    Serial.println("[ERR] Failed to create SD mutex!");
  }

  // Initialize WiFi FIRST before any peripherals touch GPIO2
  // This must happen before Wire, SPI, or any other peripheral init
  pinMode(2, INPUT);  // Release GPIO2
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  Serial.println("[WIFI] PHY initialized early");

  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);

  // Battery monitoring (PowerBoost 1000C)
  setupBattery();
  updateBattery();  // Initial reading
  Serial.printf("[BATT] Initial: %.2fV (%d%%)\n", battery.voltage, battery.percent);
  if (battery.critical) {
    Serial.println("[BATT] WARNING: Battery critical on startup!");
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setBufferSize(512);  // BNO085 SHTP needs larger buffer
  Wire.setClock(100000);  // Start slow for reliable init
  delay(100);  // Let I2C bus stabilize

  // BNO085 needs up to 1 second to boot after power-on
  Serial.println("[I2C] Waiting for BNO085 boot (1s)...");
  delay(1000);

  // I2C Scanner - check all addresses to debug
  Serial.println("[I2C] Scanning bus...");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C] Found device at 0x%02X\n", addr);
    }
  }

  // Check expected devices
  Serial.println("[I2C] Checking expected devices...");
  Wire.beginTransmission(BNO085_ADDR);
  bool bnoFound = (Wire.endTransmission() == 0);
  Wire.beginTransmission(DPS310_ADDR);
  bool dpsFound = (Wire.endTransmission() == 0);
  Serial.printf("[I2C] BNO085 0x4B: %s\n", bnoFound ? "YES" : "NO");
  Serial.printf("[I2C] DPS310 0x77: %s\n", dpsFound ? "YES" : "NO");

  // IMU — BNO085 (init early, before SPI peripherals)
  Serial.println("[IMU] Initializing BNO085...");
  bool imuInitOK = bno08x.begin_I2C(BNO085_ADDR, &Wire);

  if (imuInitOK) {
    imuOK = true;
    Wire.setClock(400000);  // Switch to fast mode after init
    Serial.println("[IMU] BNO085 detected, enabling reports");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_INTERVAL_MS * 1000)) {
      Serial.println("[IMU] WARNING: Failed to enable game rotation vector");
    }
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, IMU_INTERVAL_MS * 1000)) {
      Serial.println("[IMU] WARNING: Failed to enable rotation vector");
    }
    if (!bno08x.enableReport(SH2_ACCELEROMETER, IMU_INTERVAL_MS * 1000)) {
      Serial.println("[IMU] WARNING: Failed to enable accelerometer");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_INTERVAL_MS * 1000)) {
      Serial.println("[IMU] WARNING: Failed to enable gyroscope");
    }
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, IMU_INTERVAL_MS * 1000)) {
      Serial.println("[IMU] WARNING: Failed to enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER, 500000)) {
      Serial.println("[IMU] WARNING: Failed to enable stability classifier");
    }
    if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, IMU_INTERVAL_MS * 1000)) {
      Serial.println("[IMU] WARNING: Failed to enable magnetometer");
    }
    Serial.println("[IMU] BNO085 OK");
  } else {
    Serial.println("[IMU] BNO085 not found!");
  }

  // Set up CS pins before any SPI init
  pinMode(TFT_CS_PIN, OUTPUT);
  digitalWrite(TFT_CS_PIN, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);

  // SD Card - Initialize on HSPI bus (separate from TFT's VSPI)
  Serial.println("[SD] Initializing on HSPI bus (separate from TFT)...");
  Serial.printf("[SD] Pins: CLK=%d, MISO=%d, MOSI=%d, CS=%d\n", SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  // Create HSPI instance for SD card - completely separate from TFT's VSPI
  static SPIClass sdSPI(HSPI);
  sdSPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);  // CLK=14, MISO=12, MOSI=13, CS=27
  delay(50);

  // Try different SPI speeds
  Serial.println("[SD] Trying 4MHz...");
  sdOK = SD.begin(SD_CS_PIN, sdSPI, 4000000);
  if (!sdOK) {
    Serial.println("[SD] 4MHz failed, trying 1MHz...");
    sdOK = SD.begin(SD_CS_PIN, sdSPI, 1000000);
  }
  if (!sdOK) {
    Serial.println("[SD] 1MHz failed, trying 400kHz...");
    sdOK = SD.begin(SD_CS_PIN, sdSPI, 400000);
  }

  if (sdOK) {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("[SD] No card detected!");
      sdOK = false;
    } else {
      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("[SD] OK - Card size: %llu MB\n", cardSize);
      Serial.printf("[SD] Card type: %s\n",
        cardType == CARD_MMC ? "MMC" :
        cardType == CARD_SD ? "SD" :
        cardType == CARD_SDHC ? "SDHC" : "UNKNOWN");
      loadConfig();
      loadClassRegistry();  // Stage 5.5 — RC-only, no-op for racing boats
      loadIMUCalibration();

      // Append a boot record so we can reconstruct reset history later
      // even without a USB cable attached at the moment of failure.
      File bootLog = SD.open("/boot.log", FILE_APPEND);
      if (bootLog) {
        bootLog.printf("boot fw=%s reset=%s heap=%u min_heap=%u\n",
                       FW_VERSION,
                       resetReasonStr(esp_reset_reason()),
                       ESP.getFreeHeap(),
                       (unsigned)esp_get_minimum_free_heap_size());
        bootLog.close();
        Serial.println("[SD] Boot record appended to /boot.log");
      }
    }
  } else {
    Serial.println("[SD] === SD CARD FAILED ===");
    Serial.println("[SD] Troubleshooting:");
    Serial.println("[SD]   1. Check SD module wiring (CS=GPIO27)");
    Serial.println("[SD]   2. Insert card before power-on");
    Serial.println("[SD]   3. Card must be FAT32 (not exFAT)");
    Serial.println("[SD]   4. Try different SD card");
  }

  // TFT Display - Initialize AFTER SD
  Serial.println("[TFT] Initializing ST7796U...");
  // Backlight via PWM so we can dim during idle. Init at IDLE level —
  // updateBacklight() in the loop pushes to RECORDING when logging starts.
  // Core 3.x ledcAttach: one call attaches a pin with freq + resolution,
  // and ledcWrite addresses the PIN (not a channel) thereafter.
  ledcAttach(TFT_BL_PIN, TFT_BL_PWM_FREQ, TFT_BL_PWM_RES);
  ledcWrite(TFT_BL_PIN, TFT_BL_DUTY_IDLE);
  tft.init();
  tft.setRotation(2);  // Portrait orientation (180° from rotation 0)
  tft.invertDisplay(true);  // Required for correct colors on this ST7796 panel
  tft.fillScreen(COLOR_BG);
  oledOK = true;
  Serial.println("[TFT] ST7796U initialized (320x480 portrait)");

  // SD-card fault gate. If the card never came up, loadConfig() never ran
  // and config.boat_id is still its compile-time default ("E1"). Booting on
  // would put a SECOND "E1" on the mesh — duplicate FNV-1a sender_id —
  // corrupting peer state, OCS, and the class registry (this is exactly how
  // an E6 with a half-seated card silently impersonated E1). Refuse to boot:
  // show a persistent fault screen and stop here, BEFORE meshInit() ever
  // broadcasts a bogus identity. We are past tft.init() but before
  // esp_task_wdt_add(NULL), so the task WDT is not yet armed; the delay()
  // in the loop below still yields to the IDF idle task, keeping its
  // watchdog fed. Recovery is operator action: reseat the card + power-cycle.
  if (!sdOK) {
    tft.fillScreen(COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COLOR_ERROR, COLOR_BG);
    tft.setTextSize(2);
    tft.drawString("SD CARD", SCREEN_WIDTH/2, 70, 4);
    tft.drawString("FAILURE", SCREEN_WIDTH/2, 125, 4);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setTextSize(1);
    tft.drawString("Contact", SCREEN_WIDTH/2, 215, 4);
    tft.drawString("Paul Avillach", SCREEN_WIDTH/2, 260, 4);
    tft.setTextSize(2);
    tft.drawString("857 891 0512", SCREEN_WIDTH/2, 325, 2);
    tft.setTextSize(1);
    Serial.println("[SD] FATAL: SD unreadable at boot — refusing to start "
                   "(would impersonate default boat_id). Reseat card + power-cycle.");
    while (true) {
      Serial.println("[SD] HALTED: SD card failure — see TFT for contact info.");
      delay(5000);  // yields to idle task so the IDF idle WDT stays fed
    }
  }

  // Splash screen - show device ID, domain, and firmware version
  tft.fillScreen(COLOR_BG);
  tft.setTextDatum(MC_DATUM);

  // Draw device ID in HUGE font (fill most of the screen)
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextSize(8);
  tft.drawString(config.boat_id, SCREEN_WIDTH/2, SCREEN_HEIGHT/2 - 60, 4);

  // "Sailframes.com" - black, medium size
  tft.setTextColor(TFT_BLACK, COLOR_BG);
  tft.setTextSize(1);
  tft.drawString("Sailframes.com", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 + 80, 4);

  // Firmware version at bottom - large enough to read across the cabin
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.setTextSize(2);
  tft.drawString(FW_VERSION, SCREEN_WIDTH/2, SCREEN_HEIGHT - 40, 4);
  tft.setTextSize(1);

  delay(2500);  // Show splash screen

  // Reset text size for rest of display
  tft.setTextSize(1);

  // DPS310 Pressure/Temperature sensor
  Serial.println("[PRES] Initializing DPS310...");
  if (dps.begin_I2C(DPS310_ADDR, &Wire)) {
    presOK = true;
    // Configure for high-rate sampling (good for gust detection)
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    Serial.println("[PRES] DPS310 OK");

    // Take initial reading
    sensors_event_t temp_event, pressure_event;
    if (dps.getEvents(&temp_event, &pressure_event)) {
      pressure.pressure_hpa = pressure_event.pressure;
      pressure.temperature_c = temp_event.temperature;
      pressure.valid = true;
      Serial.printf("[PRES] Initial: %.2f hPa, %.1f°C\n",
        pressure.pressure_hpa, pressure.temperature_c);
    }
  } else {
    Serial.println("[PRES] DPS310 not found");
  }

  // GPS
  // Enlarge the RX FIFO (default 256 B ≈ 5.5 ms at 460800). The RTK base path
  // (readGPSBase) can spend a few ms broadcasting RTCM mid-read; a 2 KB buffer
  // (~44 ms) absorbs that so outgoing base RTCM isn't dropped on RX overflow.
  Serial2.setRxBufferSize(2048);   // must precede begin()
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("[GPS] UART2 at %d baud (RX=GPIO%d, TX=GPIO%d)\n", GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);

  // Diagnostic: check for incoming data
  Serial.println("[GPS] Checking for data (2 sec)...");
  delay(100);
  unsigned long testStart = millis();
  int byteCount = 0;
  while (millis() - testStart < 2000) {
    while (Serial2.available()) {
      char c = Serial2.read();
      byteCount++;
      if (byteCount <= 100) Serial.print(c);  // Print first 100 chars
    }
    delay(1);
  }
  Serial.printf("\n[GPS] Received %d bytes in 2 sec\n", byteCount);
  if (byteCount == 0) {
    Serial.println("[GPS] WARNING: No data received! Check:");
    Serial.println("[GPS]   - Wiring: GPS TXD3 -> ESP32 GPIO16");
    Serial.println("[GPS]   - Baud rate: try 115200 or 460800");
    Serial.println("[GPS]   - GPS power and antenna");
  }

  // RTCM3 raw-observation capture retired in .09 — the CFGRTCM probe
  // here previously drained the response buffer; configureLG290P() below
  // does its own command/response handling so no explicit probe is needed.

  delay(500);
  gnssConfigure();   // RTK off ⇒ exactly configureLG290P(); on ⇒ base/rover per role+chip

  // Don't block waiting for GPS fix - let main loop handle it
  // This allows WiFi/telnet access while GPS is searching
  Serial.println("[GPS] Will acquire fix in background...");

  // Initialize wind sensor (Calypso BLE)
#if ENABLE_WIND
  initWindSensor();
#endif

  // Device-protocol BLE GATT relay (docs/device-protocol.md §8) — brings
  // up BLE itself if the wind sensor didn't. First-class upload path:
  // starts unconditionally, not gated on WiFi being absent/down.
  bleRelayInit();

  // Connect to WiFi EARLY (for OTA and telnet access during GPS search)
  // WiFi connection is handled by upload task on Core 0 - non-blocking
  // Don't connect at boot to avoid blocking the display
  if (config.wifi_count > 0) {
    Serial.println("[WIFI] WiFi configured - will connect in background when needed");
  }

  // Apply recording thresholds from config
  startSpeedKnots = config.start_speed_knots;
  stopSpeedKnots = config.stop_speed_knots;
  startDelayMs = config.start_delay_sec * 1000UL;
  stopDelayMs = config.stop_delay_sec * 1000UL;
  Serial.printf("[REC] Thresholds: start>%.1f kt (%ds), stop<%.1f kt (%ds)\n",
    startSpeedKnots, config.start_delay_sec, stopSpeedKnots, config.stop_delay_sec);

  // DON'T start logging immediately - GPS speed state machine controls this
  // Recording will auto-start when GPS speed > threshold
  recState = REC_IDLE;
  Serial.println("[REC] Auto-recording enabled - waiting for GPS speed trigger");

  // Watchdog timeout: 300s (5 min). HTTP PUTs of 600KB+ RTCM3 files at
  // marginal signal can stretch past 120s in a single sendRequest() —
  // we don't get to call esp_task_wdt_reset() inside HTTPClient. The
  // 2026-05-03 fleet simultaneous-reboot event is consistent with the
  // wdt firing on a slow PUT across multiple devices when signal briefly
  // degraded. 300s gives ~3.6 KB/s for a 1 MB file before tripping.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 300000,
    .idle_core_mask = 0,       // Don't monitor IDLE tasks on any core
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdt_config);

  // Create upload task on Core 0 (sensor reading stays on Core 1)
  xTaskCreatePinnedToCore(
    uploadTaskFunc,     // Function
    "uploadTask",       // Name
    12288,              // Stack size (needs room for HTTP + BLE deinit/reinit)
    NULL,               // Parameters
    1,                  // Priority
    &uploadTaskHandle,  // Handle
    0                   // Core 0
  );

  // Subscribe the Arduino main loop task (Core 1) to the watchdog. The upload
  // task subscribes itself in its first iteration. Without this the wdt is
  // configured but watching nothing, so a Core 1 hang stays silent (firmware
  // 2026.05.03.01 hard hang). With this, a 120s stall produces a panic +
  // backtrace and reboots the device.
  esp_err_t wdt_err = esp_task_wdt_add(NULL);
  if (wdt_err != ESP_OK) {
    Serial.printf("[WDT] Failed to subscribe loopTask: %d\n", wdt_err);
  } else {
    Serial.println("[WDT] loopTask subscribed");
  }

  // Diagnostic heartbeat task. Runs independently of loopTask so it keeps
  // printing even when Core 1 hangs. The g_loopSection it prints is the
  // last section Core 1 entered before stalling.
  xTaskCreatePinnedToCore(
    diagnosticsTask,    // Function
    "diagTask",         // Name
    4096,               // Stack
    NULL,               // Params
    1,                  // Priority
    &diagTaskHandle,    // Handle
    0                   // Core 0 (Core 1 is the one we're watching)
  );

  // Stage 2 re-enabled in .13 after the root cause of the .10 fleet-brick
  // was found: a single-byte stack overflow in meshBuildAndSendBoatState
  // writing to p->reserved[2] when reserved was sized [2]. Fixed at the
  // source. setup() is now back to the .10 sequence (meshInit + radio
  // mode transition). __stack_chk_fail was a TRUE positive — exactly
  // doing its job catching the smash on every meshTick call.
  meshInit();
  rtkRelayInit();   // RTK Phase-2: arm relay callbacks + rover ring (inert unless rtk_enabled)
  radioModeTransition(MODE_IDLE, "setup complete");

  Serial.println("[SETUP] Complete - WiFi/telnet available, GPS acquiring in background");
}

void loop() {
  static unsigned long lastDisp = 0, lastFlush = 0, lastIMU = 0, lastWind = 0;

  esp_task_wdt_reset();  // feed wdt — without this a stuck loop iteration
                         // panics in 300s with a backtrace pointing at the
                         // call that hung (firmware 2026.05.03.01 hard hang)
  g_loopIter++;
  g_loopSection = "top";
  unsigned long now = millis();

  // ----------------------------------------------------------
  // WiFi state management — MUST run before any handler call.
  //
  // (1) Sync wifiConnected with reality. WiFi.setAutoReconnect(false) means
  //     if the iPhone hotspot kicks an idle device, WiFi.status() flips to
  //     WL_DISCONNECTED but our flag stays stale. Calling handleTelnet()
  //     against a dead stack panics the device.
  //
  // (2) Honor teardown requests from the upload task. Core 0 sets the flag
  //     after a successful upload cycle; we tear down here on Core 1 because
  //     we own the handlers. Gated on !uploading && !triggerUpload to avoid
  //     racing a new upload cycle that just kicked off.
  // ----------------------------------------------------------
  // Helper: disconnect from AP without powering down the radio.
  // WiFi.disconnect(true) (wifioff=true) reconfigures the radio, which
  // races with the BLE wind-sensor scanner on the shared ESP32 radio
  // (CLAUDE.md known issues #11/#12) and panicked Core 1 in firmware
  // 2026.05.02.03. WiFi.disconnect(false, false) just leaves the AP —
  // the iPhone hotspot slot is freed, which was the goal — while the
  // radio stays in STA mode so BLE coexistence is undisturbed.
  g_loopSection = "wifi-state-sync";
  // Skip while Core 0 is mid-WiFi-work — these calls go through LWIP and
  // deadlock under contention.
  if (!wifiBusy && wifiConnected && WiFi.status() != WL_CONNECTED) {
    Serial.printf("[WIFI] Lost connection (status=%d) — clearing stale flag\n", WiFi.status());
    Serial.flush();
    g_loopSection = "wifi-state-sync.telnet-stop";
    if (telnetClient && telnetClient.connected()) telnetClient.stop();
    g_loopSection = "wifi-state-sync.server-end";
    telnetServer.end();
    g_loopSection = "wifi-state-sync.wifi-disconnect";
    WiFi.disconnect(false, false);
    connectedSSID[0] = '\0';
    wifiConnected = false;
    wifiTeardownRequested = false;  // Already torn down
  }

  g_loopSection = "wifi-teardown-check";
  if (wifiTeardownRequested && !uploading && !triggerUpload && !wifiBusy && wifiConnected) {
    Serial.println("[WIFI] Honoring teardown request (Core 1)");
    Serial.flush();
    g_loopSection = "teardown.telnet-stop";
    if (telnetClient && telnetClient.connected()) telnetClient.stop();
    g_loopSection = "teardown.server-end";
    telnetServer.end();
    g_loopSection = "teardown.wifi-disconnect";
    WiFi.disconnect(false, false);
    connectedSSID[0] = '\0';
    wifiConnected = false;
    wifiTeardownRequested = false;
    Serial.println("[WIFI] Teardown complete");
    Serial.flush();
  }

  // Telnet is skipped while wifiBusy: handleTelnet's WiFiServer/WiFiClient
  // calls share LWIP locks with Core 0's HTTP uploads and deadlock under
  // sustained contention (firmware 2026.05.03.03 hang).
  if (wifiConnected && !wifiBusy) {
    g_loopSection = "telnet";
    handleTelnet();
  }

  // BLE relay runs independent of WiFi state — it's not gated on wifiBusy
  // like the central wind-sensor scan (pauseBLEForWiFi()): GATT-server
  // notify/write servicing doesn't share NimBLEScan's documented
  // WiFi-contention hang, and pausing it here would drop an in-flight
  // phone relay transfer every time the device happens to also do a
  // periodic WiFi health check.
  g_loopSection = "ble-relay";
  bleRelayTick();

  g_loopSection = "mesh";
  meshTick();

  g_loopSection = "ocs";
  ocsTick();

  g_loopSection = "rc-ocs";
  rcComputeFleetOCS();

  g_loopSection = "fleetwatch";
  fleetWatchTick();

  g_loopSection = "serial-cmd";
  handleSerialCommand();


  g_loopSection = "gps";
  if (config.rtk_enabled && roleIsBase()) readGPSBase();   // demux RTCM-out + 1 Hz NMEA
  else                                    readGPS();        // unchanged NMEA-only path

  // RTK Phase-2 — rover: drain reassembled RTCM from the ring (filled in the
  // ESP-NOW recv callback) to the GNSS UART. Bounded + non-blocking: write only
  // what fits the UART TX buffer this iteration, never flush a backlog.
  if (config.rtk_enabled && roleIsRover() && g_rtcmRing) {
    g_loopSection = "rtcm-drain";
    uint8_t tmp[256];
    for (int budget = 4; budget > 0; budget--) {
      int canWrite = Serial2.availableForWrite();
      if (canWrite <= 0) break;
      if (canWrite > (int)sizeof(tmp)) canWrite = sizeof(tmp);
      size_t n = xStreamBufferReceive(g_rtcmRing, tmp, (size_t)canWrite, 0);
      if (n == 0) break;
      Serial2.write(tmp, n);
    }
  }

  // Once per boot: when GPS time first becomes valid, stamp boot.log with
  // wall-clock + battery so we can correlate this session with the previous
  // "alive" tail and tell battery-died from clean-power-off.
  if (!g_bootSessionLogged) {
    char iso[24];
    if (formatGpsIso(iso, sizeof(iso))) {
      char line[80];
      snprintf(line, sizeof(line), "session t=%s batt=%.2fV %d%%",
               iso, battery.voltage, battery.percent);
      appendBootLog(line);
      g_bootSessionLogged = true;
    }
  }

  g_loopSection = "rec-state";
  updateRecordingState();

  // Sensor reads are I2C on Core 1, upload runs on Core 0 — no conflict.
  // SD logging (logIMU/logPressure) is guarded by `logging` which is always
  // false during upload (task checks !logging before starting).
  if (now - lastIMU >= IMU_INTERVAL_MS) {
    g_loopSection = "imu";
    readIMU();
    if (logging) { g_loopSection = "imu.log"; logIMU(); }
    lastIMU = now;
  }

  // Pressure sensor (0.1 Hz - weather trends only, not gust detection)
  static unsigned long lastPres = 0;
  if (presOK && now - lastPres >= PRES_INTERVAL_MS) {
    g_loopSection = "pres";
    readPressure();
    if (logging) { g_loopSection = "pres.log"; logPressure(); }
    lastPres = now;
  }

  // Reset pressure min/max every 10 seconds for fresh gust window
  static unsigned long lastPresReset = 0;
  if (presOK && now - lastPresReset >= 10000) {
    g_loopSection = "pres-reset";
    resetPressureMinMax();
    lastPresReset = now;
  }

  if (logging && gps.newGGA) {
    g_loopSection = "nav.log";
    logNav();
    gps.newGGA = false;
  }

#if ENABLE_WIND
  // Handle wind sensor
  if (config.wind_enabled) {
    g_loopSection = "wind-check";
    checkWindConnection();

    // Log wind data at configured interval
    if (logging && now - lastWind >= WIND_INTERVAL_MS) {
      g_loopSection = "wind.log";
      logWind();
      lastWind = now;
    }
  }
#endif

  // The OCS alarm blinks at ~2 Hz (inverts every 250 ms); the normal 500 ms
  // display cadence is too slow to render that (and aliases to a static
  // frame), so tick the display ~every 120 ms while the alarm is up. The RC
  // fleet panel also wants a faster, live refresh.
  bool fastDisp = (g_ocs.armed && g_ocs.over_line) ||
                  (g_role == ROLE_RC_SIGNAL && g_ocs.armed);
  unsigned long dispGate = fastDisp ? 120 : DISPLAY_UPDATE_MS;
  if (now - lastDisp >= dispGate) {
    g_loopSection = "display";
    updateDisplay();
    // Adaptive backlight — recheck at every display tick, only write
    // PWM register when target changes (effectively at logging
    // start/stop). Saves ~30% of backlight current during idle.
    static uint8_t bl_current = TFT_BL_DUTY_IDLE;
    uint8_t bl_target = logging ? TFT_BL_DUTY_RECORDING : TFT_BL_DUTY_IDLE;
    if (bl_target != bl_current) {
      ledcWrite(TFT_BL_PIN, bl_target);
      bl_current = bl_target;
    }
    lastDisp = now;
  }

  if (logging && now - lastFlush >= FLUSH_INTERVAL_MS) {
    navFile.flush();
    if (imuFile) imuFile.flush();
#if ENABLE_WIND
    if (windFile) windFile.flush();
#endif
    if (presFile) presFile.flush();
    lastFlush = now;
  }

  // WiFi upload is handled entirely by uploadTaskFunc on Core 0
  // (removed checkWiFiUpload from main loop — was causing race condition crashes)

  // Battery monitoring (every 10 seconds)
  static unsigned long lastBattCheck = 0;
  if (now - lastBattCheck >= 10000) {
    g_loopSection = "battery";
    updateBattery();
    handleLowBattery();  // Will warn and halt if critical
    lastBattCheck = now;
  }
  g_loopSection = "loop-end";

  // RTCM3 debug output (every 30 seconds) - helps diagnose PPK data logging
  static unsigned long lastRtcmDebug = 0;
  if (now - lastRtcmDebug >= 30000) {
    // RTCM3 stats tracked but not printed (use 'status' command to see)
    lastRtcmDebug = now;
  }
}
