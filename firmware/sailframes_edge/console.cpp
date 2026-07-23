// Console command dispatcher glue — see console.h.
#include "console.h"
#include "config.h"
#include "v2_types.h"
#include "gnss.h"
#include "imu.h"
#include "wind_sensor.h"
#include "battery.h"
#include "pressure.h"
#include "mesh.h"
#include "rtk_relay.h"
#include "ocs.h"
#include "recording.h"
#include "storage.h"
#include "display.h"
#include "telnet.h"
#include "upload.h"
#include "device_auth.h"
#include "shared_state.h"
#include <SD.h>

void tprint(const char* msg) {
  Serial.print(msg);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(msg);
  }
}

void tprintf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  tprint(buf);
}

void tprintln(const char* msg) {
  Serial.println(msg);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(msg);
  }
}

// ============================================================

void processCommand(String cmd, bool fromTelnet) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "ls" || cmd == "list") {
    if (!sdOK) {
      tprintln("SD card not available");
      return;
    }
    tprintln("=== SD Card Contents ===");
    listDirOutput("/", 0, fromTelnet);
    tprintln("========================");

  } else if (cmd == "lssf") {
    if (!sdOK) {
      tprintln("SD card not available");
      return;
    }
    tprintln("=== /sf/ Contents ===");
    if (SD.exists("/sf")) {
      listDirOutput("/sf", 0, fromTelnet);
    } else {
      tprintln("/sf directory does not exist!");
      tprintln("Creating /sf...");
      if (SD.mkdir("/sf")) {
        tprintln("Created /sf successfully");
      } else {
        tprintln("Failed to create /sf");
      }
    }
    tprintf("Logging: %s\n", logging ? "ACTIVE" : "STOPPED");

  } else if (cmd == "status") {
    tprintln("=== Status ===");
    tprintf("GPS: %s, SAT:%d, HDOP:%.1f\n",
      gps.valid ? "FIX" : "NO FIX", gps.satellites, gps.hdop);
    tprintf("Position: %.6f, %.6f\n", gps.lat, gps.lon);
    tprintf("Speed: %.1f kt, Course: %.0f\n", gps.speed_kts, gps.course);
    tprintf("IMU: %s (heel:%.0f pitch:%.0f)%s\n",
      imuOK ? "BNO085" : "NONE", imu.heel, imu.pitch,
      g_imuFailed ? " ⚠ FAILED (no events)" :
        (g_imuSilentReads > 10 ? " (silent reads warning)" : ""));
    tprintf("Pres: %s", presOK ? "" : "NONE");
    if (presOK) tprintf("%.1f hPa, %.1f°C", pressure.pressure_hpa, pressure.temperature_c);
    tprintln("");
    tprintf("SD:  %s\n", sdOK ? "OK" : "FAILED");
    tprintf("Battery: %.2fV (%d%%)%s\n", battery.voltage, battery.percent,
      battery.critical ? " CRITICAL!" : "");
    tprintf("Logging: %s\n", logging ? "YES" : "NO");
    tprintf("Data logged: %lu KB\n", totalBytes / 1024);
    tprintf("WiFi: %s\n", wifiConnected ? connectedSSID : "disconnected");
    if (wifiConnected) {
      tprintf("IP: %s\n", WiFi.localIP().toString().c_str());
    }
#if ENABLE_WIND
    if (config.wind_enabled) {
      if (wind.connected) {
        tprintf("Wind: %.1f kt @ %d deg", wind.speed_kts, wind.angle_deg);
        if (wind.battery >= 0) tprintf(" (%d%%)", wind.battery);
        tprintln("");
      } else {
        tprintln("Wind: scanning...");
      }
    } else {
      tprintln("Wind: disabled");
    }
#endif
    tprintln("===============");

  } else if (cmd.startsWith("cat ")) {
    String path = cmd.substring(4);
    path.trim();
    if (!sdOK) {
      tprintln("SD card not available");
      return;
    }
    File f = SD.open(path.c_str());
    if (!f) {
      tprintf("Cannot open: %s\n", path.c_str());
      return;
    }
    tprintf("=== %s (%lu bytes) ===\n", path.c_str(), f.size());
    int lines = 0;
    while (f.available() && lines < 50) {
      String line = f.readStringUntil('\n');
      tprintf("%s\n", line.c_str());
      lines++;
      yield();
    }
    if (f.available()) tprintln("... (truncated at 50 lines)");
    f.close();

  } else if (cmd == "telneton") {
    telnetEnabled = true;
    if (wifiConnected && !telnetServerRunning) {
      startTelnetServer();
      tprintln("Telnet listener enabled and started");
    } else {
      tprintln("Telnet enabled — will start on next WiFi connect");
    }
  } else if (cmd == "telnetoff") {
    telnetEnabled = false;
    if (telnetServerRunning) {
      if (telnetClient && telnetClient.connected()) telnetClient.stop();
      telnetServer.end();
      telnetServerRunning = false;
      tprintln("Telnet listener stopped");
    } else {
      tprintln("Telnet was not running");
    }
  } else if (cmd == "upload") {
    if (!sdOK) {
      tprintln("SD card not available");
      return;
    }
    if (config.wifi_count == 0) {
      tprintln("WiFi not configured in config.txt");
      return;
    }
    tprintln("Starting manual upload...");
    // BLE deinit NOT needed — uploads use plain HTTP, no TLS memory pressure.
    // BLE and WiFi coexist fine for basic HTTP PUTs.

    tprintln("Connecting to WiFi...");
    if (connectWiFi()) {
      tprintf("Connected to: %s, IP: %s\n", connectedSSID, WiFi.localIP().toString().c_str());
      tprintf("Free heap: %u bytes\n", ESP.getFreeHeap());

      // Test API connectivity
      tprintln("Testing API connection...");
      if (!testApiConnectivity()) {
        tprintln("API connection FAILED");
        return;
      }
      tprintln("API OK");

      // Set uploading flag so Core 0 task doesn't interfere, and OLED shows progress
      uploading = true;
      uploadCount = 0;
      uploadSuccess = 0;
      uploadFailed = 0;
      uploadCurrentFile[0] = '\0';
      uploadTotal = countFilesToUpload("/sf");
      tprintf("Found %d files to upload\n", uploadTotal);

      tprintln("Calling uploadDirectory...");
      yield();
      delay(100);
      uploadDirectory("/sf");
      uploading = false;
      tprintln("Upload complete");
    }

  } else if (cmd == "wifi") {
    if (wifiConnected) {
      tprintf("Already connected to %s\n", connectedSSID);
      tprintf("IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
      tprintln("Connecting to WiFi...");
      if (connectWiFi()) {
        tprintf("Connected to %s\n", connectedSSID);
        tprintf("IP: %s\n", WiFi.localIP().toString().c_str());
      } else {
        tprintln("WiFi connection failed");
      }
    }

  } else if (cmd == "disconnect") {
    if (wifiConnected) {
      tprintln("Disconnecting WiFi...");
      // Set flag first to stop Core 1 from using WiFi services
      wifiConnected = false;
      delay(100);  // Let any pending WiFi operations complete
      if (telnetClient && telnetClient.connected()) {
        telnetClient.stop();
      }
      telnetServer.end();
      WiFi.disconnect(true);
      connectedSSID[0] = '\0';
      Serial.println("[WIFI] Disconnected via command");
    } else {
      tprintln("Not connected");
    }

  } else if (cmd.startsWith("claim ")) {
    // Device-protocol claim flow (docs/device-protocol.md §2). Needs
    // WiFi — connects first if not already up.
    String code = cmd.substring(6);
    code.trim();
    if (code.length() == 0) {
      tprintln("usage: claim <CODE>");
    } else if (isClaimed()) {
      tprintf("Already claimed (device_id=%s) — nothing to do\n", deviceId());
    } else {
      if (!wifiConnected) {
        tprintln("Connecting to WiFi...");
        connectWiFi();
      }
      if (!wifiConnected) {
        tprintln("claim: WiFi connect failed");
      } else {
        bool ok = claimDevice(code.c_str());
        tprintf("claim: %s\n", ok ? "OK" : "failed (see serial log)");
      }
    }

  } else if (cmd == "device" || cmd == "deviceinfo") {
    tprintln("=== Device ===");
    tprintf("external_id: %s\n", externalId());
    if (isClaimed()) {
      tprintf("claimed: yes (device_id=%s)\n", deviceId());
    } else {
      tprintln("claimed: no — use 'claim <CODE>' or set claim_code= in config.txt");
    }
    tprintf("api_base_url: %s\n", strlen(config.api_base_url) ? config.api_base_url : "(not set)");
    tprintln("==============");

  } else if (cmd == "reboot") {
    tprintln("Rebooting...");
    delay(500);
    ESP.restart();

  } else if (cmd == "gps") {
    tprintln("=== GPS Details ===");
    tprintf("Fix: %s (quality %d)\n", gps.valid ? "YES" : "NO", gps.fix_quality);
    tprintf("Satellites in fix: %d\n", gps.satellites);
    tprintf("Satellites in view: %d (GP:%d GL:%d GA:%d GB:%d)\n",
            satsInView, gsvGP, gsvGL, gsvGA, gsvGB);
    if (satsInView < gps.satellites) {
      tprintln("  NOTE: GLGSV/GAGSV messages may not be enabled");
    }
    tprintf("HDOP: %.1f%s\n", gps.hdop, gps.hdop > 50 ? " (no data)" : "");
    tprintf("Position: %.8f, %.8f\n", gps.lat, gps.lon);
    tprintf("Altitude: %.1f m\n", gps.alt);
    tprintf("Speed: %.2f kt\n", gps.speed_kts);
    tprintf("Course: %.1f deg\n", gps.course);
    tprintf("UTC: %s\n", gps.utc_time);
    tprintf("Date: %s\n", gps.date);
    tprintln("===================");

  } else if (cmd == "imu") {
    tprintln("=== IMU Details ===");
    tprintf("Type: %s\n", imuOK ? "BNO085" : "NONE");
    tprintf("Heading: %.0f deg (magnetic)\n", imu.heading);
    tprintf("Heel: %.1f deg (starboard +, port -)\n", imu.heel);
    tprintf("Pitch: %.1f deg (bow up +, bow down -)\n", imu.pitch);
    tprintf("Accel: X=%.2f Y=%.2f Z=%.2f\n", imu.accel_x, imu.accel_y, imu.accel_z);
    tprintf("Calibration offsets: heel=%.1f, pitch=%.1f\n", imuHeelOffset, imuPitchOffset);
    tprintln("===================");

  } else if (cmd == "pres" || cmd == "pressure") {
    tprintln("=== Pressure/Temperature ===");
    if (presOK) {
      tprintf("Pressure: %.2f hPa (mbar)\n", pressure.pressure_hpa);
      tprintf("Temperature: %.1f °C\n", pressure.temperature_c);
      tprintf("Pressure range (10s window):\n");
      tprintf("  Min: %.2f hPa\n", pressure.pressure_min);
      tprintf("  Max: %.2f hPa\n", pressure.pressure_max);
      tprintf("  Delta: %.2f hPa (gust indicator)\n",
        pressure.pressure_max - pressure.pressure_min);
    } else {
      tprintln("DPS310 not detected");
    }
    tprintln("============================");

  } else if (cmd == "imutest") {
    tprintln("=== IMU Axis Test (10 seconds) ===");
    tprintln("Tilt the device and watch which values change:");
    tprintln("  - Heel should change when tilting PORT/STARBOARD");
    tprintln("  - Pitch should change when tilting BOW UP/DOWN");
    tprintln("");
    unsigned long start = millis();
    while (millis() - start < 10000) {
      readIMU();
      // Show raw values (before calibration offset)
      float rawHeel = imu.heel + imuHeelOffset;
      float rawPitch = imu.pitch + imuPitchOffset;
      tprintf("H:%+6.1f P:%+6.1f  Accel X:%+5.2f Y:%+5.2f Z:%+5.2f\r",
        rawHeel, rawPitch, imu.accel_x, imu.accel_y, imu.accel_z);
      delay(200);
      yield();
    }
    tprintln("\n=== Test complete ===");
    tprintln("If axes are wrong, note which accel axis changes for each tilt");

  } else if (cmd == "cal" || cmd == "calibrate") {
    tprintln("=== IMU Calibration ===");
    tprintln("Place boat level on flat surface");
    tprintln("Current readings:");
    tprintf("  Heel: %.1f, Pitch: %.1f\n", imu.heel, imu.pitch);
    tprintln("Setting current position as zero...");
    calibrateIMU();
    tprintln("Calibration saved to SD card");
    tprintf("New offsets: heel=%.1f, pitch=%.1f\n", imuHeelOffset, imuPitchOffset);
    tprintln("=======================");

  } else if (cmd == "calreset") {
    tprintln("Resetting IMU calibration to defaults...");
    resetIMUCalibration();
    tprintln("Calibration reset to zero");

  } else if (cmd == "cleanup" || cmd == "delup") {
    if (!sdOK) {
      tprintln("SD card not available");
      return;
    }
    tprintln("Deleting uploaded files...");
    int deleted = deleteUploadedFiles("/sf");
    tprintf("Deleted %d files\n", deleted);

  } else if (cmd == "clearmarkers") {
    if (!sdOK) {
      tprintln("SD card not available");
      return;
    }
    tprintln("Clearing .uploaded markers (keeping data files)...");
    int count = 0;
    // Clear markers in all session directories
    File root = SD.open("/sf");
    if (root) {
      File dir = root.openNextFile();
      while (dir) {
        if (dir.isDirectory()) {
          String dirPath = String("/sf/") + dir.name();
          File subdir = SD.open(dirPath);
          if (subdir) {
            File f = subdir.openNextFile();
            while (f) {
              String fname = String(f.name());
              f.close();
              if (fname.endsWith(".uploaded")) {
                String fullPath = dirPath + "/" + fname;
                if (SD.remove(fullPath.c_str())) {
                  count++;
                }
              }
              f = subdir.openNextFile();
            }
            subdir.close();
          }
        }
        dir.close();
        dir = root.openNextFile();
      }
      root.close();
    }
    // Also clear markers in /sf root
    File sfRoot = SD.open("/sf");
    if (sfRoot) {
      File f = sfRoot.openNextFile();
      while (f) {
        String fname = String(f.name());
        f.close();
        if (fname.endsWith(".uploaded")) {
          String fullPath = String("/sf/") + fname;
          if (SD.remove(fullPath.c_str())) {
            count++;
          }
        }
        f = sfRoot.openNextFile();
      }
      sfRoot.close();
    }
    tprintf("Cleared %d marker files\n", count);

  } else if (cmd == "gpsraw") {
    tprintln("=== Raw GPS data (10 seconds) ===");
    tprintln("Press any key to stop early...");
    unsigned long start = millis();
    while (millis() - start < 10000) {
      while (Serial2.available()) {
        char c = Serial2.read();
        if (c >= 32 || c == '\n' || c == '\r') {
          Serial.print(c);
          if (telnetClient && telnetClient.connected()) {
            telnetClient.print(c);
          }
        }
      }
      // Check for keypress to stop
      if (Serial.available() || (telnetClient && telnetClient.available())) {
        while (Serial.available()) Serial.read();
        while (telnetClient && telnetClient.available()) telnetClient.read();
        break;
      }
      yield();
    }
    tprintln("\n=== End raw GPS ===");

  } else if (cmd == "gpscfg") {
    tprintln("Reconfiguring GPS...");
    gnssConfigure();   // RTK off ⇒ configureLG290P(); on ⇒ base/rover per role+chip
    tprintln("GPS reconfigured");

  } else if (cmd == "rtk") {
    // RTK Phase-2 relay status (bench verification).
    tprintf("rtk_enabled=%d role=%s (%s) hw=%s\n", config.rtk_enabled,
            roleName(g_role), roleIsBase() ? "base/produce" : "rover/consume", hwName(g_hw));
    tprintf("gps fix_quality=%d (4=RTK-FIXED 5=float 2=DGPS 1=GPS) sat=%d hdop=%.1f\n",
            gps.fix_quality, gps.satellites, gps.hdop);
    tprintf("accuracy: h=%.3f m (1sigma; GST=LG290P / PQTMEPE=LC29HEA)%s\n",
            gps.hacc_m, (gps.hacc_m == 0) ? "  (no data yet)" : "");
    if (roleIsBase()) {
      tprintf("base: tx_msg_id=%u (frames fragmented+broadcast 2x)\n", (unsigned)g_rtcmTxMsgId);
    } else {
      tprintf("rover: pkts=%lu complete=%lu crc_fail=%lu dropped=%lu dup=%lu bad=%lu ring=%u\n",
              g_rtcmRx.s_pkts, g_rtcmRx.s_complete, g_rtcmRx.s_crc_fail, g_rtcmRx.s_dropped,
              g_rtcmRx.s_dup, g_rtcmRx.s_bad,
              g_rtcmRing ? (unsigned)xStreamBufferBytesAvailable(g_rtcmRing) : 0);
    }

  } else if (cmd.startsWith("setcfg ")) {
    // Bench helper: append a key=value to /config.txt so config can be set over
    // USB/telnet without pulling the SD. APPEND-only ⇒ never rewrites existing
    // identity/wifi lines (no corruption risk); loadConfig() takes the LAST
    // occurrence of a key, so the appended value wins. Reboot to apply.
    String kv = cmd.substring(7); kv.trim();
    int eq = kv.indexOf('=');
    if (eq < 1 || eq >= (int)kv.length() - 1) {
      tprintln("usage: setcfg key=value   (e.g. setcfg rtk_enabled=1)");
    } else {
      File f = SD.open("/config.txt", FILE_APPEND);
      if (!f) {
        tprintln("setcfg: cannot open /config.txt");
      } else {
        f.print("\n"); f.print(kv); f.print("\n"); f.close();
        tprintf("setcfg: appended '%s' — power-cycle/reset to apply (config is read at boot)\n",
                kv.c_str());
      }
    }

  } else if (cmd == "wind") {
#if ENABLE_WIND
    tprintln("=== Wind Sensor ===");
    tprintf("Enabled: %s\n", config.wind_enabled ? "yes" : "no");
    tprintf("Connected: %s\n", wind.connected ? "yes" : "no");
    if (strlen(wind.deviceName) > 0) {
      tprintf("Device: %s (%s)\n", wind.deviceName, wind.deviceAddr);
    }
    if (strlen(wind.firmware) > 0) {
      tprintf("Firmware: %s\n", wind.firmware);
    }
    if (wind.connected) {
      tprintf("Speed: %.1f kts (%.1f m/s)\n", wind.speed_kts, wind.speed_mps);
      tprintf("Direction: %d deg (apparent)\n", wind.angle_deg);
      if (wind.battery >= 0) {
        tprintf("Battery: %d%%\n", wind.battery);
      }
      tprintf("Last update: %lu ms ago\n", millis() - wind.lastUpdate);
    }
    if (strlen(config.wind_mac) > 0) {
      tprintf("Saved MAC: %s\n", config.wind_mac);
    }
    tprintln("===================");
#else
    tprintln("Wind sensor support not compiled in");
#endif

  } else if (cmd == "windscan") {
#if ENABLE_WIND
    tprintln("Scanning for Calypso wind sensors...");
    if (scanForCalypso()) {
      tprintf("Found: %s at %s\n", wind.deviceName, wind.deviceAddr);
      tprintln("Attempting connection...");
      if (connectToCalypso()) {
        tprintln("Connected successfully!");
      } else {
        tprintln("Connection failed");
      }
    } else {
      tprintln("No Calypso device found");
    }
#else
    tprintln("Wind sensor support not compiled in");
#endif

  } else if (cmd == "blescan") {
#if ENABLE_WIND
    tprintln("BLE scan (5 sec)...");
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->clearResults();
    pScan->start(5, false);
    delay(6000);
    pScan->stop();
    NimBLEScanResults r = pScan->getResults();
    tprintf("Found %d\n", r.getCount());
    for (int i = 0; i < r.getCount(); i++) {
      const NimBLEAdvertisedDevice* d = r.getDevice(i);
      if (d) tprintf("%s %s\n", d->getAddress().toString().c_str(), d->getName().c_str());
    }
#else
    tprintln("No BLE");
#endif

  } else if (cmd == "bledeinit") {
#if ENABLE_WIND
    if (!bleInitialized) {
      tprintln("BLE not initialized");
      return;
    }
    tprintln("Deinitializing BLE to free memory...");
    tprintf("Heap before: %u bytes\n", ESP.getFreeHeap());
    if (pWindClient && pWindClient->isConnected()) {
      pWindClient->disconnect();
    }
    pWindClient = nullptr;
    pWindSpeedChar = nullptr;
    pWindDirChar = nullptr;
    pBatteryChar = nullptr;
    pDataChar = nullptr;
    wind.connected = false;
    NimBLEDevice::deinit(false);
    bleInitialized = false;
    delay(500);
    tprintf("Heap after: %u bytes\n", ESP.getFreeHeap());
    tprintln("BLE disabled. Run 'bleinit' to restart.");
#else
    tprintln("No BLE");
#endif

  } else if (cmd == "bleinit") {
#if ENABLE_WIND
    tprintln("Reinitializing BLE...");
    tprintf("Heap before: %u bytes\n", ESP.getFreeHeap());
    if (bleInitialized) {
      tprintln("Deinitializing first...");
      NimBLEDevice::deinit(false);
      bleInitialized = false;
      delay(500);
    }
    tprintln("Calling NimBLEDevice::init()...");
    NimBLEDevice::init("SailFrames-E1");
    bleInitialized = true;
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    tprintf("BLE address: %s\n", NimBLEDevice::getAddress().toString().c_str());
    tprintf("Heap after: %u bytes\n", ESP.getFreeHeap());
    tprintln("Done. Try 'blescan' now.");
#else
    tprintln("BLE not compiled in");
#endif

  } else if (cmd.startsWith("bleconnect ")) {
#if ENABLE_WIND
    String mac = cmd.substring(11);
    mac.trim();
    tprintf("Connecting to %s...\n", mac.c_str());
    strncpy(wind.deviceAddr, mac.c_str(), sizeof(wind.deviceAddr) - 1);
    strncpy(config.wind_mac, mac.c_str(), sizeof(config.wind_mac) - 1);
    if (connectToCalypso()) {
      tprintln("Connected! Saving MAC.");
      saveWindMAC(mac.c_str());
    } else {
      tprintln("Connection failed");
    }
#else
    tprintln("No BLE");
#endif

  } else if (cmd == "display") {
    displayMode = (displayMode >= 3) ? 1 : displayMode + 1;
    // Force layout redraw on mode switch
    d2LayoutDrawn = false;
    d3LayoutDrawn = false;
    tprintf("Display mode: D%d\n", displayMode);
    updateDisplay();

  } else if (cmd == "heap") {
    tprintln("=== Memory Status ===");
    tprintf("Free heap: %u bytes\n", ESP.getFreeHeap());
    tprintf("Min free heap: %u bytes\n", ESP.getMinFreeHeap());
    tprintf("Max alloc heap: %u bytes\n", ESP.getMaxAllocHeap());
    tprintf("PSRAM: %u bytes free\n", ESP.getFreePsram());
    tprintf("Sketch size: %u bytes\n", ESP.getSketchSize());
    tprintf("Free sketch space: %u bytes\n", ESP.getFreeSketchSpace());
    tprintln("SSL needs ~45KB free heap");

  } else if (cmd == "testssl") {
    tprintf("Free heap before test: %u bytes\n", ESP.getFreeHeap());
    tprintln("Testing SSL to google.com:443...");
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(10);  // 10 second timeout
    if (client.connect("www.google.com", 443)) {
      tprintln("Google SSL OK!");
      client.stop();
    } else {
      tprintln("Google SSL FAILED");
    }

    tprintln("Testing SSL to AWS API Gateway...");
    WiFiClientSecure client2;
    client2.setInsecure();
    client2.setTimeout(10);
    if (client2.connect("p9s9eia0t6.execute-api.us-east-1.amazonaws.com", 443)) {
      tprintln("AWS SSL OK!");
      client2.println("PUT /prod/upload?boat=E1&file=test.txt HTTP/1.1");
      client2.println("Host: p9s9eia0t6.execute-api.us-east-1.amazonaws.com");
      client2.println("Content-Type: text/plain");
      client2.println("Content-Length: 4");
      client2.println("Connection: close");
      client2.println();
      client2.print("test");
      delay(2000);
      while (client2.available()) {
        String line = client2.readStringUntil('\n');
        tprintf("%s\n", line.c_str());
      }
      client2.stop();
    } else {
      tprintln("AWS SSL FAILED");
      char errBuf[128];
      client2.lastError(errBuf, sizeof(errBuf));
      tprintf("Error: %s\n", errBuf);
    }

    tprintln("Testing plain HTTP to httpbin...");
    WiFiClient client3;
    if (client3.connect("httpbin.org", 80)) {
      tprintln("HTTP connected!");
      client3.println("GET /get HTTP/1.1");
      client3.println("Host: httpbin.org");
      client3.println("Connection: close");
      client3.println();
      delay(1000);
      int lines = 0;
      while (client3.available() && lines < 5) {
        String line = client3.readStringUntil('\n');
        tprintf("%s\n", line.c_str());
        lines++;
      }
      client3.stop();
    } else {
      tprintln("HTTP FAILED");
    }

  } else if (cmd == "rec" || cmd == "startrec") {
    // Manual start recording — same entry point the button/BLE use.
    if (logging) {
      tprintln("Already recording");
    } else if (startRecording()) {
      tprintf("Recording session %d started\n", sessionCount);
    } else {
      tprintln("Could not start recording (no SD card, or SD busy)");
    }

  } else if (cmd == "stoprec") {
    // Manual stop recording — same entry point the button/BLE use.
    if (!logging) {
      tprintln("Not recording");
    } else {
      stopRecording();
      tprintf("Recording session %d stopped\n", sessionCount);
    }

  } else if (cmd == "recstate") {
    // Show recording state
    tprintln("=== Recording State ===");
    tprintf("State: %s\n", getRecStateStr());
    tprintf("Logging: %s\n", logging ? "YES" : "NO");
    tprintf("Session: %d\n", sessionCount);
    tprintf("Speed: %.1f kt\n", gps.speed_kts);
    tprintln("Start/stop: button, console rec/stoprec, or BLE start-rec/stop-rec (no auto-trigger)");
    tprintf("Move-detect threshold (aborts uploads): >%.1f kt\n", config.start_speed_knots);

  // v2.0.0 foundation commands (SF_FIRMWARE_V2_SPEC.md Stage 1)
  } else if (cmd == "hwid") {
    tprintf("platform=%s (config=%s)\n", hwName(g_hw), config.hardware_platform);

  } else if (cmd == "role") {
    tprintf("role=%s radio_mode=%s\n", roleName(g_role), radioModeName(g_radio_mode));

  } else if (cmd == "flags") {
    tprintf("imu_interval_ms=%d (baked)\n", IMU_INTERVAL_MS);
    tprintf("gnss_fix_rate=10 Hz (baked, PQTMCFGFIXRATE)\n");
    tprintf("wind_enabled=%s\n", config.wind_enabled ? "on" : "off");
    tprintf("telnet=%s\n", telnetEnabled ? "on" : "off");

  } else if (cmd == "radiomode") {
    tprintf("radio_mode=%s\n", radioModeName(g_radio_mode));

  } else if (cmd == "health") {
    // Force a health snapshot POST now (docs/device-protocol.md §4.4).
    // Manual trigger for testing — the upload task does this once per
    // boot automatically.
    if (!wifiConnected) {
      tprintln("health: WiFi not connected; run `wifi` to bring it up");
    } else if (!isClaimed()) {
      tprintln("health: device not claimed");
    } else {
      bool ok = uploadHealthSnapshot();
      tprintf("health: %s\n", ok ? "OK" : "failed (see Serial)");
    }

  } else if (cmd == "ocs") {
    // Stage 4 — boat-local OCS state.
    if (!g_ocs.armed) {
      tprintln("ocs: NOT ARMED. Use `race arm <pin_lat> <pin_lon> <rc_lat> <rc_lon> <secs>`");
    } else {
      uint32_t now = millis();
      int32_t to_start = (int32_t)(g_ocs.start_time_ms - now);
      tprintln("ocs: ARMED");
      tprintf("  PIN: %.7f, %.7f\n", g_ocs.pin_lat, g_ocs.pin_lon);
      tprintf("  RC:  %.7f, %.7f\n", g_ocs.rc_lat, g_ocs.rc_lon);
      if (to_start > 0) {
        tprintf("  Start in: %d s\n", to_start / 1000);
      } else {
        tprintf("  Started: %d s ago\n", -to_start / 1000);
      }
      const char* side = g_ocs.distance_to_line_m >= 0 ? "pre-start" : "course";
      tprintf("  Distance to line: %+.2f m (%s side)\n",
              g_ocs.distance_to_line_m, side);
      tprintf("  Closure rate: %+.2f m/s%s\n", g_ocs.closure_rate_m_s,
              g_ocs.closure_rate_m_s < 0 ? " (approaching line)" : "");
      tprintf("  Over line: %s\n", g_ocs.over_line ? "YES" : "no");
      tprintf("  Was over at start: %s\n",
              g_ocs.was_over_at_start ? "YES" : "no");
    }

  } else if (cmd.startsWith("race arm ")) {
    // race arm <pin_lat> <pin_lon> <rc_lat> <rc_lon> <secs_from_now>
    // Stage 4.5: also broadcasts MSG_RACE_ARMED over ESP-NOW so all
    // other boats arm at the same instant. 3x transmission for
    // reliability — single packet losses don't lose the race start.
    double pln, plg, rln, rlg;
    int secs = 0;
    int n = sscanf(cmd.c_str(), "race arm %lf %lf %lf %lf %d",
                   &pln, &plg, &rln, &rlg, &secs);
    if (n != 5) {
      tprintln("usage: race arm <pin_lat> <pin_lon> <rc_lat> <rc_lon> <secs_from_now>");
      tprintln("       example: race arm 42.3601 -71.0589 42.3604 -71.0578 300");
    } else {
      uint32_t start_ms = millis() + (uint32_t)(secs * 1000);
      ocsArm(pln, plg, rln, rlg, start_ms);
      bool sent = meshBroadcastRaceArmed(pln, plg, rln, rlg, secs, 0, 30);
      tprintf("race armed locally: PIN(%.5f,%.5f) RC(%.5f,%.5f) T+0 in %d s\n",
              pln, plg, rln, rlg, secs);
      tprintf("mesh broadcast: %s (3x for reliability)\n", sent ? "OK" : "FAILED");
    }

  } else if (cmd.startsWith("race armrtk")) {
    // Increment 2 — capture the start line in the RTK frame, so cm-accurate
    // boats are measured against a cm-accurate line (not a ±2 m typed line).
    //   RC end  = own position (RC base = committee end = RTK frame origin)
    //   PIN end = the rc_pin peer's latest RTK-FIXED position over the mesh
    // Then the existing ocsArm + MSG_RACE_ARMED fleet path, with cm coords.
    int secs = -1;
    if (sscanf(cmd.c_str(), "race armrtk %d", &secs) != 1 || secs < 0) {
      tprintln("usage: race armrtk <secs_from_now>   (RC-only; captures line from base + rc_pin RTK)");
    } else if (!config.rtk_enabled) {
      tprintln("race armrtk: rtk_enabled is OFF (SD config). Use `race arm <coords>` for the manual line.");
    } else if (g_role != ROLE_RC_SIGNAL) {
      tprintln("race armrtk: RC-only — this boat is not unit_role=rc_signal (the base).");
    } else if (!gps.valid || (gps.lat == 0 && gps.lon == 0)) {
      tprintln("race armrtk: RC base has no position yet (survey-in not complete?).");
    } else {
      uint32_t now = millis();
      int pin = -1;
      for (int i = 0; i < g_mesh_peer_count; i++) {
        if (g_mesh_peers[i].unit_role == ROLE_RC_PIN &&
            (now - g_mesh_peers[i].last_seen_ms) < 5000) { pin = i; break; }
      }
      if (pin < 0) {
        tprintln("race armrtk: no rc_pin peer in last 5s — is the pin boat on (unit_role=rc_pin) + in the mesh?");
      } else if (g_mesh_peers[pin].fix_quality != 4) {
        tprintf("race armrtk: rc_pin peer NOT RTK FIXED (q=%d) — wait for q=4 so the pin end is cm-accurate.\n",
                g_mesh_peers[pin].fix_quality);
      } else {
        double pln = g_mesh_peers[pin].last_lat_e7 / 1e7;
        double plg = g_mesh_peers[pin].last_lon_e7 / 1e7;
        double rln = gps.lat, rlg = gps.lon;
        // line-length sanity (equirectangular)
        double refLat = ((pln + rln) / 2.0) * PI / 180.0;
        double dx = (plg - rlg) * 111320.0 * cos(refLat);
        double dy = (pln - rln) * 111320.0;
        double lineLen = sqrt(dx * dx + dy * dy);
        if (lineLen < 10.0 || lineLen > 1000.0) {
          tprintf("race armrtk: line length %.1f m out of sane range (10-1000 m) — check positions. NOT armed.\n",
                  lineLen);
        } else {
          uint32_t start_ms = now + (uint32_t)(secs * 1000);
          ocsArm(pln, plg, rln, rlg, start_ms);
          bool sent = meshBroadcastRaceArmed(pln, plg, rln, rlg, secs, 0, 30);
          tprintf("race ARMED (RTK frame): PIN(%.7f,%.7f q=4)  RC(%.7f,%.7f base)  len=%.1f m  T+0 in %d s\n",
                  pln, plg, rln, rlg, lineLen, secs);
          tprintf("mesh broadcast: %s (3x). NOTE: RC end taken from base GGA — verify it equals the surveyed ARP (1005).\n",
                  sent ? "OK" : "FAILED");
        }
      }
    }

  } else if (cmd == "race disarm" || cmd == "race off") {
    ocsDisarm();
    tprintln("race: disarmed locally (no mesh disarm message yet)");

  } else if (cmd == "mesh") {
    // ESP-NOW peer mesh status (Stage 2)
    if (!g_mesh_enabled) {
      tprintln("mesh: DISABLED");
    } else {
      tprintf("mesh: enabled, sender_id=0x%08lx, peers=%d/%d\n",
              (unsigned long)g_mesh_local_sender_id,
              g_mesh_peer_count, MESH_PEER_MAX);
      tprintf("  tx=%lu (fail %lu), rx=%lu (bad %lu)\n",
              (unsigned long)g_mesh_tx_count,
              (unsigned long)g_mesh_tx_fail_count,
              (unsigned long)g_mesh_rx_count,
              (unsigned long)g_mesh_rx_dropped_bad_magic);
      unsigned long now = millis();
      for (int i = 0; i < g_mesh_peer_count; i++) {
        const MeshPeerState& p = g_mesh_peers[i];
        tprintf("  %-3s 0x%08lx role=%u age=%lus rssi=%ddBm msgs=%lu lat=%.7f lon=%.7f sog=%.1fkt cog=%d hdg.heel=%d\n",
                boatNameForSender(p.sender_id),
                (unsigned long)p.sender_id,
                (unsigned)p.unit_role,
                (now - p.last_seen_ms) / 1000,
                (int)p.last_rssi,
                (unsigned long)p.msg_count,
                p.last_lat_e7 / 1e7,
                p.last_lon_e7 / 1e7,
                p.last_sog_cm_s / 51.4444,
                p.last_cog_deg10 / 10,
                p.last_heel_deg);
      }
    }

  } else if (cmd == "fleet") {
    // Stage 5 — RC unit's fleet OCS view.
    // Shows per-peer distance from start line + RC-side OCS state.
    // RC-only because boats don't compute fleet-wide OCS.
    if (g_role != ROLE_RC_SIGNAL) {
      tprintf("fleet: only meaningful when role=rc_signal (current role=%d)\n",
              (int)g_role);
    } else if (!g_ocs.armed) {
      tprintln("fleet: OCS not armed (no race armed; use 'race arm ...')");
    } else {
      unsigned long now = millis();
      int32_t time_to_start = (int32_t)(g_ocs.start_time_ms - now);
      tprintf("fleet (RC view): %d peers, T%+ds, line %.6f,%.6f -> %.6f,%.6f\n",
              g_mesh_peer_count, time_to_start / 1000,
              g_ocs.pin_lat, g_ocs.pin_lon, g_ocs.rc_lat, g_ocs.rc_lon);
      for (int i = 0; i < g_mesh_peer_count; i++) {
        const MeshPeerState& p = g_mesh_peers[i];
        const char* ocs_state =
            p.rc_ocs_called ? "OCS"
                            : (p.rc_distance_m < 0 ? "over" : "ok ");
        tprintf("  0x%08lx role=%u fix=%u sat=%2u sog=%.1fkt hdg=%4.0f bow=%.2fm d=%+6.2fm %s%s\n",
                (unsigned long)p.sender_id,
                (unsigned)p.unit_role,
                (unsigned)p.fix_quality,
                (unsigned)p.sat_count,
                p.last_sog_cm_s / 51.4444,
                p.last_heading_deg10 / 10.0,
                bowOffsetForSender(p.sender_id),
                p.rc_distance_m,
                ocs_state,
                p.rc_ocs_called ? "*" : "");
      }
    }

  } else if (cmd == "fleetwatch") {
    // Toggle the live RC fleet dashboard (refreshes from fleetWatchTick()
    // in the main loop — non-blocking). VT100 terminal required.
    g_fleetWatch = !g_fleetWatch;
    if (g_fleetWatch) {
      g_fleetWatchLast = 0;          // paint on the next tick immediately
      Serial.print("\033[2J");       // clear screen on start
      tprintln("fleetwatch: ON (live ~2 Hz; type 'fleetwatch' again to stop)");
    } else {
      tprintln("fleetwatch: OFF");
    }

  } else if (cmd == "classes") {
    // Stage 5.5 — dump per-class bow_offset registry loaded from
    // /sf/classes.csv. RC-only (boats use OCS_BOW_OFFSET_M directly).
    if (g_class_registry_count == 0) {
      tprintf("classes: registry empty (default bow=%.2fm applied to all peers)\n",
              OCS_BOW_OFFSET_M);
    } else {
      tprintf("classes: %d entries loaded from /sf/classes.csv\n",
              g_class_registry_count);
      for (int i = 0; i < g_class_registry_count; i++) {
        const ClassRegistryEntry& e = g_class_registry[i];
        tprintf("  %-12s (0x%08lx) class=%-12s bow=%.2fm\n",
                e.boat_id,
                (unsigned long)e.sender_id,
                e.class_name,
                e.bow_offset_m);
      }
    }

  } else if (cmd == "help") {
    tprintln("=== Commands ===");
    tprintln("  status     - Show device status");
    tprintln("  gps        - Detailed GPS info");
    tprintln("  gpsraw     - Show raw GPS serial data");
    tprintln("  gpscfg     - Reconfigure GPS module");
    tprintln("  imu        - Detailed IMU info");
    tprintln("  imutest    - Test IMU axes (5 sec)");
    tprintln("  cal        - Calibrate IMU (set level)");
    tprintln("  calreset   - Reset IMU calibration");
    tprintln("  pres       - Pressure/temperature sensor");
    tprintln("  rec        - Manual start recording");
    tprintln("  stoprec    - Manual stop recording");
    tprintln("  recstate   - Show recording state");
    tprintln("  wind       - Wind sensor info");
    tprintln("  windscan   - Scan for wind sensor");
    tprintln("  blescan    - Scan ALL BLE devices");
    tprintln("  bledeinit  - Deinit BLE (free memory)");
    tprintln("  bleinit    - Reinitialize BLE");
    tprintln("  bleconnect <mac> - Connect to BLE MAC");
    tprintln("  display    - Toggle display mode (D1/D2)");
    tprintln("  heap       - Show memory status");
    tprintln("  testssl    - Test SSL connection");
    tprintln("  ls, list   - List SD card files");
    tprintln("  cat <file> - Show file contents");
    tprintln("  upload     - Manual upload via device protocol");
    tprintln("  cleanup    - Delete uploaded files");
    tprintln("  telneton   - Enable telnet listener (off by default)");
    tprintln("  telnetoff  - Disable telnet listener");
    tprintln("  wifi       - Connect to WiFi");
    tprintln("  disconnect - Disconnect WiFi");
    tprintln("  claim <CODE> - Redeem a device-protocol claim code");
    tprintln("  device     - Show external_id + claim status");
    tprintln("  reboot     - Restart device");
    tprintln("  hwid       - Show detected hardware platform");
    tprintln("  role       - Show unit role + radio mode");
    tprintln("  flags      - Show v2.0.0 feature flag state");
    tprintln("  radiomode  - Show current radio mode");
    tprintln("  mesh       - ESP-NOW peer mesh status + peers seen");
    tprintln("  fleet      - RC view of fleet OCS (RC-only)");
    tprintln("  fleetwatch - live RC fleet OCS dashboard, ~2 Hz (RC-only; VT100 term)");
    tprintln("  classes    - Show /sf/classes.csv bow_offset registry (RC-only)");
    tprintln("  health     - Push a health snapshot now");
    tprintln("  ocs        - Show OCS state (Stage 4)");
    tprintln("  race arm <pin_lat> <pin_lon> <rc_lat> <rc_lon> <secs>");
    tprintln("             - Arm OCS state machine for a race start");
    tprintln("  race disarm - Clear OCS arming");
    tprintln("  help       - Show this help");
    tprintln("================");

  } else {
    tprintf("Unknown command: %s (type 'help')\n", cmd.c_str());
  }
}

void handleSerialCommand() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.printf("\n> %s\n", cmd.c_str());
  processCommand(cmd, false);
}
