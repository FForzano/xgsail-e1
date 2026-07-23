// Calypso Mini wind sensor — BLE client (Environmental Sensing Service +
// Calypso's combined Data Service) providing apparent wind speed/angle.
#ifndef SAILFRAMES_WIND_SENSOR_H
#define SAILFRAMES_WIND_SENSOR_H

#include <Arduino.h>
#include <NimBLEDevice.h>

struct WindData {
  float speed_kts = 0;      // Apparent wind speed in knots
  float speed_mps = 0;      // Apparent wind speed in m/s
  int angle_deg = 0;        // Apparent wind angle (0-360)
  int battery = -1;         // Battery level (0-100, -1 = unknown)
  bool connected = false;
  bool newData = false;
  unsigned long lastUpdate = 0;
  char deviceName[32] = "";
  char deviceAddr[20] = "";
  char firmware[16] = "";   // Firmware version from Device Information Service
};

extern WindData wind;
extern bool windScanning;
extern bool windOK;
extern bool bleInitialized;  // Track BLE init state for safe deinit

// Initialize BLE + connect if /wind_mac.txt is present on SD (enables wind).
void initWindSensor();
// Stop any in-flight BLE scan/connection before WiFi work begins (shared
// ESP32 radio — sustained WiFi traffic can stall NimBLE without a timeout).
void pauseBLEForWiFi();
// Reconnect if disconnected and enough time has passed since the last try.
void checkWindConnection();
// Appends the current wind reading to the session's wind CSV.
void logWind();

#endif  // SAILFRAMES_WIND_SENSOR_H
