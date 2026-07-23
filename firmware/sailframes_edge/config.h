// SailFrames E1 — pin assignments, firmware-wide constants, and the
// on-SD-card Config struct (config.txt). Split out of the original
// monolithic sailframes_edge.ino so every module can pull in the
// constants it needs without depending on the whole sketch.
#ifndef SAILFRAMES_CONFIG_H
#define SAILFRAMES_CONFIG_H

#include <Arduino.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define GPS_RX_PIN    16
#define GPS_TX_PIN    17
#define SDA_PIN       21
#define SCL_PIN       22

// TFT Display (Hosyond 3.5" IPS ST7796U) - SPI
#define TFT_CS_PIN    5   // LCD chip select
#define TFT_DC_PIN    2   // Data/Command (also used as LED_PIN on old board)
#define TFT_RST_PIN   4   // LCD reset
#define TFT_BL_PIN    25  // Backlight PWM

// Adaptive backlight (2026.05.26.04). Backlight is the dominant load
// in the power budget (~40% of total system draw). Dimming when not
// recording recovers ~30% of backlight current during the "between
// sails / pre-start" idle periods, gaining ~10-15% of total system
// runtime. Levels chosen per operator preference: 80% gives plenty of
// daylight readability while shaving 20% off the recording-mode
// backlight current; 50% is still legible in shade and saves half the
// idle backlight current.
//
// Uses the ESP32 Arduino Core 3.x ledcAttach API (pin-addressed), NOT
// the legacy 2.x ledcSetup/ledcAttachPin channel-addressed API which
// doesn't exist in Core 3.3.7.
#define TFT_BL_PWM_FREQ     5000   // 5 kHz — well above flicker perception
#define TFT_BL_PWM_RES      8      // 8-bit duty (0-255)
#define TFT_BL_DUTY_RECORDING 204  // ~80%
#define TFT_BL_DUTY_IDLE      128  // ~50%

// SD Card - standalone module on SEPARATE HSPI bus (eliminates TFT flicker)
// HSPI pins - completely independent from TFT's VSPI bus
// NOTE: GPIO12 is a strapping pin - avoid it! Using GPIO35 for MISO instead
#define SD_CS_PIN     27  // SD card CS (GPIO27)
#define SD_CLK_PIN    14  // HSPI CLK
#define SD_MISO_PIN   35  // MISO on GPIO35 (input-only pin, perfect for MISO)
#define SD_MOSI_PIN   13  // HSPI MOSI

// LED_PIN disabled - was causing backlight to blink during logging
#define LED_PIN       -1  // Set to -1 to disable LED blinking

// Battery monitoring (DWEII USB-C Boost Converter)
// 100K/100K voltage divider from LiPo B+ to GPIO34
#define BATT_VOLTAGE_PIN  34   // ADC pin for voltage divider (input-only, no pullup)

// Power control: Hardware switch on boost converter.
// No software deep sleep - hardware switch cuts all power when OFF.

// ============================================================
// CONFIGURATION
// ============================================================
// Firmware version: YYYY.MM.DD.N (date + daily build number)
#define FW_VERSION    "2026.06.29.02"   // E (LG290P)

// v2.0.0 foundation: HW platform / unit role / radio mode skeleton.
// 10 Hz GNSS + 10 Hz IMU are now baked-in firmware defaults (no longer
// per-boat config knobs). config.txt holds per-boat / per-club state
// only (WiFi creds, boat_id, wind sensor, role, etc.).

// Telnet listener is OFF by default. The 2026.05.03.04 fleet test confirmed
// (via diag heartbeat) that handleTelnet() blocks Core 1 inside LWIP when
// Core 0 is doing concurrent HTTP uploads — even with the wifiBusy gate,
// because Core 1 may already be INSIDE handleTelnet when the upload fires
// and the gate only prevents new entries. Easiest robust fix: don't run
// the listener during automated post-sail uploads. Set telnetEnabled=true
// at runtime via the serial 'telneton' command if you need to debug live.
#define TELNET_ENABLED_DEFAULT  false

// ArduinoOTA registers an mDNS multicast UDP listener. On ESP32 Arduino
// Core 3.3.7 with NimBLE active for the wind sensor, the mDNS init at
// WiFi-up time crosses into the BLE/WiFi shared-radio coexistence path
// and panics with "spinlock_release ... core_owner_id == lock->owner"
// (firmware 2026.05.02.04 fleet test). The user does not currently use
// ArduinoOTA — fleet firmware is flashed via USB, and the manifest-pull
// OTA update (ota.h/.cpp) pulls binaries via Update.h on a manifest GET
// (no passive listener).
#define ENABLE_ARDUINO_OTA  0

// Home WiFi SSID. Boats prefer this network when in range. OTA pull
// is gated to this SSID — see performOTAUpdate.
#define HOME_WIFI_SSID "Home-IOT"

#define GPS_BAUD      460800  // LG290P configured rate
#define SERIAL_BAUD   115200
#define SCREEN_WIDTH  320     // E1: 3.5" ST7796 portrait width
#define SCREEN_HEIGHT 480     // E1: 3.5" ST7796 portrait height
#define BNO085_ADDR   0x4B    // GY-BNO08X breakout (ADO pin high)
#define DPS310_ADDR   0x77    // Pressure/temperature sensor
#define GPS_FIX_TIMEOUT_MS  300000
#define DISPLAY_UPDATE_MS   500   // TFT can handle faster updates (no I2C contention)
#define FLUSH_INTERVAL_MS   10000
#define IMU_INTERVAL_MS     100    // 10 Hz BNO085 reports. Baked-in fleet default.
#define PRES_INTERVAL_MS    10000  // 0.1 Hz (every 10 sec - weather trends only)

// Wind sensor (Calypso Mini BLE)
#define ENABLE_WIND         true
#define WIND_SCAN_TIMEOUT_MS    10000
#define WIND_RECONNECT_MS       30000
#define WIND_INTERVAL_MS        1000   // Log at 1Hz

// Color scheme for sailing dashboard - WHITE background, BLACK numbers
#define COLOR_BG        TFT_WHITE
#define COLOR_TEXT      TFT_BLACK
#define COLOR_VALUE     TFT_CYAN
#define COLOR_LABEL     TFT_DARKGREY
#define COLOR_GOOD      TFT_GREEN
#define COLOR_WARN      TFT_YELLOW
#define COLOR_ERROR     TFT_RED
#define COLOR_DIVIDER   0x4208  // Dark gray

#define MAX_WIFI_NETWORKS 5

struct WiFiNetwork {
  char ssid[64];
  char pass[64];
};

struct Config {
  WiFiNetwork wifi[MAX_WIFI_NETWORKS];
  int wifi_count = 0;
  char upload_url[256] = "https://p9s9eia0t6.execute-api.us-east-1.amazonaws.com/prod/upload";  // Legacy, not used
  char s3_bucket[128] = "sailframes-fleet-data-prod";
  char s3_region[32] = "us-east-1";
  char boat_id[16] = "UNCFG";  // Non-colliding sentinel. If config.txt is missing/blank (SD reads OK
                               // but no boat_id), the device joins the mesh as "UNCFG" rather than
                               // impersonating a real boat. A default of a real ID ("E1") would put a
                               // duplicate FNV-1a sender_id on the mesh, corrupting peers/OCS/registry.
                               // (SD-unreadable is handled separately by the boot-time SD fault gate.)
  char wind_mac[20] = "";  // Calypso Mini MAC (loaded from /wind_mac.txt if present)
  bool wind_enabled = false;  // Auto-enabled if /wind_mac.txt exists on SD
  int wind_offset = 0;  // Heading offset in degrees (added to raw AWA for sensor mounting correction)
  // Recording thresholds
  float start_speed_knots = 1.5;
  float stop_speed_knots = 0.5;
  int start_delay_sec = 10;
  int stop_delay_sec = 180;

  // v2.0.0 foundation (SF_FIRMWARE_V2_SPEC.md Stage 1)
  char hardware_platform[8] = "e1";       // always "e1" in this repo
  char unit_role[24]        = "racing_boat";
  int  config_version       = 0;          // bumped by cloud config sync (Stage 3)
  // RTK Phase-2 (docs/RTK_PHASE2_DESIGN.md). SD-config ONLY — deliberately NOT
  // cloud-allow-listed: flipping it reconfigures the GNSS (base/rover RTK) and
  // is a physical bring-up act, not a remote push. Default off ⇒ byte-identical
  // to pre-RTK behavior, so an OTA that ships this code changes nothing until set.
  bool rtk_enabled          = false;
};

extern Config config;

// Loads/parses /config.txt on SD into `config` (WiFi creds, boat_id, wind
// sensor, role, recording thresholds, RTK enable).
void loadConfig();

#endif  // SAILFRAMES_CONFIG_H
