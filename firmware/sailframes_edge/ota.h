// Firmware updates: the (disabled-by-default) ArduinoOTA listener, the
// telnet remote console transport, and the manifest-pull HTTP OTA update
// used in practice (CI publishes firmware/{boat_id}/latest.json + the
// .bin; this fetches, verifies SHA256, and flashes via Update.h).
#ifndef SAILFRAMES_OTA_H
#define SAILFRAMES_OTA_H

#include <Arduino.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

extern bool otaInProgress;

// Telnet server for remote console.
extern WiFiServer telnetServer;
extern WiFiClient telnetClient;
extern bool telnetEnabled;
extern bool telnetServerRunning;
extern String telnetBuffer;

// Hang-watchdog breadcrumbs for OTA specifically (diagnosticsTask forces a
// restart if an OTA cycle overruns these bounds).
extern volatile unsigned long g_otaDeadlineMs;   // 0 = no OTA in flight
extern volatile bool          g_otaCheckedThisBoot;
extern const unsigned long OTA_MAX_MS;    // hard ceiling per OTA cycle
extern const unsigned long OTA_STALL_MS;  // abort if no bytes received for this long
extern const unsigned long LOOP_HANG_MS;  // Core 1 must tick at least this often

// Registers the (disabled-by-default) ArduinoOTA callbacks.
void setupOTA();
void startTelnetServer();
// Services the telnet client: line-buffered input -> processCommand().
void handleTelnet();

// Manifest-pull OTA: fetches latest.json, verifies SHA256, flashes via
// Update.h. `manual` bypasses the once-per-boot auto-check guard.
bool performOTAUpdate(bool manual);
// Paints a one-time progress layout, then updates just the % + bar.
void drawOTAProgress(int percent, const char* targetVersion, const char* phase);
// Lowercase hex encoding of a raw digest (SHA256 verification, shared with
// cloud_config.cpp's config-sync integrity check).
String otaHexDigest(const uint8_t* digest, size_t len);

#endif  // SAILFRAMES_OTA_H
