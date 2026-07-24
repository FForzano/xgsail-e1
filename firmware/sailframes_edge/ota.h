// OTA firmware update: manifest fetch, version compare, download/verify/
// apply, and the state surfaced over BLE `status` for progress/result
// polling. See docs/ota.md. Runs on Core 0, called from upload.cpp's
// upload task (which owns the WiFi connect/teardown window this reuses).
#ifndef SAILFRAMES_OTA_H
#define SAILFRAMES_OTA_H

#include <Arduino.h>

// Automatic OTA firmware update check. No-op unless config.ota_auto_update
// is set, and never while recording. On a strictly newer manifest version it
// downloads, verifies, and reboots (does not return on success).
void checkForFirmwareUpdate();

// Runs one OTA cycle: manifest -> version compare -> download+apply. Assumes
// WiFi is already up. `manual` bypasses the ota_auto_update opt-in (the BLE
// control `ota-update` trigger, serviced by upload.cpp's task loop) but never
// the recording gate. Reboots on a successful update.
void runOtaCheck(bool manual);

// Manual OTA trigger, set by ble_relay.cpp's control `ota-update` command
// (any bonded phone, works even with ota_auto_update off). The upload task
// services it: brings WiFi up and calls runOtaCheck(true). Refused while
// recording.
extern volatile bool otaManualRequested;

// Coarse OTA state for the BLE `status` characteristic: "idle" | "checking" |
// "up_to_date" | "downloading" | "applying" | "suspended" | "error".
// g_otaProgress is a 0-100 percentage while downloading, else -1. otaMessage()
// is a short human-readable detail (e.g. the error reason), or "".
extern volatile const char* g_otaState;
extern volatile int         g_otaProgress;
const char* otaMessage();

#endif  // SAILFRAMES_OTA_H
