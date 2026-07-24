// WiFi connect + XGSail device-protocol upload pipeline, running as a
// dedicated FreeRTOS task pinned to Core 0 (sensor reads + logging stay on
// Core 1). Also owns the dual-core hang-diagnostics task and the
// once-per-boot health-snapshot push.
#ifndef SAILFRAMES_UPLOAD_H
#define SAILFRAMES_UPLOAD_H

#include <Arduino.h>
#include <WiFi.h>
#include <SD.h>

extern bool uploading;
extern int  pendingUploads;   // sessions with files still to upload
extern bool wifiConnected;
extern char connectedSSID[64];
extern int  uploadCount, uploadTotal;
extern int  uploadSuccess, uploadFailed;
extern char uploadCurrentFile[32];  // short name of file being uploaded

// Where Core 0's upload task currently is. Complements g_loopSection so
// the [DIAG] heartbeat names the stuck section on either core when a task
// wdt fires.
extern volatile const char* g_uploadSection;

extern unsigned long lastUploadCheck;
extern const unsigned long UPLOAD_CHECK_INTERVAL_MS;
extern int uploadRetryCount;
extern const int MAX_UPLOAD_RETRIES;
extern unsigned long lastUploadAttempt;
extern const unsigned long UPLOAD_RETRY_DELAY_MS;

// boot.log session record is written once per power cycle, after the first
// valid GPS time + date arrives; the diagnostics task then appends an
// "alive" heartbeat every 5 min.
extern bool g_bootSessionLogged;
extern unsigned long g_lastAliveLog;

// Marks/checks a file's `.uploaded` sentinel so a session's files upload
// exactly once.
bool isUploaded(const char* filepath);
int  deleteUploadedFiles(const char* dirname);
void markUploaded(const char* filepath);
// Deletes filepath + its .uploaded marker if config.auto_cleanup_uploads
// is set — called right after markUploaded() by both upload paths (this
// WiFi one and ble_relay.cpp's ack-uploaded). No-op otherwise. Caller
// holds sdMutex.
void cleanupIfAutoDelete(const char* filepath);

// Uploads one file via the XGSail device protocol (docs/device-protocol.md
// §4.1): POST /api/devices/me/session-uploads, then PUT the raw bytes to
// the returned presigned upload_url. No-ops (returns false) if the device
// isn't claimed yet.
bool uploadFile(const char* filepath);
int  countFilesToUpload(const char* dirname);
// Cheap DNS+TCP reachability check against config.api_base_url — run
// before a batch of uploads so a dead backend fails fast instead of
// timing out per-file.
bool testApiConnectivity();
void uploadDirectory(const char* dirname);
// Connects to the strongest configured WiFi network. Sets wifiConnected +
// connectedSSID on success.
bool connectWiFi();

// Health snapshot (docs/device-protocol.md §4.4), pushed once per boot.
bool uploadHealthSnapshot();
void countPendingUploads();

// Automatic OTA firmware update check (docs/ota.md). Called from the upload
// task's WiFi window, right after the health snapshot. No-op unless
// config.ota_auto_update is set, and never runs while recording. On a newer
// build it downloads + verifies + reboots (see upload.cpp).
void checkForFirmwareUpdate();

// Manual OTA trigger, set by ble_relay.cpp's control `ota-update` command
// (any bonded phone, works even with ota_auto_update off). The upload task
// services it: brings WiFi up and runs a check/apply. Refused while recording.
extern volatile bool otaManualRequested;

// Coarse OTA state for the BLE `status` characteristic: "idle" | "checking" |
// "up_to_date" | "downloading" | "applying" | "suspended" | "error".
// g_otaProgress is a 0-100 percentage while downloading, else -1. otaMessage()
// is a short human-readable detail (e.g. the error reason), or "".
extern volatile const char* g_otaState;
extern volatile int         g_otaProgress;
const char* otaMessage();

// Core-0 tasks (created from setup()).
void diagnosticsTask(void* param);
void uploadTaskFunc(void* param);

#endif  // SAILFRAMES_UPLOAD_H
