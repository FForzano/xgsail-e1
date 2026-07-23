// SD-card session logging: file lifecycle (nav/imu/wind/pressure CSVs),
// session folder naming, and the /boot.log diagnostic trail.
#ifndef SAILFRAMES_STORAGE_H
#define SAILFRAMES_STORAGE_H

#include <Arduino.h>
#include <SD.h>
#include <esp_system.h>

extern bool sdOK;
extern bool logging;
extern File navFile, imuFile, windFile, presFile;

// millis() at which the current session's logging started; every log
// line's elapsed-time column is `millis() - logStart`.
extern unsigned long logStart;
// Running count of bytes written this session, for the status display.
extern unsigned long totalBytes;

// Reads/increments /sf/session.txt — used as the session-folder fallback
// name when no GPS date/time is available yet.
int getNextSessionNumber();
// Opens a new session's nav/imu/wind/pressure CSVs under /sf/<folder>/ and
// writes their headers. Sets `logging = true` and `logStart` on success.
// Every session becomes its own xgsail session server-side regardless
// (ingestion.find_or_create_session); the two optional overrides below
// just say more about it, both picked from the app when starting the
// recording (recording.h) — omit either/both for the device's defaults:
//   sessionBoatId     — the XGSail boat (a backend UUID, unrelated to
//                        config.h's `config.boat_id` mesh-identity label
//                        — see recording.h) to file this session under
//                        instead of the device's own boat.
//   sessionActivityId — an existing XGSail activity (race, training
//                        session, ...) to attach this session to,
//                        instead of getting its own private "solo"
//                        activity — see docs/device-protocol.md §4.4's
//                        session-uploads activity_id.
// Both are written to the session folder's boat_id.txt/activity_id.txt
// markers so upload.cpp's uploadFile() can include them on the
// session-uploads POST — the server already defaults each to
// device.owner_boat_id / a fresh solo activity when the upload omits
// them, so nothing else needs to happen for the default case.
void startLogging(const char* boatIdOverride = nullptr, const char* activityIdOverride = nullptr);
void logNav();
void logIMU();

// Recursively lists an SD directory to the current console (serial/telnet).
void listDirOutput(const char* dirname, int depth, bool toTelnet);

// Derives a file's session's started_at (ISO 8601 UTC, required by
// xgsail's device-protocol session-uploads API) from its enclosing
// "/sf/YYYYMMDD_HHMMSS/" folder name — see startLogging()'s naming
// above. Falls back to the live GPS clock (only relevant for the
// "session_NNN" fallback folder name); returns "" if neither source is
// available. Shared by upload.cpp (WiFi path) and ble_relay.cpp (BLE
// relay manifest) — both need the same session timestamp for the same
// files.
String sessionStartedAtIso(const char* filepath);

// Reads back a file's session's boat_id.txt/activity_id.txt markers (see
// startLogging()) — "" if the session used the device's default (no
// marker written). Shared by upload.cpp (WiFi path's session-uploads
// POST) and ble_relay.cpp (BLE relay manifest).
String sessionBoatId(const char* filepath);
String sessionActivityId(const char* filepath);

// Converts esp_reset_reason_t to a short label for /boot.log readability.
const char* resetReasonStr(esp_reset_reason_t r);
// Appends one line to /boot.log, taking sdMutex with a short timeout so a
// busy SD path (logging, upload) can't stall the caller.
void appendBootLog(const char* line);

#endif  // SAILFRAMES_STORAGE_H
