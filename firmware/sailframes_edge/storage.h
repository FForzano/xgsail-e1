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
void startLogging();
void logNav();
void logIMU();

// Recursively lists an SD directory to the current console (serial/telnet).
void listDirOutput(const char* dirname, int depth, bool toTelnet);

// Converts esp_reset_reason_t to a short label for /boot.log readability.
const char* resetReasonStr(esp_reset_reason_t r);
// Appends one line to /boot.log, taking sdMutex with a short timeout so a
// busy SD path (logging, upload) can't stall the caller.
void appendBootLog(const char* line);

#endif  // SAILFRAMES_STORAGE_H
