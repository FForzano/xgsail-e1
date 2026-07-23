// Cross-cutting runtime bookkeeping that doesn't belong to one sensor/
// subsystem: the dual-core hang-watchdog breadcrumbs, the WiFi-busy gate
// that every radio-sharing module (mesh, wind BLE, telnet) checks before
// touching the shared ESP32 radio, and the SD-card mutex serializing
// logging (Core 1) against upload (Core 0).
#ifndef SAILFRAMES_SHARED_STATE_H
#define SAILFRAMES_SHARED_STATE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// SD-card mutex for dual-core safety (Core 1 logging vs. Core 0 upload).
extern SemaphoreHandle_t sdMutex;
extern TaskHandle_t      uploadTaskHandle;
extern TaskHandle_t      diagTaskHandle;

// Where Core 1's main loop currently is. Set by the loop, read by the diag
// task. When Core 1 hangs, the last value here pinpoints the stuck section.
extern volatile const char* g_loopSection;
extern volatile uint32_t    g_loopIter;

// True while Core 0 is mid-WiFi-work: WiFi scan, connect, upload cycle.
// Core 1 must NOT touch the WiFi/LWIP stack (handleTelnet, telnetServer,
// WiFi.* APIs other than fast-status reads) during this window — under
// sustained Core 0 traffic, especially with weak signal, LWIP mutex
// contention blocks Core 1 inside otherwise-cheap calls and hangs the
// device.
extern volatile bool wifiBusy;

// Flag to skip display updates during SD writes.
extern volatile bool sdWriting;

// Set by recording.cpp when a session stops; consumed by upload.cpp to
// kick off an upload cycle. Cleared by upload.cpp once it picks it up.
extern bool triggerUpload;
// Set by upload.cpp once an upload cycle is fully done; consumed by the
// main loop (Core 1) to actually disconnect WiFi — teardown must happen
// on Core 1 since it owns the telnet/WiFi handlers upload.cpp must not
// race against.
extern bool wifiTeardownRequested;

// Core 1 must call back into the main loop at least this often; the diag
// task force-restarts the device if g_loopIter hasn't advanced for this
// long (turns a permanent hang into a recoverable reboot).
extern const unsigned long LOOP_HANG_MS;

#endif  // SAILFRAMES_SHARED_STATE_H
