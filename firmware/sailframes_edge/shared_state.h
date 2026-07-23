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

#endif  // SAILFRAMES_SHARED_STATE_H
