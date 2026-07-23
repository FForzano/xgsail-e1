// Button-triggered recording: a short button press (button.cpp), the
// console's `rec`/`stoprec` commands, or the BLE relay's control
// characteristic (ble_relay.cpp) all start/stop through the same two
// functions below — no GPS-speed auto-start/stop.
#ifndef SAILFRAMES_RECORDING_H
#define SAILFRAMES_RECORDING_H

#include <Arduino.h>

enum RecordState { REC_IDLE, REC_RECORDING };

extern RecordState recState;
extern int sessionCount;                // increments each recording session

// Retained only as the "boat is moving" heuristic upload.cpp uses to
// abort an in-progress upload cycle — see config.h's start_speed_knots.
extern float startSpeedKnots;

// Starts a new session (SD-mutex guarded) if not already recording.
// `boatId`/`activityId` are the optional per-session overrides an app can
// pick when it triggers the start over BLE (see storage.h's
// startLogging()) — both default to null ("use the device's normal
// boat, a fresh solo activity") for the button/console entry points.
// Returns true if a session was actually started.
bool startRecording(const char* boatId = nullptr, const char* activityId = nullptr);

// Flushes and closes the current session's files if recording. Returns
// true if a session was actually stopped.
bool stopRecording();

// Toggles start/stop off the current state — button.cpp's short-press
// handler calls this with no overrides (physical button, no app in the
// loop to supply them).
void toggleRecording();

const char* getRecStateStr();

// Refreshes startSpeedKnots from config.start_speed_knots. Called once at
// boot (sailframes_edge.ino's setup()) and again by ble_relay.cpp's
// device_config write handler so a live threshold change takes effect
// without a reboot.
void applyRecordingThresholds();

#endif  // SAILFRAMES_RECORDING_H
