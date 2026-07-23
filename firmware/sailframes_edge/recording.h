// GPS speed-triggered recording state machine: auto-starts a session once
// boat speed sustains above a threshold; stops only on a clean operator
// action (hardware power-off or the `stoprec` command).
#ifndef SAILFRAMES_RECORDING_H
#define SAILFRAMES_RECORDING_H

#include <Arduino.h>

enum RecordState { REC_IDLE, REC_ARMED, REC_RECORDING, REC_STOPPING };

extern RecordState recState;
extern unsigned long armStartTime;      // when speed first exceeded start threshold
extern unsigned long stopStartTime;     // when speed first dropped below stop threshold
extern int sessionCount;                // increments each recording session

// Recording thresholds (configurable via config.txt)
extern float startSpeedKnots;         // Start recording above this speed
extern float stopSpeedKnots;          // Stop recording below this speed
extern unsigned long startDelayMs;    // sustained duration before start
extern unsigned long stopDelayMs;     // sustained duration before stop

// Advances the state machine off the current GPS speed; starts logging
// (via storage.h) once the sustained-speed threshold is met.
void updateRecordingState();
const char* getRecStateStr();

// Refreshes startSpeedKnots/stopSpeedKnots/startDelayMs/stopDelayMs from
// config.start_speed_knots/etc. Called once at boot (sailframes_edge.ino's
// setup()) and again by ble_relay.cpp's device_config write handler so a
// live threshold change takes effect without a reboot.
void applyRecordingThresholds();

#endif  // SAILFRAMES_RECORDING_H
