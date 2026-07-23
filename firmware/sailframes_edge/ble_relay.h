// XGSail device-protocol BLE GATT relay (docs/device-protocol.md §8) — a
// NimBLE peripheral (GATT server) letting the owner's phone app relay the
// claim + session-upload calls documented there over Bluetooth, for a
// device with no WiFi at all or whose WiFi isn't reachable right now.
// Runs concurrently with the existing NimBLE central role used for the
// Calypso wind sensor (wind_sensor.cpp).
//
// Also carries E1-specific extensions beyond xgsail's protocol, on the
// same service: a `device_config` characteristic for live remote
// configuration (WiFi creds, boat_id, thresholds, ...), two extra
// `control` commands for IMU calibration, and a read-only `status`
// characteristic for live runtime state (battery, WiFi, GPS, sensors,
// recording). Not part of docs/device-protocol.md (that's the ingestion
// contract, owned by xgsail) — see docs/ble-config.md for the full spec.
#ifndef SAILFRAMES_BLE_RELAY_H
#define SAILFRAMES_BLE_RELAY_H

#include <Arduino.h>

// Starts the GATT server + advertising. Call once from setup(),
// unconditionally — this is a first-class upload path, not a WiFi-down
// fallback, so it must come up whether or not WiFi is configured at all.
void bleRelayInit();

// Services any in-flight session_data transfer (sends a bounded number of
// chunk notifications) — call every loop() iteration. Non-blocking.
void bleRelayTick();

// Opens the pairing window: for BLE_BOND_WINDOW_MS (config.h), a
// not-yet-bonded phone is allowed to write `provisioning`/`device_config`
// (the two characteristics that can carry secrets — device_api_key, WiFi
// password). Outside the window, only a connection already recognized as
// bonded from an earlier pairing may write them. Called by button.cpp's
// long-press handler — see docs/ble-config.md.
void bleOpenBondWindow();

#endif  // SAILFRAMES_BLE_RELAY_H
