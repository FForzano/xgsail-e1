// XGSail device-protocol BLE GATT relay (docs/device-protocol.md §8) — a
// NimBLE peripheral (GATT server) letting the owner's phone app relay the
// claim + session-upload calls documented there over Bluetooth, for a
// device with no WiFi at all or whose WiFi isn't reachable right now.
// Runs concurrently with the existing NimBLE central role used for the
// Calypso wind sensor (wind_sensor.cpp).
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

#endif  // SAILFRAMES_BLE_RELAY_H
