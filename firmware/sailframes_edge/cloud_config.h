// Cloud config sync: fetches /config/<boat_id>/latest.json (mirrors the
// OTA manifest pattern), verifies SHA256, and merges allow-listed
// key=value overrides into /sf/config.txt. One-shot per boot, gated so a
// push can't disturb an armed race start or an in-progress recording.
#ifndef SAILFRAMES_CLOUD_CONFIG_H
#define SAILFRAMES_CLOUD_CONFIG_H

#include <Arduino.h>

// One-shot-per-boot guard + last-known cloud version, surfaced by the
// `configver`/`status` console commands.
extern bool g_configSyncCheckedThisBoot;
extern int  g_cloud_config_version;   // -1 = unknown / not fetched

// Set by performConfigSync() after a successful config rewrite; loop()
// restarts the device once this fires (lets the upload task unwind first).
extern bool     g_configRebootPending;
extern uint32_t g_configRebootAtMs;

// Fetches the manifest, verifies + applies a newer config body if one
// exists. Reboots (after a short delay) to pick up the new config.txt.
bool performConfigSync();

#endif  // SAILFRAMES_CLOUD_CONFIG_H
