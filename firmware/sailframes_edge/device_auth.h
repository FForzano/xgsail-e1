// Device identity, claim flow, and the shared authenticated-HTTP helper
// for XGSail's device-integration protocol (xgsail's docs/device-protocol.md).
// See docs/firmware-architecture.md for the wire-level contract this
// implements: claim/confirm, then `Authorization: DeviceKey <key>` on
// every call after.
#ifndef SAILFRAMES_DEVICE_AUTH_H
#define SAILFRAMES_DEVICE_AUTH_H

#include <Arduino.h>

// Stable device identity: the ESP32's WiFi MAC address, formatted
// "AA:BB:CC:DD:EE:FF". Computed once, cached — safe to call before WiFi
// is connected (MAC is readable as soon as the radio is initialized).
const char* externalId();

// True once /device.txt holds a usable device_api_key (i.e. the claim
// flow in claimDevice() — or a BLE-relayed claim via persistDeviceApiKey()
// — has succeeded at some point). device_id is display metadata only,
// never required for this to be true: the BLE `provisioning`
// characteristic (docs/device-protocol.md §8.3) delivers only the key,
// since the phone app that ran the claim call already knows device_id.
bool isClaimed();
// "" if not claimed.
const char* deviceApiKey();
// "" if unknown — e.g. claimed via BLE relay, which doesn't tell the
// device its own device_id.
const char* deviceId();

// Runs the claim flow: POST /api/devices/claim/confirm with
// {external_id, claim_code}. On success, persists device_id +
// device_api_key to /device.txt and returns true. Per protocol: 400
// (bad external_id/claim_code), 404 (code not found), and 409 (expired
// or external_id already claimed) are none of them retryable — the user
// must supply a fresh code. 429 means back off and retry. This function
// does not itself retry; callers decide whether/when to call again.
bool claimDevice(const char* claimCode);

// Persists a device_api_key received via a channel that doesn't also
// supply device_id — namely the BLE `provisioning` characteristic (§8.3):
// the phone app calls claim/confirm itself and already holds device_id,
// so it only relays the key to the device. Whatever device_id is already
// on file (possibly none) is left as-is.
bool persistDeviceApiKey(const char* apiKey);

// Shared HTTP+JSON call used by every device-protocol endpoint. `path`
// is relative to config.api_base_url (e.g. "/api/devices/me/health").
// When `authenticated` is true, adds "Authorization: DeviceKey <key>"
// (fails fast, returns -1, if not yet claimed). `jsonBody` may be empty
// for a bodyless call. On return, `responseBody` holds the raw response
// text (caller parses with ArduinoJson for the fields it needs).
// Returns the HTTP status code, or a negative value if the request
// itself failed (no connection, api_base_url unset, etc).
int apiRequest(const char* method, const char* path, const String& jsonBody,
               String& responseBody, bool authenticated);

#endif  // SAILFRAMES_DEVICE_AUTH_H
