// OCS (On Course Side / over-the-line) detection: boat-local bow-position
// projection onto the start line, RC-side fleet aggregation across the
// ESP-NOW mesh, and the live serial fleet dashboard. See
// SF_FIRMWARE_V2_SPEC.md "OCS state machine" / "RC unit fleet OCS
// aggregation" (ported to docs/firmware-architecture.md in this repo).
#ifndef SAILFRAMES_OCS_H
#define SAILFRAMES_OCS_H

#include <Arduino.h>

#define OCS_BOW_OFFSET_M           2.4f    // Sonar 23 default (used when class registry has no entry)
#define OCS_THRESHOLD_M            0.5f    // must be >50cm over to call OCS
#define OCS_CLEAR_THRESHOLD_M      0.5f    // must be >50cm back to clear
#define OCS_CLEAR_DWELL_MS         2000    // sustained-clear time before un-latching
#define OCS_TICK_INTERVAL_MS       100     // 10 Hz — matches GPS fix rate
#define RC_TICK_INTERVAL_MS        200     // 5 Hz — receivers broadcast at 2 Hz so this is fast enough

// Per-class bow_offset registry (RC-only). Loaded from /sf/classes.csv at
// boot when role=rc_signal. CSV format (header optional):
//   boat_id,class,bow_offset_m
//   E1,Sonar23,2.4
// FNV-1a hash of boat_id is the key — same hash used as ESP-NOW sender_id.
#define CLASS_REGISTRY_MAX  32

struct ClassRegistryEntry {
    uint32_t sender_id;        // FNV-1a hash of boat_id, used to match peer
    char     boat_id[16];      // raw boat_id for telnet display
    char     class_name[16];   // e.g. "Sonar23", "J80"
    float    bow_offset_m;
};

extern ClassRegistryEntry g_class_registry[CLASS_REGISTRY_MAX];
extern int g_class_registry_count;

struct OCSState {
  bool     armed;
  uint32_t start_time_ms;        // millis() value at which T+0 fires
  double   pin_lat, pin_lon;
  double   rc_lat,  rc_lon;
  // Live:
  bool     over_line;
  bool     was_over_at_start;
  float    distance_to_line_m;   // signed, positive = pre-start side
  float    closure_rate_m_s;     // negative = approaching line from pre-start
  uint32_t over_since_ms;
  uint32_t cleared_at_ms;
  // Internal for closure-rate calc:
  float    _last_d;
  uint32_t _last_t_ms;
};

extern OCSState g_ocs;
extern unsigned long g_ocs_last_tick_ms;
extern unsigned long g_rc_last_tick_ms;
extern bool g_fleetWatch;
extern unsigned long g_fleetWatchLast;  // millis() of the last fleetwatch repaint (ocs.cpp's ~2 Hz throttle)

void ocsDisarm();
// Arms boat-local OCS with the start line (PIN->RC) and start time.
void ocsArm(double pin_lat, double pin_lon,
            double rc_lat,  double rc_lon,
            uint32_t start_time_ms);
// Per-tick boat-local over-line detection (bow position vs. start line).
void ocsTick();
// Called when this boat is the target of a MSG_INDIVIDUAL_RECALL: forces
// over_line=true regardless of local computation (RC is authoritative).
void ocsForceOver(int16_t rc_distance_cm);

// RC-only: computes OCS for every mesh peer and broadcasts
// MSG_INDIVIDUAL_RECALL for any peer that crosses the threshold post-T+0.
void rcComputeFleetOCS();

// Non-blocking live RC fleet dashboard over serial (toggle via `fleetwatch`).
void fleetWatchTick();

// Loads /sf/classes.csv into g_class_registry (RC-only, no-op otherwise).
void loadClassRegistry();
// Per-class bow_offset_m lookup; falls back to OCS_BOW_OFFSET_M.
float bowOffsetForSender(uint32_t sender_id);

#endif  // SAILFRAMES_OCS_H
