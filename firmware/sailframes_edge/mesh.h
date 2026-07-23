// SailFrames Firmware v2.0.0 Stage 2 — ESP-NOW peer-mesh wire types.
// See SF_FIRMWARE_V2_SPEC.md "ESP-NOW peer mesh" for the contract.
//
// Same .h-not-.ino discipline as v2_types.h: the Arduino preprocessor
// auto-generates forward declarations that reference these structs
// before the .ino-level definitions reach the compiler.

#ifndef SAILFRAMES_MESH_H
#define SAILFRAMES_MESH_H

#include <stdint.h>
#include <string.h>
#include "v2_types.h"  // RadioMode

#define MESH_MAGIC_0  0x53  // 'S'
#define MESH_MAGIC_1  0x46  // 'F'
#define MESH_VERSION  1

enum MeshMsgType {
    MSG_BOAT_STATE       = 0x01,
    MSG_LINE_ENDPOINT    = 0x02,  // not yet implemented
    MSG_RACE_ARMED       = 0x10,  // not yet implemented
    MSG_START_LOCKED     = 0x11,  // not yet implemented
    MSG_GENERAL_RECALL   = 0x12,  // not yet implemented
    MSG_INDIVIDUAL_RECALL= 0x13,  // not yet implemented
    MSG_ABANDON          = 0x14,  // not yet implemented
    MSG_SHORTEN_COURSE   = 0x15,  // not yet implemented
    MSG_ACK              = 0x20,  // not yet implemented
    MSG_RTCM_FRAG        = 0x30,  // RTK Phase-2: fragmented RTCM3 from RC base
};

struct __attribute__((packed)) MeshHeader {
    uint8_t  magic[2];      // 'S','F' (0x53, 0x46)
    uint8_t  version;       // MESH_VERSION
    uint8_t  msg_type;      // MeshMsgType
    uint16_t seq;           // monotonic per sender (rolls 16-bit)
    uint8_t  ttl;           // hops remaining; 0 = no rebroadcast
    uint8_t  reserved;
    uint32_t sender_id;     // FNV1a hash of boat_id string, stable across boots
    uint32_t gps_time_ms;   // GPS time-of-day in ms (rolls daily; 0 if no fix)
};
static_assert(sizeof(MeshHeader) == 16, "MeshHeader must be 16 bytes");

struct __attribute__((packed)) BoatStatePayload {
    int32_t  lat_e7;        // latitude * 1e7  (signed, ~1 cm/lsb at equator)
    int32_t  lon_e7;        // longitude * 1e7
    int16_t  sog_cm_s;      // SOG in cm/s
    int16_t  cog_deg10;     // COG in 0.1°
    int16_t  heading_deg10; // IMU heading in 0.1°
    int8_t   heel_deg;      // heel in degrees, signed
    uint8_t  fix_quality;   // NMEA fix indicator
    uint8_t  sat_count;
    uint8_t  unit_role;     // sender's UnitRole enum value
    // The former reserved[2] pad, now carrying per-boat quality so the RC
    // pre-race panel can show it. SAME 20-byte wire format (no size change, no
    // rollout compat surface): old firmware sent these as 0 and old receivers
    // ignored them. 0 == "no data" (old FW / no fix), NOT "perfect" — render as
    // "--". Named fields (not an array) to avoid the gotcha-#25 index footgun.
    uint8_t  hdop_x10;      // HDOP * 10, saturated 0..255; 0 = no data
    uint8_t  hacc_mm;       // GST horizontal 1-sigma in mm, saturated 0..255; 0 = no data
};
static_assert(sizeof(BoatStatePayload) == 20, "BoatStatePayload must be 20 bytes");

// Stage 4.5 — race-armed broadcast. Sent by any boat acting as
// race ops (typically the one tethered to the laptop / RC unit).
// Every receiver translates the relative start time into its own
// millis() clock and arms its boat-local OCS state machine.
//
// We use `seconds_until_start` (signed int32, relative to the
// receiver's millis() at packet arrival) rather than GPS time-of-day
// because not every boat has a GPS fix at the dock. Network latency
// adds ~few-ms drift — acceptable for second-level race timing.
struct __attribute__((packed)) RaceArmedPayload {
    int32_t  pin_lat_e7;
    int32_t  pin_lon_e7;
    int32_t  rc_lat_e7;
    int32_t  rc_lon_e7;
    int32_t  seconds_until_start;   // signed; negative = race already underway
    uint8_t  race_num;              // informational, 1-99
    uint8_t  sequence_mode;         // ISAF Rule 26 = 30, Short = 27, etc. (Stage 7)
    uint8_t  reserved[2];
};
static_assert(sizeof(RaceArmedPayload) == 24, "RaceArmedPayload must be 24 bytes");

// Stage 5 — RC unit broadcasts when it detects a boat over the
// start line at T+0. Target boat (matched by sender_id hash)
// receives this and overrides its local OCS state to over=true,
// regardless of what its own computation said. The RC unit's
// call is authoritative because it knows the canonical line
// endpoints and applies a unified bow_offset_m per class.
//
// Distance is the RC's measurement at the moment of recall —
// useful for post-race auditing if the boat's local state
// disagrees with the RC's.
struct __attribute__((packed)) IndividualRecallPayload {
    uint32_t target_sender_id;   // FNV1a hash of boat being recalled
    int16_t  distance_cm;        // signed; negative = course side (OCS)
    uint8_t  reserved[2];
};
static_assert(sizeof(IndividualRecallPayload) == 8, "IndividualRecallPayload must be 8 bytes");

// RTK Phase-2 — one fragment of an RC base's RTCM3 frame, relayed to rovers.
// A complete RTCM3 frame (≤1029 B) is split into up to 5 of these. The 16 B
// MeshHeader + 4 B meta + ≤230 B data = 250 B = the ESP-NOW single-packet cap.
// Only frag_len data bytes go on the wire (send = 16 + 4 + frag_len). The
// data[] is fixed-size only so static_assert can pin the max; never send sizeof.
// See docs/RTK_PHASE2_DESIGN.md §2 and rtk_relay.h. (gotcha #25)
#define RTCM_FRAG_MAX 230
struct __attribute__((packed)) RtcmFragPayload {
    uint8_t msg_id;       // rolls per complete RTCM frame at the RC
    uint8_t frag_index;   // 0 .. frag_count-1
    uint8_t frag_count;   // total fragments for this frame (max 5)
    uint8_t frag_len;     // RTCM bytes in this fragment (<= RTCM_FRAG_MAX)
    uint8_t data[RTCM_FRAG_MAX];
};
static_assert(sizeof(RtcmFragPayload) == 4 + RTCM_FRAG_MAX, "RtcmFragPayload size (gotcha #25)");
static_assert(sizeof(MeshHeader) + 4 + RTCM_FRAG_MAX == 250, "RTCM frag packet must fit ESP-NOW 250 B cap");

// FNV-1a 32-bit hash of a NUL-terminated string. Stable across boots
// and across the fleet — every E1/E2/.../B1 hashes its boat_id the
// same way, so receivers can identify senders without a peer registry.
static inline uint32_t boatIdHash(const char* s) {
    uint32_t h = 2166136261u;
    while (s && *s) {
        h ^= (uint8_t)(*s++);
        h *= 16777619u;
    }
    return h;
}

// ============================================================
// v2.0.0 Stage 2 — ESP-NOW peer mesh state (defined in mesh.cpp)
// ============================================================
// MVP: always-on after WiFi PHY init, gated off only during HTTP uploads
// via wifiBusy. Radio-mode integration (init only in MODE_RACING/_RC_ACTIVE
// per spec) deferred to a later stage once mode transitions actually fire.
#define MESH_CHANNEL                 1     // ESP-NOW broadcast channel (configurable in spec; hardcoded for MVP)
#define MESH_BROADCAST_INTERVAL_MS   500   // 2 Hz boat-state broadcast
#define MESH_PEER_MAX                32    // 25 boats + RC + marks + spares + headroom
#define MESH_PEER_EXPIRY_MS          30000 // drop peers we haven't heard from in 30 s

struct MeshPeerState {
    uint32_t sender_id;
    int32_t  last_lat_e7;
    int32_t  last_lon_e7;
    int16_t  last_sog_cm_s;
    int16_t  last_cog_deg10;
    int16_t  last_heading_deg10;   // Stage 5: from BoatStatePayload.heading_deg10
    int8_t   last_heel_deg;
    uint8_t  unit_role;
    uint8_t  fix_quality;
    uint8_t  sat_count;
    uint8_t  hdop_x10;             // RTK Phase-2: HDOP*10 from peer (0 = no data)
    uint8_t  hacc_mm;             // RTK Phase-2: GST horiz 1-sigma mm from peer (0 = no data)
    uint32_t last_seen_ms;
    int8_t   last_rssi;            // ESP-NOW RX RSSI (dBm) of last packet — link-budget/range debug (0 = no data)
    uint32_t msg_count;
    uint16_t last_seq;
    // Stage 5 — RC-side OCS computation per peer.
    float    rc_distance_m;        // signed perpendicular distance from RC view
    bool     rc_ocs_called;        // RC has broadcast MSG_INDIVIDUAL_RECALL for this boat
    uint32_t rc_ocs_called_at_ms;
};

extern MeshPeerState     g_mesh_peers[MESH_PEER_MAX];
extern volatile int      g_mesh_peer_count;
extern volatile uint16_t g_mesh_seq;
extern volatile bool     g_mesh_enabled;
extern volatile uint32_t g_mesh_rx_count;
extern volatile uint32_t g_mesh_tx_count;
extern volatile uint32_t g_mesh_tx_fail_count;
extern volatile uint32_t g_mesh_rx_dropped_bad_magic;
extern unsigned long     g_mesh_last_broadcast;
extern uint32_t          g_mesh_local_sender_id;
extern const uint8_t     MESH_BROADCAST_ADDR[6];

// v2.0.0 radio mode transition (SF_FIRMWARE_V2_SPEC.md Stage 1). Logs +
// records the transition; callers invoke this to record intent.
void radioModeTransition(RadioMode target, const char* reason);

// Initializes ESP-NOW + registers the receive callback. Idempotent.
void meshInit();
// Called from the main loop: (re)inits if needed, broadcasts boat-state at
// MESH_BROADCAST_INTERVAL_MS, and expires stale peers.
void meshTick();
// Broadcasts a MSG_RACE_ARMED to the fleet (telnet `race arm`).
bool meshBroadcastRaceArmed(double pin_lat, double pin_lon,
                            double rc_lat, double rc_lon,
                            int seconds_until_start,
                            uint8_t race_num, uint8_t sequence_mode);
// Stage 5 — RC broadcasts MSG_INDIVIDUAL_RECALL for a boat it judged OCS.
bool meshBroadcastIndividualRecall(uint32_t target_id, int16_t distance_cm);
// Reverse FNV-1a lookup: known fleet boat_id whose hash matches `id`, or "??".
const char* boatNameForSender(uint32_t id);

#endif
