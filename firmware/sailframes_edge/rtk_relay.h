// SailFrames RTK Phase-2 — ESP-NOW RTCM3 relay logic (chip-agnostic).
//
// Carries the RC base's RTCM3 corrections to the rover boats over the existing
// ESP-NOW mesh. This is the code Gate A proved (self-test + 2-board radio,
// 2026-06-05): byte-exact reassembly via CRC-24Q, two-level dedup, clean
// frame-drop under loss. See docs/RTK_PHASE2_DESIGN.md.
//
// Lives in a .h (same discipline as mesh.h / v2_types.h) so the Arduino
// preprocessor does not hoist auto-prototypes above these struct definitions.
//
// THREADING CONTRACT (critical):
//   - RtcmFramer       is touched ONLY in the loop task (readGPSBase()).
//   - RtcmReassembler  is touched ONLY in the ESP-NOW recv callback. Its
//     staleness check lives inside onPacket() — never call it from another
//     task. The ONLY object that crosses the callback↔loop boundary is the
//     StreamBuffer ring (in the .ino), written by the reassembler's onFrame.

#ifndef SAILFRAMES_RTK_RELAY_H
#define SAILFRAMES_RTK_RELAY_H

#include <stdint.h>
#include <string.h>
#include "freertos/stream_buffer.h"  // StreamBufferHandle_t: RTCM ring rover-side
#include "mesh.h"   // MeshHeader, MSG_RTCM_FRAG, RtcmFragPayload, RTCM_FRAG_MAX

#define RTK_MAX_RTCM_FRAME 1029   // 3 (hdr) + 1023 (max payload) + 3 (CRC)
#define RTK_MAX_FRAGS      5      // ceil(1029 / 230)

// CRC-24Q (RTCM/Qualcomm, poly 0x1864CFB, init 0). Used as the byte-exact
// integrity check: a reassembled frame whose CRC validates was relayed bit-perfect.
static inline uint32_t rtcmCrc24q(const uint8_t* d, int len) {
  uint32_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= (uint32_t)d[i] << 16;
    for (int b = 0; b < 8; b++) { crc <<= 1; if (crc & 0x1000000) crc ^= 0x1864CFB; }
  }
  return crc & 0xFFFFFF;
}

static inline bool rtcmFrameValid(const uint8_t* f, int len) {
  if (len < 6 || f[0] != 0xD3) return false;
  int pl = ((f[1] & 0x03) << 8) | f[2];
  if (3 + pl + 3 != len) return false;
  uint32_t want = ((uint32_t)f[3+pl] << 16) | ((uint32_t)f[3+pl+1] << 8) | f[3+pl+2];
  return rtcmCrc24q(f, 3 + pl) == want;
}

// RC side: byte-stream → complete CRC-valid RTCM frames (length-driven, NOT
// '$'-keyed — a 0x24 inside a binary payload must not trigger NMEA mode).
// feed() returns true if the byte was consumed as part of an RTCM frame; the
// caller routes a `false` byte to the NMEA parser. onFrame fires per CRC-valid
// frame. A mis-sync produces a bad CRC → no onFrame → framer resyncs.
struct RtcmFramer {
  uint8_t  st = 0;            // 0=sync 1=len1 2=len2 3=payload 4=crc
  uint16_t len = 0, got = 0;
  uint8_t  buf[RTK_MAX_RTCM_FRAME];
  void (*onFrame)(const uint8_t*, int) = nullptr;

  bool feed(uint8_t c) {
    switch (st) {
      case 0: if (c == 0xD3) { buf[0] = c; st = 1; return true; } return false;
      case 1: buf[1] = c; len = (uint16_t)(c & 0x03) << 8; st = 2; return true;
      case 2: buf[2] = c; len |= c; got = 0; st = (len == 0 || len > 1023) ? 0 : 3; return true;
      case 3: buf[3 + got] = c; if (++got >= len) { got = 0; st = 4; } return true;
      case 4: buf[3 + len + got] = c;
              if (++got >= 3) { st = 0; int t = 3 + len + 3;
                if (onFrame && rtcmFrameValid(buf, t)) onFrame(buf, t); }
              return true;
    }
    st = 0; return false;
  }
};

// Rover side: ESP-NOW MSG_RTCM_FRAG packets → complete RTCM frames.
// Two-level dedup: got_mask per frag_index (in-flight dups) + last_done_msg_id
// (2×-tx trailing dups — Gate A bug 2026-06-05; without it a 1-frag frame
// re-completes and double-writes the GNSS). Staleness check is INSIDE onPacket
// (single-context). onFrame fires per complete CRC-valid frame.
struct RtcmReassembler {
  bool     active = false;
  uint8_t  cur_msg_id = 0, frag_count = 0;
  uint32_t got_mask = 0;
  int      total_len = -1;
  uint8_t  buf[RTK_MAX_RTCM_FRAME + 16];
  unsigned long start_ms = 0;
  bool     have_last_done = false;
  uint8_t  last_done_msg_id = 0;
  // diagnostics (read-only from other contexts; eventual consistency is fine)
  unsigned long s_pkts=0, s_bad=0, s_dup=0, s_dropped=0, s_complete=0, s_crc_fail=0;
  void (*onFrame)(const uint8_t*, int) = nullptr;

  void onPacket(const uint8_t* pkt, int len) {
    s_pkts++;
    if (active && millis() - start_ms > 2000) { s_dropped++; active = false; }  // stale partial
    if (len < (int)(sizeof(MeshHeader) + 4)) { s_bad++; return; }
    const MeshHeader* h = (const MeshHeader*)pkt;
    if (h->magic[0] != MESH_MAGIC_0 || h->magic[1] != MESH_MAGIC_1 ||
        h->version != MESH_VERSION  || h->msg_type != MSG_RTCM_FRAG) { s_bad++; return; }
    const RtcmFragPayload* p = (const RtcmFragPayload*)(pkt + sizeof(MeshHeader));
    if (p->frag_count == 0 || p->frag_count > RTK_MAX_FRAGS ||
        p->frag_index >= p->frag_count || p->frag_len > RTCM_FRAG_MAX ||
        (int)(sizeof(MeshHeader) + 4 + p->frag_len) != len) { s_bad++; return; }
    // Bound the SUM, not just each field (gotcha #25): a malformed/spoofed packet
    // with max-but-individually-valid frag_index+frag_len (e.g. 4*230+230=1150)
    // would otherwise memcpy past buf[]. A real ≤1029 B frame never trips this.
    if (p->frag_index * RTCM_FRAG_MAX + p->frag_len > RTK_MAX_RTCM_FRAME) { s_bad++; return; }

    if (have_last_done && p->msg_id == last_done_msg_id) { s_dup++; return; }  // trailing dup
    if (!active || p->msg_id != cur_msg_id) {
      if (active) s_dropped++;
      active = true; cur_msg_id = p->msg_id; frag_count = p->frag_count;
      got_mask = 0; total_len = -1; start_ms = millis();
    }
    uint32_t bit = 1u << p->frag_index;
    if (got_mask & bit) { s_dup++; return; }   // in-flight dup
    got_mask |= bit;
    memcpy(buf + p->frag_index * RTCM_FRAG_MAX, p->data, p->frag_len);
    if (p->frag_index == p->frag_count - 1) total_len = p->frag_index * RTCM_FRAG_MAX + p->frag_len;

    uint32_t full = (1u << frag_count) - 1;
    if (got_mask == full && total_len > 0) {
      active = false; have_last_done = true; last_done_msg_id = cur_msg_id;
      s_complete++;
      if (!rtcmFrameValid(buf, total_len)) { s_crc_fail++; return; }
      if (onFrame) onFrame(buf, total_len);
    }
  }
};

// RTK Phase-2 relay state (docs/RTK_PHASE2_DESIGN.md). All inert unless
// config.rtk_enabled. The RC base (rc_signal) PRODUCES; everyone else
// CONSUMES. Defined in rtk_relay.cpp.
extern RtcmFramer           g_rtcmTx;      // RC base: Serial2 RTCM frames (loop ctx only)
extern RtcmReassembler      g_rtcmRx;      // rover: ESP-NOW frags (recv-cb ctx only)
extern StreamBufferHandle_t g_rtcmRing;    // recv-cb -> loop: reassembled RTCM bytes to GNSS
extern uint8_t              g_rtcmTxMsgId; // rolling msg_id for fragmentation (loop ctx)

// RTK relay init — set callbacks + alloc the rover ring. Inert unless enabled.
void rtkRelayInit();
// recv-cb context: push a completed rover-side RTCM frame into the ring.
void rtcmRingPush(const uint8_t* frame, int len);
// loop context (readGPSBase): fragment + broadcast an RC-base RTCM frame.
void rtcmBroadcastFrame(const uint8_t* frame, int len);

#endif
