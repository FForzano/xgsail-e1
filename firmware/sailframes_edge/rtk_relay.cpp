// RTK Phase-2 relay glue — see rtk_relay.h for the framer/reassembler and
// the threading contract. docs/RTK_PHASE2_DESIGN.md has the wire design.
#include "rtk_relay.h"
#include "mesh.h"
#include "gnss.h"
#include "upload.h"
#include "storage.h"
#include "config.h"
#include "v2_types.h"
#include "shared_state.h"  // wifiBusy
#include <esp_now.h>       // esp_now_send

RtcmFramer           g_rtcmTx;             // RC base: Serial2 RTCM frames (loop ctx only)
RtcmReassembler      g_rtcmRx;             // rover: ESP-NOW frags (recv-cb ctx only)
StreamBufferHandle_t g_rtcmRing = nullptr; // recv-cb -> loop: reassembled RTCM bytes to GNSS
uint8_t              g_rtcmTxMsgId = 0;    // rolling msg_id for fragmentation (loop ctx)

// RTK relay init — set callbacks + alloc the rover ring. Inert unless enabled.
void rtkRelayInit() {
  if (!config.rtk_enabled) return;
  if (roleIsBase()) {
    g_rtcmTx.onFrame = rtcmBroadcastFrame;
    Serial.println("[RTK] relay PRODUCER (RC base) armed");
  } else {
    g_rtcmRx.onFrame = rtcmRingPush;
    g_rtcmRing = xStreamBufferCreate(4096, 1);   // SPSC byte ring, trigger level 1
    Serial.printf("[RTK] relay CONSUMER (rover) armed, ring=%s\n", g_rtcmRing ? "ok" : "ALLOC FAIL");
  }
  appendBootLog(roleIsBase() ? "rtk relay base" : "rtk relay rover");
}

void rtcmRingPush(const uint8_t* frame, int len) {
  if (g_rtcmRing) xStreamBufferSend(g_rtcmRing, frame, (size_t)len, 0);
}

// RC base, loop context (readGPSBase): fragment a complete RTCM frame and
// broadcast it 2× over ESP-NOW. Gated off during uploads to avoid the WiFi/RF
// contention that caused past fleet hangs (gotchas #21/#22) — racing never
// overlaps an upload, so nothing is lost.
void rtcmBroadcastFrame(const uint8_t* frame, int len) {
  if (!g_mesh_enabled || wifiBusy || uploading) return;
  uint8_t msg_id = g_rtcmTxMsgId++;
  int fc = (len + RTCM_FRAG_MAX - 1) / RTCM_FRAG_MAX;
  if (fc > RTK_MAX_FRAGS) return;   // shouldn't happen (≤1029 B), defensive
  uint8_t buf[sizeof(MeshHeader) + 4 + RTCM_FRAG_MAX];
  for (int rep = 0; rep < 2; rep++) {                 // 2×-tx for loss margin
    for (int fi = 0; fi < fc; fi++) {
      int off = fi * RTCM_FRAG_MAX;
      int flen = len - off; if (flen > RTCM_FRAG_MAX) flen = RTCM_FRAG_MAX;
      MeshHeader* h = (MeshHeader*)buf;
      h->magic[0] = MESH_MAGIC_0; h->magic[1] = MESH_MAGIC_1; h->version = MESH_VERSION;
      h->msg_type = MSG_RTCM_FRAG; h->seq = g_mesh_seq++; h->ttl = 0; h->reserved = 0;
      h->sender_id = g_mesh_local_sender_id; h->gps_time_ms = 0;
      RtcmFragPayload* p = (RtcmFragPayload*)(buf + sizeof(MeshHeader));
      p->msg_id = msg_id; p->frag_index = (uint8_t)fi; p->frag_count = (uint8_t)fc;
      p->frag_len = (uint8_t)flen;
      memcpy(p->data, frame + off, flen);
      esp_now_send(MESH_BROADCAST_ADDR, buf, sizeof(MeshHeader) + 4 + flen);
      delayMicroseconds(250);   // light pacing for peers' recv; small so readGPSBase RX doesn't overflow
    }
  }
}
