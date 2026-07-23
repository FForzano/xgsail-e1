// SailFrames v2.0.0 Stage 2 — ESP-NOW peer-mesh glue (state + radio-mode
// transitions + broadcast/receive logic). Wire types live in mesh.h.
#include "mesh.h"
#include "v2_types.h"
#include "config.h"
#include "storage.h"
#include "ocs.h"
#include "rtk_relay.h"
#include "gnss.h"
#include "imu.h"
#include "shared_state.h"
#include "upload.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

MeshPeerState     g_mesh_peers[MESH_PEER_MAX];
volatile int      g_mesh_peer_count = 0;
volatile uint16_t g_mesh_seq = 0;
volatile bool     g_mesh_enabled = false;
volatile uint32_t g_mesh_rx_count = 0;
volatile uint32_t g_mesh_tx_count = 0;
volatile uint32_t g_mesh_tx_fail_count = 0;
volatile uint32_t g_mesh_rx_dropped_bad_magic = 0;
unsigned long     g_mesh_last_broadcast = 0;
uint32_t          g_mesh_local_sender_id = 0;
const uint8_t     MESH_BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// v2.0.0 radio mode transition stub (SF_FIRMWARE_V2_SPEC.md Stage 1).
// Stage 1 ships the state variable and logging only — the actual WiFi/BLE/
// ESP-NOW teardown and bringup move into this function in Stage 2 once
// the existing implicit "WiFi STA + BLE-C" coexistence is converted to a
// single Core-1 owner. Callers may invoke this now to record intent.
void radioModeTransition(RadioMode target, const char* reason) {
  if (target == g_radio_mode) return;
  RadioMode prev = g_radio_mode;
  g_radio_mode = target;
  Serial.printf("[RADIO] %s -> %s (%s)\n",
                radioModeName(prev), radioModeName(target),
                reason ? reason : "");
  char line[96];
  snprintf(line, sizeof(line), "radio %s->%s reason=%s",
           radioModeName(prev), radioModeName(target),
           reason ? reason : "-");
  appendBootLog(line);
}

static void meshOnReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  // ESP-NOW RX RSSI (dBm) for link-budget / range debugging. Captured
  // here, surfaced per-peer in the `mesh` command. Guard the pointer
  // chain in case a core build doesn't populate rx_ctrl.
  int8_t rx_rssi = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0;
  if (len < (int)sizeof(MeshHeader)) {
    g_mesh_rx_dropped_bad_magic++;
    return;
  }
  const MeshHeader* h = (const MeshHeader*)data;
  if (h->magic[0] != MESH_MAGIC_0 || h->magic[1] != MESH_MAGIC_1) {
    g_mesh_rx_dropped_bad_magic++;
    return;
  }
  if (h->version != MESH_VERSION) return;
  if (h->sender_id == g_mesh_local_sender_id) return;  // own packet (broadcast loopback)

  g_mesh_rx_count++;

  if (h->msg_type == MSG_BOAT_STATE &&
      len >= (int)(sizeof(MeshHeader) + sizeof(BoatStatePayload))) {
    const BoatStatePayload* p = (const BoatStatePayload*)(data + sizeof(MeshHeader));

    // Find or add peer in the in-memory table.
    int idx = -1;
    for (int i = 0; i < g_mesh_peer_count; i++) {
      if (g_mesh_peers[i].sender_id == h->sender_id) { idx = i; break; }
    }
    if (idx < 0) {
      if (g_mesh_peer_count >= MESH_PEER_MAX) return;  // full
      idx = g_mesh_peer_count++;
      g_mesh_peers[idx].sender_id = h->sender_id;
      g_mesh_peers[idx].msg_count = 0;
    }
    g_mesh_peers[idx].last_lat_e7       = p->lat_e7;
    g_mesh_peers[idx].last_lon_e7       = p->lon_e7;
    g_mesh_peers[idx].last_sog_cm_s     = p->sog_cm_s;
    g_mesh_peers[idx].last_cog_deg10    = p->cog_deg10;
    g_mesh_peers[idx].last_heading_deg10= p->heading_deg10;
    g_mesh_peers[idx].last_heel_deg     = p->heel_deg;
    g_mesh_peers[idx].unit_role         = p->unit_role;
    g_mesh_peers[idx].fix_quality       = p->fix_quality;
    g_mesh_peers[idx].sat_count         = p->sat_count;
    g_mesh_peers[idx].hdop_x10          = p->hdop_x10;   // 0 = no data
    g_mesh_peers[idx].hacc_mm           = p->hacc_mm;    // 0 = no data
    g_mesh_peers[idx].last_seen_ms      = millis();
    g_mesh_peers[idx].last_rssi         = rx_rssi;
    g_mesh_peers[idx].last_seq          = h->seq;
    g_mesh_peers[idx].msg_count++;
  }
  else if (h->msg_type == MSG_RACE_ARMED &&
           len >= (int)(sizeof(MeshHeader) + sizeof(RaceArmedPayload))) {
    // Stage 4.5 — race-armed broadcast. Translate relative
    // seconds_until_start into local millis() and arm boat-local OCS.
    // Forward-declared ocsArm in this TU (defined later in the file).
    const RaceArmedPayload* p = (const RaceArmedPayload*)(data + sizeof(MeshHeader));
    double pin_lat = p->pin_lat_e7 / 1e7;
    double pin_lon = p->pin_lon_e7 / 1e7;
    double rc_lat  = p->rc_lat_e7  / 1e7;
    double rc_lon  = p->rc_lon_e7  / 1e7;
    uint32_t start_ms = millis() + (uint32_t)(p->seconds_until_start * 1000);
    extern void ocsArm(double, double, double, double, uint32_t);
    ocsArm(pin_lat, pin_lon, rc_lat, rc_lon, start_ms);
    Serial.printf("[MESH] MSG_RACE_ARMED from 0x%08lx — race %d T+0 in %ds\n",
                  (unsigned long)h->sender_id, p->race_num, p->seconds_until_start);
  }
  else if (h->msg_type == MSG_INDIVIDUAL_RECALL &&
           len >= (int)(sizeof(MeshHeader) + sizeof(IndividualRecallPayload))) {
    // Stage 5 — RC unit recalled a specific boat. If that's us,
    // override boat-local OCS to over_line=true. RC is authoritative.
    const IndividualRecallPayload* p =
        (const IndividualRecallPayload*)(data + sizeof(MeshHeader));
    if (p->target_sender_id == g_mesh_local_sender_id) {
      // Forward-declare; defined later in the OCS section.
      extern void ocsForceOver(int16_t rc_distance_cm);
      ocsForceOver(p->distance_cm);
      Serial.printf("[MESH] INDIVIDUAL_RECALL for us! RC d=%d cm — forcing OCS=true\n",
                    p->distance_cm);
    }
  }
  else if (h->msg_type == MSG_RTCM_FRAG) {
    // RTK Phase-2 — rover ingests RC base corrections. This callback is the
    // ONLY context that touches g_rtcmRx; completed frames go to the ring via
    // its onFrame (rtcmRingPush). Gated off during uploads (RF contention,
    // gotchas #21/#22). Inert unless rtk_enabled — old-firmware boats and
    // non-RTK boats simply fall through here, ignoring the new msg_type.
    if (config.rtk_enabled && roleIsRover() && !wifiBusy && !uploading) {
      g_rtcmRx.onPacket(data, len);
    }
  }
}

// Broadcast a MSG_RACE_ARMED to the fleet. Called from the telnet
// `race arm` command after we've armed our own OCS state. Other
// boats receive this in meshOnReceive and arm their own.
bool meshBroadcastRaceArmed(double pin_lat, double pin_lon,
                            double rc_lat, double rc_lon,
                            int seconds_until_start,
                            uint8_t race_num, uint8_t sequence_mode) {
  if (!g_mesh_enabled) return false;
  uint8_t buf[sizeof(MeshHeader) + sizeof(RaceArmedPayload)];
  MeshHeader* h = (MeshHeader*)buf;
  RaceArmedPayload* p = (RaceArmedPayload*)(buf + sizeof(MeshHeader));

  h->magic[0] = MESH_MAGIC_0;
  h->magic[1] = MESH_MAGIC_1;
  h->version  = MESH_VERSION;
  h->msg_type = MSG_RACE_ARMED;
  h->seq      = g_mesh_seq++;
  h->ttl      = 0;
  h->reserved = 0;
  h->sender_id = g_mesh_local_sender_id;
  h->gps_time_ms = 0;

  p->pin_lat_e7 = (int32_t)(pin_lat * 1e7);
  p->pin_lon_e7 = (int32_t)(pin_lon * 1e7);
  p->rc_lat_e7  = (int32_t)(rc_lat  * 1e7);
  p->rc_lon_e7  = (int32_t)(rc_lon  * 1e7);
  p->seconds_until_start = seconds_until_start;
  p->race_num = race_num;
  p->sequence_mode = sequence_mode;
  p->reserved[0] = p->reserved[1] = 0;

  // Send multiple times — small payload, race-critical, no ack mechanism
  // in MVP. Three transmissions ~100 ms apart raise reliability against
  // single-packet losses without saturating airtime.
  esp_err_t err = ESP_OK;
  for (int i = 0; i < 3; i++) {
    esp_err_t e = esp_now_send(MESH_BROADCAST_ADDR, buf, sizeof(buf));
    if (e == ESP_OK) g_mesh_tx_count++;
    else { g_mesh_tx_fail_count++; err = e; }
    if (i < 2) delay(100);
  }
  return err == ESP_OK;
}

// Stage 5 — RC broadcasts when it sees a boat over the line.
// Target boat receives in meshOnReceive and overrides its local
// OCS state. Sent 3x for resilience.
bool meshBroadcastIndividualRecall(uint32_t target_id, int16_t distance_cm) {
  if (!g_mesh_enabled) return false;
  uint8_t buf[sizeof(MeshHeader) + sizeof(IndividualRecallPayload)];
  MeshHeader* h = (MeshHeader*)buf;
  IndividualRecallPayload* p =
      (IndividualRecallPayload*)(buf + sizeof(MeshHeader));

  h->magic[0] = MESH_MAGIC_0;
  h->magic[1] = MESH_MAGIC_1;
  h->version  = MESH_VERSION;
  h->msg_type = MSG_INDIVIDUAL_RECALL;
  h->seq      = g_mesh_seq++;
  h->ttl      = 0;
  h->reserved = 0;
  h->sender_id = g_mesh_local_sender_id;
  h->gps_time_ms = 0;

  p->target_sender_id = target_id;
  p->distance_cm      = distance_cm;
  p->reserved[0] = p->reserved[1] = 0;

  esp_err_t err = ESP_OK;
  for (int i = 0; i < 3; i++) {
    esp_err_t e = esp_now_send(MESH_BROADCAST_ADDR, buf, sizeof(buf));
    if (e == ESP_OK) g_mesh_tx_count++;
    else { g_mesh_tx_fail_count++; err = e; }
    if (i < 2) delay(50);
  }
  return err == ESP_OK;
}

void meshInit() {
  if (g_mesh_enabled) return;
  g_mesh_local_sender_id = boatIdHash(config.boat_id);

  // ESP-NOW needs the WiFi radio enabled to transmit, not just the
  // driver initialised. Early-setup() does WiFi.mode(WIFI_STA) then
  // WiFi.disconnect(true) which turns the radio OFF (esp_wifi_stop).
  // Re-enable STA mode here. This is idempotent if WiFi was already
  // up from a later connectWiFi().
  WiFi.mode(WIFI_STA);

  // ESP-NOW range fixes (2026.06.08):
  // 1) Disable modem power-save. Default STA power-save duty-cycles the
  //    receiver, so it sleeps through broadcasts — at the link margin
  //    this looks like a sharp short-range cliff (peers vanish past a
  //    few metres). WIFI_PS_NONE keeps the RX always listening.
  // 2) Pin max TX power for the ALWAYS-ON mesh. setTxPower was only
  //    applied inside connectWiFi() (the upload window), so mesh-only
  //    operation ran at the post-mode default. Make it explicit here.
  WiFi.setSleep(false);                    // esp_wifi_set_ps(WIFI_PS_NONE)
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  esp_err_t err = esp_now_init();
  if (err != ESP_OK) {
    Serial.printf("[MESH] esp_now_init failed: %d\n", err);
    return;
  }

  esp_now_register_recv_cb(meshOnReceive);

  // Pin the WiFi channel explicitly so esp_now_send always knows which
  // channel to transmit on, even when STA is not associated with any
  // AP. .13 used peerInfo.channel=0 which means "use current STA
  // channel" — but when STA hasn't connected to anything yet, that
  // channel is undefined and esp_now_send returns ESP_ERR_ESPNOW_IF
  // (tx=0 fail=N pattern observed on the fleet's .13 boot).
  esp_wifi_set_channel(MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MESH_BROADCAST_ADDR, 6);
  peerInfo.channel = MESH_CHANNEL;     // explicit channel, not 0
  peerInfo.ifidx   = WIFI_IF_STA;      // be explicit; was zero-init
  peerInfo.encrypt = false;
  err = esp_now_add_peer(&peerInfo);
  if (err != ESP_OK) {
    Serial.printf("[MESH] add broadcast peer failed: %d\n", err);
    esp_now_deinit();
    return;
  }

  g_mesh_enabled = true;
  Serial.printf("[MESH] ESP-NOW init OK, sender_id=0x%08lx ch=%d\n",
                (unsigned long)g_mesh_local_sender_id, MESH_CHANNEL);
  appendBootLog("mesh init ok");
}

static void meshBuildAndSendBoatState() {
  uint8_t buf[sizeof(MeshHeader) + sizeof(BoatStatePayload)];
  MeshHeader* h = (MeshHeader*)buf;
  BoatStatePayload* p = (BoatStatePayload*)(buf + sizeof(MeshHeader));

  h->magic[0]   = MESH_MAGIC_0;
  h->magic[1]   = MESH_MAGIC_1;
  h->version    = MESH_VERSION;
  h->msg_type   = MSG_BOAT_STATE;
  h->seq        = g_mesh_seq++;
  h->ttl        = 0;     // peer-to-peer for MVP, no rebroadcast
  h->reserved   = 0;
  h->sender_id  = g_mesh_local_sender_id;
  h->gps_time_ms = 0;    // TODO: HHMMSS.sss -> ms-of-day; 0 = unknown for MVP

  p->lat_e7         = (int32_t)(gps.lat * 1e7);
  p->lon_e7         = (int32_t)(gps.lon * 1e7);
  // 1 kt = 51.4444 cm/s
  p->sog_cm_s       = (int16_t)(gps.speed_kts * 51.4444f);
  p->cog_deg10      = (int16_t)(gps.course * 10.0f);
  p->heading_deg10  = (int16_t)(imu.heading * 10.0f);
  p->heel_deg       = (int8_t)imu.heel;
  p->fix_quality    = (uint8_t)gps.fix_quality;
  p->sat_count      = (uint8_t)gps.satellites;
  p->unit_role      = (uint8_t)g_role;
  // Per-boat quality for the RC pre-race panel (former reserved[2], same 20 B
  // wire). 0 = "no data" (no fix / no GST) so the RC renders "--", never 0.
  // Clamp: hdop is 99.9 with no fix (×10 overflows u8), hacc could exceed 255 mm.
  if (gps.valid && gps.hdop > 0.1f && gps.hdop < 25.5f) {
    p->hdop_x10 = (uint8_t)lroundf(gps.hdop * 10.0f);
  } else {
    p->hdop_x10 = 0;   // no data
  }
  float hacc_mm = gps.hacc_m * 1000.0f;   // unified: GST (LG290P) or PQTMEPE (LC29HEA)
  p->hacc_mm = (hacc_mm > 0.5f) ? (uint8_t)fminf(255.0f, lroundf(hacc_mm)) : 0;  // 0 = no data

  esp_err_t err = esp_now_send(MESH_BROADCAST_ADDR, buf, sizeof(buf));
  if (err == ESP_OK) {
    g_mesh_tx_count++;
  } else {
    g_mesh_tx_fail_count++;
    static int s_logged = 0;
    if (s_logged < 5) {
      Serial.printf("[MESH] esp_now_send failed: 0x%x\n", err);
      s_logged++;
    }
    // ESP_ERR_ESPNOW_NOT_INIT (0x3001) fires when WiFi.disconnect(true)
    // in connectWiFi() called esp_wifi_stop() as a side effect, which
    // tears down ESP-NOW too. Recover by re-initializing on the next
    // tick. Observed on .15: tx=71 then fail=N with IDF logging
    // "E ESPNOW: esp now not init!" at 500 ms intervals after the
    // first upload-task WiFi reconnect cycle.
    if (err == ESP_ERR_ESPNOW_NOT_INIT) {
      g_mesh_enabled = false;
      Serial.println("[MESH] ESP-NOW torn down by WiFi cycle — re-init next tick");
    }
  }
}

void meshTick() {
  // Auto-recover if ESP-NOW got torn down by a WiFi cycle. meshInit
  // is idempotent (returns early if already enabled), so calling it
  // here is safe both for normal operation and post-teardown.
  // Throttled to once per second so we don't spam if the radio is
  // genuinely down.
  if (!g_mesh_enabled && !wifiBusy && !uploading) {
    static unsigned long lastReinit = 0;
    unsigned long now2 = millis();
    if (now2 - lastReinit >= 1000) {
      lastReinit = now2;
      meshInit();
    }
    return;
  }
  if (!g_mesh_enabled) return;
  // Don't compete with HTTP uploads — esp_now_send is cheap but the
  // RF airtime steals from the upload. Existing wifiBusy + uploading
  // gates already protect telnet; we use the same convention.
  if (wifiBusy || uploading) return;

  unsigned long now = millis();
  if (now - g_mesh_last_broadcast >= MESH_BROADCAST_INTERVAL_MS) {
    g_mesh_last_broadcast = now;
    meshBuildAndSendBoatState();
  }

  // Expire stale peers (linear scan is fine for ≤32 peers).
  for (int i = g_mesh_peer_count - 1; i >= 0; i--) {
    if (now - g_mesh_peers[i].last_seen_ms > MESH_PEER_EXPIRY_MS) {
      g_mesh_peers[i] = g_mesh_peers[g_mesh_peer_count - 1];
      g_mesh_peer_count--;
    }
  }
}

const char* boatNameForSender(uint32_t id) {
  static const char* names[] = {"E1","E2","E3","E4","E5","E6","B1","F1"};
  for (unsigned i = 0; i < sizeof(names)/sizeof(names[0]); i++) {
    if (boatIdHash(names[i]) == id) return names[i];
  }
  return "??";
}
