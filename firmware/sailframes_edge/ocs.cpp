// OCS glue — see ocs.h.
#include "ocs.h"
#include "config.h"
#include "v2_types.h"
#include "gnss.h"
#include "imu.h"
#include "mesh.h"
#include "shared_state.h"
#include "upload.h"
#include <SD.h>

ClassRegistryEntry g_class_registry[CLASS_REGISTRY_MAX];
int g_class_registry_count = 0;

OCSState g_ocs = {};
unsigned long g_ocs_last_tick_ms = 0;
unsigned long g_rc_last_tick_ms = 0;
bool g_fleetWatch = false;

void ocsDisarm() {
  g_ocs.armed = false;
  g_ocs.over_line = false;
  g_ocs.was_over_at_start = false;
  g_ocs.distance_to_line_m = 0;
  g_ocs.closure_rate_m_s = 0;
  g_ocs.over_since_ms = 0;
  g_ocs.cleared_at_ms = 0;
  g_ocs._last_d = 0;
  g_ocs._last_t_ms = 0;
}

void ocsArm(double pin_lat, double pin_lon,
            double rc_lat,  double rc_lon,
            uint32_t start_time_ms) {
  ocsDisarm();
  // A re-arm is a new start sequence — a clean slate. Clear any prior RC-side
  // OCS latches on every peer so boats recalled in a previous start don't stay
  // flagged OCS into the new one. (rc_ocs_called is RC-only state; clearing it
  // on a racing boat is harmless.)
  for (int i = 0; i < g_mesh_peer_count; i++) {
    g_mesh_peers[i].rc_ocs_called = false;
    g_mesh_peers[i].rc_ocs_called_at_ms = 0;
  }
  g_ocs.armed = true;
  g_ocs.pin_lat = pin_lat;
  g_ocs.pin_lon = pin_lon;
  g_ocs.rc_lat = rc_lat;
  g_ocs.rc_lon = rc_lon;
  g_ocs.start_time_ms = start_time_ms;
}

void ocsTick() {
  if (!g_ocs.armed) return;
  if (!gps.valid) return;

  unsigned long now = millis();
  if (now - g_ocs_last_tick_ms < OCS_TICK_INTERVAL_MS) return;
  g_ocs_last_tick_ms = now;

  // Use GPS COG when boat is moving — reliable above ~2 kt. Below
  // that, use IMU heading (magnetic / fusion may drift but it's
  // the best we have when stationary or in low-speed pre-start
  // tactical maneuvering).
  float heading_deg = (gps.speed_kts > 2.0f) ? gps.course : imu.heading;
  float heading_rad = heading_deg * (float)PI / 180.0f;

  double ref_lat_rad = ((g_ocs.pin_lat + g_ocs.rc_lat) / 2.0) * PI / 180.0;
  double m_per_deg_lat = 111320.0;
  double m_per_deg_lon = 111320.0 * cos(ref_lat_rad);

  // Bow position = boat position + bow_offset along heading
  double bow_lat = gps.lat + (OCS_BOW_OFFSET_M * cos(heading_rad)) / m_per_deg_lat;
  double bow_lon = gps.lon + (OCS_BOW_OFFSET_M * sin(heading_rad)) / m_per_deg_lon;

  // Project bow onto line PIN(A) -> RC(B). Local equirectangular
  // meters frame anchored at PIN.
  double Bx = (g_ocs.rc_lon - g_ocs.pin_lon) * m_per_deg_lon;
  double By = (g_ocs.rc_lat - g_ocs.pin_lat) * m_per_deg_lat;
  double Px = (bow_lon - g_ocs.pin_lon) * m_per_deg_lon;
  double Py = (bow_lat - g_ocs.pin_lat) * m_per_deg_lat;

  // 2D cross product gives signed perpendicular distance × |AB|.
  // Sign convention: positive = "left" of AB walking PIN -> RC.
  // The user/RC convention (which side is pre-start) is decided
  // at arm time by how PIN and RC are passed. Standard fleet
  // convention: PIN on port hand approaching start, RC on stbd;
  // pre-start side is the side the cross-product convention
  // picks positive.
  double cross = Bx * Py - By * Px;
  double lenAB = sqrt(Bx * Bx + By * By);
  float d_signed = (lenAB > 0.001) ? (float)(cross / lenAB) : 0.0f;

  // Closure-rate numerical derivative over last tick.
  if (g_ocs._last_t_ms > 0) {
    float dt = (now - g_ocs._last_t_ms) / 1000.0f;
    if (dt > 0.005f) g_ocs.closure_rate_m_s = (d_signed - g_ocs._last_d) / dt;
  }
  g_ocs._last_d = d_signed;
  g_ocs._last_t_ms = now;
  g_ocs.distance_to_line_m = d_signed;

  // Snapshot whether we were over at T+0 (within ±500 ms window).
  int32_t time_to_start = (int32_t)(g_ocs.start_time_ms - now);  // positive = pre-start
  if (time_to_start > -500 && time_to_start < 500) {
    if (d_signed < -OCS_THRESHOLD_M) g_ocs.was_over_at_start = true;
  }

  // Over-line latching is only meaningful at/after T+0.
  if (time_to_start <= 0) {
    if (d_signed < -OCS_THRESHOLD_M) {
      if (!g_ocs.over_line) {
        g_ocs.over_line = true;
        g_ocs.over_since_ms = now;
        g_ocs.cleared_at_ms = 0;
        Serial.printf("[OCS] Bow over line: d=%.2f m\n", d_signed);
      }
    } else if (g_ocs.over_line && d_signed > OCS_CLEAR_THRESHOLD_M) {
      if (g_ocs.cleared_at_ms == 0) {
        g_ocs.cleared_at_ms = now;
      } else if (now - g_ocs.cleared_at_ms > OCS_CLEAR_DWELL_MS) {
        g_ocs.over_line = false;
        g_ocs.cleared_at_ms = 0;
        Serial.printf("[OCS] Bow cleared line: d=%.2f m\n", d_signed);
      }
    } else {
      g_ocs.cleared_at_ms = 0;
    }
  }
}

// Stage 5 — called from meshOnReceive when this boat is the
// target of MSG_INDIVIDUAL_RECALL. Overrides local OCS to true
// regardless of local computation. RC is authoritative.
//
// Stage 5.5 — also logs RC-vs-local OCS disagreement to
// /sf/ocs_disagree.log when the deltas exceed a threshold.
// Disagreement is interesting data: if RC's bow_offset_m for this
// boat is wrong, or the boat's IMU heading is bad, or there's
// large clock skew between fixes, the boat's local OCS state will
// not match RC's. Post-race we mine this log to tune the registry.
void ocsForceOver(int16_t rc_distance_cm) {
  float rc_d_m = rc_distance_cm / 100.0f;
  float local_d_m = g_ocs.distance_to_line_m;
  bool local_over = g_ocs.over_line;

  if (g_ocs.armed && !g_ocs.over_line) {
    g_ocs.over_line = true;
    g_ocs.over_since_ms = millis();
    g_ocs.cleared_at_ms = 0;
    Serial.printf("[OCS] Forced over_line by RC recall (rc_d=%.2fm, local_d=%.2fm)\n",
                  rc_d_m, local_d_m);
  }

  // Stage 5.5 — log every recall to /sf/ocs_disagree.log with
  // RC's view and our local state at the moment of recall. Files
  // are uploaded post-race like the rest of the session CSVs.
  char iso[24];
  bool have_time = formatGpsIso(iso, sizeof(iso));
  File f = SD.open("/sf/ocs_disagree.log", FILE_APPEND);
  if (f) {
    f.printf("t=%s armed=%d local_over=%d rc_d=%.2fm local_d=%.2fm delta=%.2fm\n",
             have_time ? iso : "no-fix",
             g_ocs.armed ? 1 : 0,
             local_over ? 1 : 0,
             rc_d_m, local_d_m, rc_d_m - local_d_m);
    f.close();
  }
}

void rcComputeFleetOCS() {
  if (g_role != ROLE_RC_SIGNAL) return;
  if (!g_ocs.armed) return;
  unsigned long now = millis();
  if (now - g_rc_last_tick_ms < RC_TICK_INTERVAL_MS) return;
  g_rc_last_tick_ms = now;

  int32_t time_to_start = (int32_t)(g_ocs.start_time_ms - now);
  // Only call OCS post-T+0 (with small grace window for clock drift)
  if (time_to_start > 500) return;

  double ref_lat_rad = ((g_ocs.pin_lat + g_ocs.rc_lat) / 2.0) * PI / 180.0;
  double m_per_deg_lat = 111320.0;
  double m_per_deg_lon = 111320.0 * cos(ref_lat_rad);

  double Bx = (g_ocs.rc_lon - g_ocs.pin_lon) * m_per_deg_lon;
  double By = (g_ocs.rc_lat - g_ocs.pin_lat) * m_per_deg_lat;
  double lenAB = sqrt(Bx * Bx + By * By);
  if (lenAB < 0.001) return;

  for (int i = 0; i < g_mesh_peer_count; i++) {
    MeshPeerState& peer = g_mesh_peers[i];
    // Skip if no recent fix
    if (peer.fix_quality == 0 || peer.last_lat_e7 == 0) continue;
    if (now - peer.last_seen_ms > 5000) continue;  // stale (>5s no msg)

    double peer_lat = peer.last_lat_e7 / 1e7;
    double peer_lon = peer.last_lon_e7 / 1e7;

    // Bow position: use heading from BoatStatePayload when boat is
    // slow, COG when fast. peer.last_sog_cm_s is cm/s; 100 cm/s ≈ 2 kt.
    float heading_deg = (peer.last_sog_cm_s > 100)
                          ? (peer.last_cog_deg10 / 10.0f)
                          : (peer.last_heading_deg10 / 10.0f);
    float heading_rad = heading_deg * (float)PI / 180.0f;

    // Stage 5.5 — per-class bow offset from /sf/classes.csv lookup.
    // Unknown peers fall through to OCS_BOW_OFFSET_M.
    float bow_offset = bowOffsetForSender(peer.sender_id);

    double bow_lat = peer_lat +
        (bow_offset * cos(heading_rad)) / m_per_deg_lat;
    double bow_lon = peer_lon +
        (bow_offset * sin(heading_rad)) / m_per_deg_lon;

    double Px = (bow_lon - g_ocs.pin_lon) * m_per_deg_lon;
    double Py = (bow_lat - g_ocs.pin_lat) * m_per_deg_lat;
    double cross = Bx * Py - By * Px;
    float d_signed = (float)(cross / lenAB);
    peer.rc_distance_m = d_signed;

    if (d_signed < -OCS_THRESHOLD_M && !peer.rc_ocs_called) {
      peer.rc_ocs_called = true;
      peer.rc_ocs_called_at_ms = now;
      int16_t d_cm = (int16_t)(d_signed * 100.0f);
      Serial.printf("[RC] OCS: peer 0x%08lx d=%.2fm — broadcasting recall\n",
                    (unsigned long)peer.sender_id, d_signed);
      meshBroadcastIndividualRecall(peer.sender_id, d_cm);
    }
  }
}

unsigned long g_fleetWatchLast = 0;

void fleetWatchTick() {
  if (!g_fleetWatch) return;
  unsigned long now = millis();
  if (now - g_fleetWatchLast < 500) return;  // ~2 Hz
  g_fleetWatchLast = now;
  static uint32_t refresh = 0;
  refresh++;

  Serial.print("\033[H");  // cursor home (overwrite in place)
  if (g_role != ROLE_RC_SIGNAL) {
    Serial.printf("fleetwatch: role is not rc_signal (role=%d) — no fleet OCS here\033[K\r\n", (int)g_role);
    Serial.print("\033[J");
    return;
  }
  if (!g_ocs.armed) {
    Serial.print("fleetwatch: OCS not armed — use 'race arm <pinLat> <pinLon> <rcLat> <rcLon> <secs>'\033[K\r\n");
    Serial.print("\033[J");
    return;
  }
  int32_t tts = (int32_t)(g_ocs.start_time_ms - now);
  Serial.printf("RC FLEET LIVE  T%+ds  peers=%d  bow=%.2fm  #%lu   (type 'fleetwatch' to stop)\033[K\r\n",
                tts / 1000, g_mesh_peer_count, OCS_BOW_OFFSET_M, (unsigned long)refresh);
  Serial.printf("line %.6f,%.6f -> %.6f,%.6f\033[K\r\n",
                g_ocs.pin_lat, g_ocs.pin_lon, g_ocs.rc_lat, g_ocs.rc_lon);
  Serial.print("NAME ID          FIX SAT  SOG   HDG    d(m)    STATE age\033[K\r\n");
  for (int i = 0; i < g_mesh_peer_count; i++) {
    const MeshPeerState& p = g_mesh_peers[i];
    const char* st = p.rc_ocs_called ? "OCS*" : (p.rc_distance_m < 0 ? "over" : "ok");
    Serial.printf("%-4s 0x%08lx  %u  %2u  %4.1f  %4.0f  %+7.2f  %-4s  %lus\033[K\r\n",
                  boatNameForSender(p.sender_id), (unsigned long)p.sender_id,
                  (unsigned)p.fix_quality, (unsigned)p.sat_count,
                  p.last_sog_cm_s / 51.4444, p.last_heading_deg10 / 10.0,
                  p.rc_distance_m, st, (now - p.last_seen_ms) / 1000);
  }
  Serial.print("\033[J");  // erase anything below the table
}

void loadClassRegistry() {
  g_class_registry_count = 0;
  if (g_role != ROLE_RC_SIGNAL) return;

  File f = SD.open("/sf/classes.csv", FILE_READ);
  if (!f) f = SD.open("/classes.csv", FILE_READ);  // fallback to root
  if (!f) {
    Serial.println("[CLASS] No classes.csv — RC will use OCS_BOW_OFFSET_M for all peers");
    return;
  }
  Serial.println("[CLASS] Loading classes.csv");

  while (f.available() && g_class_registry_count < CLASS_REGISTRY_MAX) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;
    // Skip header row if it starts with "boat_id" (case-insensitive)
    String low = line; low.toLowerCase();
    if (low.startsWith("boat_id")) continue;

    int c1 = line.indexOf(',');
    if (c1 < 0) continue;
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 < 0) continue;

    String boat = line.substring(0, c1); boat.trim();
    String cls  = line.substring(c1 + 1, c2); cls.trim();
    String bow  = line.substring(c2 + 1); bow.trim();
    if (boat.length() == 0) continue;

    ClassRegistryEntry& e = g_class_registry[g_class_registry_count];
    boat.toCharArray(e.boat_id, sizeof(e.boat_id));
    cls.toCharArray(e.class_name, sizeof(e.class_name));
    e.bow_offset_m = bow.toFloat();
    if (e.bow_offset_m <= 0.0f) e.bow_offset_m = OCS_BOW_OFFSET_M;
    e.sender_id = boatIdHash(e.boat_id);
    g_class_registry_count++;
  }
  f.close();

  Serial.printf("[CLASS] Loaded %d entries:\n", g_class_registry_count);
  for (int i = 0; i < g_class_registry_count; i++) {
    Serial.printf("[CLASS]   %s (0x%08lx) class=%s bow=%.2fm\n",
                  g_class_registry[i].boat_id,
                  (unsigned long)g_class_registry[i].sender_id,
                  g_class_registry[i].class_name,
                  g_class_registry[i].bow_offset_m);
  }
}

// Lookup bow_offset_m for a given peer sender_id. Returns the
// hardcoded default when registry has no entry — safe fallback so
// new boats joining the fleet without a registry entry still get
// OCS computed (just with the class-default bow offset).
float bowOffsetForSender(uint32_t sender_id) {
  for (int i = 0; i < g_class_registry_count; i++) {
    if (g_class_registry[i].sender_id == sender_id)
      return g_class_registry[i].bow_offset_m;
  }
  return OCS_BOW_OFFSET_M;
}

