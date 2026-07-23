# Firmware architecture

How the E1 firmware is put together: the dual-core split, the ESP-NOW
peer mesh, on-course-side (OCS) race-start detection, unit roles, and
cloud config sync. Written against the actual module layout in
`firmware/sailframes_edge/` (see `CLAUDE.md` for the file-per-module map)
— if something here and the source disagree, the source wins.

## Dual-core split

`setup()` creates two additional FreeRTOS tasks pinned to **Core 0**,
leaving Arduino's own `loop()` running on **Core 1**:

- `uploadTaskFunc` (`upload.cpp`) — WiFi connect, S3 upload, OTA pull,
  cloud config sync, and the periodic fleet-health/boot-log snapshots.
  Everything that touches the WiFi/HTTP stack lives here so Core 1's
  sensor reads and SD logging are never blocked by a slow upload.
- `diagnosticsTask` (`upload.cpp`) — an independent heartbeat that keeps
  running even if Core 1 hangs, printing `g_loopSection` (Core 1's last
  checkpoint, `shared_state.h`) and `g_uploadSection` (Core 0's, in
  `upload.h`) so a watchdog panic's backtrace can be matched to a human-
  readable "stuck here" location instead of just a raw stack trace.

SD-card access is shared between the two cores (Core 1 logs, Core 0
reads closed session files to upload) and serialized by `sdMutex`
(`shared_state.h`) — the upload task only ever touches files after
`logging` has already closed them, never the actively-written session.

`wifiBusy` (`shared_state.h`) is the other cross-core signal: while Core 0
is mid-WiFi-work, Core 1 must not touch the WiFi/LWIP stack (telnet,
`WiFi.*` calls) — under sustained traffic, LWIP mutex contention between
the two cores has hung the device in the field. Every radio-sharing
subsystem (ESP-NOW mesh, RTK relay, telnet) checks this gate before
touching the radio.

## ESP-NOW peer mesh (`mesh.{h,cpp}`)

Every unit broadcasts its own boat-state at 2 Hz (`MESH_BROADCAST_INTERVAL_MS`)
on a fixed channel (`MESH_CHANNEL`), unencrypted, to the broadcast address
— there's no peer registration or per-boat pairing. `meshOnReceive()`
(the ESP-NOW receive callback, running in the WiFi task context) parses
the packet and updates an in-memory peer table (`g_mesh_peers`,
`MESH_PEER_MAX` = 32 slots); peers not heard from in `MESH_PEER_EXPIRY_MS`
(30 s) are dropped.

Wire format (`mesh.h`): a 16-byte `MeshHeader` (magic, version, message
type, sequence, sender ID — an FNV-1a hash of `boat_id`, stable across
reboots and MAC changes) followed by a per-message-type payload. Field
widths are deliberately small — `sog_cm_s`/`cog_deg10`/`heading_deg10` as
scaled `int16_t`, `heel_deg` as `int8_t` — to keep the aggregate mesh
traffic (up to ~25 boats × 2 Hz) well inside ESP-NOW's per-packet budget.
Message types in use: `MSG_BOAT_STATE`, `MSG_RACE_ARMED`,
`MSG_INDIVIDUAL_RECALL`, and `MSG_RTCM_FRAG` (the RTK relay, see
`docs/gnss-rtk.md`) — the enum also reserves a few message types that
aren't sent by this firmware.

## OCS (on-course-side) detection (`ocs.{h,cpp}`)

Per-boat, boat-local: `ocsArm()` records the start line (PIN and RC
endpoints, both `double` lat/lon) and start time. Once armed, `ocsTick()`
runs at up to `OCS_TICK_INTERVAL_MS` (100 ms, matching the GPS fix rate):
it projects the boat's bow position (GPS position advanced by
`bow_offset_m` along the current heading — GPS course over ground above
2 kt, IMU heading below that speed, since COG is unreliable at low
speed) onto the start line, computing a **signed perpendicular
distance** — positive is the pre-start side. Past the start time, a
distance beyond `OCS_THRESHOLD_M` (0.5 m) latches `over_line`; clearing
requires being back past `OCS_CLEAR_THRESHOLD_M` for
`OCS_CLEAR_DWELL_MS` (2 s) continuously, so a boat bobbing right at the
line doesn't flicker in and out of the OCS state.

The `rc_signal` unit additionally runs `rcComputeFleetOCS()`: the same
geometry applied to every mesh peer's last-known position, using the
RC's own (authoritative) line endpoints and each boat's `bow_offset_m`
looked up from `/sf/classes.csv` (`loadClassRegistry()`/
`bowOffsetForSender()` — falls back to `OCS_BOW_OFFSET_M` for boats not
in the registry). When the RC's computation calls a boat OCS, it
broadcasts `MSG_INDIVIDUAL_RECALL`; the target boat's `ocsForceOver()`
sets `over_line = true` regardless of what its own local computation
said — **the RC's call is authoritative**, and a boat's local vs. the
RC's view disagreeing is logged to `/sf/ocs_disagree.log` for post-race
review rather than acted on live.

`fleetWatchTick()` (`ocs.cpp`) is a non-blocking live serial dashboard
(toggle with the `fleetwatch` console command) for watching the whole
fleet's OCS state from an RC unit's USB-serial console during a start
sequence.

## Unit roles and radio mode (`v2_types.h`)

`UnitRole` (`config.txt`'s `unit_role`, read at boot in
`loadConfig()`/`config.cpp`): `racing_boat` (default), `rc_signal`
(computes fleet OCS, is the RTK base when RTK is on), `rc_pin`, `mark`,
`committee_chase`, `spare`. `RadioMode` (`g_radio_mode`:
`MODE_BOOT`/`MODE_IDLE`/`MODE_DOCK`/`MODE_RACING`/`MODE_RC_ACTIVE`) is
tracked and logged on transition (`radioModeTransition()`, `mesh.cpp`)
for `/boot.log` forensics; this repo's build is E1-only, so
`HardwarePlatform` (`g_hw`) is always `HW_E1` regardless of what
`hardware_platform` says in `config.txt` (kept in `Config` for on-disk/
cloud-config compatibility with the wider SailFrames Core fleet, which
also has other hardware platforms — see `README.md`).

## Cloud config sync (`cloud_config.{h,cpp}`)

Mirrors the OTA manifest-pull pattern (`docs/gnss-rtk.md`'s sibling,
`ota.cpp`): fetches a manifest, verifies a SHA256 over the config body,
and merges only **allow-listed** keys into `/config.txt` —
`CLOUD_CONFIG_ALLOW_KEYS` in `cloud_config.cpp` is exactly:
`wind_enabled`, `wind_offset`, `start_speed_knots`, `stop_speed_knots`,
`start_delay_sec`, `stop_delay_sec`, `unit_role`, `rtk_enabled`. Identity
and connectivity keys — `boat_id`, `wifi*`, `wind_mac`, `s3_*` — are
**deliberately excluded**: a bad push must never be able to lock a boat
off the network or change the identity hash its mesh peers and the class
registry key off of. Applying a synced config schedules a reboot 3
seconds out (`g_configRebootPending`/`g_configRebootAtMs`, applied from
`loop()` once no upload is in flight) rather than rebooting immediately,
so the upload task gets to unwind cleanly first.

## Console (`console.{h,cpp}`)

One command dispatcher (`processCommand()`) shared by USB-serial
(always on) and telnet (off by default — see `firmware/README.md` for
why, and the full command reference). Status/debug commands read state
directly out of the owning module (`gps`, `imu`, `battery`, `g_ocs`,
`g_mesh_peers`, ...) rather than duplicating it — if you add a command
that needs a new piece of state, that state belongs in its owning
module's header, not a new global in `console.cpp`.
