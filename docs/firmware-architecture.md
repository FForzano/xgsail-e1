# Firmware architecture

How the E1 firmware is put together: the dual-core split, the XGSail
device-protocol claim/upload flow (WiFi and its BLE relay fallback), the
ESP-NOW peer mesh, on-course-side (OCS) race-start detection, and unit
roles. Written against the actual module layout in
`firmware/sailframes_edge/` (see `CLAUDE.md` for the file-per-module map)
— if something here and the source disagree, the source wins.

## Dual-core split

`setup()` creates two additional FreeRTOS tasks pinned to **Core 0**,
leaving Arduino's own `loop()` running on **Core 1**:

- `uploadTaskFunc` (`upload.cpp`) — WiFi connect, the device-protocol
  session-upload cycle, claim attempts, and the periodic health-snapshot
  push. Everything that touches the WiFi/HTTP stack lives here so Core 1's
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
`hardware_platform` says in `config.txt` (kept in `Config` for on-disk
compatibility with the wider SailFrames Core fleet, which also has other
hardware platforms — see `README.md`).

## Device protocol: identity, claim, and uploads (`device_auth.{h,cpp}`, `upload.{h,cpp}`)

Implements xgsail's `docs/device-protocol.md` exactly — that document is
the source of truth for the wire shapes; this is how the E1 firmware
happens to drive them.

- **Identity** (`device_auth.h`'s `externalId()`): the ESP32's WiFi MAC
  address, stable across reboots, requiring no configuration.
- **Claim** (`claimDevice()`): `config.txt`'s `claim_code=` (read once at
  boot by `uploadTaskFunc` as soon as WiFi first connects) or the
  `claim <CODE>` console command call `POST /api/devices/claim/confirm`.
  On success, `device_id` + `device_api_key` persist to `/device.txt` — a
  firmware-owned file, parallel to `/boot.log`, never the user-edited
  `config.txt`. `isClaimed()` only requires a usable `device_api_key`;
  `device_id` is display metadata, not a precondition — the BLE relay's
  `provisioning` characteristic (below) only ever delivers the key.
- **Authenticated calls** (`apiRequest()`): every `/api/devices/me/...`
  call adds `Authorization: DeviceKey <key>`. Chooses `WiFiClientSecure`
  or `WiFiClient` based on `config.api_base_url`'s scheme.
- **Uploads** (`uploadFile()`, called once per pending SD file — nav/imu/
  wind/pres CSVs are independent files, uploaded independently, matching
  the backend's one-`session_upload`-row-per-file model): `POST
  /api/devices/me/session-uploads` with `is_final=true` (E1 never uses
  the incremental/chunked case), then a manually-chunked `PUT` of the raw
  bytes to the returned presigned `upload_url` (own chunk loop rather
  than `HTTPClient::sendRequest`, so the task watchdog gets fed every
  chunk on multi-minute transfers). A PUT failure `PATCH`es the upload as
  `"failed"`; success sends no `PATCH` at all — `is_final` was already
  true on the opening `POST`, and the object-storage `PUT` completing is
  what actually finalizes the data server-side.
- **Health** (`uploadHealthSnapshot()`): the 5-field `POST
  /api/devices/me/health` snapshot, once per boot.
- Sessions with only some sensors present (no wind sensor paired, no
  pressure chip detected) upload exactly the files that exist — nothing
  waits for or assumes all four CSVs.

## BLE relay (`ble_relay.{h,cpp}`)

A NimBLE **peripheral** GATT server implementing `docs/device-protocol.md`
§8: lets the owner's phone app relay the claim + session-upload calls
above over Bluetooth instead of WiFi. This is a first-class upload path,
not a WiFi-down fallback — `bleRelayInit()` runs unconditionally from
`setup()` and brings up BLE itself (`NimBLEDevice::init()`) if the
wind-sensor central role (`wind_sensor.cpp`) hasn't already, since a boat
with no wind sensor paired and no WiFi configured still needs a way in.
NimBLE-Arduino runs both roles concurrently (`sailframes_edge.ino`'s
`CONFIG_BT_NIMBLE_ROLE_*` — central for the Calypso wind sensor, peripheral
for this relay).

Each pending SD file (same file-driven pending list `upload.cpp` walks) is
one `session_manifest` entry, keyed by its SD path as the manifest's
`session_id` — the device never learns the backend's real
`session_id`/`session_upload_id`, since per the protocol it's the phone
app that calls `session-uploads`/`PUT`/`PATCH` itself; the device only
serves its own file inventory (`session_manifest`) and streams bytes on
request (`session_data`, notified in `bleRelayTick()` from the main
loop, sequence-indexed 4KB-ish chunks sized to the connection's actual
negotiated MTU). `control`'s `ack-uploaded` reuses `upload.cpp`'s own
`.uploaded` marker (`markUploaded()`) — a file relayed over BLE is never
re-offered by either upload path afterward. `provisioning` (the only
characteristic that can carry `device_api_key`) requires a bonded,
encrypted link (`NimBLEDevice::setSecurityAuth(true, true, true)`); so
does `device_config`'s write, for the same reason (it can carry a WiFi
password) — see below.

The same service also carries E1-specific extensions beyond
`docs/device-protocol.md` §8: a `device_config` characteristic for live
remote configuration (WiFi credentials, boat identity, recording
thresholds, RTK enable, ...), two extra `control` commands
(`calibrate`/`calibrate-reset`) for IMU calibration, and a read-only
`status` characteristic exposing live runtime state (battery, WiFi, GPS,
sensor presence, wind reading, recording activity) — the same fields the
console's `status` command prints, just as JSON. Full field-by-field
spec in `docs/ble-config.md` — not duplicated here. Every `device_config`
field applies live, no reboot, by calling the small refresh helper its
owning module already exposes (`applyUnitRole()`,
`applyRecordingThresholds()`, `forceWindReconnect()`, `gnssConfigure()`,
`boatIdHash()`) rather than requiring a fresh `loadConfig()` pass.

Because BLE callbacks run on the NimBLE host's own FreeRTOS task — a
third execution context beyond Core 1's `loop()` and Core 0's upload
task — every SD access reachable from any characteristic's callback
(not just the new ones) takes `sdMutex` with a short skip-if-busy
timeout, the same convention `storage.cpp`'s `appendBootLog()` and
`upload.cpp`'s `countPendingUploads()` already use.

## Console (`console.{h,cpp}`)

One command dispatcher (`processCommand()`) shared by USB-serial
(always on) and telnet (off by default — see `firmware/README.md` for
why, and the full command reference). Status/debug commands read state
directly out of the owning module (`gps`, `imu`, `battery`, `g_ocs`,
`g_mesh_peers`, ...) rather than duplicating it — if you add a command
that needs a new piece of state, that state belongs in its owning
module's header, not a new global in `console.cpp`.
