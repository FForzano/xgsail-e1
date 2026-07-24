# BLE device configuration

An E1-specific extension to the BLE GATT relay in `firmware/sailframes_edge/ble_relay.cpp`,
letting a companion app read and write the device's configurable
parameters — WiFi credentials, boat identity, recording thresholds, and
so on — without pulling the SD card, plus trigger IMU calibration.

This is **not** part of xgsail's `docs/device-protocol.md` §8 (that
document is the ingestion contract — claim, session uploads, health —
owned by the `xgsail` repo). This lives on the *same* BLE service so an
app only needs one bonded connection, but the characteristic and
commands below are firmware-specific and documented here instead.

If you're building the app-side integration, this document plus
`firmware/sailframes_edge/ble_relay.cpp` (the `device_config`
characteristic and `control`'s `calibrate`/`calibrate-reset` commands)
are the full spec.

---

## Service

Same service as the device-protocol relay:
`24e6db2c-3c8a-4b5b-ba5a-23bc4c818046`.

## Pairing window (first-time bonding)

The device's BLE bonding flag is **off by default** — a phone that
isn't already a recognized bond can still connect and read plain
characteristics, but a write to `provisioning` (`docs/device-protocol.md`
§8.2, carries `device_api_key`) or `device_config` (can carry a WiFi
password) is rejected with `{"status":"error","reason":"pairing_window_closed"}`
unless the connection is already bonded from an earlier pairing.

A long press (`button.h`, `BUTTON_LONG_PRESS_MS` in `config.h`, 2s) on the
device's physical button opens a pairing window
(`BLE_BOND_WINDOW_MS`, 60s) during which bonding is enabled and a new
phone's write to either characteristic is allowed and, on success,
persists as a bond — that phone reconnects and writes normally forever
after, with no further button presses. There's no in-band way to open
the window over BLE itself: physical access to the boat is the point.

## `control` characteristic — start/stop recording

```json
{ "cmd": "start-rec" }
```
```json
{ "cmd": "start-rec", "boat_id": "<uuid>", "activity_id": "<uuid>" }
```
```json
{ "cmd": "stop-rec" }
```

Same entry point as the physical button's short press and the console's
`rec`/`stoprec` commands (`recording.h`'s `startRecording()`/
`stopRecording()`) — recording is button/app-triggered only, there is no
GPS-speed auto-start. Neither command requires bonding (same as
`calibrate`/`calibrate-reset` below — nothing here is a secret; `boat_id`/
`activity_id` are backend record identifiers, not credentials). Notifies
back on `control`:

```json
{ "cmd": "start-rec", "ok": true, "logging": true }
```

`ok` is `false` if the command was a no-op (already recording / already
stopped, or no SD card for `start-rec`).

**`boat_id`/`activity_id` are both optional**, and independent of each
other — this is "start a session, optionally naming the boat and/or
activity it belongs to", not "pick a boat":

- Omit both (or send bare `{"cmd":"start-rec"}`) and the session gets
  the device's own boat (`device.owner_boat_id`, set at claim time) and
  a fresh private "solo" activity — today's only behavior, unchanged.
- `activity_id` — an existing XGSail activity (a race, a training
  session, ...) to fold this session into, instead of a new solo one.
  This is the field worth exposing prominently in an app's "start
  recording" flow (docs/device-protocol.md §4.4's `session-uploads`
  `activity_id` — same field, same semantics).
- `boat_id` — overrides which XGSail boat the session is filed under
  (**not** the same thing as `device_config`'s `boat_id` field, which is
  only the ESP-NOW mesh identity / log-filename prefix — see
  `docs/firmware-architecture.md`'s note on `boatIdHash()`). Only
  useful for a device that sails on more than one boat; most apps can
  leave this out entirely.

Neither value is validated by the firmware — an unrecognized UUID simply
fails the `session-uploads` call server-side once this session's files
try to upload. The device writes whichever were given to
`boat_id.txt`/`activity_id.txt` markers in the session's SD folder
(`storage.cpp`'s `startLogging()`) and echoes them back in the BLE
relay's `session_manifest` entries too (an addition beyond
`docs/device-protocol.md` §8.2's manifest shape) so an app relaying the
`session-uploads` call itself later knows what was originally intended.

## `control` characteristic — manual OTA update

```json
{ "cmd": "ota-update" }
```

Triggers an immediate firmware update check-and-apply (`docs/ota.md`),
regardless of `device_config`'s `ota_auto_update` setting. `ota-check` is
accepted as an alias. Like `start-rec`/`stop-rec`/`calibrate` above, this
doesn't require bonding — it doesn't touch persisted config, just kicks off a
check. Notifies back immediately:

```json
{ "cmd": "ota-update", "ok": true, "message": "ota update requested" }
```

`ok` is `false` only if the device refused outright — currently just
`"recording in progress"`. This ack does **not** report the update's outcome:
a successful update reboots the device, so there's nothing to notify back on
`control`. Poll the `status` characteristic's `ota` object (below) for
progress and result instead.

## `device_config` characteristic

| | |
|---|---|
| UUID | `042dfd7c-88f4-4ae8-af9a-eb1d7be7a3c6` |
| Properties | `read`, `write` (bonded + encrypted), `notify` |

**Security**: writing requires an encrypted link, and — unless this
connection is already a recognized bond from an earlier pairing —
requires the pairing window to be open (see above), same as
`provisioning` in `docs/device-protocol.md` §8.2, since a write can carry
a WiFi password. Reading is plain (no bonding required): nothing this characteristic
returns is sensitive, because **`wifi[].pass` is never included in a
read response** — it's write-only, always sent back as `""` regardless
of connection state. Set a password, but don't expect to read it back.

### Read: current configuration

**`boat_id` here is not an XGSail boat.** It's the mesh-identity/
log-filename-prefix/splash-screen label (≤15 chars, e.g. `"E1"`) —
`config.txt`'s `boat_id`, hashed into the ESP-NOW sender id
(`docs/firmware-architecture.md`). The XGSail boat a session uploads
under is `device.owner_boat_id` (set once, at claim time, server-side —
not writable here) unless overridden per-session via `control`'s
`start-rec` `boat_id` above. An app UI editing this field should label
it something like "device name on the mesh", not "boat", to avoid
implying it changes which XGSail boat the device's data belongs to.

```json
{
  "boat_id": "E1",
  "unit_role": "racing_boat",
  "api_base_url": "https://xgsail.example.com",
  "wind_mac": "AA:BB:CC:DD:EE:FF",
  "wind_offset": 0,
  "start_speed_knots": 1.5,
  "stop_speed_knots": 0.5,
  "start_delay_sec": 10,
  "stop_delay_sec": 180,
  "rtk_enabled": false,
  "auto_cleanup_uploads": true,
  "ota_auto_update": false,
  "ota_base_url": "https://xgsail.com/ota",
  "display_mode": 2,
  "wifi": [
    { "ssid": "YourHomeNetwork", "pass": "" },
    { "ssid": "YachtClubNetwork", "pass": "" }
  ]
}
```

### Write: partial update

Only the keys you include are changed — omit anything you don't want to
touch. `wifi`, if present, **replaces the entire array** (not a
per-slot merge): send every network you want kept, up to 5.

```json
{
  "wifi": [
    { "ssid": "NewHomeNetwork", "pass": "newpassword" }
  ],
  "boat_id": "E7",
  "start_speed_knots": 2.0
}
```

Response, notified back on the same characteristic:

```json
{ "status": "ok" }
```

or, on failure:

```json
{ "status": "error", "reason": "bad_json" }
```

(`reason` is one of `pairing_window_closed` — this connection isn't
already bonded and no long-press has opened the pairing window (see
above) — `bad_json` — the write body didn't parse as JSON — or
`sd_busy` — the SD card was in use by the recording/upload path at
the moment of the write; safe to retry).

### Field reference and live-apply behavior

Every field below takes effect **immediately, without a reboot** — that
was a deliberate design goal, not a limitation to work around. How each
one applies:

| Field | Type | Applies |
|---|---|---|
| `wifi` | array of `{ssid, pass}`, ≤5 | Immediately — the next connection attempt reads it fresh. |
| `api_base_url` | string | Immediately — the next device-protocol API call reads it fresh. |
| `boat_id` | string, ≤15 chars | Immediately — also recomputes the ESP-NOW mesh identity, so the boat doesn't keep broadcasting under its old name. |
| `unit_role` | string: `racing_boat`, `rc_signal`, `rc_pin`, `mark`, `committee_chase`, `spare` | Immediately. |
| `wind_mac` | string, Calypso MAC (`AA:BB:CC:DD:EE:FF`) or `""` | Immediately — an empty string disables the wind sensor; any other value enables it and forces a reconnect to the new device on the next cycle. |
| `wind_offset` | int, degrees | Immediately. |
| `start_speed_knots` | float, knots | Immediately — but only fed into upload.cpp's "boat is moving, don't upload right now" check. Recording itself is button/console/BLE-triggered (`control`'s `start-rec`/`stop-rec` above), not speed-triggered. |
| `stop_speed_knots` / `start_delay_sec` / `stop_delay_sec` | float / int / int | Round-tripped for older cards' `config.txt` compatibility — **unused by the firmware**. Don't build UI around them. |
| `rtk_enabled` | bool | Immediately — reconfigures the GNSS module (base/rover RTK) right away, the same live path the console's `gpscfg` command already uses. |
| `auto_cleanup_uploads` | bool | Immediately — the next successful upload (WiFi or BLE relay) reads it fresh before deciding whether to delete the file. Default `true`. |
| `ota_auto_update` | bool | Persisted opt-in for automatic OTA firmware update checks (`docs/ota.md`). Default `false`. A manual update can still be triggered with `control`'s `ota-update` (above) even when this is off. Never runs while recording either way. |
| `ota_base_url` | string, URL | Persisted. Base URL of the OTA firmware feed (`<url>/manifest.json`). Default `https://xgsail.com/ota` — only override for staging/self-hosted. Read fresh on the next OTA check. |
| `display_mode` | int, `1`\|`2`\|`3` | Immediately — same effect as cycling the console's `display` command: `1` = D1 (simple, large numbers), `2` = D2 (Vakaros-style nav + wind, default), `3` = D3 (wind focus). Out-of-range values are ignored (rest of the write still applies). Persisted, so it survives a reboot. |

**Not configurable here, and why:**
- `claim_code` — irrelevant to this characteristic. BLE claiming already
  happens through `provisioning` (`docs/device-protocol.md` §8.3): the
  app calls `claim/confirm` itself and relays the resulting
  `device_api_key` directly, so there's never a raw claim code for the
  device to consume over BLE.
- `hardware_platform` — fixed to `"e1"` in this firmware; not a real
  setting.

### Persistence and `config.txt`

Every write rewrites `/config.txt` in full, from the device's current
in-memory configuration. This keeps the SD card consistent with
whatever was last set over BLE (or by hand before the first BLE write),
but **any hand-written comments or custom formatting in `config.txt` are
lost the first time a BLE write happens** — the file that comes back out
is plain `key=value` lines, nothing else. If you rely on a heavily
annotated `config.txt`, keep your own copy of it; the device will no
longer preserve one.

---

## `control` characteristic — calibration commands

Two additional commands on the existing `control` characteristic
(`ec88dd3e-2562-420c-aebe-30a4ae40bdf9`, `docs/device-protocol.md` §8.2),
alongside its documented `start-transfer`/`ack-uploaded`. `control` now
also has `notify` (an addition beyond §8.2 — purely additive, a client
that never subscribes sees no difference for the two existing commands).

```json
{ "cmd": "calibrate" }
```

Zeroes heel/pitch at the boat's **current** attitude — same as the
console's `cal` command. Only meaningful with the boat sitting level;
the app is responsible for telling the user to do that first.

```json
{ "cmd": "calibrate-reset" }
```

Resets heel/pitch offsets back to zero (undoes any prior calibration) —
same as the console's `calreset` command.

Both notify a status back on `control`:

```json
{ "cmd": "calibrate", "status": "ok", "heel_offset": 2.3, "pitch_offset": -0.5 }
```

```json
{ "cmd": "calibrate", "status": "error", "reason": "no_imu" }
```

(`reason` is `no_imu` — the BNO085 wasn't detected at boot — or
`sd_busy`, safe to retry.)

---

## `status` characteristic (read-only)

Live runtime state — distinct from `device_config` (how the device is
*set up*) and `identity` (its stable identity). Read on demand; there is
no periodic push. Same fields the console's `status` command already
prints, just as JSON instead of a serial dump — nothing here is
computed specially for BLE.

| | |
|---|---|
| UUID | `bfef7865-f3f7-486c-93fe-bbae78cfdc43` |
| Properties | `read` only — no write, no notify |

No bonding required: nothing in the payload is a secret (your own
boat's GPS position and WiFi SSID, not a password).

```json
{
  "claimed": true,
  "firmware_version": "2026.06.29.02",
  "uptime_s": 5423,
  "heap_free": 142300,
  "battery": { "pct": 78, "v": 3.91, "critical": false },
  "sd_ok": true,
  "wifi": { "connected": true, "ssid": "YachtClub", "ip": "192.168.1.42" },
  "sensors": { "imu": true, "pressure": true, "wind": true },
  "gps": { "fix": true, "satellites": 9, "hdop": 1.2, "lat": 42.36012, "lon": -71.05821, "speed_kts": 5.2, "course": 210 },
  "wind": { "connected": true, "speed_kts": 12.1, "angle_deg": 45, "battery": 88 },
  "recording": { "logging": true, "session_count": 3, "pending_uploads": 1, "elapsed_s": 842 },
  "ota": { "state": "downloading", "progress": 42 }
}
```

Field notes:
- `firmware_version` — the running build, `YYYY.MM.DD.N` (date + daily
  build number). Always present. It's the basis for OTA's "is a newer build
  available?" decision (`docs/ota.md`), and the same value the device reports
  to the backend in its health snapshot.
- `ota` — state of the last automatic or manual (`control`'s `ota-update`)
  update check (`docs/ota.md`). `state` is one of `idle` / `checking` /
  `up_to_date` / `downloading` / `applying` / `suspended` / `error`.
  `progress` (0-100) is only present while `downloading`. `message` is a
  short detail, present on `error`/`suspended`.
- `wifi.ssid`/`wifi.ip` are only present when `wifi.connected` is `true`.
- `sensors` reports **presence**, not live readings — whether the IMU/
  pressure chip were detected at boot, and whether a wind sensor is
  currently paired (`config.wind_enabled`). Wind's live reading is the
  separate `wind` object below.
- `gps.fix` is `false` and the rest of `gps` is stale/zeroed until the
  GNSS module gets its first fix.
- `wind.speed_kts`/`wind.angle_deg`/`wind.battery` are only present when
  `wind.connected` is `true` (no wind sensor paired, or not currently
  connected, means just `{"connected": false}`).
- `recording.pending_uploads` counts sessions with files still to
  upload, over WiFi or BLE relay — same count the `status` console
  command and the device's own display show.
- `recording.elapsed_s` — seconds since the current session started
  (`storage.cpp`'s `logStart`). Only present while `logging` is `true`;
  omitted rather than stale when not recording.

## Concurrency note (for firmware maintainers, not app authors)

Every SD-card access reachable from a BLE callback — `device_config`'s
read/write, `control`'s calibration commands, and the existing
`provisioning`/`session_manifest`/`session_data` handlers — takes the
firmware's `sdMutex` (skip-if-busy, not a hard block) before touching the
SD card. BLE callbacks run on the NimBLE host's own FreeRTOS task, a
third execution context distinct from Core 1's sensor/display loop and
Core 0's upload task; without this, a BLE-triggered write could race
against either. See `ble_relay.cpp`'s `BLE_SD_MUTEX_TIMEOUT` and its
usages for the exact pattern. `status` is the one exception — it only
reads already-maintained in-memory globals, never the SD card, so its
`onRead` doesn't take `sdMutex`.
