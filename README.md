# XGSail-E1

[![License](https://img.shields.io/github/license/FForzano/xgsail-e1)](LICENSE)

E1 device firmware and hardware for [XGSail](https://github.com/FForzano/xgsail) —
an ESP32 sailboat tracker: GNSS + IMU + wind + pressure logging, an
ESP-NOW peer mesh for live fleet position sharing and on-course-side
(OCS) race-start detection, and uploads over the XGSail device
protocol — direct over WiFi, or relayed over BLE by the owner's phone
when WiFi isn't available.

> **xgsail-e1 is derived from `sailframes/core`**, the upstream hardware/
> firmware project. It is not a GitHub fork of that repository (see
> "Relationship to SailFrames Core" below for why, and where this
> content actually came from) — it's a from-scratch export of the E1
> device's firmware and hardware, reorganized into per-responsibility
> modules and stripped down to E1 only.

## What xgsail-e1 Is

xgsail-e1 is the firmware and PCB design for one specific device: **E1**,
an ESP32-based fleet tracker.

It provides:

- Firmware (`firmware/sailframes_edge/`) — GNSS/IMU/wind/pressure sensor
  reads, SD-card session logging, an ESP-NOW peer mesh with on-course-side
  (OCS) race-start detection, the XGSail device-protocol claim/upload flow
  (direct over WiFi and a BLE GATT relay for when WiFi isn't available),
  and a serial/telnet console.
- Hardware (`hardware/`) — the KiCad 8+ schematic, PCB layout, Gerbers,
  and BOM for the E1 board.
- Documentation (`docs/`) — the firmware's internal architecture, GNSS/RTK
  configuration and PPK post-processing workflow, and the current battery/
  display/power hardware.

The device talks to the XGSail backend through the hardware-agnostic
ingestion contract documented in xgsail's `docs/device-protocol.md`
(device claim, `DeviceKey` auth, presigned session uploads, and the BLE
relay contract for WiFi-less operation) — this repo implements that
contract, it doesn't redefine it.

## What xgsail-e1 Is Not

xgsail-e1 is **not** the software platform — users, roles, clubs, the
ingestion API, analytics, or the web/native apps. That's
[XGSail](https://github.com/FForzano/xgsail).

It is also not the general SailFrames Core hardware project: other
device families from that upstream (a Raspberry-Pi-based "Edge-S" unit,
a sealed-enclosure successor board, and a standalone BLE wind-sensor
accessory) are out of scope here on purpose — this repository is E1 only.

## Relationship to SailFrames Core

XGSail's own `xgsail` repository is itself a GitHub-level fork of
`sailframes/core`, the upstream hardware+software monorepo — before its
software layer was redesigned into what is now XGSail, that repository's
history contained `edge-e/` (the E1 device: this firmware + hardware),
`edge-b/` (a later, different hardware platform), and `edge-s/` (the
Raspberry-Pi device), alongside the cloud-side code that became XGSail's
backend/frontend/workers.

xgsail-e1 is an export of `edge-e/` from that history: the firmware and
hardware for E1 specifically, with the other device families and the
now-superseded cloud-side code left out, and the firmware's `#ifdef`s for
the non-E1 hardware variant removed rather than carried forward as dead
code. It is not a `git fork` of `sailframes/core` (GitHub only allows one
fork per repo per account, and that slot is already used by `xgsail`) —
its origin is recorded here in `docs/firmware-architecture.md` and this
README instead of in GitHub's fork metadata.

License: Apache 2.0, same as upstream and same as xgsail — no relicensing.

## Display

3.5" TFT dashboard, three nav layouts cycled live (console or BLE), plus
two race-committee-only panels for fleet OCS detection:

<img src="docs/images/display-modes/d2.png" width="220" alt="D2 nav display">
<img src="docs/images/display-modes/rc-fleet-ocs.png" width="220" alt="RC in-race OCS panel">
<br>D2 — nav + wind (default) &nbsp;·&nbsp; RC panel — live OCS per boat

See `docs/hardware.md`'s Display section for all three nav modes and
both RC panels.

## Quick start

### Firmware

```bash
cd firmware/sailframes_edge
# Open in Arduino IDE, or build with PlatformIO
```

See `firmware/README.md` for board settings, library dependencies, pin
assignments, SD card setup, and the full serial/telnet console command
reference.

### Hardware (KiCad)

```bash
cd hardware
kicad kicad_sailframes-e1.kicad_pro
```

## Repository layout

`firmware/sailframes_edge/` (main sketch, one file/pair per
responsibility — see `CLAUDE.md` for the full module map) ·
`firmware/{e1_sd_card,i2c_scan,sailframes_e1_diag}/` (sample SD contents
and standalone bring-up sketches) · `hardware/` (KiCad project + fab
outputs) · `docs/{firmware-architecture,gnss-rtk,hardware}.md`.

## License

Apache 2.0.
