# XGSail-E1 — E1 Device Firmware

## Project Context for Claude Code

This repository is **xgsail-e1**: the E1 device firmware and hardware
only — an ESP32 sailboat-tracker sketch plus its KiCad PCB design. It is
**not** the software platform (users, roles, ingestion API, analytics,
web/native apps) — that lives in the sibling repository **xgsail**.

This content derives from `sailframes/core`, the upstream hardware/
firmware project — see "Relationship to SailFrames Core" in `README.md`
for how it got here and what was deliberately left out (other device
families, retired PPK capture, non-E1 accessories).

xgsail-e1 implements the ingestion contract documented in xgsail's
`docs/device-protocol.md` (claim flow, `DeviceKey` auth, session-upload
API) — that document is the source of truth for anything upload/
ingestion-related in this firmware; don't duplicate it here.

---

## Project Overview

- **License:** Apache 2.0
- **Hardware:** ESP32 DevKit V1, Waveshare LG290P GNSS, BNO085 IMU, DPS310
  pressure sensor, Hosyond 3.5" ST7796U TFT, MicroSD, Calypso Mini BLE wind
  sensor. See `hardware/` for the KiCad PCB project and `firmware/README.md`
  for the full bring-up/pin/build reference.
- **Build:** Arduino IDE or PlatformIO against `firmware/sailframes_edge/`
  (see `firmware/README.md` for board/library settings).
- **Status:** ported from `sailframes/core` (via xgsail's own fork history)
  and reorganized into per-responsibility modules; behavior is
  byte-equivalent to the source firmware with the B1/successor-hardware
  code paths removed (this repo is E1-only, see `docs/firmware-architecture.md`).

---

## Code Style Guidelines

Same principles as xgsail, applied to firmware idioms rather than a web
backend/frontend:

- **Simple and readable over clever.** Optimize for the next person
  reading the code (or debugging it over a telnet console at a regatta),
  not for fewest lines.
- **Isolate responsibilities.** Each module owns one subsystem's state and
  behavior — `gnss.{h,cpp}` doesn't know about S3 upload, `display.{h,cpp}`
  doesn't parse NMEA. `sailframes_edge.ino` stays a thin composition root:
  global object wiring, `setup()`, `loop()` — business logic (sensor
  reads, mesh/OCS logic, upload, console commands) lives in the modules
  below, not inline in the sketch.
- **Reuse before writing.** Before adding a new global or helper, check
  whether the owning module already exposes what you need via its `.h`
  (e.g. GNSS state lives in `gnss.h`'s `gps` struct — don't add a second
  copy of position/fix-quality state elsewhere). Prefer extending an
  existing module over bypassing it with a new cross-cutting global.
- **No duplicated logic.** If two modules need the same computation
  (e.g. FNV-1a boat-ID hashing, ISO-timestamp formatting), it lives once
  in the module that owns the underlying data (`mesh.h`'s `boatIdHash`,
  `gnss.h`'s `formatGpsIso`) and is included, not copy-pasted.
- **This is a standing rule, not a one-time cleanup.** The module split
  in this repo is the result of applying this rule to what was previously
  a single 8000+ line `.ino`; if you find a new piece of logic drifting
  back into the composition root or duplicated across modules, refactor
  it into the right module rather than leaving it or copying it again.

---

## Repository Structure

```
xgsail-e1/
├── CLAUDE.md              # This file
├── README.md              # Project scope: what xgsail-e1 is / isn't
├── LICENSE                # Apache 2.0
├── docs/
│   ├── firmware-architecture.md  # setup/loop, dual-core split, mesh, OCS, cloud config
│   ├── gnss-rtk.md               # LG290P config, RTCM3/MSM, RTK, PPK post-processing
│   └── hardware.md               # Power/battery, display, GPS-triggered recording
├── firmware/
│   ├── README.md          # Build/flash, wiring, console commands, S3 upload/OTA
│   ├── config.txt         # Sample on-SD-card config (WiFi, boat_id, thresholds)
│   ├── e1_sd_card/        # Sample imu_cal.txt / wind_mac.txt
│   ├── i2c_scan/          # Standalone I2C bus scanner (bring-up/debug)
│   ├── sailframes_e1_diag/  # Standalone board bring-up diagnostic sketch
│   └── sailframes_edge/   # Main sketch — one file/pair per responsibility:
│       ├── sailframes_edge.ino   # Composition root: setup()/loop() only
│       ├── config.h/.cpp         # Pins, firmware-wide constants, Config struct + loader
│       ├── v2_types.h/.cpp       # Shared HardwarePlatform/UnitRole/RadioMode vocabulary
│       ├── gnss.h/.cpp           # NMEA parsing, PQTM/LG290P config, GNSS read
│       ├── rtk_relay.h/.cpp      # RTCM3 framer/reassembler + ESP-NOW relay glue
│       ├── mesh.h/.cpp           # ESP-NOW peer mesh (wire types + state + glue)
│       ├── imu.h/.cpp            # BNO085 read + on-SD calibration
│       ├── wind_sensor.h/.cpp    # Calypso Mini BLE client
│       ├── pressure.h/.cpp       # DPS310 read + gust-detection min/max
│       ├── battery.h/.cpp        # LiPo voltage monitoring + low-battery halt
│       ├── recording.h/.cpp      # GPS-speed-triggered recording state machine
│       ├── ocs.h/.cpp            # On-course-side detection + RC fleet aggregation
│       ├── display.h/.cpp        # TFT dashboard rendering (D1/D2/D3 + RC panels)
│       ├── storage.h/.cpp        # SD session logging + /boot.log
│       ├── ota.h/.cpp            # Telnet transport + manifest-pull firmware update
│       ├── console.h/.cpp        # Serial/telnet interactive command dispatcher
│       ├── cloud_config.h/.cpp   # Cloud config-sync fetch/verify/apply
│       ├── upload.h/.cpp         # WiFi connect + S3 upload (Core-0 task) + diagnostics
│       └── shared_state.h/.cpp   # Cross-cutting: hang-watchdog breadcrumbs, WiFi-busy gate, SD mutex
└── hardware/              # KiCad 8+ project: schematic, PCB, Gerbers, BOM
```

Arduino compiles every `.h`/`.cpp` in the sketch directory as one build —
the module split above is a compilation-unit-per-responsibility layout,
not a change to how the firmware is built or flashed.

---

## Firmware Data Flow

```
[Sensors: GNSS UART2, IMU/pressure I2C, wind sensor BLE]
  → per-module read (gnss.cpp/imu.cpp/pressure.cpp/wind_sensor.cpp)
  → session CSVs on SD (storage.cpp), 10 Hz nav+IMU / 1 Hz wind / 0.1 Hz pressure

[ESP-NOW peer mesh] (mesh.cpp)
  → boat-state broadcast at 2 Hz; OCS module (ocs.cpp) computes over-line
    state locally and, for the RC unit, across the whole fleet

[Upload] (upload.cpp, Core-0 FreeRTOS task)
  → once stationary + WiFi in range: session CSVs → presigned S3 PUT,
    matching xgsail's docs/device-protocol.md ingestion contract
  → fleet-health snapshot + /boot.log tail, once per boot
  → manifest-pull firmware update (ota.cpp) and cloud config sync
    (cloud_config.cpp) piggyback on the same WiFi window
```

See `docs/firmware-architecture.md` for the mesh/OCS/cloud-config design
in more detail, and xgsail's `docs/device-protocol.md` for the claim flow
and upload API this firmware talks to.
