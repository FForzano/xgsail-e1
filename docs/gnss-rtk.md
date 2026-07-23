# GNSS / RTK reference

How E1 configures and uses its GNSS module (Waveshare LG290P), and how
its real-time-kinematic (RTK) correction relay actually works today. All
commands and behavior below are taken from `firmware/sailframes_edge/
gnss.cpp` and `rtk_relay.{h,cpp}` — this is what the shipped firmware
does, not a design proposal.

## Module and wiring

- **Waveshare LG290P** GNSS receiver, ESP32 UART2 at 460800 baud
  (`GPS_RX_PIN`/`GPS_TX_PIN` = GPIO16/17, `GPS_BAUD` in `config.h`).
- Configuration happens entirely over that UART using **PQTM** commands
  (`sendPQTM()` in `gnss.cpp` appends the NMEA-style checksum and reads
  back the module's response).

## Rover configuration (the default, RTK off)

`configureLG290P()` runs at boot and on the `gpscfg` console command. It:

1. Queries firmware version (`PQTMVERNO`, for `/boot.log` forensics).
2. Sets **Rover mode** — `PQTMCFGRCVRMODE,W,1` — required to unlock the
   10 Hz fix rate (Base mode locks the module to 1 Hz).
3. Enables GGA/RMC/GSA/GSV NMEA sentences at the current rate
   (`PQTMCFGMSGRATE,W,<sentence>,1` for each).
4. Sets the fix rate to 10 Hz — `PQTMCFGFIXRATE,W,100` (100 ms) —
   *before* the save+restart below, so the new rate is in NVM and applied
   by the same-boot restart.
5. Saves to NVM and restarts the module (`PQTMSAVEPAR` then `PQTMSRR`,
   ~6 s wait), then reads back `PQTMCFGRCVRMODE,R` / `PQTMCFGFIXRATE,R`
   to confirm.

This NMEA-only path was the entire GNSS config for a long time (see "PPK
capture — retired" below for what used to run alongside it). It is
**byte-identical** whether or not RTK is compiled in — `gnssConfigure()`
falls straight through to `configureLG290P()` when `config.rtk_enabled`
is false, which is the default.

## RTK relative mode

When `config.rtk_enabled` is true, `gnssConfigure()` instead calls one of
two role-specific configurators:

- **Rover** (`lg290pConfigRover()`): runs `configureLG290P()` unchanged,
  then enables RTK relative mode — `PQTMCFGRTK,W,1,2,120` (auto diff
  mode, relative mode, 120 s max correction age) — and turns on the
  accuracy-output sentence: `PQTMCFGMSGRATE,W,GST,1`. GST gives
  1-sigma lat/lon/alt error in metres; `parseNMEA()` combines
  `sqrt(lat_std² + lon_std²)` into `gps.hacc_m`, the single
  "how accurate is this fix" number the rest of the firmware reads (mesh
  broadcast, RC panels, nav display).
- **Base** (`lg290pConfigBase()`, used by the `rc_signal` role): sets
  Base mode (`PQTMCFGRCVRMODE,W,2`, locks 1 Hz), a short/loose survey-in
  (`PQTMCFGSVIN,W,1,60,0,0,0,0`), saves + restarts, then — **every
  boot, never saved** — requests RTCM3 output:
  `PQTMCFGRTCM,W,7,0,-90,07,06,1,0` (MSM7 requested, elevation mask off,
  GPS+GLONASS+Galileo, L1+L2, ephemeris on-update). **The module actually
  emits MSM4** (1074/1084/1094/1124) even though MSM7 was requested —
  this is a known LG290P firmware quirk, not a bug in this configuration;
  MSM4 is sufficient for centimeter-level positioning. Because this
  command doesn't persist, it's resent on every boot rather than saved —
  do not add a `PQTMSAVEPAR`/`PQTMSRR` after it. Base mode also
  auto-disables NMEA, so GGA/RMC/GSA/GST are re-enabled after the RTCM3
  command so the RC unit still gets its own position/HDOP for the
  pre-race panel.

## RTK correction relay (ESP-NOW, not a cable)

There's no direct RTK radio link between boats — corrections ride the
same ESP-NOW mesh used for boat-state broadcasts (see
`docs/firmware-architecture.md`):

- **Base** (`readGPSBase()`, `rtk_relay.h`'s `RtcmFramer`): demuxes the
  Base's UART stream — length-driven, not `$`-keyed, since a `0x24` byte
  can appear inside a binary RTCM3 payload — into complete, CRC-24Q
  validated frames. Each valid frame is handed to
  `rtcmBroadcastFrame()`, which splits it into up to 5 fragments
  (`RTCM_FRAG_MAX` = 230 B, capped by the 250 B ESP-NOW packet limit) and
  sends each one twice for loss margin.
- **Rover** (`RtcmReassembler` in `rtk_relay.h`): reassembles fragments
  by `msg_id`, with a two-level dedup (in-flight `got_mask` per fragment
  index, plus a `last_done_msg_id` guard against the base's own 2×
  retransmission). A complete, CRC-valid frame goes into a small
  FreeRTOS stream-buffer ring (`g_rtcmRing`) — the only object that
  crosses the ESP-NOW receive-callback / main-loop boundary — and
  `loop()` drains it straight to the GNSS UART a few bytes at a time,
  non-blocking.
- Both directions are gated off during an active upload (`wifiBusy` /
  `uploading`) to avoid RF contention with HTTP traffic — a lesson from
  earlier fleet hangs, not a design choice made lightly (see the "gotcha"
  comments in `mesh.cpp`/`rtk_relay.cpp` for the specific incidents).

RTK is **off by default** (`config.rtk_enabled = false`) and is SD-config
only — deliberately not part of the cloud config allow-list, since
flipping it reconfigures the GNSS module, a physical bring-up act rather
than something safe to push remotely.

## PPK post-processing — retired, kept for history

An earlier firmware generation logged raw RTCM3 to the SD card
(`*_raw.rtcm3` per session) for **offline** post-processed kinematic
(PPK) positioning: upload the file, pull the matching hour(s) of RINEX
observations from a CORS base station, and run RTKLIB's `rnx2rtkp`
locally for centimeter-level positions after the fact. That capture path
was retired in firmware `2026.05.20.09` in favor of the live RTK relay
above — 10 Hz real-time fixes matter more for on-the-water OCS detection
than decimeter-accurate positions available only after the race.

If you ever resurrect offline PPK from archived `.rtcm3` files, the one
non-obvious pitfall worth knowing about: matching RINEX 2 (from most CORS
stations) against RINEX 3 (from `convbin`/RTKCONV's own RTCM3 conversion)
observation codes isn't automatic in stock RTKLIB. RINEX 2's plain `C2`/
`L2` (P(Y)-code) doesn't get matched against RINEX 3's `C2X`/`L2X`
(L2C civilian) even when both sides are tracking compatible dual-frequency
signals, which silently drops dual-frequency observations and produces a
0% fix rate that looks like a signal-compatibility problem but isn't. The
[RTKExplorer demo5 fork](https://github.com/rtklibexplorer/RTKLIB) handles
mixed-version code matching; on stock RTKLIB, set `pos1-frequency=l1+l2`
and an explicit code priority (`pos1-codepri_gps_l2=XSLWPYMCQ`, preferring
L2C over P(Y)) in `ppk.conf`.
