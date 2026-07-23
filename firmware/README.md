# SailFrames E1 Firmware

ESP32 fleet-tracker firmware: GNSS + IMU + wind + pressure logging, an
ESP-NOW peer mesh for live fleet position sharing and OCS (on-course-side)
race-start detection, uploads over the XGSail device protocol (direct
WiFi, or a BLE relay via the owner's phone when WiFi isn't available),
and a serial/telnet console.

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | ESP32 DevKit V1 | — |
| GNSS | Waveshare LG290P | UART2, 460800 baud |
| IMU | BNO085 (Adafruit breakout, address 0x4B) | I2C |
| Pressure/temp | DPS310 (address 0x77) | I2C |
| Display | Hosyond 3.5" IPS ST7796U, 480x320 | SPI (VSPI) |
| Storage | MicroSD, standalone module | SPI (HSPI, separate bus from the display) |
| Wind sensor | Calypso Mini (BLE) | BLE (NimBLE) |
| Power | DWEII USB-C 5V boost converter + LiPo | 100K/100K divider on GPIO34 |
| Recording/pairing button | Momentary pushbutton, active-low | GPIO32, internal pull-up |

### Pin assignments (see `sailframes_edge/config.h`)

| ESP32 Pin | Function | Device |
|-----------|----------|--------|
| GPIO16 | UART2 RX | LG290P TX |
| GPIO17 | UART2 TX | LG290P RX |
| GPIO21 | I2C SDA | BNO085 + DPS310 |
| GPIO22 | I2C SCL | BNO085 + DPS310 |
| GPIO5 | SPI CS (VSPI) | TFT |
| GPIO2 | Data/Command | TFT |
| GPIO4 | Reset | TFT |
| GPIO25 | Backlight (PWM) | TFT |
| GPIO27 | SPI CS (HSPI) | SD card |
| GPIO14 | SPI CLK (HSPI) | SD card |
| GPIO35 | SPI MISO (HSPI) | SD card |
| GPIO13 | SPI MOSI (HSPI) | SD card |
| GPIO34 | ADC (battery divider) | LiPo voltage sense |
| GPIO32 | Digital in, pull-up (active-low) | Recording/pairing pushbutton |

The display and SD card are deliberately on separate SPI buses (VSPI vs.
HSPI) — sharing one bus caused visible flicker during SD writes.

## Building and flashing

Arduino IDE or PlatformIO, sketch directory `sailframes_edge/`:

- **Board:** ESP32 Dev Module
- **Upload speed:** 921600
- **Flash frequency:** 80 MHz
- **Partition scheme:** Default 4MB with spiffs

### Libraries (Arduino Library Manager)

- **TFT_eSPI** — display driver (configured via `User_Setup.h` in this
  sketch directory, loaded automatically ahead of the library's own config)
- **Adafruit BNO08x** — IMU driver (SHTP protocol)
- **Adafruit DPS310** — pressure/temperature driver
- **NimBLE-Arduino** (pinned to 2.4.0 — see `docs/firmware-architecture.md`
  for why the version is pinned) — Calypso wind sensor BLE client
  (central role) and the device-protocol BLE relay GATT server
  (peripheral role), running concurrently
- **ArduinoJson** (7.x) — JSON building/parsing for the device-protocol
  HTTP calls (claim, session-uploads, health) and the BLE relay's
  characteristic payloads

ESP-NOW, WiFi, and SD support come from the ESP32 Arduino core (pinned to
3.3.7 — also documented in `docs/firmware-architecture.md`).

### Prebuilt binaries

`.github/workflows/firmware-release.yml` builds this sketch with
`arduino-cli` (same board settings and pinned versions as above) on every
`v*` tag, and on demand from the Actions tab. It attaches to the release:

- `...-merged.bin` — bootloader + partition table + app in one image,
  flashed at offset `0x0`
- `...-app.bin` / `...-bootloader.bin` / `...-partitions.bin` — the
  individual images, for flashing the app alone at `0x10000`
- `SHA256SUMS.txt`

Flashing a downloaded build needs only esptool (`pip install esptool`),
no toolchain:

```
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 \
  write_flash 0x0 sailframes_edge-<version>-merged.bin
```

Flashing replaces the firmware only — `config.txt`, `device.txt` and the
recorded sessions live on the SD card and are untouched.

## Repository layout

```
sailframes_edge/       # Main sketch — see CLAUDE.md for the per-file module map
e1_sd_card/             # Sample imu_cal.txt / wind_mac.txt for the SD card
i2c_scan/               # Standalone I2C bus scanner sketch (bring-up/debug)
sailframes_e1_diag/     # Standalone board-bring-up diagnostic sketch
```

## SD card setup

1. Format FAT32 (32GB or less recommended).
2. Copy `sailframes_edge/config.txt` to the card's root and edit it:
   WiFi networks (`wifi1_ssid`/`wifi1_pass`, ...), `boat_id`, recording
   speed thresholds, `unit_role` (`racing_boat`, `rc_signal`, ...),
   `api_base_url` (the XGSail backend this device talks to), and
   `claim_code` (see "Claiming the device" below).
3. A `/wind_mac.txt` file (Calypso Mini MAC address) enables the wind
   sensor and speeds up reconnection — see `e1_sd_card/wind_mac.txt` for
   the format. Without it, the wind sensor is disabled.
4. `/imu_cal.txt` stores the heel/pitch zero-offsets saved by the `cal`
   console command — see `e1_sd_card/imu_cal.txt`.
5. `/device.txt` is firmware-owned (like `/boot.log`) — it holds the
   `device_id`/`device_api_key` written by a successful claim. Don't
   create or edit it by hand.

## Log files

Per session, under `/sf/<session-folder>/` (GPS-datetime named when a fix
is available, `session_NNN` otherwise):

```
<boat_id>_<date>_<time>_nav.csv    # 10 Hz: lat, lon, speed, course, sat, hdop, fix, hacc
<boat_id>_<date>_<time>_imu.csv    # 10 Hz: accel, gyro, linear accel, mag, heel, pitch, heading
<boat_id>_<date>_<time>_wind.csv   # 1 Hz, only when a wind sensor is paired
<boat_id>_<date>_<time>_pres.csv   # 0.1 Hz: pressure, temperature, min/max (gust window)
```

A recording session starts and stops manually — a short press of the
device's button, the console's `rec`/`stoprec` commands, or the BLE
relay's `start-rec`/`stop-rec` (`docs/ble-config.md`) — or ends on a
clean power-off. There is no GPS-speed auto-start/stop (see
`docs/hardware.md`'s "Button-triggered recording").

## Claiming the device

A device cannot upload anything until it's claimed (xgsail's
`docs/device-protocol.md` §2) — there is no auto-registration on first
upload. From the XGSail app, create a claim for this device to get a
one-time `claim_code`, then either:

- write it into `config.txt` as `claim_code=XXXXXXXX` before boot (the
  upload task redeems it automatically, once, the first time WiFi
  connects), or
- power the device on and run the `claim <CODE>` console command, or
- use the XGSail app's BLE relay (no WiFi needed at all — see "BLE relay"
  below).

Check status any time with the `device` console command. A lost
`device_api_key` needs a fresh key from the app (`rotate-key`) rewritten
onto the device by one of the methods above — the device cannot
regenerate its own key.

## Device-protocol upload

Files upload automatically once the device is stationary (speed below
~0.5 kt, or no GPS fix) for 30 seconds, the device is claimed, and a
configured WiFi network is in range — no manual step needed. Each of the
session's CSVs (nav/imu/wind/pres — whichever sensors were actually
present) uploads as its own call; a session missing a sensor (no wind
paired, no pressure chip) uploads exactly the files it has. Each file
gets a `.uploaded` marker so it's never re-sent by either upload path
(WiFi or BLE relay). Trigger a cycle immediately with the `upload`
console command, or check pending state with `status`.

## BLE relay

Alongside direct WiFi upload, the device always advertises a BLE GATT
service (xgsail's `docs/device-protocol.md` §8) that the XGSail phone app
can use to relay the same claim + upload calls over Bluetooth — for a
device with no WiFi configured at all, or when WiFi is temporarily out of
range. It's a first-class path, not just a fallback: the service comes up
at boot regardless of WiFi state. See `docs/firmware-architecture.md` for
how `ble_relay.cpp` maps pending SD files onto the protocol's
`session_manifest`/`session_data` characteristics.

The same BLE service also lets a companion app configure the device
remotely — WiFi credentials, boat identity, recording thresholds, and
more — trigger IMU calibration, and start/stop a recording session, all
without pulling the SD card. This is firmware-specific, not part of
xgsail's device protocol — see `docs/ble-config.md` for the full spec.

A phone pairing for the first time (to claim the device or write any of
the above) needs the device's button held for the long-press pairing
window — see `docs/hardware.md`'s "Recording/pairing button" and
`docs/ble-config.md`'s "Pairing window" for why and how.

## Serial / telnet console

USB-serial always accepts commands. The telnet listener (port 23) is off
by default — enable it with the serial `telneton` command if you need to
debug over WiFi (see `sailframes_edge/config.h`'s `TELNET_ENABLED_DEFAULT`
for why it defaults off).

```
telnet <device-ip> 23
```

Commands (`help` prints this list from the device):

| Command | Description |
|---|---|
| `status` | Device status: GPS, IMU, SD, WiFi, uploads |
| `gps` / `gpsraw` / `gpscfg` | GNSS detail / raw NMEA / reconfigure |
| `imu` / `imutest` / `cal` / `calreset` | IMU detail, axis test, calibrate/reset |
| `pres` | Pressure/temperature reading |
| `rec` / `stoprec` / `recstate` | Manual recording control + state |
| `wind` / `windscan` / `blescan` / `bleinit` / `bledeinit` / `bleconnect <mac>` | Wind sensor BLE |
| `display` | Toggle display mode |
| `heap` | Memory status |
| `ls`, `list` / `cat <file>` / `cleanup` | SD card browsing + uploaded-file cleanup |
| `upload` | Manual device-protocol upload cycle now |
| `claim <CODE>` / `device` | Redeem a claim code / show external_id + claim status |
| `wifi` / `disconnect` / `reboot` | Network + power control |
| `hwid` / `role` / `flags` / `radiomode` | Identity/config/state readouts |
| `mesh` / `fleet` / `fleetwatch` / `classes` | ESP-NOW peer mesh + RC fleet OCS view |
| `ocs` / `race arm <pinLat> <pinLon> <rcLat> <rcLon> <secs>` / `race disarm` | OCS state machine |
| `health` | Push a health snapshot now |

See `docs/firmware-architecture.md` for what the mesh/OCS/device-protocol
commands actually do, and `docs/gnss-rtk.md` for the GNSS-specific ones.

## WiFi configuration

Up to 5 networks in `config.txt`, tried in order until one connects:

```
wifi1_ssid=Home-IOT
wifi1_pass=password1

wifi2_ssid=YachtClub
wifi2_pass=password2
```

## License

Apache 2.0.
