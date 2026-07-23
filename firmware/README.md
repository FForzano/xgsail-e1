# SailFrames E1 Firmware

ESP32 fleet-tracker firmware: GNSS + IMU + wind + pressure logging, an
ESP-NOW peer mesh for live fleet position sharing and OCS (on-course-side)
race-start detection, S3 upload, and a serial/telnet console.

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

ESP-NOW, WiFi, SD, and OTA/Update support come from the ESP32 Arduino
core (pinned to 3.3.7 — also documented in `docs/firmware-architecture.md`).

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
   speed thresholds, `unit_role` (`racing_boat`, `rc_signal`, ...).
3. A `/wind_mac.txt` file (Calypso Mini MAC address) enables the wind
   sensor and speeds up reconnection — see `e1_sd_card/wind_mac.txt` for
   the format. Without it, the wind sensor is disabled.
4. `/imu_cal.txt` stores the heel/pitch zero-offsets saved by the `cal`
   console command — see `e1_sd_card/imu_cal.txt`.

## Log files

Per session, under `/sf/<session-folder>/` (GPS-datetime named when a fix
is available, `session_NNN` otherwise):

```
<boat_id>_<date>_<time>_nav.csv    # 10 Hz: lat, lon, speed, course, sat, hdop, fix, hacc
<boat_id>_<date>_<time>_imu.csv    # 10 Hz: accel, gyro, linear accel, mag, heel, pitch, heading
<boat_id>_<date>_<time>_wind.csv   # 1 Hz, only when a wind sensor is paired
<boat_id>_<date>_<time>_pres.csv   # 0.1 Hz: pressure, temperature, min/max (gust window)
```

A recording session auto-starts once boat speed sustains above
`start_speed_knots` (config.txt) and only stops on a clean power-off or
the `stoprec` command (see `docs/firmware-architecture.md` for why
speed-triggered *stop* was removed).

## S3 upload

Files upload automatically once the device is stationary (speed below
~0.5 kt, or no GPS fix) for 30 seconds and a configured WiFi network is in
range — no manual step needed. Each file gets a `.uploaded` marker so a
session never re-uploads. Trigger it immediately with the `upload` console
command, or check pending state with `status`.

## Firmware updates (OTA)

`ArduinoOTA` (the Arduino-IDE-over-WiFi upload path) is disabled by
default — see `docs/firmware-architecture.md` for why. In practice,
firmware ships as a manifest-pull update: CI publishes
`firmware/<boat_id>/latest.json` (version, URL, SHA256) plus the `.bin` to
the configured S3 bucket, and the device pulls + verifies + flashes it via
the `update` console command (or automatically, once per boot, from the
upload task). The very first flash must be over USB.

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
| `upload` / `update` / `configsync` | Manual S3 upload / OTA pull / cloud config pull |
| `wifi` / `disconnect` / `reboot` | Network + power control |
| `hwid` / `role` / `configver` / `flags` / `radiomode` | Identity/config/state readouts |
| `mesh` / `fleet` / `fleetwatch` / `classes` | ESP-NOW peer mesh + RC fleet OCS view |
| `ocs` / `race arm <pinLat> <pinLon> <rcLat> <rcLon> <secs>` / `race disarm` | OCS state machine |
| `statusup` | Upload a fleet-health snapshot to S3 now |

See `docs/firmware-architecture.md` for what the mesh/OCS/cloud-config
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
