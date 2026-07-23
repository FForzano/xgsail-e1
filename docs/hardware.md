# Hardware reference

The current battery/power, display, and recording-control setup, as
implemented in `firmware/sailframes_edge/` (`config.h`, `battery.cpp`,
`display.cpp`, `recording.cpp`) and the KiCad project in `hardware/`.
Earlier hardware revisions (an OLED display, a different battery pack
and voltage divider, a momentary power button with deep-sleep) are not
described here — see "Superseded designs" at the end if you're trying to
make sense of an older E1 unit in the field.

## Power

**No software power control at all.** A hardware SPDT slide switch cuts
all power to the board directly — there's no deep sleep, no button-hold
shutdown sequence, and no GPIO involved in powering the unit off. This is
deliberate: the firmware's own header comment states the design plainly —
*"Power control: Hardware switch on boost converter. No software deep
sleep — hardware switch cuts all power when OFF."* The only thing
software does around a power-off is make sure it doesn't happen mid-write:
`updateRecordingState()`'s `REC_STOPPING` path and `handleLowBattery()`
both flush and close any open session files before anything else, so a
switch flip mid-race doesn't corrupt the CSV in progress.

### Battery monitoring

- **Divider**: 100 kΩ / 100 kΩ from the LiPo's B+ straight to GPIO34
  (`BATT_VOLTAGE_PIN`, ADC1 — input-only, no pull-up, avoids ADC2's WiFi
  conflict). Nominal ratio is 2.0; the firmware uses a calibrated
  `BATT_DIVIDER_RATIO = 2.25` (`battery.cpp`) to correct for the ESP32
  ADC's non-linearity (empirically: a multimeter-measured 4.165 V read as
  3.70 V through the raw divider math, so `4.165 / 3.70 * 2.0 = 2.25`).
  16 averaged samples per read (`BATT_SAMPLES`) smooth out ADC noise.
- **Percent estimate**: a piecewise-linear lookup table over the LiPo
  discharge curve (`getBatteryPercent()`), not a straight voltage-to-percent
  line — the curve is genuinely non-linear (steep at both ends, flat in
  the middle), so a linear mapping over- or under-reports charge in the
  middle of the range:

  | Voltage | 4.20 | 4.15 | 4.10 | 4.05 | 4.00 | 3.90 | 3.80 | 3.70 | 3.60 | 3.50 | 3.40 | 3.30 |
  |---|---|---|---|---|---|---|---|---|---|---|---|---|
  | Percent | 100 | 95 | 85 | 75 | 65 | 50 | 35 | 20 | 12 | 6 | 2 | 0 |

- **Critical/low-battery behavior**: `isBatteryCritical()` fires below
  3.3 V (and only when the reading is plausible — above 0.5 V — so an
  unconnected/faulty sense line doesn't false-trigger). `handleLowBattery()`
  then flushes and closes any open session files, paints a red "LOW
  BATTERY / Flip power switch to OFF" screen, and halts in a `delay(1000)`
  loop — recovery is the operator flipping the hardware switch, not an
  automatic power-off, since there's no software path to cut power.

### GPS-speed-triggered recording

No manual start/stop for the common case. `updateRecordingState()`
(`recording.cpp`) auto-starts a session once GPS speed sustains above
`start_speed_knots` (default 1.5 kt) for `start_delay_sec` (default 10 s)
— well above dock/mooring GPS noise, with the delay guarding against a
single noisy fix. It does **not** auto-stop on low speed: sailors
routinely sit near-stationary before a start or between races in a
series, and an earlier speed-triggered stop was removed because it
chopped sessions mid-event. A session only ends on a clean power-off (the
slide switch) or the `stoprec` console command — see
`docs/firmware-architecture.md` for the OCS/mesh context this recording
state feeds into, and `firmware/README.md` for the resulting log files.

## Display

**Hosyond 3.5" IPS ST7796U TFT, 480×320, SPI** (`TFT_eSPI`, configured via
`firmware/sailframes_edge/User_Setup.h`). Portrait orientation
(`tft.setRotation(2)`), inverted colors (`tft.invertDisplay(true)` — this
specific panel needs it for correct colors), PWM-dimmable backlight on
GPIO25 (`TFT_BL_PIN`) — full brightness while recording
(`TFT_BL_DUTY_RECORDING`, ~80%) and dimmed otherwise
(`TFT_BL_DUTY_IDLE`, ~50%) to save power, since the backlight is the
single largest current draw in the system.

The TFT and the microSD card are on **separate SPI buses** — the display
on the default VSPI bus, the SD card on HSPI with its own explicit pins
(`SD_CS_PIN`/`SD_CLK_PIN`/`SD_MISO_PIN`/`SD_MOSI_PIN` in `config.h`).
This is deliberate, not incidental: sharing one bus between the two
caused visible display flicker during SD writes on an earlier board
revision.

Three nav display modes exist (`display.cpp`'s `updateDisplayD1/D2/D3`,
cycled with the `display` console command), plus two RC-only panels
(`drawRcFleetPanel`/`drawRcPreRacePanel`) that take over the screen
in place of the nav display while the unit's role is `rc_signal` — see
`docs/firmware-architecture.md` for what those panels show and why.

## Superseded designs (for context only — not what's on the board today)

Earlier E1 hardware revisions used a 2.42" SSD1309 OLED (128×64,
monochrome) instead of the TFT — replaced because it was unreadable
through polarized sunglasses and too small for a multi-value dashboard —
and a different battery/divider combination (first a PowerBoost-style
boost converter with a 200Ω/200Ω divider drawing a continuous ~10 mA,
later corrected to the 100 kΩ/100 kΩ divider described above once the
continuous drain was recognized as wasteful). An even earlier revision
used a momentary pushbutton (ESP32 deep sleep, wake-on-button) rather
than the hardware slide switch — none of this is present in the current
firmware or the KiCad project in `hardware/`; it's noted here only so a
differently-wired older unit doesn't look like a bug.
