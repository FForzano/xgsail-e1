# OTA firmware updates

The E1 can fetch and apply firmware updates over WiFi, either automatically
(opt-in, once-per-boot) or on demand from the owner's phone over BLE. It talks
to a public, unauthenticated OTA feed — the same manifest shape xgsail's own
`ota-service` uses for native-app bundles — and verifies the downloaded image's
SHA256 before ever committing it to flash.

## Why the partition table had to be decided up front

OTA needs two application slots (A/B): the device runs from one, writes the
new image into the other, and boot-swaps. The board's original `no_ota` scheme
had a single app slot — impossible to OTA onto, and a partition table can't be
changed by an update that runs *inside* that table. So the dual-app table
(`firmware/sailframes_edge/partitions.csv`, selected by `PartitionScheme=custom`
in the build FQBN) was adopted ahead of implementing OTA itself, to avoid ever
needing a fleet-wide USB reflash:

| Name    | Type | SubType | Offset    | Size     | Notes                        |
|---------|------|---------|-----------|----------|-------------------------------|
| nvs     | data | nvs     | 0x9000    | 0x5000   | WiFi radio calibration store |
| otadata | data | ota     | 0xe000    | 0x2000   | active-slot selector         |
| app0    | app  | ota_0   | 0x10000   | 0x1F0000 | ~1.94 MB app slot A          |
| app1    | app  | ota_1   | 0x200000  | 0x1F0000 | ~1.94 MB app slot B          |

SPIFFS is dropped entirely (this firmware stores everything on the SD card,
`storage.cpp`) — reclaiming its 2 MB is what makes room for a second app slot
on 4 MB flash. The current image is ~1.6 MB, leaving comfortable headroom per
slot. Offsets line up with the release workflow's `esptool merge_bin` step
unchanged: the app lands at `0x10000` (= `app0`) and `boot_app0.bin` at `0xe000`
(= `otadata`).

> **One-time reflash of already-deployed devices.** A device flashed on the old
> `no_ota` table must be reflashed once over USB with the new merged image to
> adopt this table — a partition table can't rewrite itself.

## Config

| Field              | Default                    | Meaning |
|---------------------|----------------------------|---------|
| `ota_auto_update`    | `false`                    | Owner opt-in for automatic checks. Independent of the manual BLE trigger below, which works either way. |
| `ota_base_url`       | `https://xgsail.com/ota`   | Base URL of the firmware feed. Override for staging/self-hosted. |

Both are on-SD (`config.txt`) and live over BLE `device_config`
(`docs/ble-config.md`).

## Manifest contract

`GET <ota_base_url>/manifest.json` — public, **no auth** (firmware isn't a
secret; integrity is the SHA256 check below). Same response shape as xgsail's
`ota-service` (which serves native-app bundle manifests the same way):

```
200 OK
{ "version": "2026.08.01.01", "url": "https://.../app.bin", "checksum": "<sha256 hex>" }
```

or `{}` — nothing published / device is already current. `version` follows the
firmware's `YYYY.MM.DD.N` scheme (`config.h`'s `FW_VERSION`), compared as a
tuple against the running build — only a strictly newer build triggers a
download. A malformed/missing version on either side is treated as "not
newer" (fail closed — never flash on ambiguous data).

## Flow (`upload.cpp`)

1. **Trigger.** Either automatic — `checkForFirmwareUpdate()`, called once per
   boot (and on each subsequent health-check WiFi wake) from the Core-0 upload
   task, gated on `config.ota_auto_update` — or manual, via the BLE `control`
   `ota-update` command (see below), which runs regardless of that flag.
2. **Manifest.** `otaFetchManifest()` GETs `manifest.json` and parses
   `version`/`url`/`checksum`.
3. **Version compare.** `otaVersionIsNewer()` parses both versions with the
   same `sscanf("%d.%d.%d.%d", ...)` pattern `display.cpp`'s `fwShortTag()`
   uses and compares the tuples.
4. **Download + flash.** `otaDownloadAndApply()` streams the image via
   `HTTPClient`/`Update.h` straight into the inactive OTA slot
   (`Update.begin()` targets the next slot automatically), computing a running
   SHA256 (`mbedtls_sha256_*`) as it writes. Uses the same watchdog-fed,
   stall/ceiling-guarded chunk loop shape as the session-upload path's
   `putFileBytes()`.
5. **Verify before commit.** The computed SHA256 is compared against the
   manifest's `checksum` **before** `Update.end()` is called. A mismatch aborts
   the update — the device keeps running the current (still-active) slot.
6. **Apply.** `Update.end(true)` sets the new slot as the boot partition;
   `ESP.restart()` reboots into it. The bootloader automatically rolls back to
   the previous slot if the new image fails to boot.

## Recording gate (hard requirement)

An OTA flash must never contend with an active recording for SD/CPU/WiFi:

- **Won't start:** `checkForFirmwareUpdate()` and the manual path both refuse
  outright while `logging` (`storage.h`) is true.
- **Suspended if already running:** the download loop polls `logging` every
  chunk (same pattern as `putFileBytes()`'s abort checks) and aborts the
  transfer the moment a recording starts, reporting state `"suspended"` rather
  than `"error"`. It is retried on a later WiFi window — automatically if
  `ota_auto_update` is on, or the next time the owner manually triggers it.

## Manual trigger over BLE

The `control` characteristic's `ota-update` (alias `ota-check`) command
queues an immediate check-and-apply, usable from **any bonded phone at any
time** — no pairing-window requirement, unlike `device_config` writes, since
it doesn't touch persisted config. The device refuses it outright (with
`{"ok": false, "message": "recording in progress"}`) while recording.

Because a successful update reboots the device, there is no final reply on
`control` itself — the ack only confirms the check was queued. Progress and
outcome are surfaced instead in the `status` characteristic's `ota` object
(`docs/ble-config.md`):

```json
"ota": { "state": "downloading", "progress": 42 }
```

`state` is one of `idle` / `checking` / `up_to_date` / `downloading` /
`applying` / `suspended` / `error`; `progress` (0-100) is present only while
downloading; `message` carries a short detail on `error`/`suspended`.

## Security: SHA256 only, no secure boot

Deliberately no secure boot, no image signing, no flash encryption — only the
SHA256 check above. Rationale:

- Secure boot v2 requires burning **irreversible eFuses** at flash time,
  signing every build (including dev builds), and careful key management —
  with a real risk of permanently bricking a device if keys are lost.
- It defends against an attacker with **physical access to the flash**
  injecting firmware or reading firmware IP off the chip. That threat isn't
  compelling for a sailboat tracker, which holds no secrets on flash beyond a
  per-device `DeviceKey` that is already revocable server-side.
- The realistic OTA threat — a **corrupted or tampered image in transit** — is
  fully covered by verifying the SHA256 before committing the image.

Note that HTTP/TLS is currently `setInsecure()` in `device_auth.cpp` (the
ESP32 core 3.3.7 TLS stack is unreliable), so certificate validation is not a
transport guarantee today — which is exactly why the SHA256 manifest check is
the **primary** integrity control for OTA, not a secondary one. If the threat
model ever changes, secure boot cannot be retrofitted onto deployed devices; it
would require a hardware/manufacturing decision for new units.

## Server side (xgsail repo)

The firmware feed is expected to live alongside xgsail's existing
`ota-service` (native-app bundle updates) — same manifest shape
(`version`/`url`/`checksum`, `{}` = up to date), same public/no-auth posture,
serving from the same object storage. Standing up
`https://xgsail.com/ota/manifest.json` for firmware builds (populating it from
the firmware release workflow's app binary + its SHA256) is server-side work
in that repo, not this one.
