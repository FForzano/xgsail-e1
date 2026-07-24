# @xgsail-e1/capacitor

Client-side **BLE transport for the E1** sailing tracker. It packages the
E1's GATT contract — service/characteristic UUIDs, wire types, and every
operation to talk to the device over Bluetooth LE — so any Capacitor app that
manages an E1 can reuse it instead of re-implementing the protocol.

This is the client half of the same contract the firmware in this repo
implements. See [`docs/ble-config.md`](../../docs/ble-config.md) for the
E1-specific configuration/calibration spec.

## Scope

**BLE transport (low level):** scan & discovery, connect/disconnect, read
`identity`, read/write `device_config`, read `status`, calibrate / start-rec /
stop-rec / manual OTA-update commands, and the session-relay primitives
(`readManifest`, `transferSession`, `ackUploaded`, `writeProvisioning`).

**Device management (high level):** `createE1Client({ backend, keyStore })`
gives you `claim(...)` and `uploadSessions(...)` — the full BLE-plus-backend
orchestration. The two seams are pluggable:

- **`E1Backend`** — the HTTP calls claim/upload end in. `httpBackend({ baseUrl })`
  is the ready-made implementation of the standard device-protocol (see
  [Backend contract](#backend-contract)); implement the interface yourself for
  a different server.
- **`E1KeyStore`** — where per-device keys are persisted. `secureStorageKeyStore()`
  (Capacitor Secure Storage) is the default; supply your own to store keys
  elsewhere.

Nothing here is tied to any particular app — point it at a base URL and go.

Native-only: every function no-ops on web (Web Bluetooth is unavailable on
iOS Safari and unreliable elsewhere), gated by `isNative()`.

## Install

```bash
npm install @xgsail-e1/capacitor
```

Peer dependencies (already present in a Capacitor app):

```bash
npm install @capacitor/core @capacitor-community/bluetooth-le
```

## Usage

### Configure & manage a device (claim + upload)

```ts
import { createE1Client, httpBackend, secureStorageKeyStore, scanForDevices } from "@xgsail-e1/capacitor";

const client = createE1Client({
  backend: httpBackend({ baseUrl: "https://app.example.com/api" }),
  keyStore: secureStorageKeyStore(), // Capacitor Secure Storage
});

// Claim a nearby device with a code, then relay its buffered sessions:
const [scanned] = await scanForDevices();
const { deviceId } = await client.claim(scanned, "ABC123");
const results = await client.uploadSessions(scanned, deviceId);
```

### Read status / edit config (low-level BLE)

```ts
import * as e1 from "@xgsail-e1/capacitor";

const devices = await e1.scanForDevices();
const status = await e1.withConnection(devices[0].bleId, () => e1.readStatus(devices[0].bleId));

// Live panel: hold one connection open, poll while mounted
await e1.connect(bleId);
const config = await e1.readConfig(bleId);
await e1.writeConfig(bleId, { display_mode: 2 });
await e1.disconnect(bleId);
```

`readStatus(...).firmware_version` is the device's running build
(`YYYY.MM.DD.N`). `E1Config` includes `ota_auto_update` (owner's opt-in for
automatic OTA firmware updates) and `ota_base_url` (which feed it checks) —
see the repo's `docs/ota.md`. Regardless of `ota_auto_update`,
`requestOtaUpdate(bleId)` (`control.ts`) triggers a manual check-and-apply from
any bonded phone; poll `readStatus(...).ota` for progress
(`checking`/`downloading`/`applying`/`up_to_date`/`suspended`/`error`) — a
successful update reboots the device, so there's no final reply on the control
characteristic itself. No update, automatic or manual, is ever attempted while
the device is recording.

### Custom backend or key store

Implement `E1Backend` / `E1KeyStore` and pass them to `createE1Client` — the
BLE relay logic is unchanged, only where it reads/writes differs.

## Backend contract

`httpBackend({ baseUrl })` speaks the E1 device-protocol (the same one the
firmware implements). A compatible server exposes:

| Call | Endpoint | Auth |
| --- | --- | --- |
| Confirm claim | `POST {baseUrl}/devices/claim/confirm` — body `{ external_id, claim_code }` → `{ device_id, device_api_key }` | none (the claim code is the credential) |
| Open session upload | `POST {baseUrl}/devices/me/session-uploads` — body `{ started_at, ended_at, filename, boat_id?, activity_id? }` → `{ session_upload_id, upload_url }` | `Authorization: DeviceKey <device_api_key>` |
| Store bytes | `PUT <upload_url>` (self-authorising presigned/HMAC URL) | none |
| Finalize | `PATCH {baseUrl}/devices/me/session-uploads/{id}` — body `{ is_final: true }` | `Authorization: DeviceKey <device_api_key>` |

`filename` must keep the file's real sensor-type suffix (nav/imu/wind/pres) —
the ingestion side keys sensor type off it. See `docs/ble-config.md` for the
BLE side of the same flow.

## Build

```bash
npm install
npm run build   # tsc → dist/ (ESM + .d.ts)
```

## License

Apache-2.0
