import { BleClient, dataViewToText, textToDataView } from "@capacitor-community/bluetooth-le";
import { SERVICE_UUID, CHAR_IDENTITY, CHAR_PROVISIONING, isNative } from "./protocol";
import { withConnection } from "./connection";
import type { IdentityPayload, ScannedDevice } from "./types";

/** Scans for nearby E1 devices for `timeoutMs` (default 8s) and resolves with
 * whatever was found by then. A claim picker needs "what's here" once, not a
 * live-updating feed, so results are collected into one list rather than
 * streamed back through a callback. */
export async function scanForDevices(timeoutMs = 8000): Promise<ScannedDevice[]> {
  if (!isNative()) return [];
  await BleClient.initialize();
  const found = new Map<string, ScannedDevice>();
  await BleClient.requestLEScan({ services: [SERVICE_UUID] }, (result) => {
    found.set(result.device.deviceId, {
      bleId: result.device.deviceId,
      name: result.device.name ?? result.localName ?? null,
    });
  });
  await new Promise((resolve) => setTimeout(resolve, timeoutMs));
  await BleClient.stopLEScan();
  return [...found.values()];
}

/** Reads the device `identity` characteristic (external_id, firmware). */
export async function readIdentity(bleId: string): Promise<IdentityPayload> {
  const raw = await BleClient.read(bleId, SERVICE_UUID, CHAR_IDENTITY);
  return JSON.parse(dataViewToText(raw)) as IdentityPayload;
}

/** Writes the device_api_key back onto a device during the claim flow (the
 * value the backend returned from claim/confirm). */
export async function writeProvisioning(bleId: string, deviceApiKey: string): Promise<void> {
  await BleClient.write(
    bleId,
    SERVICE_UUID,
    CHAR_PROVISIONING,
    textToDataView(JSON.stringify({ device_api_key: deviceApiKey })),
  );
}

// external_id -> bleId, populated by findByExternalId. There's no other way to
// map a claimed device back to a nearby peripheral: every E1 advertises under
// the same name, so the only disambiguator is external_id read off `identity`
// after connecting — this cache just avoids re-scanning+re-connecting-to-
// everything on every poll.
const externalIdToBleId = new Map<string, string>();

/** Scans for `timeoutMs`, connecting to each candidate in turn until one's
 * `identity.external_id` matches, or none do. Cached by external_id; a failed
 * connect to the cached bleId (device moved out of range, rebooted with a new
 * OS-level connection id) evicts the cache entry so the next call re-scans
 * instead of retrying a dead id forever. */
export async function findByExternalId(
  externalId: string,
  timeoutMs = 8000,
): Promise<ScannedDevice | null> {
  if (!isNative()) return null;
  const cached = externalIdToBleId.get(externalId);
  if (cached) {
    try {
      const identity = await withConnection(cached, () => readIdentity(cached));
      if (identity.external_id === externalId) return { bleId: cached, name: null };
    } catch {
      // fall through to a fresh scan
    }
    externalIdToBleId.delete(externalId);
  }

  const candidates = await scanForDevices(timeoutMs);
  for (const candidate of candidates) {
    try {
      const identity = await withConnection(candidate.bleId, () => readIdentity(candidate.bleId));
      if (identity.external_id === externalId) {
        externalIdToBleId.set(externalId, candidate.bleId);
        return candidate;
      }
    } catch {
      // unreachable/errored candidate — try the next one
    }
  }
  return null;
}
