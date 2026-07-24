import { isNative } from "./protocol";
import { withConnection } from "./connection";
import { readIdentity, writeProvisioning } from "./discovery";
import { readManifest, transferSession, ackUploaded } from "./relay";
import type { E1Backend } from "./backend";
import type { E1KeyStore } from "./keystore";
import type { ScannedDevice } from "./types";

export interface E1ClientOptions {
  backend: E1Backend;
  keyStore: E1KeyStore;
}

export interface UploadRelayResult {
  sessionId: string;
  uploaded: boolean;
  error?: string;
}

export interface E1Client {
  /** Claims a device over BLE instead of typing the code onto it by hand:
   * reads its `external_id`, redeems the claim code against the backend, writes
   * the issued key back onto the device, and stores it. This client becomes
   * the device's natural first upload relay, since it now holds the key.
   * Native-only (throws on web). */
  claim(scanned: ScannedDevice, claimCode: string): Promise<{ deviceId: string; externalId: string }>;

  /** Relays every session the device buffered but couldn't upload itself (no
   * WiFi at the time) through this client's connection. Safe to call
   * repeatedly, including after a dropped link: the device only frees a
   * session's buffer once it receives `ack-uploaded` (sent here only after the
   * backend upload resolves), so an interrupted relay is simply retried next
   * time — nothing is lost or duplicated. No-op on web. */
  uploadSessions(scanned: ScannedDevice, deviceId: string): Promise<UploadRelayResult[]>;
}

/** The real basename the device would send if it uploaded this file itself
 * (e.g. `E1_20260723_1405_nav.csv`) — the manifest's `session_id` is that
 * file's full SD path. The backend keys sensor type off the filename suffix
 * (nav/imu/wind/pres) and collapses same-named files onto one storage key, so
 * a fixed name would silently drop every file in a session but the last. */
function basenameOf(sdPath: string): string {
  return sdPath.slice(sdPath.lastIndexOf("/") + 1);
}

/** Builds an E1 device manager from a backend + key store. This is the
 * high-level entry point: the BLE relay logic (connect, read identity/manifest,
 * transfer, ack, per-session error handling) lives here once; only the
 * backend and storage are pluggable. */
export function createE1Client({ backend, keyStore }: E1ClientOptions): E1Client {
  return {
    async claim(scanned, claimCode) {
      if (!isNative()) throw new Error("Not available on web");
      return withConnection(scanned.bleId, async () => {
        const identity = await readIdentity(scanned.bleId);
        const { deviceId, deviceApiKey } = await backend.confirmClaim(identity.external_id, claimCode);
        await writeProvisioning(scanned.bleId, deviceApiKey);
        await keyStore.save(deviceId, deviceApiKey);
        return { deviceId, externalId: identity.external_id };
      });
    },

    async uploadSessions(scanned, deviceId) {
      if (!isNative()) return [];
      const key = await keyStore.load(deviceId);
      if (!key) throw new Error("No stored key for this device — claim it again");

      return withConnection(scanned.bleId, async () => {
        const results: UploadRelayResult[] = [];
        for (const entry of await readManifest(scanned.bleId)) {
          try {
            const bytes = await transferSession(scanned.bleId, entry.session_id, entry.byte_size);
            await backend.uploadSession(key, entry, basenameOf(entry.session_id), bytes);
            await ackUploaded(scanned.bleId, entry.session_id);
            results.push({ sessionId: entry.session_id, uploaded: true });
          } catch (err) {
            results.push({
              sessionId: entry.session_id,
              uploaded: false,
              error: err instanceof Error ? err.message : String(err),
            });
          }
        }
        return results;
      });
    },
  };
}
