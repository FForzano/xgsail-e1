import { BleClient, dataViewToText } from "@capacitor-community/bluetooth-le";
import { SERVICE_UUID, CHAR_SESSION_MANIFEST, CHAR_SESSION_DATA } from "./protocol";
import { writeControl } from "./control";
import type { ManifestEntry } from "./types";

// A dropped/stalled BLE link during a transfer must not hang forever — the
// caller's retry (the device hasn't received ack-uploaded and so hasn't freed
// its buffer) is the real recovery path; this timeout just bounds a single
// attempt.
const SESSION_TRANSFER_TIMEOUT_MS = 120_000;

/** Reads the pending-session manifest — every buffered session the device
 * couldn't upload itself (no WiFi at the time). */
export async function readManifest(bleId: string): Promise<ManifestEntry[]> {
  const raw = await BleClient.read(bleId, SERVICE_UUID, CHAR_SESSION_MANIFEST);
  return JSON.parse(dataViewToText(raw)) as ManifestEntry[];
}

/** Reassembles one session's bytes from `session_data` notifications, each
 * framed as a 4-byte big-endian sequence index + chunk payload. Resolves once
 * it has received exactly `byteSize` bytes total — the manifest-declared size
 * is the completion signal, no separate end-of-stream marker is needed. */
async function receiveSessionBytes(bleId: string, byteSize: number): Promise<Uint8Array<ArrayBuffer>> {
  const chunks = new Map<number, Uint8Array>();
  let received = 0;
  await new Promise<void>((resolve, reject) => {
    const timeout = setTimeout(
      () => reject(new Error("Session transfer timed out")),
      SESSION_TRANSFER_TIMEOUT_MS,
    );
    BleClient.startNotifications(bleId, SERVICE_UUID, CHAR_SESSION_DATA, (value) => {
      if (value.byteLength < 4) return; // malformed frame — drop it, wait for a good one
      const index = value.getUint32(0, false);
      if (!chunks.has(index)) {
        chunks.set(index, new Uint8Array(value.buffer, value.byteOffset + 4, value.byteLength - 4));
        received += value.byteLength - 4;
      }
      if (received >= byteSize) {
        clearTimeout(timeout);
        resolve();
      }
    }).catch(reject);
  });
  await BleClient.stopNotifications(bleId, SERVICE_UUID, CHAR_SESSION_DATA).catch(() => {});

  // Explicitly a fresh (non-shared) ArrayBuffer, satisfying Blob's BlobPart
  // type in the consumer — the source chunks are views into DataView.buffer,
  // which TS widens to ArrayBufferLike (it could theoretically be a
  // SharedArrayBuffer), but this allocation is unambiguously a plain
  // ArrayBuffer.
  const out: Uint8Array<ArrayBuffer> = new Uint8Array(received);
  let offset = 0;
  for (const [, chunk] of [...chunks.entries()].sort(([a], [b]) => a - b)) {
    out.set(chunk, offset);
    offset += chunk.length;
  }
  return out;
}

/** Requests one buffered session over BLE and returns its reassembled bytes:
 * sends `start-transfer` then collects the `session_data` stream. The caller
 * uploads the bytes to the backend, then calls `ackUploaded` so the device
 * frees its local buffer. */
export async function transferSession(
  bleId: string,
  sessionId: string,
  byteSize: number,
): Promise<Uint8Array<ArrayBuffer>> {
  await writeControl(bleId, "start-transfer", { session_id: sessionId });
  return receiveSessionBytes(bleId, byteSize);
}

/** Tells the device a session was successfully uploaded, so it can free the
 * local buffer. Until this arrives the device keeps the session, so an
 * interrupted relay is simply retried next time — nothing is lost. */
export async function ackUploaded(bleId: string, sessionId: string): Promise<void> {
  await writeControl(bleId, "ack-uploaded", { session_id: sessionId });
}
