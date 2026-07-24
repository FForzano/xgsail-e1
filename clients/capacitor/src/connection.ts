import { BleClient } from "@capacitor-community/bluetooth-le";

/** Connects, runs `fn`, always disconnects — the shared shape every one-shot
 * BLE operation needs (claim, upload relay, config read/write, commands).
 * Centralized so connect/disconnect bookkeeping doesn't get duplicated (and
 * drift) across each operation. */
export async function withConnection<T>(bleId: string, fn: () => Promise<T>): Promise<T> {
  await BleClient.initialize();
  await BleClient.connect(bleId);
  try {
    return await fn();
  } finally {
    await BleClient.disconnect(bleId).catch(() => {});
  }
}

/** Opens a connection the caller manages the lifetime of directly (unlike
 * withConnection's one-shot pattern) — for a panel that polls `status`/
 * `device_config` repeatedly while mounted instead of reconnecting per call.
 * Caller must call `disconnect()` on unmount. */
export async function connect(bleId: string): Promise<void> {
  await BleClient.initialize();
  await BleClient.connect(bleId);
}

export async function disconnect(bleId: string): Promise<void> {
  await BleClient.disconnect(bleId).catch(() => {});
}
