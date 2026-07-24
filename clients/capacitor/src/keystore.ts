import { isNative } from "./protocol";

// Where per-device keys live. A device key is issued once at claim time and
// then reused to authenticate that device's uploads, so it must be persisted
// securely and keyed per device (one client can hold several E1s). The
// mechanism is the consuming app's concern — this interface abstracts it so
// the claim/upload orchestration doesn't hardcode a particular store.
export interface E1KeyStore {
  load(deviceId: string): Promise<string | null>;
  save(deviceId: string, deviceApiKey: string): Promise<void>;
}

/** Default `E1KeyStore` backed by Capacitor Secure Storage (iOS Keychain /
 * Android Keystore). `namespace` prefixes the stored keys, so multiple apps or
 * device families don't collide. Returns null (never throws) on web or on a
 * corrupt/inaccessible entry — callers treat "no key" as "not claimed here".
 *
 * `@aparajita/capacitor-secure-storage` is an *optional* peer dependency,
 * imported dynamically so consumers that supply their own store never need it
 * installed. */
export function secureStorageKeyStore(namespace = "e1_device_key"): E1KeyStore {
  const storageKey = (deviceId: string): string => `${namespace}:${deviceId}`;
  return {
    async load(deviceId) {
      if (!isNative()) return null;
      try {
        const { SecureStorage } = await import("@aparajita/capacitor-secure-storage");
        return (await SecureStorage.getItem(storageKey(deviceId))) as string | null;
      } catch {
        return null;
      }
    },
    async save(deviceId, deviceApiKey) {
      const { SecureStorage } = await import("@aparajita/capacitor-secure-storage");
      await SecureStorage.setItem(storageKey(deviceId), deviceApiKey);
    },
  };
}
