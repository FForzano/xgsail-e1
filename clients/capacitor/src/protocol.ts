import { Capacitor } from "@capacitor/core";

// The E1 GATT contract — the values firmware and any client app must agree
// on. Documented in this repo's docs/ble-config.md (the E1-specific
// extensions) and docs/firmware-architecture.md (the BLE relay design).
// Nothing else in this package depends on the specific values beyond app and
// firmware agreeing on them.
//
// The UUIDs were freshly generated (v4, random), not borrowed from a known
// public service like Nordic UART, so scanning for SERVICE_UUID can't
// false-positive-match an unrelated nearby BLE device that happens to
// implement that service.
export const SERVICE_UUID = "24e6db2c-3c8a-4b5b-ba5a-23bc4c818046";
export const CHAR_IDENTITY = "985a1aae-858e-4727-9d5c-c8670bd6bd06";
export const CHAR_PROVISIONING = "db2c2e63-9e13-4fa9-867c-0b579ce2ae57";
export const CHAR_SESSION_MANIFEST = "ed9efdc8-70d4-4ce5-a0a3-9fa6d88b9b9e";
export const CHAR_SESSION_DATA = "728d2815-0409-49ce-ad73-ecca6fc6d981";
export const CHAR_CONTROL = "ec88dd3e-2562-420c-aebe-30a4ae40bdf9";
export const CHAR_DEVICE_CONFIG = "042dfd7c-88f4-4ae8-af9a-eb1d7be7a3c6";
export const CHAR_STATUS = "bfef7865-f3f7-486c-93fe-bbae78cfdc43";

/** True only inside a Capacitor native app (iOS/Android). BLE for the E1 is
 * native-only by design: Web Bluetooth isn't available on iOS Safari at all
 * and is unreliable elsewhere, so every operation here no-ops on web. */
export const isNative = (): boolean => Capacitor.isNativePlatform();
