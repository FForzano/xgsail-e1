import { BleClient, dataViewToText, textToDataView } from "@capacitor-community/bluetooth-le";
import { SERVICE_UUID, CHAR_DEVICE_CONFIG } from "./protocol";
import { ConfigWriteError } from "./types";
import type { ConfigWriteErrorReason, E1Config, E1ConfigPatch } from "./types";

export async function readConfig(bleId: string): Promise<E1Config> {
  const raw = await BleClient.read(bleId, SERVICE_UUID, CHAR_DEVICE_CONFIG);
  return JSON.parse(dataViewToText(raw)) as E1Config;
}

/** Writes a partial config update and awaits the device's notified result. A
 * write outside the pairing window (no recent long-press, no existing bond)
 * is rejected with `pairing_window_closed` — the caller is responsible for
 * telling the user to long-press the physical button first (docs/ble-config.md:
 * no in-band way to open it). */
export async function writeConfig(bleId: string, patch: E1ConfigPatch): Promise<void> {
  await new Promise<void>((resolve, reject) => {
    BleClient.startNotifications(bleId, SERVICE_UUID, CHAR_DEVICE_CONFIG, (value) => {
      const status = JSON.parse(dataViewToText(value)) as { status: "ok" | "error"; reason?: ConfigWriteErrorReason };
      void BleClient.stopNotifications(bleId, SERVICE_UUID, CHAR_DEVICE_CONFIG).catch(() => {});
      if (status.status === "ok") resolve();
      else reject(new ConfigWriteError(status.reason ?? "bad_json"));
    })
      .then(() =>
        BleClient.write(bleId, SERVICE_UUID, CHAR_DEVICE_CONFIG, textToDataView(JSON.stringify(patch))),
      )
      .catch(reject);
  });
}
