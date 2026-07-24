import { BleClient, dataViewToText, textToDataView } from "@capacitor-community/bluetooth-le";
import { SERVICE_UUID, CHAR_CONTROL } from "./protocol";
import type { CalibrateResult, RecCommandResult } from "./types";

/** Fire-and-forget write to the `control` characteristic. Used both by the
 * command helpers here and by the session-relay flow (start-transfer /
 * ack-uploaded). */
export async function writeControl(
  bleId: string,
  cmd: string,
  extra?: Record<string, unknown>,
): Promise<void> {
  await BleClient.write(
    bleId,
    SERVICE_UUID,
    CHAR_CONTROL,
    textToDataView(JSON.stringify({ cmd, ...extra })),
  );
}

/** Sends a `control` command and awaits its notified reply, matched by `cmd`
 * (control is used for several concurrent purposes — start-transfer/
 * ack-uploaded during a relay, calibrate/rec commands here — so replies are
 * correlated by the `cmd` field, not by connection state). */
async function sendControlCommand<T>(
  bleId: string,
  cmd: string,
  extra?: Record<string, unknown>,
): Promise<T> {
  return new Promise<T>((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error(`No response to '${cmd}'`)), 10_000);
    BleClient.startNotifications(bleId, SERVICE_UUID, CHAR_CONTROL, (value) => {
      const payload = JSON.parse(dataViewToText(value)) as { cmd?: string } & Record<string, unknown>;
      if (payload.cmd !== cmd) return;
      clearTimeout(timeout);
      void BleClient.stopNotifications(bleId, SERVICE_UUID, CHAR_CONTROL).catch(() => {});
      resolve(payload as T);
    })
      .then(() => writeControl(bleId, cmd, extra))
      .catch((err) => {
        clearTimeout(timeout);
        reject(err);
      });
  });
}

/** Zeroes heel/pitch at the boat's current attitude — only meaningful with the
 * boat sitting level; the caller is responsible for telling the user that
 * first. */
export function calibrate(bleId: string): Promise<CalibrateResult> {
  return sendControlCommand<CalibrateResult>(bleId, "calibrate");
}

/** Resets heel/pitch calibration offsets back to zero. */
export function calibrateReset(bleId: string): Promise<CalibrateResult> {
  return sendControlCommand<CalibrateResult>(bleId, "calibrate-reset");
}

/** Starts a recording on the device, same entry point as the physical
 * button's short press. `boatId`/`activityId` are both optional and
 * independent of each other (docs/ble-config.md): omitting both files the
 * session under the device's own boat and a fresh solo activity. */
export function startRec(
  bleId: string,
  opts?: { boatId?: string; activityId?: string },
): Promise<RecCommandResult> {
  return sendControlCommand<RecCommandResult>(bleId, "start-rec", {
    ...(opts?.boatId ? { boat_id: opts.boatId } : {}),
    ...(opts?.activityId ? { activity_id: opts.activityId } : {}),
  });
}

export function stopRec(bleId: string): Promise<RecCommandResult> {
  return sendControlCommand<RecCommandResult>(bleId, "stop-rec");
}
