import { BleClient, dataViewToText } from "@capacitor-community/bluetooth-le";
import { SERVICE_UUID, CHAR_STATUS } from "./protocol";
import type { E1Status } from "./types";

export async function readStatus(bleId: string): Promise<E1Status> {
  const raw = await BleClient.read(bleId, SERVICE_UUID, CHAR_STATUS);
  return JSON.parse(dataViewToText(raw)) as E1Status;
}
