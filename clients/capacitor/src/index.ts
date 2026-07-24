// Client-side BLE transport and device management for the E1 sailing
// tracker. Low level: scan/connect, identity, config/status, calibration and
// recording commands, and session-relay primitives — the E1's GATT contract.
// High level: createE1Client(...) wraps that into claim()/uploadSessions(),
// with which backend to call and where device keys are stored left pluggable
// via E1Backend/E1KeyStore, so nothing here is tied to a specific server.
//
// Native-only (Capacitor): every operation no-ops on web (see `isNative`).

export * from "./protocol";
export * from "./connection";
export * from "./discovery";
export * from "./config";
export * from "./status";
export * from "./control";
export * from "./relay";

// High-level device management (claim + upload relay) + its pluggable seams.
export * from "./backend";
export * from "./keystore";
export * from "./client";

export { ConfigWriteError } from "./types";
export type {
  ScannedDevice,
  IdentityPayload,
  ManifestEntry,
  E1WifiNetwork,
  E1Config,
  E1ConfigPatch,
  ConfigWriteErrorReason,
  E1Status,
  CalibrateResult,
  RecCommandResult,
} from "./types";
