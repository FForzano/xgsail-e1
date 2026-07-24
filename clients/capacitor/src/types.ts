// Wire types for the E1 BLE protocol. These are the typed source of truth
// shared between the firmware (which emits them) and any client app (which
// consumes them) — see docs/ble-config.md.

export interface ScannedDevice {
  bleId: string; // transient BLE connection id (BleClient.connect target)
  name: string | null;
}

export interface IdentityPayload {
  external_id: string;
  firmware_version?: string;
}

export interface ManifestEntry {
  session_id: string; // the device-local SD path of the pending file — also
  // the source of the real filename; NOT a consuming app's own session id.
  byte_size: number;
  started_at: string;
  ended_at: string | null;
  // The boat/activity the operator picked at recording start over `start-rec`,
  // if any (see docs/ble-config.md). Absent = device/session-uploads defaults
  // apply, same as a direct-WiFi upload with no override.
  boat_id?: string;
  activity_id?: string;
}

// --- device_config (docs/ble-config.md) -------------------------------------

export interface E1WifiNetwork {
  ssid: string;
  pass: string; // always "" on read (write-only) — see ble-config.md
}

export interface E1Config {
  boat_id: string; // mesh-identity/log-filename label — NOT a consuming
  // app's own boat record
  unit_role: "racing_boat" | "rc_signal" | "rc_pin" | "mark" | "committee_chase" | "spare";
  api_base_url: string;
  wind_mac: string;
  wind_offset: number;
  // stop_speed_knots/start_delay_sec/stop_delay_sec are round-tripped by the
  // firmware for older cards' config.txt compatibility but unused by it
  // (docs/ble-config.md) — deliberately not modeled here.
  start_speed_knots: number;
  rtk_enabled: boolean;
  auto_cleanup_uploads: boolean;
  // Owner opt-in for automatic OTA firmware updates (docs/ota.md). A manual
  // update can still be triggered with requestOtaUpdate() (control.ts) even
  // when this is off. No update is ever attempted while the device is
  // recording.
  ota_auto_update: boolean;
  // Base URL of the OTA firmware feed the device checks (manifest.json under
  // it). Defaults to the production feed; only override for staging/
  // self-hosted.
  ota_base_url: string;
  // 1 = D1 (simple SOG/COG), 2 = D2 (nav + wind, default), 3 = D3 (wind
  // focus) — see docs/hardware.md "Display" section for what each layout
  // shows; out-of-range values are ignored by the firmware.
  display_mode: 1 | 2 | 3;
  wifi: E1WifiNetwork[];
}

export type E1ConfigPatch = Partial<Omit<E1Config, "wifi">> & { wifi?: E1WifiNetwork[] };

export type ConfigWriteErrorReason = "pairing_window_closed" | "bad_json" | "sd_busy";

export class ConfigWriteError extends Error {
  constructor(public reason: ConfigWriteErrorReason) {
    super(`device_config write failed: ${reason}`);
  }
}

// --- status (docs/ble-config.md) --------------------------------------------

export interface E1Status {
  claimed: boolean;
  // Running firmware version, YYYY.MM.DD.N (date + daily build). Stable,
  // always-present read — the basis for an OTA "is a newer build available?"
  // decision once OTA ships (docs/ota.md).
  firmware_version: string;
  uptime_s: number;
  heap_free: number;
  battery: { pct: number; v: number; critical: boolean };
  sd_ok: boolean;
  wifi: { connected: boolean; ssid?: string; ip?: string };
  sensors: { imu: boolean; pressure: boolean; wind: boolean };
  gps: {
    fix: boolean;
    satellites: number;
    hdop: number;
    lat: number;
    lon: number;
    speed_kts: number;
    course: number;
  };
  wind: { connected: boolean; speed_kts?: number; angle_deg?: number; battery?: number };
  // elapsed_s is only present while logging is true (docs/ble-config.md).
  recording: { logging: boolean; session_count: number; pending_uploads: number; elapsed_s?: number };
  // OTA state (docs/ota.md) — reflects the last automatic or manual
  // (requestOtaUpdate()) update check. `progress` is 0-100 while
  // downloading, absent otherwise. `message` is a short human-readable
  // detail, present on "error"/"suspended" and while "downloading".
  ota: {
    state: "idle" | "checking" | "up_to_date" | "downloading" | "applying" | "suspended" | "error";
    progress?: number;
    message?: string;
  };
}

// --- control command replies ------------------------------------------------

export interface CalibrateResult {
  status: "ok" | "error";
  heel_offset?: number;
  pitch_offset?: number;
  reason?: "no_imu" | "sd_busy";
}

export interface RecCommandResult {
  ok: boolean;
  logging: boolean;
}

// Ack for requestOtaUpdate() (control.ts) — only confirms the device queued
// the check, e.g. "ok: false, message: recording in progress". Poll
// readStatus(...).ota for the actual outcome (docs/ota.md).
export interface OtaUpdateResult {
  ok: boolean;
  message?: string;
}
