import type { ManifestEntry } from "./types";

// The backend seam. Claiming a device and relaying its buffered sessions both
// end in HTTP calls to a server that speaks the E1 device-protocol (claim
// confirmation + DeviceKey-authenticated session uploads). `E1Backend`
// abstracts exactly those calls so the BLE orchestration in client.ts never
// hardcodes a particular server, and a project with a different backend can
// implement this one interface instead of reimplementing the relay logic.
//
// `httpBackend` below is the reference implementation of the standard
// contract — most consumers just point it at a base URL.
export interface E1Backend {
  /** Redeems a claim code for a device (identified by the `external_id` read
   * off its BLE `identity`) and returns the issued device key. The endpoint is
   * unauthenticated by contract — the claim code is the credential — so this
   * works from a signed-out client. */
  confirmClaim(
    externalId: string,
    claimCode: string,
  ): Promise<{ deviceId: string; deviceApiKey: string }>;

  /** Stores one buffered session's bytes on the backend as the device itself
   * would have over WiFi: opens a session-upload, PUTs the bytes to the
   * returned URL, and finalizes it. Authenticated as the *device* via its
   * `deviceApiKey`. `filename` must carry the file's real sensor-type suffix
   * (see client.ts `basenameOf`). */
  uploadSession(
    deviceApiKey: string,
    entry: ManifestEntry,
    filename: string,
    bytes: Uint8Array<ArrayBuffer>,
  ): Promise<void>;
}

export interface HttpBackendConfig {
  /** Base URL of the device-protocol API, e.g. `https://app.example.com/api`
   * or a same-origin `/api`. Endpoint paths are appended to it. */
  baseUrl: string;
  /** Override `fetch` (tests, non-browser runtimes). Defaults to global fetch. */
  fetchImpl?: typeof fetch;
}

/** Raw byte PUT to a self-authorising upload URL (presigned S3/MinIO or an
 * HMAC-signed proxy URL) — no cookies, no auth header, by contract. */
async function putBytes(url: string, bytes: Uint8Array<ArrayBuffer>): Promise<void> {
  await new Promise<void>((resolve, reject) => {
    const xhr = new XMLHttpRequest();
    xhr.open("PUT", url);
    xhr.onload = () => {
      if (xhr.status >= 200 && xhr.status < 300) resolve();
      else reject(new Error(`Upload failed (${xhr.status})`));
    };
    xhr.onerror = () => reject(new Error("Upload failed (network error)"));
    xhr.send(new Blob([bytes]));
  });
}

/** Reference `E1Backend` for the standard device-protocol: `POST
 * /devices/claim/confirm`, `POST /devices/me/session-uploads` + PUT +
 * finalizing `PATCH`. All calls use `credentials: "omit"` — the protocol is
 * DeviceKey/token authenticated, never cookie-based, so this stays correct
 * whether the base URL is same-origin or cross-origin. */
export function httpBackend(config: HttpBackendConfig): E1Backend {
  const baseUrl = config.baseUrl.replace(/\/$/, "");
  const doFetch = config.fetchImpl ?? fetch;
  // Origin of an absolute base URL, so a relative `upload_url` (built to work
  // same-origin on the web) resolves against the real API host inside a native
  // WebView instead of the app's virtual origin. Empty for a relative base
  // (same-origin web), leaving relative upload URLs untouched.
  const origin = /^https?:\/\//.test(baseUrl) ? new URL(baseUrl).origin : "";
  const resolveUrl = (u: string): string => (/^https?:\/\//.test(u) ? u : `${origin}${u}`);

  async function deviceApiFetch<T>(
    deviceApiKey: string,
    path: string,
    method: string,
    body?: unknown,
  ): Promise<T> {
    const res = await doFetch(`${baseUrl}${path}`, {
      method,
      credentials: "omit",
      headers: {
        Authorization: `DeviceKey ${deviceApiKey}`,
        ...(body !== undefined ? { "Content-Type": "application/json" } : {}),
      },
      body: body !== undefined ? JSON.stringify(body) : undefined,
    });
    if (!res.ok) throw new Error(`Device API ${method} ${path} failed (${res.status})`);
    if (res.status === 204) return null as T;
    return (await res.json()) as T;
  }

  return {
    async confirmClaim(externalId, claimCode) {
      const res = await doFetch(`${baseUrl}/devices/claim/confirm`, {
        method: "POST",
        credentials: "omit",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ external_id: externalId, claim_code: claimCode }),
      });
      if (!res.ok) throw new Error(`claim/confirm failed (${res.status})`);
      const body = (await res.json()) as { device_id: string; device_api_key: string };
      return { deviceId: body.device_id, deviceApiKey: body.device_api_key };
    },

    async uploadSession(deviceApiKey, entry, filename, bytes) {
      // session-uploads is idempotent on (session, device, sequence_number),
      // so a retried relay re-opens the same upload rather than duplicating it.
      const upload = await deviceApiFetch<{ session_upload_id: string; upload_url: string }>(
        deviceApiKey,
        "/devices/me/session-uploads",
        "POST",
        {
          started_at: entry.started_at,
          ended_at: entry.ended_at,
          filename,
          ...(entry.boat_id ? { boat_id: entry.boat_id } : {}),
          ...(entry.activity_id ? { activity_id: entry.activity_id } : {}),
        },
      );
      await putBytes(resolveUrl(upload.upload_url), bytes);
      await deviceApiFetch(deviceApiKey, `/devices/me/session-uploads/${upload.session_upload_id}`, "PATCH", {
        is_final: true,
      });
    },
  };
}
