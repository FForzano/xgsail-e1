// GNSS (Waveshare LG290P) glue — see gnss.h.
#include "gnss.h"
#include "config.h"
#include "v2_types.h"
#include "rtk_relay.h"

GPSData gps;

char nmeaBuf[256];
int  nmeaIdx = 0;

int satsInView = 0;
int gsvGP = 0, gsvGL = 0, gsvGA = 0, gsvGB = 0, gsvGQ = 0, gsvGI = 0;
unsigned long lastValidGPS = 0;  // Track when we last had a valid fix

bool sendPQTM(const char* body) {
  uint8_t cs = 0;
  for (int i = 0; body[i] != '\0'; i++) cs ^= body[i];
  char buf[128];
  snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, cs);

  // Flush any pending data before sending
  while (Serial2.available()) Serial2.read();

  Serial2.print(buf);
  Serial.printf("[CMD] %s", buf);

  // Wait longer for response (some commands take time)
  delay(100);

  // Read all response lines (may be multiple)
  char resp[256];
  int idx = 0;
  int lineCount = 0;
  bool gotOK = false;
  bool gotError = false;
  unsigned long start = millis();

  while (millis() - start < 500 && lineCount < 3) {
    if (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n') {
        if (idx > 0) {
          resp[idx] = '\0';
          // Check for PQTM response (not NMEA)
          if (resp[0] == '$' && resp[1] == 'P') {
            Serial.printf("[RSP] %s\n", resp);
            // Check for error response
            if (strstr(resp, "ERROR") || strstr(resp, "NACK")) {
              Serial.printf("[GPS] FAILED: %s\n", body);
              gotError = true;
            } else if (strstr(resp, "OK") || strstr(resp, "PQTM")) {
              gotOK = true;
            }
          }
          idx = 0;
          lineCount++;
        }
      } else if (c != '\r' && idx < (int)sizeof(resp) - 1) {
        resp[idx++] = c;
      }
    }
  }

  // Show if no response received
  if (lineCount == 0) {
    Serial.printf("[RSP] (no response for: %s)\n", body);
  }

  return gotOK && !gotError;
}

void configureLG290P() {
  // Configures the Waveshare LG290P for 10 Hz NMEA-only operation.
  //
  // PPK / RTCM3 raw-observation capture was removed in 2026.05.20.09 —
  // see docs/RTCM_PPK_ARCHIVE.md for the previous architecture and
  // git SHA 08cdadfe (firmware .08) for the last working PPK-era
  // configureLG290P + parsers + upload path. The trade-off:
  //   * 10 Hz nav fixes are critical for on-the-water OCS (over-line
  //     detection at race start) and per-tack motion analysis.
  //   * PPK gave decimeter accuracy *after* the race — useful but the
  //     LG290P's "MSM in Base mode only" lock meant we couldn't have
  //     both. OCS won.
  // The LG290P drives NMEA at the fix rate, so 10 Hz fix = 10 Hz
  // RMC/GGA/GSA/GSV. Rover mode is required to unlock the 10 Hz rate
  // per LG290P&LGx80P Protocol Spec v1.1 §2.3.28.
  Serial.println("[GPS] Configuring LG290P for Rover @ 10 Hz NMEA...");

  // Step 1: Query firmware version (for boot.log forensics)
  Serial.println("[GPS] Querying firmware version...");
  sendPQTM("PQTMVERNO");

  // Step 2: Check current receiver mode
  Serial.println("[GPS] Checking receiver mode...");
  sendPQTM("PQTMCFGRCVRMODE,R");
  delay(300);

  // Step 3: Set Rover mode (unlocks 10 Hz fix rate)
  Serial.println("[GPS] Setting Rover mode...");
  sendPQTM("PQTMCFGRCVRMODE,W,1");
  delay(200);

  // Step 4: Enable NMEA messages. Rover mode keeps NMEA on by default
  // but explicit rates ensure a previous base-mode session (which
  // auto-disabled NMEA) re-enables cleanly.
  Serial.println("[GPS] Enabling NMEA messages...");
  sendPQTM("PQTMCFGMSGRATE,W,GGA,1");
  sendPQTM("PQTMCFGMSGRATE,W,RMC,1");
  sendPQTM("PQTMCFGMSGRATE,W,GSA,1");
  sendPQTM("PQTMCFGMSGRATE,W,GSV,1");

  // Step 5: Set fix rate to 100 ms (10 Hz). BEFORE save+restart so the
  // new rate is in NVM and applied by the same-boot restart.
  Serial.println("[GPS] Setting fix rate to 10 Hz (100 ms)...");
  sendPQTM("PQTMCFGFIXRATE,W,100");
  delay(200);

  // Step 6: Save NVM + restart to apply mode + rate together
  Serial.println("[GPS] Saving to NVM...");
  sendPQTM("PQTMSAVEPAR");
  delay(500);

  Serial.println("[GPS] Restarting module...");
  sendPQTM("PQTMSRR");
  delay(6000);

  // Drain any buffered data after restart
  while (Serial2.available()) Serial2.read();

  // Step 7: Verify — read back active configuration
  Serial.println("[GPS] Verifying configuration...");
  sendPQTM("PQTMCFGRCVRMODE,R");
  sendPQTM("PQTMCFGFIXRATE,R");

  Serial.println("[GPS] Configuration complete:");
  Serial.println("[GPS]   Mode: Rover @ 10 Hz");
  Serial.println("[GPS]   NMEA: GGA, RMC, GSA, GSV @ 10 Hz");
  Serial.println("[GPS]   RTCM3: disabled (PPK retired — see docs/RTCM_PPK_ARCHIVE.md)");
}

void lg290pConfigRover() {
  configureLG290P();                       // unchanged: rover, 10 Hz, NMEA on (+save+restart)
  Serial.println("[GPS] Enabling RTK rover (PQTMCFGRTK,W,1,2,120) + GST accuracy...");
  sendPQTM("PQTMCFGRTK,W,1,2,120");        // DiffMode=Auto, RelMode=relative, 120 s diff-age
  // Accuracy output — enable BOTH so the SAME rover config works on either chip
  // (this config also runs on LC29HEA boats left as hardware_platform=e1):
  //   LG290P  -> GST (2-param form); LC29HEA NAKs it.
  //   LC29HEA -> PQTMEPE (3-param form); LG290P NAKs it.
  sendPQTM("PQTMCFGMSGRATE,W,GST,1");       // LG290P GST -> gps.hacc_m
  sendPQTM("PQTMCFGMSGRATE,W,PQTMEPE,1,2"); // LC29HEA $PQTMEPE EPE_2D -> gps.hacc_m
  sendPQTM("PQTMSAVEPAR");
  delay(300);
}

void lg290pConfigBase() {
  Serial.println("[GPS] Configuring LG290P as RTK BASE (1 Hz, MSM7 out)...");
  sendPQTM("PQTMCFGRCVRMODE,W,2");         // base mode (locks 1 Hz)
  delay(200);
  sendPQTM("PQTMCFGSVIN,W,1,60,0,0,0,0");  // survey-in: short/loose (base err is common-mode)
  delay(200);
  sendPQTM("PQTMSAVEPAR");
  delay(400);
  sendPQTM("PQTMSRR");                      // restart to apply base mode
  delay(6000);
  while (Serial2.available()) Serial2.read();
  // post-restart: RTCM out (non-persistent → re-issue each boot) + re-enable NMEA
  sendPQTM("PQTMCFGRTCM,W,7,0,-90,07,06,1,0");  // MSM7 1077/1087/1097/1127 + eph + 1006
  delay(200);
  sendPQTM("PQTMCFGMSGRATE,W,GGA,1");       // base mode auto-disables NMEA; RC still needs GGA
  sendPQTM("PQTMCFGMSGRATE,W,RMC,1");
  sendPQTM("PQTMCFGMSGRATE,W,GSA,1");        // GSA -> base hdop for the RC panel
  sendPQTM("PQTMCFGMSGRATE,W,GST,1");        // LG290P base accuracy -> gps.hacc_m
  sendPQTM("PQTMCFGMSGRATE,W,PQTMEPE,1,2");  // (if base is ever LC29HEA) -> gps.hacc_m
  Serial.println("[GPS] LG290P base: MSM7 + 1006 + ephemeris @ 1 Hz");
}

// Single entry point used at boot + on the `gps` reconfig command. With RTK
// off this is EXACTLY configureLG290P() (byte-identical legacy path).
void gnssConfigure() {
  if (!config.rtk_enabled) {
    configureLG290P();
    return;
  }
  if (roleIsBase()) lg290pConfigBase(); else lg290pConfigRover();
}

void readGPS() {
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    if (c == '$') {
      nmeaIdx = 0;
      nmeaBuf[nmeaIdx++] = c;
    } else if (c == '\n' || c == '\r') {
      if (nmeaIdx > 5) {
        nmeaBuf[nmeaIdx] = '\0';
        parseNMEA(nmeaBuf);
        nmeaIdx = 0;
      }
    } else if (nmeaIdx < (int)sizeof(nmeaBuf) - 1) {
      nmeaBuf[nmeaIdx++] = c;
    }
  }
}

void readGPSBase() {
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    if (g_rtcmTx.feed(c)) continue;          // consumed as part of an RTCM frame
    if (c == '$') {
      nmeaIdx = 0; nmeaBuf[nmeaIdx++] = c;
    } else if (c == '\n' || c == '\r') {
      if (nmeaIdx > 5) { nmeaBuf[nmeaIdx] = '\0'; parseNMEA(nmeaBuf); nmeaIdx = 0; }
    } else if (nmeaIdx < (int)sizeof(nmeaBuf) - 1) {
      nmeaBuf[nmeaIdx++] = c;
    }
  }
}

bool getField(const char* s, int n, char* out, int mx) {
  int f = 0, i = 0, o = 0;
  while (s[i]) {
    if (s[i] == ',') {
      if (++f == n) {
        i++;
        while (s[i] && s[i] != ',' && s[i] != '*' && o < mx - 1)
          out[o++] = s[i++];
        out[o] = '\0';
        return o > 0;
      }
    }
    i++;
  }
  return false;
}

void parseNMEA(const char* s) {
  if (strstr(s, "GGA")) {
    char f[32];
    if (getField(s, 1, f, sizeof(f))) strncpy(gps.utc_time, f, sizeof(gps.utc_time) - 1);
    if (getField(s, 2, f, sizeof(f))) {
      double raw = atof(f);              // double: preserve ddmm.mmmmmm to cm
      int deg = (int)(raw / 100);
      gps.lat = deg + (raw - deg * 100) / 60.0;
      char ns[4];
      if (getField(s, 3, ns, sizeof(ns)) && ns[0] == 'S') gps.lat = -gps.lat;
    }
    if (getField(s, 4, f, sizeof(f))) {
      double raw = atof(f);              // double: preserve ddmm.mmmmmm to cm
      int deg = (int)(raw / 100);
      gps.lon = deg + (raw - deg * 100) / 60.0;
      char ew[4];
      if (getField(s, 5, ew, sizeof(ew)) && ew[0] == 'W') gps.lon = -gps.lon;
    }
    if (getField(s, 6, f, sizeof(f))) {
      int fq = atoi(f);
      // Validate: 0=none, 1=GPS, 2=DGPS, 4=RTK, 5=RTK float
      if (fq >= 0 && fq <= 5) {
        gps.fix_quality = fq;
        gps.valid = fq > 0;
        if (gps.valid) lastValidGPS = millis();
      }
    }
    if (getField(s, 7, f, sizeof(f))) {
      int sats = atoi(f);
      if (sats >= 0 && sats <= 50) gps.satellites = sats;
    }
    if (getField(s, 8, f, sizeof(f))) {
      float hdop = atof(f);
      if (hdop > 0.1 && hdop < 50) gps.hdop = hdop;  // HDOP is never 0 or near-zero
    }
    if (getField(s, 9, f, sizeof(f))) gps.alt = atof(f);
    gps.newGGA = true;
  } else if (strstr(s, "RMC")) {
    char f[32];
    if (getField(s, 7, f, sizeof(f))) {
      float spd = atof(f);
      if (spd >= 0 && spd < 100) gps.speed_kts = spd;  // Reject impossible speeds (>100kt)
    }
    if (getField(s, 8, f, sizeof(f))) {
      float crs = atof(f);
      if (crs >= 0 && crs <= 360) gps.course = crs;  // Reject invalid course
    }
    if (getField(s, 9, f, sizeof(f))) strncpy(gps.date, f, sizeof(gps.date) - 1);
  } else if (strstr(s, "GSV")) {
    // GSV sentences: GPGSV (GPS), GLGSV (GLONASS), GAGSV (Galileo), GBGSV (BeiDou), GNGSV (combined)
    // Field 1 = total messages, Field 2 = message number, Field 3 = total sats in view
    // Only parse if sentence looks valid (starts with $G and has reasonable length)
    if (s[0] == '$' && s[1] == 'G' && strlen(s) > 20) {
      char f[32];
      if (getField(s, 2, f, sizeof(f))) {
        int msgNum = atoi(f);
        if (msgNum == 1) {  // First message in sequence has total count
          if (getField(s, 3, f, sizeof(f))) {
            int count = atoi(f);
            // Sanity check: count should be 0-50
            if (count >= 0 && count <= 50) {
              // Track each constellation separately (global vars for status display)
              if (strstr(s, "GPGSV")) {
                gsvGP = count;
              } else if (strstr(s, "GLGSV")) {
                gsvGL = count;
              } else if (strstr(s, "GAGSV")) {
                gsvGA = count;
              } else if (strstr(s, "GBGSV")) {
                gsvGB = count;
              } else if (strstr(s, "GQGSV")) {
                gsvGQ = count;  // QZSS
              } else if (strstr(s, "GIGSV")) {
                gsvGI = count;  // NavIC
              }

              // Sum all constellations
              satsInView = gsvGP + gsvGL + gsvGA + gsvGB + gsvGQ + gsvGI;
            }
          }
        }
      }
    }
  } else if (strstr(s, "GST")) {
    // GST — position error statistics (RTK Phase-2 accuracy readout). Fields:
    // 6 = latitude σ (m), 7 = longitude σ (m), 8 = altitude σ (m). Horizontal
    // 1σ ≈ √(latσ²+lonσ²); ~1-2 cm at RTK FIXED, ~decimetres-metre at FLOAT.
    // Enabled only on the RTK rover path (PQTMCFGMSGRATE,W,GST,1).
    char f[32];
    if (getField(s, 6, f, sizeof(f))) { float v = atof(f); if (v >= 0 && v < 1000) gps.lat_std = v; }
    if (getField(s, 7, f, sizeof(f))) { float v = atof(f); if (v >= 0 && v < 1000) gps.lon_std = v; }
    if (getField(s, 8, f, sizeof(f))) { float v = atof(f); if (v >= 0 && v < 1000) gps.alt_std = v; }
    gps.hacc_m = sqrtf(gps.lat_std * gps.lat_std + gps.lon_std * gps.lon_std);
  } else if (strstr(s, "PQTMEPE")) {
    // LC29HEA accuracy: $PQTMEPE,<ver>,<N>,<E>,<D>,<2D>,<3D>. Field 5 = horizontal
    // (2D) error in metres. The LC29HEA supports neither GST nor float-GST, so
    // this is its accuracy source (enabled via PQTMCFGMSGRATE,W,PQTMEPE,1,2).
    char f[32];
    if (getField(s, 5, f, sizeof(f))) { float v = atof(f); if (v >= 0 && v < 1000) gps.hacc_m = v; }
  }
}

bool formatGpsIso(char* out, size_t outSize) {
  if (strlen(gps.utc_time) < 6) return false;
  if (strlen(gps.date) < 6) return false;
  if (gps.date[4] == '0' && gps.date[5] == '0') return false;  // year 00 = invalid
  snprintf(out, outSize, "20%c%c-%c%c-%c%cT%c%c:%c%c:%c%cZ",
           gps.date[4], gps.date[5],          // YY
           gps.date[2], gps.date[3],          // MM
           gps.date[0], gps.date[1],          // DD
           gps.utc_time[0], gps.utc_time[1],  // HH
           gps.utc_time[2], gps.utc_time[3],  // MM
           gps.utc_time[4], gps.utc_time[5]); // SS
  return true;
}
