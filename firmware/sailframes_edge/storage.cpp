// SD-card session logging glue — see storage.h.
#include "storage.h"
#include "config.h"
#include "gnss.h"
#include "imu.h"
#include "pressure.h"
#include "shared_state.h"
#include "console.h"

bool sdOK = false;
bool logging = false;
File navFile, imuFile, windFile, presFile;

unsigned long logStart = 0;
unsigned long totalBytes = 0;

const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:    return "POWERON";
    case ESP_RST_EXT:        return "EXT";
    case ESP_RST_SW:         return "SW";
    case ESP_RST_PANIC:      return "PANIC";
    case ESP_RST_INT_WDT:    return "INT_WDT";
    case ESP_RST_TASK_WDT:   return "TASK_WDT";
    case ESP_RST_WDT:        return "WDT";
    case ESP_RST_DEEPSLEEP:  return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:   return "BROWNOUT";
    case ESP_RST_SDIO:       return "SDIO";
    default:                 return "UNKNOWN";
  }
}

void appendBootLog(const char* line) {
  if (!sdOK) return;
  if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;
  File f = SD.open("/boot.log", FILE_APPEND);
  if (f) {
    f.println(line);
    f.close();
  }
  if (sdMutex) xSemaphoreGive(sdMutex);
}

int getNextSessionNumber() {
  int n = 1;
  File f = SD.open("/sf/session.txt", FILE_READ);
  if (f) {
    n = f.parseInt() + 1;
    f.close();
  }
  f = SD.open("/sf/session.txt", FILE_WRITE);
  if (f) {
    f.print(n);
    f.close();
  }
  return n;
}

// Writes `<dd>/<name>.txt` with `value`, if `value` is non-null/non-empty.
static void writeSessionMarker(const char* dd, const char* name, const char* value) {
  if (!value || value[0] == '\0') return;
  char markerPath[80];
  snprintf(markerPath, sizeof(markerPath), "%s/%s.txt", dd, name);
  File marker = SD.open(markerPath, FILE_WRITE);
  if (marker) {
    marker.print(value);
    marker.close();
  } else {
    Serial.printf("[LOG] Failed to write %s\n", markerPath);
  }
}

void startLogging(const char* boatIdOverride, const char* activityIdOverride) {
  Serial.println("[LOG] Starting logging...");

  // Folder naming: GPS datetime preferred, session counter as fallback
  char dd[32], ds[20], ts[12];
  // Check for valid GPS date: year portion (chars 4-5) must not be "00" (default)
  // Date format is DDMMYY, so "050426" = April 5, 2026
  bool hasGpsDate = (strlen(gps.date) >= 6 && (gps.date[4] != '0' || gps.date[5] != '0'));
  // Check for valid GPS time: must have a fix and time length OK
  bool hasGpsTime = gps.valid && (strlen(gps.utc_time) >= 6);

  if (hasGpsDate && hasGpsTime) {
    // Best case: GPS date + time as folder name (e.g., /sf/20260402_163325/)
    snprintf(dd, sizeof(dd), "/sf/20%c%c%c%c%c%c_%c%c%c%c%c%c",
      gps.date[4], gps.date[5], gps.date[2], gps.date[3], gps.date[0], gps.date[1],
      gps.utc_time[0], gps.utc_time[1], gps.utc_time[2],
      gps.utc_time[3], gps.utc_time[4], gps.utc_time[5]);
    snprintf(ds, sizeof(ds), "20%c%c%c%c%c%c",
      gps.date[4], gps.date[5], gps.date[2], gps.date[3], gps.date[0], gps.date[1]);
    snprintf(ts, sizeof(ts), "%c%c%c%c%c%c",
      gps.utc_time[0], gps.utc_time[1], gps.utc_time[2],
      gps.utc_time[3], gps.utc_time[4], gps.utc_time[5]);
  } else {
    // Fallback: sequential session number (e.g., /sf/session_001/)
    int sessionNum = getNextSessionNumber();
    snprintf(dd, sizeof(dd), "/sf/session_%03d", sessionNum);
    snprintf(ds, sizeof(ds), "s%03d", sessionNum);
    // Use millis for timestamp portion
    snprintf(ts, sizeof(ts), "%06lu", (millis() / 1000) % 1000000);
    Serial.printf("[LOG] No GPS datetime, using session_%03d\n", sessionNum);
  }

  // Create directories
  Serial.println("[LOG] Creating /sf directory...");
  if (!SD.mkdir("/sf")) {
    Serial.println("[LOG] /sf mkdir failed (may already exist)");
  }
  Serial.printf("[LOG] Creating %s directory...\n", dd);
  if (!SD.mkdir(dd)) {
    Serial.printf("[LOG] %s mkdir failed (may already exist)\n", dd);
  }

  // Session-scoped boat/activity overrides (see startLogging()'s doc
  // comment) — written once here, read back by sessionBoatId()/
  // sessionActivityId() at upload time. No marker at all means "device's
  // defaults", matching the backend falling back to device.owner_boat_id
  // / a fresh solo activity when session-uploads omits the field.
  writeSessionMarker(dd, "boat_id", boatIdOverride);
  writeSessionMarker(dd, "activity_id", activityIdOverride);

  // Build file paths (RTCM3 raw capture retired in .09 — see archive doc)
  char np[64], ip[64], wp[64], pp[64];
  snprintf(np, sizeof(np), "%s/%s_%s_%s_nav.csv", dd, config.boat_id, ds, ts);
  snprintf(ip, sizeof(ip), "%s/%s_%s_%s_imu.csv", dd, config.boat_id, ds, ts);
  snprintf(wp, sizeof(wp), "%s/%s_%s_%s_wind.csv", dd, config.boat_id, ds, ts);
  snprintf(pp, sizeof(pp), "%s/%s_%s_%s_pres.csv", dd, config.boat_id, ds, ts);

  Serial.printf("[LOG] Opening NAV: %s\n", np);
  navFile = SD.open(np, FILE_WRITE);
  Serial.printf("[LOG] NAV file %s\n", navFile ? "OK" : "FAILED");

  Serial.printf("[LOG] Opening IMU: %s\n", ip);
  imuFile = SD.open(ip, FILE_WRITE);
  Serial.printf("[LOG] IMU file %s\n", imuFile ? "OK" : "FAILED");

#if ENABLE_WIND
  if (config.wind_enabled) {
    Serial.printf("[LOG] Opening WIND: %s\n", wp);
    windFile = SD.open(wp, FILE_WRITE);
    Serial.printf("[LOG] WIND file %s\n", windFile ? "OK" : "FAILED");
  }
#endif

  // Pressure file (always open if sensor is present)
  if (presOK) {
    Serial.printf("[LOG] Opening PRES: %s\n", pp);
    presFile = SD.open(pp, FILE_WRITE);
    Serial.printf("[LOG] PRES file %s\n", presFile ? "OK" : "FAILED");
  }

  if (navFile) {
    logging = true;
    logStart = millis();
    navFile.println("ms,utc,lat,lon,alt,sog,cog,sat,hdop,fix,gps_date,hacc");
    navFile.flush();
    if (imuFile) {
      imuFile.println("ms,utc,ax,ay,az,gx,gy,gz,lax,lay,laz,mx,my,mz,heel,pitch,heading,stability,accuracy");
      imuFile.flush();
    }
#if ENABLE_WIND
    if (windFile) {
      windFile.println("ms,utc,aws_kts,aws_mps,awa_deg,battery");
      windFile.flush();
    }
#endif
    if (presFile) {
      presFile.println("ms,utc,date,pressure_hpa,temp_c,pres_min,pres_max");
      presFile.flush();
      resetPressureMinMax();  // Start fresh min/max tracking
    }
    Serial.println("[LOG] ========================================");
    Serial.printf("[LOG] NAV: %s\n", np);
    Serial.printf("[LOG] IMU: %s\n", ip);
#if ENABLE_WIND
    if (config.wind_enabled) Serial.printf("[LOG] WIND: %s\n", wp);
#endif
    if (presOK) Serial.printf("[LOG] PRES: %s\n", pp);
    Serial.println("[LOG] ========================================");
  } else {
    Serial.println("[LOG] ERROR: Failed to open NAV file!");
    Serial.println("[LOG] Check SD card is properly inserted and formatted FAT32");
  }
}

// ============================================================
// LOG NAV + IMU
// ============================================================
void logNav() {
  if (!navFile || !logging) return;
  sdWriting = true;
  unsigned long e = millis() - logStart;
  // hacc = horizontal 1-sigma (m); GST (LG290P) or PQTMEPE (LC29HEA); 0 = none.
  float hacc = gps.hacc_m;
  navFile.printf("%lu,%s,%.10f,%.10f,%.3f,%.3f,%.2f,%d,%.2f,%d,%s,%.3f\n",
    e, gps.utc_time, gps.lat, gps.lon, gps.alt,
    gps.speed_kts, gps.course, gps.satellites, gps.hdop, gps.fix_quality, gps.date, hacc);
  totalBytes += 98;
  sdWriting = false;
}

String sessionStartedAtIso(const char* filepath) {
  String path = String(filepath);
  int sfIdx = path.indexOf("/sf/");
  if (sfIdx >= 0) {
    int start = sfIdx + 4;
    int slash = path.indexOf('/', start);
    String folder = (slash >= 0) ? path.substring(start, slash) : path.substring(start);
    if (folder.length() == 15 && folder.charAt(8) == '_') {
      return folder.substring(0, 4) + "-" + folder.substring(4, 6) + "-" + folder.substring(6, 8) +
             "T" + folder.substring(9, 11) + ":" + folder.substring(11, 13) + ":" + folder.substring(13, 15) + "Z";
    }
  }
  char iso[24];
  if (formatGpsIso(iso, sizeof(iso))) return String(iso);
  return "";
}

// Shared by sessionBoatId()/sessionActivityId(): finds `filepath`'s
// enclosing "/sf/<folder>/" directory and reads back "<folder>/<name>.txt"
// if it exists, else "".
static String readSessionMarker(const char* filepath, const char* name) {
  String path = String(filepath);
  int sfIdx = path.indexOf("/sf/");
  if (sfIdx < 0) return "";
  int start = sfIdx + 4;
  int slash = path.indexOf('/', start);
  if (slash < 0) return "";
  String markerPath = path.substring(0, slash) + "/" + name + ".txt";
  File marker = SD.open(markerPath.c_str(), FILE_READ);
  if (!marker) return "";
  String value = marker.readString();
  marker.close();
  value.trim();
  return value;
}

String sessionBoatId(const char* filepath) {
  return readSessionMarker(filepath, "boat_id");
}

String sessionActivityId(const char* filepath) {
  return readSessionMarker(filepath, "activity_id");
}

void logIMU() {
  if (!imuFile || !logging) return;
  sdWriting = true;
  unsigned long e = millis() - logStart;
  if (g_imuFailed) {
    // BNO has gone silent — every field would be stale. Write empty
    // cells for the derived/orientation fields so downstream (dashboard
    // parseFloat → NaN → row skipped) doesn't treat the stale value as
    // a real reading. Keep ms + utc_time so the row alignment with GPS
    // is preserved for forensic inspection.
    imuFile.printf("%lu,%s,,,,,,,,,,,,,,,,,\n", e, gps.utc_time);
    totalBytes += 30;
  } else {
    imuFile.printf("%lu,%s,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u\n",
      e, gps.utc_time,
      imu.accel_x, imu.accel_y, imu.accel_z,           // Raw acceleration (with gravity)
      imu.gyro_x, imu.gyro_y, imu.gyro_z,              // Angular velocity (deg/s)
      imu.linaccel_x, imu.linaccel_y, imu.linaccel_z,  // Linear acceleration (no gravity)
      imu.mag_x, imu.mag_y, imu.mag_z,                 // Magnetic field (uT)
      imu.heel, imu.pitch, imu.heading,                // Orientation
      imu.stability, imu.accuracy);                    // Motion state & calibration quality
    totalBytes += 210;
  }
  sdWriting = false;
}

// ============================================================

void listDirOutput(const char* dirname, int depth, bool toTelnet) {
  File root = SD.open(dirname);
  if (!root || !root.isDirectory()) {
    tprintf("Failed to open %s\n", dirname);
    return;
  }

  File file = root.openNextFile();
  while (file) {
    char indent[32] = "";
    for (int i = 0; i < depth && i < 10; i++) strcat(indent, "  ");
    if (file.isDirectory()) {
      tprintf("%s[DIR]  %s/\n", indent, file.name());
      char path[128];
      snprintf(path, sizeof(path), "%s/%s", dirname, file.name());
      file.close();  // Close before recursing to free file descriptor
      listDirOutput(path, depth + 1, toTelnet);
    } else {
      tprintf("%s[FILE] %s (%lu bytes)\n", indent, file.name(), file.size());
      file.close();  // Close file after reading info
    }
    file = root.openNextFile();
    yield();
  }
  root.close();  // Close directory when done
}

