// GNSS (Waveshare LG290P) — NMEA parsing, PQTM command helpers, and
// RTK-aware read/config entry points. See docs/RTK_PHASE2_DESIGN.md and
// LG290P_CONFIG_UPDATES.md (ported to docs/gnss-rtk.md in this repo).
#ifndef SAILFRAMES_GNSS_H
#define SAILFRAMES_GNSS_H

#include <Arduino.h>

struct GPSData {
  // lat/lon are DOUBLE, not float: a float32 near 42° has ~0.4 m resolution
  // (worse at the atof parse step) — it silently quantizes away the cm RTK fix
  // before the value is ever stored, capping OCS at ~0.5 m. Double preserves it.
  double lat = 0, lon = 0;
  float alt = 0;
  float speed_kts = 0, course = 0, hdop = 99.9;
  int satellites = 0, fix_quality = 0;
  char utc_time[12] = "000000.00";
  char date[8] = "010100";
  bool valid = false;
  bool newGGA = false;
  // GST 1-sigma position error std-devs in metres (LG290P; 0 until GST parses).
  float lat_std = 0, lon_std = 0, alt_std = 0;
  // Unified horizontal 1-sigma accuracy (m), 0 = no data. Set from GST
  // (LG290P, = sqrt(lat_std^2+lon_std^2)) OR from $PQTMEPE EPE_2D (LC29HEA,
  // which supports neither GST nor float GST). The whole system reads this.
  float hacc_m = 0;
};

extern GPSData gps;

// Raw NMEA line assembly state (Serial2 byte stream -> parseNMEA lines).
extern char nmeaBuf[256];
extern int  nmeaIdx;

// GSV constellation satellite-in-view counts, and last-valid-fix timestamp.
extern int satsInView;
extern int gsvGP, gsvGL, gsvGA, gsvGB, gsvGQ, gsvGI;
extern unsigned long lastValidGPS;

// Sends a PQTM command with checksum + reads back the response. Used by
// the LG290P config sequence below.
bool sendPQTM(const char* body);

// Full LG290P bring-up: Rover mode @ 10 Hz NMEA (the byte-identical legacy
// path used whenever RTK is off).
void configureLG290P();
// E rover: configureLG290P() + RTK relative mode + GST accuracy output.
void lg290pConfigRover();
// E base (rc_signal): Base mode + survey-in + RTCM3 MSM7 out @ 1 Hz.
void lg290pConfigBase();
// Single entry point used at boot + the `gps` reconfig command. With RTK
// off this is exactly configureLG290P() (byte-identical legacy path).
void gnssConfigure();

// NMEA-only read (RTCM3 capture retired in firmware .09).
void readGPS();
// RTK base read path: demuxes RTCM3 (binary) interleaved with 1 Hz NMEA.
void readGPSBase();

bool getField(const char* s, int n, char* out, int mx);
void parseNMEA(const char* s);

// Formats gps.utc_time + gps.date into ISO8601. Returns false if either
// field is not yet populated.
bool formatGpsIso(char* out, size_t outSize);

#endif  // SAILFRAMES_GNSS_H
