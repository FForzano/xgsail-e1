// DPS310 barometric pressure/temperature — used for gust detection via
// min/max pressure tracked over a rolling window.
#ifndef SAILFRAMES_PRESSURE_H
#define SAILFRAMES_PRESSURE_H

#include <Arduino.h>
#include <Adafruit_DPS310.h>

struct PressureData {
  float pressure_hpa = 0;   // Barometric pressure in hPa (mbar)
  float temperature_c = 0;  // Temperature in Celsius
  float pressure_min = 9999; // Min pressure in current window (for gust detection)
  float pressure_max = 0;   // Max pressure in current window
  bool valid = false;
  unsigned long lastRead = 0;
};

extern PressureData pressure;
extern bool presOK;
extern Adafruit_DPS310 dps;

void readPressure();
void logPressure();
// Resets the gust-detection min/max window (call periodically, e.g. every
// 10 seconds).
void resetPressureMinMax();

#endif  // SAILFRAMES_PRESSURE_H
