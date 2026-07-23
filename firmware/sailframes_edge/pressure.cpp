// DPS310 pressure/temperature glue — see pressure.h.
#include "pressure.h"
#include "storage.h"
#include "gnss.h"
#include "shared_state.h"

PressureData pressure;
bool presOK = false;
Adafruit_DPS310 dps;

void readPressure() {
  if (!presOK) return;

  sensors_event_t temp_event, pressure_event;
  if (dps.getEvents(&temp_event, &pressure_event)) {
    pressure.pressure_hpa = pressure_event.pressure;
    pressure.temperature_c = temp_event.temperature;
    pressure.valid = true;
    pressure.lastRead = millis();

    // Track min/max for gust detection
    if (pressure.pressure_hpa < pressure.pressure_min) {
      pressure.pressure_min = pressure.pressure_hpa;
    }
    if (pressure.pressure_hpa > pressure.pressure_max) {
      pressure.pressure_max = pressure.pressure_hpa;
    }
  }
}

void logPressure() {
  if (!presFile || !logging || !pressure.valid) return;

  sdWriting = true;
  unsigned long e = millis() - logStart;
  // Log: elapsed_ms, utc, date, pressure_hpa, temp_c, pressure_min, pressure_max
  presFile.printf("%lu,%s,%s,%.2f,%.2f,%.2f,%.2f\n",
    e, gps.utc_time, gps.date, pressure.pressure_hpa, pressure.temperature_c,
    pressure.pressure_min, pressure.pressure_max);
  totalBytes += 80;
  sdWriting = false;
}

void resetPressureMinMax() {
  // Reset min/max tracking (call this periodically, e.g., every 10 seconds)
  pressure.pressure_min = pressure.pressure_hpa;
  pressure.pressure_max = pressure.pressure_hpa;
}
