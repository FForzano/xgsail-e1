// Main LiPo battery monitoring (DWEII USB-C Boost Converter), via a
// 100K/100K voltage divider on GPIO34.
#ifndef SAILFRAMES_BATTERY_H
#define SAILFRAMES_BATTERY_H

#include <Arduino.h>

struct BatteryData {
  float voltage = 0;        // Battery voltage (3.0-4.2V for LiPo)
  int percent = 0;          // Estimated percentage (0-100)
  bool critical = false;    // Voltage below 3.3V (overdischarge threshold)
  bool valid = false;       // Have we read the battery yet?
  unsigned long lastRead = 0;
};

extern BatteryData battery;

void setupBattery();
float readBatteryVoltage();
int getBatteryPercent(float voltage);
bool isBatteryCritical();
void updateBattery();
// Halts the device (hardware power switch is the only way off) once the
// battery is critical, after flushing/closing any open session files.
void handleLowBattery();

#endif  // SAILFRAMES_BATTERY_H
