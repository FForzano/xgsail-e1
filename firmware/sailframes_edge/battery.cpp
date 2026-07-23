// Battery monitoring glue — see battery.h.
// 100K/100K voltage divider from LiPo B+ to GPIO34.
// Divider ratio: nominal 2.0 (100K/100K), calibrated to 2.25.
// ESP32 ADC has ~10-15% non-linearity without calibration.
// Calibrated: 4.165V actual = 3.70V displayed -> ratio = 4.165/3.70 * 2.0 = 2.25.
// LiPo range 3.0V-4.2V -> ADC sees 1.5V-2.1V (within ESP32 3.3V limit).
#include "battery.h"
#include "config.h"
#include "storage.h"
#include "display.h"

const float BATT_DIVIDER_RATIO = 2.25;
const int BATT_SAMPLES = 16;  // Average multiple readings for stability

BatteryData battery;

void setupBattery() {
  // GPIO34 is input-only, no internal pull-up (ideal for ADC)
  analogReadResolution(12);  // 12-bit ADC (0-4095)
  analogSetAttenuation(ADC_11db);  // Full 0-3.3V range
}

float readBatteryVoltage() {
  // Average multiple readings to reduce noise
  uint32_t sum = 0;
  for (int i = 0; i < BATT_SAMPLES; i++) {
    sum += analogRead(BATT_VOLTAGE_PIN);
    delayMicroseconds(100);
  }
  float raw = (float)sum / BATT_SAMPLES;
  float adcVoltage = (raw / 4095.0) * 3.3;  // Voltage at ADC pin
  float voltage = adcVoltage * BATT_DIVIDER_RATIO;  // Actual battery voltage
  return voltage;
}

int getBatteryPercent(float voltage) {
  // Li-ion discharge curve (non-linear lookup table)
  // Based on typical LiPo discharge profile
  // Voltage drops quickly at start and end, flat in middle
  static const float voltageTable[] = {
    4.20, 4.15, 4.10, 4.05, 4.00, 3.90, 3.80, 3.70, 3.60, 3.50, 3.40, 3.30
  };
  static const int percentTable[] = {
    100,   95,   85,   75,   65,   50,   35,   20,   12,    6,    2,    0
  };
  static const int tableSize = sizeof(voltageTable) / sizeof(voltageTable[0]);

  if (voltage >= voltageTable[0]) return 100;
  if (voltage <= voltageTable[tableSize - 1]) return 0;

  // Find bracketing points and interpolate
  for (int i = 0; i < tableSize - 1; i++) {
    if (voltage >= voltageTable[i + 1]) {
      float vHigh = voltageTable[i];
      float vLow = voltageTable[i + 1];
      int pHigh = percentTable[i];
      int pLow = percentTable[i + 1];
      // Linear interpolation between points
      float ratio = (voltage - vLow) / (vHigh - vLow);
      return pLow + (int)(ratio * (pHigh - pLow));
    }
  }
  return 0;
}

bool isBatteryCritical() {
  // Critical if voltage drops below 3.3V (overdischarge protection threshold)
  // Only consider critical if voltage is measurable (> 0.5V)
  // This prevents false shutdown when battery hardware not connected
  bool voltageValid = battery.voltage > 0.5;
  bool voltageLow = battery.voltage < 3.3;
  return voltageValid && voltageLow;
}

void updateBattery() {
  float v = readBatteryVoltage();
  battery.voltage = v;
  battery.percent = getBatteryPercent(v);
  battery.critical = isBatteryCritical();
  battery.valid = true;
  battery.lastRead = millis();
}

void handleLowBattery() {
  if (!battery.critical) return;

  Serial.println("[BATT] CRITICAL LOW BATTERY - Please flip power switch OFF!");

  // Flush and close all open files to prevent corruption
  if (logging) {
    if (navFile) { navFile.flush(); navFile.close(); }
    if (imuFile) { imuFile.flush(); imuFile.close(); }
    if (windFile) { windFile.flush(); windFile.close(); }
    if (presFile) { presFile.flush(); presFile.close(); }
    logging = false;
  }

  // Display warning - user must flip hardware power switch
  if (oledOK) {
    tft.fillScreen(COLOR_ERROR);
    tft.setTextColor(TFT_WHITE, COLOR_ERROR);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("LOW BATTERY", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 - 40, 4);
    tft.drawString("Flip power switch to OFF", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 + 20, 2);
  }

  // Halt here - user must use hardware switch
  while (true) {
    delay(1000);
  }
}
