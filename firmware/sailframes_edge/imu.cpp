// BNO085 IMU glue — see imu.h.
#include "imu.h"
#include "config.h"
#include "gnss.h"
#include "storage.h"

IMUData imu;
bool imuOK = false;
Adafruit_BNO08x bno08x;  // No hardware reset pin
sh2_SensorValue_t sensorValue;

float imuHeelOffset = 0.0;
float imuPitchOffset = 0.0;

unsigned long g_imuLastEventMs = 0; // millis() of last successful read
int g_imuSilentReads = 0;           // consecutive readIMU() calls with 0 events
bool g_imuFailed = false;           // sticky failure flag

void readIMU() {
  if (!imuOK) return;

  // BNO085 using Adafruit library with SHTP protocol
  if (bno08x.wasReset()) {
    Serial.println("[IMU] BNO085 was reset, re-enabling reports");
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_INTERVAL_MS * 1000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, IMU_INTERVAL_MS * 1000);
    bno08x.enableReport(SH2_ACCELEROMETER, IMU_INTERVAL_MS * 1000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_INTERVAL_MS * 1000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, IMU_INTERVAL_MS * 1000);
    bno08x.enableReport(SH2_STABILITY_CLASSIFIER, 500000);
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, IMU_INTERVAL_MS * 1000);
  }

  // Read sensor events - we have 7 reports enabled so need enough reads
  int maxReads = 10;
  int eventsThisCall = 0;
  while (maxReads-- > 0 && bno08x.getSensorEvent(&sensorValue)) {
    eventsThisCall++;
    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        // Not using quaternion for heel/pitch - accelerometer is more reliable
        // Just ignore this report, we calculate from accelerometer below
        break;
      case SH2_ACCELEROMETER:
        imu.accel_x = sensorValue.un.accelerometer.x;
        imu.accel_y = sensorValue.un.accelerometer.y;
        imu.accel_z = sensorValue.un.accelerometer.z;

        // Calculate heel and pitch from accelerometer (gravity reference)
        // Chip is mounted with X pointing UP, Y pointing STARBOARD, Z pointing BOW
        // X ≈ 9.8 when level (gravity)
        // Y changes with heel (port/starboard tilt)
        // Z changes with pitch (bow up/down tilt)

        // Heel: positive = starboard down, negative = port down
        imu.heel = atan2(imu.accel_y, imu.accel_x) * 180.0 / PI;

        // Pitch: positive = bow up, negative = bow down
        imu.pitch = atan2(-imu.accel_z, sqrt(imu.accel_y * imu.accel_y + imu.accel_x * imu.accel_x)) * 180.0 / PI;

        // Apply calibration offsets
        imu.heel -= imuHeelOffset;
        imu.pitch -= imuPitchOffset;

        // Normalize to -180 to +180 range
        while (imu.heel > 180) imu.heel -= 360;
        while (imu.heel < -180) imu.heel += 360;
        while (imu.pitch > 180) imu.pitch -= 360;
        while (imu.pitch < -180) imu.pitch += 360;
        break;

      case SH2_ROTATION_VECTOR: {
        // Full rotation vector with magnetometer - use for heading only
        float qr = sensorValue.un.rotationVector.real;
        float qi = sensorValue.un.rotationVector.i;
        float qj = sensorValue.un.rotationVector.j;
        float qk = sensorValue.un.rotationVector.k;

        // Yaw (heading) = rotation around Z axis
        float siny_cosp = 2.0 * (qr * qk + qi * qj);
        float cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk);
        float heading = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;

        // Convert to 0-360 range
        if (heading < 0) heading += 360.0;
        imu.heading = heading;

        // Accuracy estimate (0-3, 3=highest calibration)
        imu.accuracy = sensorValue.status & 0x03;
        break;
      }

      case SH2_GYROSCOPE_CALIBRATED:
        // Angular velocity in rad/s - convert to deg/s for easier interpretation
        // Useful for detecting tack/gybe maneuvers (high yaw rate = turning)
        imu.gyro_x = sensorValue.un.gyroscope.x * 180.0 / PI;  // Roll rate (deg/s)
        imu.gyro_y = sensorValue.un.gyroscope.y * 180.0 / PI;  // Pitch rate (deg/s)
        imu.gyro_z = sensorValue.un.gyroscope.z * 180.0 / PI;  // Yaw/turn rate (deg/s)
        break;

      case SH2_LINEAR_ACCELERATION:
        // Acceleration with gravity removed - pure motion acceleration
        // Useful for detecting impacts, waves, sudden movements
        imu.linaccel_x = sensorValue.un.linearAcceleration.x;
        imu.linaccel_y = sensorValue.un.linearAcceleration.y;
        imu.linaccel_z = sensorValue.un.linearAcceleration.z;
        break;

      case SH2_STABILITY_CLASSIFIER:
        // Motion state: 0=Unknown, 1=OnTable, 2=Stationary, 3=Stable, 4=Motion
        // Useful for auto-detecting sailing vs at dock
        imu.stability = sensorValue.un.stabilityClassifier.classification;
        break;

      case SH2_MAGNETIC_FIELD_CALIBRATED:
        // Raw magnetic field in microtesla (uT)
        // Useful for analyzing magnetic interference from keel, rigging, engine
        // Earth's field is ~25-65 uT depending on location
        imu.mag_x = sensorValue.un.magneticField.x;
        imu.mag_y = sensorValue.un.magneticField.y;
        imu.mag_z = sensorValue.un.magneticField.z;
        break;
    }
  }

  // Health watchdog: at 1 Hz polling we expect at least one sensor
  // event per call when the BNO is alive. Track consecutive empty
  // reads and flip into "failed" after IMU_FAIL_THRESHOLD_S of silence.
  // boot.log gets one marker on each transition (failed / recovered).
  if (eventsThisCall > 0) {
    g_imuLastEventMs = millis();
    g_imuSilentReads = 0;
    if (g_imuFailed) {
      g_imuFailed = false;
      char isoBuf[24] = {0};
      bool haveIso = formatGpsIso(isoBuf, sizeof(isoBuf));
      char line[96];
      snprintf(line, sizeof(line), "imu ok t=%s recovered",
               haveIso ? isoBuf : "?");
      appendBootLog(line);
      Serial.println("[IMU] recovered, events resuming");
    }
  } else {
    g_imuSilentReads++;
    if (!g_imuFailed && g_imuSilentReads >= IMU_FAIL_THRESHOLD_S) {
      g_imuFailed = true;
      char isoBuf[24] = {0};
      bool haveIso = formatGpsIso(isoBuf, sizeof(isoBuf));
      char line[128];
      snprintf(line, sizeof(line),
               "imu fail t=%s reason=no_events silent_reads=%d",
               haveIso ? isoBuf : "?", g_imuSilentReads);
      appendBootLog(line);
      Serial.printf("[IMU] FAILED — %d s with no sensor events\n",
                    g_imuSilentReads);
    }
  }
}

// ============================================================
// CONFIG
// ============================================================
void loadIMUCalibration() {
  File f = SD.open("/imu_cal.txt", FILE_READ);
  if (!f) {
    Serial.println("[IMU] No calibration file, using defaults");
    return;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("heel_offset=")) {
      imuHeelOffset = line.substring(12).toFloat();
    } else if (line.startsWith("pitch_offset=")) {
      imuPitchOffset = line.substring(13).toFloat();
    }
  }
  f.close();
  Serial.printf("[IMU] Loaded calibration: heel=%.1f, pitch=%.1f\n",
    imuHeelOffset, imuPitchOffset);
}

void saveIMUCalibration() {
  File f = SD.open("/imu_cal.txt", FILE_WRITE);
  if (!f) {
    Serial.println("[IMU] Failed to save calibration");
    return;
  }
  f.printf("heel_offset=%.2f\n", imuHeelOffset);
  f.printf("pitch_offset=%.2f\n", imuPitchOffset);
  f.close();
  Serial.printf("[IMU] Saved calibration: heel=%.1f, pitch=%.1f\n",
    imuHeelOffset, imuPitchOffset);
}

void calibrateIMU() {
  if (!imuOK) {
    Serial.println("[IMU] No IMU available for calibration");
    return;
  }

  // Read current raw values (before offset applied)
  float rawHeel = imu.heel + imuHeelOffset;  // Undo current offset to get raw
  float rawPitch = imu.pitch + imuPitchOffset;

  // Set new offsets so current position becomes zero
  imuHeelOffset = rawHeel;
  imuPitchOffset = rawPitch;

  // Save to SD card
  saveIMUCalibration();

  Serial.printf("[IMU] Calibrated: new offsets heel=%.1f, pitch=%.1f\n",
    imuHeelOffset, imuPitchOffset);
}

// Clears both offsets back to zero and persists — shared by console.cpp's
// `calreset` command and ble_relay.cpp's `control` "calibrate-reset" cmd.
void resetIMUCalibration() {
  imuHeelOffset = 0.0;
  imuPitchOffset = 0.0;
  saveIMUCalibration();
}

