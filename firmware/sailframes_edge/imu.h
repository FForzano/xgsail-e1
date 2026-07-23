// BNO085 IMU — heel/pitch/heading/motion read + on-SD calibration offsets.
#ifndef SAILFRAMES_IMU_H
#define SAILFRAMES_IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

struct IMUData {
  float accel_x = 0, accel_y = 0, accel_z = 0;       // Raw acceleration (includes gravity)
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;          // Angular velocity (deg/s)
  float linaccel_x = 0, linaccel_y = 0, linaccel_z = 0;  // Linear acceleration (gravity removed)
  float mag_x = 0, mag_y = 0, mag_z = 0;             // Magnetic field (uTesla) for interference analysis
  float heel = 0, pitch = 0, heading = 0;
  uint8_t stability = 0;    // 0=Unknown, 1=OnTable, 2=Stationary, 3=Stable, 4=Motion
  uint8_t accuracy = 0;     // Rotation vector accuracy (0-3, 3=highest)
};

extern IMUData imu;
extern bool imuOK;
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;

// IMU calibration offsets (stored on SD card as /imu_cal.txt).
extern float imuHeelOffset;
extern float imuPitchOffset;

// IMU health watchdog. The BNO085 lives on a 4-pin I2C header; if a header
// pin works loose (vibration), getSensorEvent() returns false every call
// and imu.heel keeps its last value. Track consecutive readIMU() calls
// that returned ZERO events; flip g_imuFailed after IMU_FAIL_THRESHOLD_S
// of no data so logIMU() writes empty cells instead of stale numbers.
#define IMU_FAIL_THRESHOLD_S 60     // 60 s of no events -> failed
extern unsigned long g_imuLastEventMs;
extern int  g_imuSilentReads;
extern bool g_imuFailed;

// Reads all pending BNO085 sensor events into `imu`, applying the stored
// calibration offsets and running the health watchdog above.
void readIMU();
void loadIMUCalibration();
void saveIMUCalibration();
// Zeroes heel/pitch at the current attitude and saves the new offsets.
void calibrateIMU();
// Clears both offsets back to zero and saves — used by console's
// `calreset` and the BLE relay's `control` "calibrate-reset" command.
void resetIMUCalibration();

#endif  // SAILFRAMES_IMU_H
