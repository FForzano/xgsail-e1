// Recording state machine glue — see recording.h.
#include "recording.h"
#include "config.h"
#include "gnss.h"
#include "storage.h"
#include "upload.h"
#include "shared_state.h"

RecordState recState = REC_IDLE;
int sessionCount = 0;

float startSpeedKnots = 1.5;

void applyRecordingThresholds() {
  startSpeedKnots = config.start_speed_knots;
}

bool startRecording(const char* boatId, const char* activityId) {
  if (recState == REC_RECORDING) return false;
  if (!sdOK) {
    Serial.println("[REC] SD card not available, can't start");
    return false;
  }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) {
    Serial.println("[REC] SD busy, try again");
    return false;
  }
  sessionCount++;
  startLogging(boatId, activityId);
  xSemaphoreGive(sdMutex);
  recState = REC_RECORDING;
  Serial.printf("[REC] Recording STARTED — session %d\n", sessionCount);
  return true;
}

bool stopRecording() {
  if (recState != REC_RECORDING) return false;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) {
    navFile.flush(); navFile.close();
    imuFile.flush(); imuFile.close();
    if (windFile) { windFile.flush(); windFile.close(); }
    if (presFile) { presFile.flush(); presFile.close(); }
    xSemaphoreGive(sdMutex);
  } else {
    Serial.println("[REC] SD busy closing session files — closing unlocked");
  }
  logging = false;
  recState = REC_IDLE;
  triggerUpload = true;
  Serial.printf("[REC] Recording STOPPED — session %d\n", sessionCount);
  return true;
}

void toggleRecording() {
  if (recState == REC_RECORDING) stopRecording();
  else startRecording();
}

const char* getRecStateStr() {
  switch (recState) {
    case REC_IDLE: return gps.valid ? "READY" : "NO GPS";
    case REC_RECORDING: return "REC";
    default: return "?";
  }
}
