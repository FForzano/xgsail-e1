// Recording state machine glue — see recording.h.
#include "recording.h"
#include "config.h"
#include "gnss.h"
#include "storage.h"
#include "upload.h"
#include "shared_state.h"

RecordState recState = REC_IDLE;
unsigned long armStartTime = 0;
unsigned long stopStartTime = 0;
int sessionCount = 0;

float startSpeedKnots = 1.5;
float stopSpeedKnots = 0.5;
unsigned long startDelayMs = 10000;
unsigned long stopDelayMs = 180000;

// Power control: Use hardware slide switch on PowerBoost EN pin
// No software shutdown - hardware switch cuts all power
void updateRecordingState() {
  float speed = gps.speed_kts;
  unsigned long now = millis();

  // Use config values. Stop-threshold fields stay in the config struct
  // for backwards compatibility with existing /sf/config.txt files but
  // are no longer consumed — recording stops only on a clean operator
  // action (SPDT power-off or `stoprec` serial/telnet command).
  float startThresh = config.start_speed_knots;
  unsigned long startDelay = config.start_delay_sec * 1000UL;

  switch (recState) {
    case REC_IDLE:
      if (gps.valid && speed > startThresh) {
        recState = REC_ARMED;
        armStartTime = now;
        Serial.printf("[REC] Arming... speed=%.1f kt\n", speed);
      }
      break;

    case REC_ARMED:
      if (speed <= startThresh) {
        // Speed dropped, reset
        recState = REC_IDLE;
        Serial.println("[REC] Speed dropped, back to idle");
      } else if (uploading) {
        // Don't start recording while upload is in progress - SD card conflict
        Serial.println("[REC] Upload in progress, waiting to start recording...");
        // Stay in ARMED state, will retry next cycle
      } else if (now - armStartTime >= startDelay) {
        // Sustained speed — start recording
        sessionCount++;
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) {
          startLogging();
          xSemaphoreGive(sdMutex);
          recState = REC_RECORDING;
          Serial.printf("[REC] Recording STARTED — session %d\n", sessionCount);
        } else {
          // Mutex timeout - upload may be holding it, stay in ARMED
          Serial.println("[REC] SD busy, retrying...");
        }
      }
      break;

    case REC_RECORDING:
      // No auto-stop. Recording continues until the operator either
      // powers the device off via the SPDT slide switch or sends the
      // `stoprec` serial/telnet command. Speed-triggered stop was
      // removed because operators routinely sit at low speed (tactics
      // before start, between starts in a series, motoring back) and
      // false-stops were chopping sessions mid-race. The stationary-
      // upload path in uploadTaskFunc only fires while `!logging`, so
      // pending files upload at next boot after a clean power-cycle.
      (void)speed; (void)now;   // unused now — silence -Wunused
      break;

    case REC_STOPPING:
      // Unreachable since REC_RECORDING no longer transitions here.
      // Kept defensively: if something forces this state, finish the
      // stop cleanly so we don't sit in a zombie state with files
      // open and `logging` still true.
      if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000))) {
        navFile.flush(); navFile.close();
        imuFile.flush(); imuFile.close();
        if (windFile) { windFile.flush(); windFile.close(); }
        if (presFile) { presFile.flush(); presFile.close(); }
        xSemaphoreGive(sdMutex);
      }
      logging = false;
      recState = REC_IDLE;
      triggerUpload = true;
      break;
  }
}

const char* getRecStateStr() {
  switch (recState) {
    case REC_IDLE: return gps.valid ? "READY" : "NO GPS";
    case REC_ARMED: return "ARMING";
    case REC_RECORDING: return "REC";
    case REC_STOPPING: return "STOPPING";
    default: return "?";
  }
}

// ============================================================
