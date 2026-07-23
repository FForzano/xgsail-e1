#include "shared_state.h"

SemaphoreHandle_t sdMutex = NULL;
TaskHandle_t uploadTaskHandle = NULL;
TaskHandle_t diagTaskHandle = NULL;

volatile const char* g_loopSection = "boot";
volatile uint32_t    g_loopIter = 0;

volatile bool wifiBusy = false;
volatile bool sdWriting = false;

bool triggerUpload = false;
bool wifiTeardownRequested = false;

const unsigned long LOOP_HANG_MS = 90UL * 1000UL;  // Core 1 must tick at least every 90 s
