// Button debounce + short/long press glue — see button.h.
#include "button.h"
#include "config.h"
#include "recording.h"
#include "ble_relay.h"

// Debounced level (true = pressed), and the millis() timestamp the
// current stable level started at.
static bool s_pressed = false;
static unsigned long s_stableSince = 0;
static bool s_raw = false;          // last raw digitalRead, pre-debounce
static unsigned long s_rawSince = 0;
static bool s_longFired = false;    // long-press already dispatched this hold

void buttonInit() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  s_raw = s_pressed = (digitalRead(BUTTON_PIN) == LOW);
  s_rawSince = s_stableSince = millis();
}

void buttonTick() {
  unsigned long now = millis();
  bool raw = (digitalRead(BUTTON_PIN) == LOW);

  if (raw != s_raw) {
    s_raw = raw;
    s_rawSince = now;
  }

  // Promote the raw reading to "stable" only after it's held steady past
  // the debounce window — filters switch bounce without a blocking delay().
  if (raw != s_pressed && (now - s_rawSince) >= BUTTON_DEBOUNCE_MS) {
    s_pressed = raw;
    s_stableSince = now;

    if (s_pressed) {
      s_longFired = false;
    } else {
      // Released before the long-press threshold fired — that's a short
      // press. (If the long press already fired during this hold, the
      // release is just the end of that gesture, not a second event.)
      if (!s_longFired) toggleRecording();
    }
  }

  // Long press fires once, while still held, so the operator gets
  // immediate feedback without needing to release the button.
  if (s_pressed && !s_longFired && (now - s_stableSince) >= BUTTON_LONG_PRESS_MS) {
    s_longFired = true;
    bleOpenBondWindow();
  }
}
