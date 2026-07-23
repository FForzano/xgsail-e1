// Single momentary pushbutton (active-low, to GND, internal pull-up) —
// owns debouncing and short/long press detection. Short press toggles
// recording (recording.h's toggleRecording()); long press opens the BLE
// pairing window (ble_relay.h's bleOpenBondWindow()) so a first-time
// phone bond requires physical presence at the boat, not just proximity.
// See docs/hardware.md for the pin and docs/ble-config.md for the
// pairing-window rationale.
#ifndef SAILFRAMES_BUTTON_H
#define SAILFRAMES_BUTTON_H

// Call once from setup().
void buttonInit();

// Call every loop() iteration — debounces the pin and dispatches short/
// long press events directly to their owning modules.
void buttonTick();

#endif  // SAILFRAMES_BUTTON_H
