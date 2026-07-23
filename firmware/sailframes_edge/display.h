// TFT dashboard rendering (Hosyond 3.5" IPS ST7796U, 320x480 portrait):
// the three nav display modes (D1/D2/D3) plus the RC fleet/pre-race panels
// shown in place of the nav display while an RC unit is armed.
#ifndef SAILFRAMES_DISPLAY_H
#define SAILFRAMES_DISPLAY_H

#include <Arduino.h>
#include "User_Setup.h"  // TFT_eSPI config (must be before TFT_eSPI.h)
#include <TFT_eSPI.h>

extern TFT_eSPI tft;
extern bool oledOK;

// Display mode: 1 = D1 (simple big numbers), 2 = D2 (nav + wind), 3 = D3 (wind focus)
extern int displayMode;
extern bool d2LayoutDrawn;  // reset to force a full D2 redraw
extern bool d3LayoutDrawn;  // reset to force a full D3 redraw

extern bool g_rcPanelShown;
extern bool g_rcPrePanelShown;

// Short WiFi indicator ("Home", "P", "WiFi", or "") for the status bar.
const char* getWifiIndicator();

// Draws battery percentage at (x, y); blinks off when critical.
void drawBatteryPercent(int x, int y);

void updateDisplayD1();
void updateDisplayD2();
void updateDisplayD3();
// Dispatches to the RC panels while armed/rc_signal, else the current
// displayMode's nav screen.
void updateDisplay();

// RC fleet OCS panel (live, while armed) and pre-race roster (while not
// armed yet) — both shown in place of the nav display on an rc_signal unit.
void drawRcFleetPanel();
void drawRcPreRacePanel();

#endif  // SAILFRAMES_DISPLAY_H
