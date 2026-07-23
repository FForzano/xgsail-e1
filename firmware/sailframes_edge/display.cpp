// TFT dashboard rendering glue — see display.h.
#include "display.h"
#include "config.h"
#include "gnss.h"
#include "imu.h"
#include "wind_sensor.h"
#include "battery.h"
#include "pressure.h"
#include "mesh.h"
#include "ocs.h"
#include "recording.h"
#include "storage.h"
#include "v2_types.h"

// TFT Display - Hosyond 3.5" IPS ST7796U (480x320, SPI)
// Using TFT_eSPI library with User_Setup.h configuration
TFT_eSPI tft = TFT_eSPI();
bool oledOK = false;

bool d2LayoutDrawn = false;  // Display layout flag - reset to redraw full screen
bool g_rcPanelShown = false;
bool g_rcPrePanelShown = false;

static unsigned long lastDisplayUpdate = 0;
static const unsigned long DISPLAY_UPDATE_INTERVAL = 200;  // Only update display every 200ms

// Get short WiFi indicator based on connected SSID
const char* getWifiIndicator() {
  if (strcmp(connectedSSID, "Home-IOT") == 0) return "Home";
  if (strcmp(connectedSSID, "paul") == 0) return "P";
  if (strlen(connectedSSID) > 0) return "WiFi";
  return "";
}

// Draw battery percentage at specified position (legacy function, battery now in main display)
void drawBatteryPercent(int x, int y) {
  if (!battery.valid) return;

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", battery.percent);

  // Blink if critical
  if (battery.critical && (millis() / 500) % 2 == 0) {
    // Don't draw (blink off)
  } else {
    uint16_t batColor = (battery.percent > 30) ? COLOR_GOOD :
                        (battery.percent > 15) ? COLOR_WARN : COLOR_ERROR;
    tft.setTextColor(batColor, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(buf, x, y, 2);
  }
}

static void drawRcClock(bool force) {
  static int prevSec = -1;
  int sod = -1;  // local seconds-of-day (-1 = no valid GPS time yet)
  if (gps.valid && strlen(gps.utc_time) >= 6) {
    int hh = (gps.utc_time[0]-'0')*10 + (gps.utc_time[1]-'0');
    int mm = (gps.utc_time[2]-'0')*10 + (gps.utc_time[3]-'0');
    int ss = (gps.utc_time[4]-'0')*10 + (gps.utc_time[5]-'0');
    sod = (((hh*3600 + mm*60 + ss) + RC_CLOCK_TZ_OFFSET_MIN*60) % 86400 + 86400) % 86400;
  }
  if (!force && sod == prevSec) return;
  prevSec = sod;
  tft.fillRect(0, 0, 150, 34, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  char tb[16];
  if (sod >= 0) snprintf(tb, sizeof(tb), "%02d:%02d:%02d", sod/3600, (sod/60)%60, sod%60);
  else          strcpy(tb, "--:--:--");
  tft.drawString(tb, 6, 8, 4);
}

static void drawRcFooter(bool force) {
  static int prevBatt = -999;
  if (!force && battery.percent == prevBatt) return;
  prevBatt = battery.percent;
  const int FY = SCREEN_HEIGHT - RC_FOOTER_H;
  tft.fillRect(0, FY, SCREEN_WIDTH, RC_FOOTER_H, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  char fb[40];
  snprintf(fb, sizeof(fb), "FW %s", FW_VERSION);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(fb, 6, FY + 6, 2);
  char bb[12];
  snprintf(bb, sizeof(bb), "BAT %d%%", battery.percent);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(bb, SCREEN_WIDTH - 6, FY + 6, 2);
}

void drawRcFleetPanel() {
  static int      prevCount = -1;
  static uint32_t prevSender[MESH_PEER_MAX];
  static int      prevDm[MESH_PEER_MAX];
  static int8_t   prevSt[MESH_PEER_MAX];
  static int      prevTsec = -99999;

  unsigned long now = millis();
  int tsec = (int)((int32_t)(g_ocs.start_time_ms - now) / 1000);

  bool full = (!g_rcPanelShown || g_mesh_peer_count != prevCount);
  if (full) {
    g_rcPanelShown = true;
    prevCount = g_mesh_peer_count;
    prevTsec = -99999;
    for (int i = 0; i < MESH_PEER_MAX; i++) { prevSender[i] = 0; prevDm[i] = -1000000; prevSt[i] = -2; }
    tft.fillScreen(COLOR_BG);
    tft.fillRect(0, 0, SCREEN_WIDTH, 34, TFT_BLACK);
    // Top-left "RC FLEET" label dropped — the time-of-day clock now lives
    // there (drawRcClock); the countdown stays on the right of the bar.
    tft.setTextColor(COLOR_LABEL, COLOR_BG);
    tft.drawString("BOAT", 8, 42, 2);
    tft.drawString("DIST", 95, 42, 2);
    tft.drawString("ST", 238, 42, 2);
    tft.drawFastHLine(0, 62, SCREEN_WIDTH, COLOR_DIVIDER);
  }

  // Countdown in the title bar (redraw only when the whole second changes).
  if (tsec != prevTsec) {
    prevTsec = tsec;
    tft.fillRect(180, 0, SCREEN_WIDTH - 180, 34, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TR_DATUM);
    char tb[16]; snprintf(tb, sizeof(tb), "T%+ds", tsec);
    tft.drawString(tb, SCREEN_WIDTH - 6, 8, 4);
  }

  const int rowH = 64, y0 = 70, maxRows = (SCREEN_HEIGHT - RC_FOOTER_H - y0) / rowH;
  for (int i = 0; i < g_mesh_peer_count && i < maxRows; i++) {
    const MeshPeerState& p = g_mesh_peers[i];
    int dm = (int)lroundf(p.rc_distance_m * 10.0f);
    int8_t st = p.rc_ocs_called ? 2 : (p.rc_distance_m < 0 ? 1 : 0);
    if (p.sender_id == prevSender[i] && dm == prevDm[i] && st == prevSt[i]) continue;
    prevSender[i] = p.sender_id; prevDm[i] = dm; prevSt[i] = st;
    int y = y0 + i * rowH;
    tft.fillRect(0, y, SCREEN_WIDTH, rowH - 4, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.drawString(boatNameForSender(p.sender_id), 8, y + 6, 4);
    char db[16]; snprintf(db, sizeof(db), "%+.1f", p.rc_distance_m);
    tft.drawString(db, 95, y + 6, 4);
    uint16_t sc = (st == 2) ? COLOR_ERROR : (st == 1) ? COLOR_WARN : COLOR_GOOD;
    const char* ss = (st == 2) ? "OCS" : (st == 1) ? "OVR" : "OK";
    tft.setTextColor(sc, COLOR_BG);
    tft.drawString(ss, 238, y + 6, 4);
    tft.setTextSize(1);
  }

  drawRcClock(full);
  drawRcFooter(full);
}

void drawRcPreRacePanel() {
  // Sunlight-readable: BIG font-4 values, ALL BLACK (no color washes out on the
  // white-background TFT). One line per boat: name · FIX · ACC · HDOP · SAT,
  // under column headers. Partial per-row redraw on change. Fix state is read
  // from the text (FIX/FLT/---), not color.
  static uint32_t prevSender[MESH_PEER_MAX];
  static int8_t   prevQ[MESH_PEER_MAX];
  static int      prevSat[MESH_PEER_MAX], prevHdop[MESH_PEER_MAX], prevHacc[MESH_PEER_MAX];
  static int      prevConn = -1, prevFixed = -1;
  static int      prevBaseSat = -1, prevBaseHdop = -1, prevBaseHacc = -1;
  static int8_t   prevBaseReady = -1;

  const int CX_NAME = 8, CX_FIX = 76, CX_ACC = 146, CX_HDOP = 234, CX_SAT = 288;
  const int HDR_Y = 46, DIV_Y = 64, ROW0 = 70, rowH = 48;

  int conn = g_mesh_peer_count, fixed = 0;
  for (int i = 0; i < g_mesh_peer_count; i++)
    if (g_mesh_peers[i].fix_quality == 4) fixed++;
  int8_t baseReady = (gps.valid && (gps.lat != 0 || gps.lon != 0)) ? 1 : 0;

  // Static layout — repaint on first show or when the peer COUNT changes.
  bool full = (!g_rcPrePanelShown || conn != prevConn);
  if (full) {
    g_rcPrePanelShown = true;
    prevConn = -999; prevFixed = -999;
    prevBaseSat = -1; prevBaseHdop = -1; prevBaseHacc = -1; prevBaseReady = -1;
    for (int i = 0; i < MESH_PEER_MAX; i++) {
      prevSender[i]=0; prevQ[i]=-2; prevSat[i]=-1; prevHdop[i]=-1; prevHacc[i]=-1;
    }
    tft.fillScreen(COLOR_BG);
    tft.fillRect(0, 0, SCREEN_WIDTH, 34, TFT_BLACK);
    // Top-left "RC PRE-RACE" label dropped — the time-of-day clock now
    // occupies the top bar (drawRcClock); FIX gauge stays on the right.
    tft.setTextColor(COLOR_TEXT, COLOR_BG); tft.setTextDatum(TL_DATUM);
    tft.drawString("BOAT", CX_NAME, HDR_Y, 2);
    tft.drawString("ST",   CX_FIX,  HDR_Y, 2);
    tft.drawString("ACC",  CX_ACC,  HDR_Y, 2);
    tft.drawString("HDOP", CX_HDOP, HDR_Y, 2);
    tft.drawString("SAT",  CX_SAT,  HDR_Y, 2);
    tft.drawFastHLine(0, DIV_Y, SCREEN_WIDTH, COLOR_DIVIDER);
  }

  // Header right: "<fixed>/<connected> FIX" readiness gauge (white on black bar).
  if (conn != prevConn || fixed != prevFixed) {
    prevConn = conn; prevFixed = fixed;
    tft.fillRect(168, 0, SCREEN_WIDTH - 168, 34, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextDatum(TR_DATUM);
    char hb[20]; snprintf(hb, sizeof(hb), "%d/%d FIX", fixed, conn);
    tft.drawString(hb, SCREEN_WIDTH - 6, 8, 4);
  }

  // BASE row (row 0) — this boat's own status in the same columns: ST=RDY/SVY,
  // ACC (gps.hacc_m), HDOP (gps.hdop), SAT (gps.satellites).
  int bHd = (gps.valid && gps.hdop > 0.1f && gps.hdop < 25.5f) ? (int)lroundf(gps.hdop * 10) : 0;
  int bHa = (gps.hacc_m > 0.0005f) ? (int)fminf(255.0f, lroundf(gps.hacc_m * 1000)) : 0;
  if (baseReady != prevBaseReady || gps.satellites != prevBaseSat ||
      bHd != prevBaseHdop || bHa != prevBaseHacc) {
    prevBaseReady = baseReady; prevBaseSat = gps.satellites; prevBaseHdop = bHd; prevBaseHacc = bHa;
    int y = ROW0;
    tft.fillRect(0, y, SCREEN_WIDTH, rowH - 4, COLOR_BG);
    tft.setTextSize(1); tft.setTextDatum(TL_DATUM); tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.drawString("BASE", CX_NAME, y + 4, 4);
    tft.drawString(baseReady ? "RDY" : "SVY", CX_FIX, y + 4, 4);
    char a[12], h[8], s[6];
    if (bHa > 0) snprintf(a, sizeof(a), "%.1fcm", bHa / 10.0); else strcpy(a, "--");
    if (bHd > 0) snprintf(h, sizeof(h), "%.1f", bHd / 10.0);   else strcpy(h, "--");
    snprintf(s, sizeof(s), "%d", gps.satellites);
    tft.drawString(a, CX_ACC,  y + 4, 4);
    tft.drawString(h, CX_HDOP, y + 4, 4);
    tft.drawString(s, CX_SAT,  y + 4, 4);
  }

  // Peer rows (below the BASE row) — name · FIX · ACC · HDOP · SAT, big + black.
  int maxRows = (SCREEN_HEIGHT - RC_FOOTER_H - (ROW0 + rowH)) / rowH;
  for (int i = 0; i < g_mesh_peer_count && i < maxRows; i++) {
    const MeshPeerState& p = g_mesh_peers[i];
    int q = p.fix_quality, sat = p.sat_count, hd = p.hdop_x10, ha = p.hacc_mm;
    if (p.sender_id == prevSender[i] && q == prevQ[i] && sat == prevSat[i] &&
        hd == prevHdop[i] && ha == prevHacc[i]) continue;
    prevSender[i]=p.sender_id; prevQ[i]=q; prevSat[i]=sat; prevHdop[i]=hd; prevHacc[i]=ha;
    int y = ROW0 + (i + 1) * rowH;
    tft.fillRect(0, y, SCREEN_WIDTH, rowH - 4, COLOR_BG);
    tft.setTextSize(1); tft.setTextDatum(TL_DATUM); tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.drawString(boatNameForSender(p.sender_id), CX_NAME, y + 4, 4);
    tft.drawString(q==4?"FIX":q==5?"FLT":q==2?"DGP":q==1?"GPS":"---", CX_FIX, y + 4, 4);
    char a[12], h[8], s[6];
    if (ha > 0) snprintf(a, sizeof(a), "%.1fcm", ha / 10.0); else strcpy(a, "--");
    if (hd > 0) snprintf(h, sizeof(h), "%.1f", hd / 10.0);   else strcpy(h, "--");
    snprintf(s, sizeof(s), "%d", sat);
    tft.drawString(a, CX_ACC,  y + 4, 4);
    tft.drawString(h, CX_HDOP, y + 4, 4);
    tft.drawString(s, CX_SAT,  y + 4, 4);
  }

  drawRcClock(full);
  drawRcFooter(full);
}

int displayMode = 2;  // D2 Vakaros-style nav + wind

// Previous values for efficient redraw (only update what changed)
static float prevSOG = -1, prevCOG = -1, prevHeel = -1, prevPitch = -1;
static int prevBattery = -1, prevSats = -1;
static bool prevRecording = false;
static unsigned long lastFullRedraw = 0;

// D1: Simple display - PORTRAIT 320x480 with large high-contrast numbers
void updateDisplayD1() {
  if (!oledOK) return;

  char buf[32];
  static bool layoutDrawn = false;
  bool forceRedraw = !layoutDrawn || (millis() - lastFullRedraw > 60000);  // Reduce blink: 60s

  // Check for warnings
  bool hasWarning = false;
  const char* warnMsg = nullptr;
  uint16_t warnColor = COLOR_ERROR;

  if (!sdOK) {
    warnMsg = "NO SD";
    hasWarning = true;
  } else if (lastValidGPS == 0 && millis() > 120000) {
    warnMsg = "NO GPS";
    warnColor = COLOR_WARN;
    hasWarning = true;
  } else if (lastValidGPS > 0 && millis() - lastValidGPS > 60000) {
    warnMsg = "GPS LOST";
    hasWarning = true;
  }

  // Full screen redraw only on first run or every 60s
  if (forceRedraw) {
    tft.fillScreen(COLOR_BG);
    lastFullRedraw = millis();
    layoutDrawn = true;

    // Draw static labels - high contrast white
    tft.setTextColor(TFT_DARKGREY, COLOR_BG);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("SOG", SCREEN_WIDTH/2, 5, 4);
    tft.drawString("COG", SCREEN_WIDTH/2, 165, 4);
    tft.drawString("HEEL", SCREEN_WIDTH/4, 325, 2);
    tft.drawString("BAT", 3*SCREEN_WIDTH/4, 325, 2);

    // Divider lines
    tft.drawFastHLine(0, 160, SCREEN_WIDTH, COLOR_DIVIDER);
    tft.drawFastHLine(0, 320, SCREEN_WIDTH, COLOR_DIVIDER);
    tft.drawFastVLine(SCREEN_WIDTH/2, 320, 120, COLOR_DIVIDER);
    tft.drawFastHLine(0, 440, SCREEN_WIDTH, COLOR_DIVIDER);

    // Force value updates
    prevSOG = prevCOG = prevHeel = -999;
    prevBattery = -1;
  }

  // Warning banner at top
  if (hasWarning && warnMsg) {
    tft.fillRect(0, 0, SCREEN_WIDTH, 30, warnColor);
    tft.setTextColor(TFT_BLACK, warnColor);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(warnMsg, SCREEN_WIDTH/2, 15, 4);
  }

  // SOG - HUGE white numbers (Font 8 = 75px)
  if (abs(gps.speed_kts - prevSOG) > 0.05 || forceRedraw) {
    prevSOG = gps.speed_kts;
    tft.fillRect(0, 35, SCREEN_WIDTH, 120, COLOR_BG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "%.1f", gps.speed_kts);
    tft.drawString(buf, SCREEN_WIDTH/2, 95, 8);  // Font 8 = 75px
  }

  // COG - HUGE white numbers (Font 8 = 75px)
  if (abs(gps.course - prevCOG) > 0.5 || forceRedraw) {
    prevCOG = gps.course;
    tft.fillRect(0, 195, SCREEN_WIDTH, 120, COLOR_BG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "%03d", (int)gps.course);
    tft.drawString(buf, SCREEN_WIDTH/2, 255, 8);  // Font 8 = 75px
  }

  // Heel - large (Font 7 = 48px)
  if (abs(imu.heel - prevHeel) > 0.5 || forceRedraw) {
    prevHeel = imu.heel;
    tft.fillRect(0, 345, SCREEN_WIDTH/2 - 5, 70, COLOR_BG);
    uint16_t heelColor = (abs(imu.heel) > 25) ? COLOR_WARN : TFT_WHITE;
    tft.setTextColor(heelColor, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "%+.0f", imu.heel);
    tft.drawString(buf, SCREEN_WIDTH/4, 380, 7);
  }

  // Battery - large (Font 7 = 48px)
  if (battery.percent != prevBattery || forceRedraw) {
    prevBattery = battery.percent;
    tft.fillRect(SCREEN_WIDTH/2 + 5, 345, SCREEN_WIDTH/2 - 5, 70, COLOR_BG);
    uint16_t batColor = (battery.percent > 30) ? COLOR_GOOD :
                        (battery.percent > 15) ? COLOR_WARN : COLOR_ERROR;
    tft.setTextColor(batColor, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "%d", battery.percent);
    tft.drawString(buf, 3*SCREEN_WIDTH/4, 380, 7);
  }

  // Status bar at bottom (always update - use overwrite)
  tft.fillRect(0, 442, SCREEN_WIDTH, 38, COLOR_BG);
  int dispSats = (gps.satellites >= 0 && gps.satellites <= 50) ? gps.satellites : 0;
  const char* fixStr = (gps.fix_quality == 2) ? "SBAS" : (gps.fix_quality == 1) ? "GPS" : "---";
  const char* recStr = getRecStateStr();

  // Left: Recording state
  uint16_t recColor = logging ? COLOR_GOOD : COLOR_LABEL;
  tft.setTextColor(recColor, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(recStr, 5, 452, 4);

  // Right: Satellites
  snprintf(buf, sizeof(buf), "%s %d", fixStr, dispSats);
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(buf, SCREEN_WIDTH - 5, 452, 4);
}

static float prevTWS = -1, prevTWA = -1, prevTWD = -1, prevAWS2 = -1, prevAWA2 = -1;
static int prevDispSats = -1;
static int prevRecState = -1;
// d2LayoutDrawn declared globally near other display flags

// ---- D2 layout geometry ----------------------------------------------------
// E (3.5" 320x480): the original constants — the #else values below are byte-
// identical to the literals this layout shipped with, so the E build is
// unchanged. B1 (2.8" 240x320): the big fonts shrink to Font 8 x1 (75px) and
// everything repacks into 320 px tall / 240 px wide; the HDOP cell is dropped
// from the top bar (no room at 240 px).

  #define D2_TOPBAR_H      30
  #define D2_TB_TXT_Y       7
  #define D2_TB_REC_X       5
  #define D2_TB_FIX_X     100
  #define D2_TB_SAT_X     145
  #define D2_TB_SATN_X    180
  #define D2_TB_HDOP_X    210
  #define D2_TB_HDOPV_X   250
  #define D2_LBL_X          5
  #define D2_LBL_FONT       4
  #define D2_COG_LBL_Y     35
  #define D2_COG_CLR_Y     60
  #define D2_COG_CLR_H    155
  #define D2_COG_VAL_Y    130
  #define D2_DIV_Y        220
  #define D2_SOG_LBL_Y    225
  #define D2_SOG_CLR_Y    250
  #define D2_SOG_CLR_H    155
  #define D2_SOG_VAL_Y    320
  #define D2_BIG_SZ         2
  #define D2_BAR_CLR_Y    430
  #define D2_BAR_CLR_H     50
  #define D2_BAR_FILL_Y   440
  #define D2_BAR_FILL_H    40
  #define D2_ROW1_Y       440
  #define D2_ROW2_Y       456

static const char* fwShortTag() {
  static char buf[16] = {0};
  if (!buf[0]) {
    int yyyy = 0, mm = 0, dd = 0, n = 0;
    if (sscanf(FW_VERSION, "%d.%d.%d.%d", &yyyy, &mm, &dd, &n) == 4) {
      snprintf(buf, sizeof(buf), "%02d.%02d.%02d.%d", yyyy % 100, mm, dd, n);
    } else {
      snprintf(buf, sizeof(buf), "%s", FW_VERSION);
    }
  }
  return buf;
}

void updateDisplayD2() {
  if (!oledOK) return;

  // Throttle display updates (SD on separate HSPI bus, so no SPI contention)
  unsigned long now = millis();
  if (now - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) return;  // 200ms
  lastDisplayUpdate = now;

  char buf[32];

  // Calculate true wind from apparent wind + boat speed
  float aws = 0, awa = 0, tws = 0, twa = 0, twd = 0;
#if ENABLE_WIND
  if (wind.connected && wind.lastUpdate > 0 && millis() - wind.lastUpdate < 5000) {
    aws = wind.speed_kts;
    awa = wind.angle_deg + config.wind_offset;
    if (awa < 0) awa += 360;
    if (awa >= 360) awa -= 360;
    float awaRad = awa * PI / 180.0;
    if (awaRad > PI) awaRad -= 2 * PI;
    float sog = gps.speed_kts;
    tws = sqrt(aws*aws + sog*sog - 2*aws*sog*cos(awaRad));
    float twaRad = atan2(aws * sin(awaRad), aws * cos(awaRad) - sog);
    twa = twaRad * 180.0 / PI;
    if (twa < 0) twa += 360;
    twd = gps.course + twa;
    if (twd >= 360) twd -= 360;
    if (twd < 0) twd += 360;
  }
#endif

  // Draw layout ONCE - labels and dividers
  if (!d2LayoutDrawn) {
    tft.fillScreen(COLOR_BG);
    d2LayoutDrawn = true;

    // VAKAROS-STYLE: White bg, bold black numbers
    // Top bar: BLACK bg, WHITE text
    // Bottom bar: BLACK bg, WHITE text (includes wind data if enabled)
    // Layout:
    // [BLACK: REC | SAT x HDOP x.x]  (30px)
    // [COG  000                   ]  (190px) - Font 8 x2
    // [SOG  00                    ]  (190px) - Font 8 x2
    // [BLACK: H P AWS AWA / BAT% W | WiFi N R ]  (50px, two rows)

    // BLACK bars for top and bottom
    tft.fillRect(0, 0, SCREEN_WIDTH, D2_TOPBAR_H, TFT_BLACK);
    tft.fillRect(0, D2_BAR_FILL_Y, SCREEN_WIDTH, D2_BAR_FILL_H, TFT_BLACK);

    // Divider between COG and SOG
    uint16_t lineColor = tft.color565(180, 180, 180);
    tft.drawFastHLine(0, D2_DIV_Y, SCREEN_WIDTH, lineColor);

    // Labels for COG and SOG - LARGE (Font 4 = 26px on E, Font 2 on B1)
    uint16_t labelColor = tft.color565(100, 100, 100);
    tft.setTextColor(labelColor, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("COG", D2_LBL_X, D2_COG_LBL_Y, D2_LBL_FONT);
    tft.drawString("SOG", D2_LBL_X, D2_SOG_LBL_Y, D2_LBL_FONT);

    // Reset prev values
    prevSOG = prevCOG = prevHeel = prevPitch = -999;
    prevTWS = prevTWA = prevTWD = prevAWS2 = prevAWA2 = -999;
    prevBattery = prevDispSats = prevRecState = -1;
  }

  // Status bar - with fix type, SAT count, HDOP
  int dispSats = (gps.satellites >= 0 && gps.satellites <= 50) ? gps.satellites : 0;
  static float prevHDOP = -1;
  static int prevFixQ = -1;
  if (dispSats != prevDispSats || recState != prevRecState ||
      abs(gps.hdop - prevHDOP) > 0.1 || gps.fix_quality != prevFixQ) {
    prevDispSats = dispSats;
    prevRecState = recState;
    prevHDOP = gps.hdop;
    prevFixQ = gps.fix_quality;

    // TOP BAR: WHITE text on BLACK background
    // Clear entire top bar first to prevent any ghosting
    tft.fillRect(0, 0, SCREEN_WIDTH, D2_TOPBAR_H, TFT_BLACK);

    // Recording state (left side)
    const char* recStr;
    switch (recState) {
      case REC_IDLE: recStr = gps.valid ? "READY" : "NO GPS"; break;
      case REC_ARMED: recStr = "ARM"; break;
      case REC_RECORDING: recStr = "REC"; break;
      case REC_STOPPING: recStr = "STOP"; break;
      default: recStr = "---"; break;
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(recStr, D2_TB_REC_X, D2_TB_TXT_Y, 2);

    // Fix type (0=none, 1=GPS, 2=DGPS/SBAS, 4=RTK fix, 5=RTK float)
    const char* fixStr;
    switch (gps.fix_quality) {
      case 1: fixStr = "GPS"; break;
      case 2: fixStr = "SBAS"; break;
      case 4: fixStr = "RTK"; break;
      case 5: fixStr = "FLT"; break;
      default: fixStr = "---"; break;
    }
    tft.drawString(fixStr, D2_TB_FIX_X, D2_TB_TXT_Y, 2);

    // SAT count
    tft.drawString("SAT", D2_TB_SAT_X, D2_TB_TXT_Y, 2);
    snprintf(buf, sizeof(buf), "%2d", dispSats);
    tft.drawString(buf, D2_TB_SATN_X, D2_TB_TXT_Y, 2);

    tft.drawString("HDOP", D2_TB_HDOP_X, D2_TB_TXT_Y, 2);
    snprintf(buf, sizeof(buf), "%.1f", gps.hdop);
    tft.drawString(buf, D2_TB_HDOPV_X, D2_TB_TXT_Y, 2);
    // WiFi status removed from top bar - shown on bottom bar instead
  }

  // COG - Font 8 x2 = 150px
  // COG area: 30-220 (190px), center at 130 (moved down to not overlap label)
  if (abs(gps.course - prevCOG) > 0.5) {
    prevCOG = gps.course;
    tft.fillRect(0, D2_COG_CLR_Y, SCREEN_WIDTH, D2_COG_CLR_H, COLOR_BG);
    tft.setTextColor(TFT_BLACK, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(D2_BIG_SZ);
    snprintf(buf, sizeof(buf), "%03d", (int)gps.course);
    tft.drawString(buf, SCREEN_WIDTH/2, D2_COG_VAL_Y, 8);
    tft.setTextSize(1);
  }

  // SOG - Font 8 x2 = 150px. SOG area: 220-440 (190px), centre at 315.
  // Below 10 kt show one decimal ("8.9") so a tactical helmsman can
  // see the kn/10 trend; ≥10 kt show integer only ("12") because
  // three glyphs ("12.x") at this size run past the screen edges.
  // The narrow "." glyph keeps "9.9" the same effective width as
  // a two-digit "12", so the font size stays unchanged either way.
  static char prevSogBuf[8] = "";
  char newSogBuf[8];
  if (gps.speed_kts < 10.0f) {
    snprintf(newSogBuf, sizeof(newSogBuf), "%.1f", gps.speed_kts);
  } else {
    snprintf(newSogBuf, sizeof(newSogBuf), "%d", (int)(gps.speed_kts + 0.5f));
  }
  if (strcmp(newSogBuf, prevSogBuf) != 0) {
    strcpy(prevSogBuf, newSogBuf);
    prevSOG = gps.speed_kts;
    tft.fillRect(0, D2_SOG_CLR_Y, SCREEN_WIDTH, D2_SOG_CLR_H, COLOR_BG);
    tft.setTextColor(TFT_BLACK, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(D2_BIG_SZ);
    tft.drawString(newSogBuf, SCREEN_WIDTH/2, D2_SOG_VAL_Y, 8);
    tft.setTextSize(1);
  }

  // Bottom status bar - two rows on BLACK background
  static bool prevWindConnected = true;
  static bool prevWifiConnected = false;
  static unsigned long lastStatusUpdate = 0;

  static int prevPendingN = -1;
  static float prevLineDist = -99999;
  static bool prevArmed = false;
  bool statusChanged = (wind.connected != prevWindConnected) ||
                       (wifiConnected != prevWifiConnected) ||
                       (abs(imu.heel - prevHeel) > 0.5) ||
                       (abs(imu.pitch - prevPitch) > 0.5) ||
                       (abs(aws - prevAWS2) > 0.3) ||
                       (abs(awa - prevAWA2) > 1) ||
                       (battery.percent != prevBattery) ||
                       (pendingUploads != prevPendingN) ||
                       (g_ocs.armed != prevArmed) ||
                       (g_ocs.armed && abs(g_ocs.distance_to_line_m - prevLineDist) > 0.1) ||
                       (millis() - lastStatusUpdate > 2000);

  if (statusChanged) {
    lastStatusUpdate = millis();
    prevWindConnected = wind.connected;
    prevWifiConnected = wifiConnected;
    prevHeel = imu.heel;
    prevPitch = imu.pitch;
    prevAWS2 = aws;
    prevAWA2 = awa;
    prevBattery = battery.percent;
    prevPendingN = pendingUploads;
    prevLineDist = g_ocs.distance_to_line_m;
    prevArmed = g_ocs.armed;

    // BOTTOM BAR: Two rows - WHITE on BLACK
    // Clear entire bottom bar first
    tft.fillRect(0, D2_BAR_CLR_Y, SCREEN_WIDTH, D2_BAR_CLR_H, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    // Row 1 (y=440): Heel + Pitch always; AWS + AWA appended when wind connected.
    // When a race is armed, append signed distance-to-line in small print
    // ("L+5.2m" / "L-4.3m") so the crew sees their line position at a glance.
    // Single row keeps heel/pitch visible even with the wind sensor active.
    char row1[64];
    char lineTag[16] = "";
    if (g_ocs.armed) snprintf(lineTag, sizeof(lineTag), " L%+.1fm", g_ocs.distance_to_line_m);
    if (config.wind_enabled && wind.connected) {
      snprintf(row1, sizeof(row1), "H%+.0f P%+.0f AWS%.1f%s",
               imu.heel, imu.pitch, aws, lineTag);
    } else {
      snprintf(row1, sizeof(row1), "H %+.0f  P %+.0f%s", imu.heel, imu.pitch, lineTag);
    }
    tft.setTextDatum(MC_DATUM);
    tft.drawString(row1, SCREEN_WIDTH/2, D2_ROW1_Y, 2);

    // Row 2 (y=458):
    //   Left:  "BAT N% W"   (W appears immediately after BAT% when wind connected)
    //   Right: WiFi indicator + upload counts (no W here — was blocking the counts)
    tft.setTextDatum(TL_DATUM);
    char left[32];
#if ENABLE_WIND
    bool windInd = (config.wind_enabled && wind.connected);
#else
    bool windInd = false;
#endif
    if (windInd) {
      snprintf(left, sizeof(left), "BAT %d%% W FW%s", battery.percent, fwShortTag());
    } else {
      snprintf(left, sizeof(left), "BAT %d%% FW%s", battery.percent, fwShortTag());
    }
    tft.drawString(left, D2_LBL_X, D2_ROW2_Y, 2);

    // Right side: WiFi + IP when idle, falls back to counts during traffic.
    // IP next to the SSID indicator gives a debug surface for telnet/curl
    // without needing the router DHCP table.
    char right[40];
    const char* wifiInd = wifiConnected ? getWifiIndicator() : "";

    if (uploading && uploadTotal > 0) {
      snprintf(right, sizeof(right), "%s %d/%d", wifiInd, uploadCount, uploadTotal);
    } else if (pendingUploads > 0) {
      snprintf(right, sizeof(right), "%s N%d", wifiInd, pendingUploads);
    } else if (wifiConnected) {
      snprintf(right, sizeof(right), "%s %s", wifiInd, WiFi.localIP().toString().c_str());
    } else {
      snprintf(right, sizeof(right), "%s", wifiInd);
    }
    tft.setTextDatum(TR_DATUM);
    tft.drawString(right, SCREEN_WIDTH - 5, D2_ROW2_Y, 2);
    tft.setTextDatum(TL_DATUM);
  }
}

static float prevD3AWS = -1, prevD3AWA = -1, prevD3TWS = -1, prevD3TWA = -1;
static float prevD3TWD = -1, prevD3SOG = -1, prevD3COG = -1, prevD3HDOP = -1;
static int prevD3RecState = -1, prevD3Sats = -1, prevD3FixQ = -1;
static bool d3LayoutDrawn = false;

void updateDisplayD3() {
  if (!oledOK) return;

  unsigned long now = millis();
  if (now - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) return;
  lastDisplayUpdate = now;

  char buf[32];

  // Calculate true wind from apparent wind + boat speed
  float aws = 0, awa = 0, tws = 0, twa = 0, twd = 0;
#if ENABLE_WIND
  if (wind.connected && wind.lastUpdate > 0 && millis() - wind.lastUpdate < 5000) {
    aws = wind.speed_kts;
    awa = wind.angle_deg + config.wind_offset;
    if (awa < 0) awa += 360;
    if (awa >= 360) awa -= 360;
    float awaRad = awa * PI / 180.0;
    if (awaRad > PI) awaRad -= 2 * PI;
    float sog = gps.speed_kts;
    tws = sqrt(aws*aws + sog*sog - 2*aws*sog*cos(awaRad));
    float twaRad = atan2(aws * sin(awaRad), aws * cos(awaRad) - sog);
    twa = twaRad * 180.0 / PI;
    if (twa < 0) twa += 360;
    twd = gps.course + twa;
    if (twd >= 360) twd -= 360;
    if (twd < 0) twd += 360;
  }
#endif

  // Row geometry: 4 rows × 2 cols, each row 105px
  // y positions: row top, label at top+2, value centered at top+58
  const int ROW_H = 105;
  const int TOP_BAR = 30;
  const int R1 = TOP_BAR;            // 30
  const int R2 = TOP_BAR + ROW_H;    // 135
  const int R3 = TOP_BAR + ROW_H*2;  // 240
  const int R4 = TOP_BAR + ROW_H*3;  // 345
  const int BOT_BAR = TOP_BAR + ROW_H*4; // 450
  const int HALF = SCREEN_WIDTH / 2;

  // Draw layout ONCE
  if (!d3LayoutDrawn) {
    tft.fillScreen(COLOR_BG);
    d3LayoutDrawn = true;

    // Layout (480 tall):
    // [BLACK: REC | SAT | BAT]         30px
    // [  AWS       |  AWA     ]       105px
    // [  TWS       |  TWA     ]       105px
    // [  SOG       |  COG     ]       105px
    // [  TWD  full width      ]       105px
    // [BLACK: H P WiFi status ]        30px

    tft.fillRect(0, 0, SCREEN_WIDTH, TOP_BAR, TFT_BLACK);
    tft.fillRect(0, BOT_BAR, SCREEN_WIDTH, 30, TFT_BLACK);

    uint16_t lineColor = tft.color565(180, 180, 180);
    tft.drawFastHLine(0, R2, SCREEN_WIDTH, lineColor);
    tft.drawFastHLine(0, R3, SCREEN_WIDTH, lineColor);
    tft.drawFastHLine(0, R4, SCREEN_WIDTH, lineColor);
    tft.drawFastVLine(HALF, R1, ROW_H * 3, lineColor);  // vertical for first 3 rows

    uint16_t labelColor = tft.color565(100, 100, 100);
    tft.setTextColor(labelColor, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    if (config.wind_enabled) {
      tft.drawString("AWS", 5, R1 + 2, 2);
      tft.drawString("AWA", HALF + 5, R1 + 2, 2);
      tft.drawString("TWS", 5, R2 + 2, 2);
      tft.drawString("TWA", HALF + 5, R2 + 2, 2);
    }
    tft.drawString("SOG", 5, R3 + 2, 2);
    tft.drawString("COG", HALF + 5, R3 + 2, 2);
    if (config.wind_enabled) {
      tft.drawString("TWD", 5, R4 + 2, 2);
    }

    prevD3AWS = prevD3AWA = prevD3TWS = prevD3TWA = -999;
    prevD3TWD = prevD3SOG = prevD3COG = -999;
    prevD3RecState = prevD3Sats = prevD3FixQ = -1;
    prevD3HDOP = -1;
  }

  // Top status bar
  int dispSats = (gps.satellites >= 0 && gps.satellites <= 50) ? gps.satellites : 0;
  if (dispSats != prevD3Sats || recState != prevD3RecState ||
      gps.fix_quality != prevD3FixQ || abs(gps.hdop - prevD3HDOP) > 0.1) {
    prevD3Sats = dispSats;
    prevD3RecState = recState;
    prevD3FixQ = gps.fix_quality;
    prevD3HDOP = gps.hdop;

    tft.fillRect(0, 0, SCREEN_WIDTH, TOP_BAR, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);

    const char* recStr;
    switch (recState) {
      case REC_IDLE: recStr = gps.valid ? "READY" : "NO GPS"; break;
      case REC_ARMED: recStr = "ARMING"; break;
      case REC_RECORDING: recStr = "REC"; break;
      case REC_STOPPING: recStr = "STOP"; break;
      default: recStr = ""; break;
    }
    tft.drawString(recStr, 5, 7, 2);

    // Fix type (0=none, 1=GPS, 2=DGPS/SBAS, 4=RTK fix, 5=RTK float)
    const char* fixStr;
    switch (gps.fix_quality) {
      case 1: fixStr = "GPS"; break;
      case 2: fixStr = "SBAS"; break;
      case 4: fixStr = "RTK"; break;
      case 5: fixStr = "FLOAT"; break;
      default: fixStr = "---"; break;
    }
    tft.drawString(fixStr, 80, 7, 2);

    snprintf(buf, sizeof(buf), "SAT %d", dispSats);
    tft.drawString(buf, 140, 7, 2);

    snprintf(buf, sizeof(buf), "HDOP %.1f", gps.hdop);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(buf, SCREEN_WIDTH - 5, 7, 2);
    tft.setTextDatum(TL_DATUM);
  }

  // Helper: value y-center for each row = row_top + 58 (label 16px + gap + 75px/2)
  // Clear area: row_top + 20 to row_top + 100 (80px tall, fits 75px font)

  // Wind values only if wind sensor enabled
  if (config.wind_enabled) {
    // AWS (row 1 left)
    if (abs(aws - prevD3AWS) > 0.2) {
      prevD3AWS = aws;
      tft.fillRect(0, R1 + 20, HALF - 2, 80, COLOR_BG);
      tft.setTextColor(TFT_BLACK, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      snprintf(buf, sizeof(buf), "%.1f", aws);
      tft.drawString(buf, HALF / 2, R1 + 58, 8);
    }

    // AWA (row 1 right)
    if (abs(awa - prevD3AWA) > 1) {
      prevD3AWA = awa;
      tft.fillRect(HALF + 2, R1 + 20, HALF - 2, 80, COLOR_BG);
      tft.setTextColor(TFT_BLACK, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      snprintf(buf, sizeof(buf), "%03d", (int)awa);
      tft.drawString(buf, HALF + HALF / 2, R1 + 58, 8);
    }

    // TWS (row 2 left)
    if (abs(tws - prevD3TWS) > 0.2) {
      prevD3TWS = tws;
      tft.fillRect(0, R2 + 20, HALF - 2, 80, COLOR_BG);
      tft.setTextColor(TFT_BLACK, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      snprintf(buf, sizeof(buf), "%.1f", tws);
      tft.drawString(buf, HALF / 2, R2 + 58, 8);
    }

    // TWA (row 2 right)
    if (abs(twa - prevD3TWA) > 1) {
      prevD3TWA = twa;
      tft.fillRect(HALF + 2, R2 + 20, HALF - 2, 80, COLOR_BG);
      tft.setTextColor(TFT_BLACK, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      snprintf(buf, sizeof(buf), "%03d", (int)twa);
      tft.drawString(buf, HALF + HALF / 2, R2 + 58, 8);
    }
  }

  // SOG (row 3 left)
  if (abs(gps.speed_kts - prevD3SOG) > 0.2) {
    prevD3SOG = gps.speed_kts;
    tft.fillRect(0, R3 + 20, HALF - 2, 80, COLOR_BG);
    tft.setTextColor(TFT_BLACK, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "%.1f", gps.speed_kts);
    tft.drawString(buf, HALF / 2, R3 + 58, 8);
  }

  // COG (row 3 right)
  if (abs(gps.course - prevD3COG) > 0.5) {
    prevD3COG = gps.course;
    tft.fillRect(HALF + 2, R3 + 20, HALF - 2, 80, COLOR_BG);
    tft.setTextColor(TFT_BLACK, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "%03d", (int)gps.course);
    tft.drawString(buf, HALF + HALF / 2, R3 + 58, 8);
  }

  // TWD (row 4 full width) - only if wind enabled
  if (config.wind_enabled) {
    if (abs(twd - prevD3TWD) > 1) {
      prevD3TWD = twd;
      tft.fillRect(0, R4 + 20, SCREEN_WIDTH, 80, COLOR_BG);
      tft.setTextColor(TFT_BLACK, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      snprintf(buf, sizeof(buf), "%03d", (int)twd);
      tft.drawString(buf, SCREEN_WIDTH / 2, R4 + 58, 8);
    }
  }

  // Bottom bar: Heel + WiFi/upload status
  static unsigned long lastD3Status = 0;
  if (millis() - lastD3Status > 2000) {
    lastD3Status = millis();
    tft.fillRect(0, BOT_BAR, SCREEN_WIDTH, 30, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);

    char line[32];
    snprintf(line, sizeof(line), "H%+.0f P%+.0f BAT%d%%", imu.heel, imu.pitch, battery.percent);
    tft.drawString(line, 3, BOT_BAR + 7, 2);

    // Right side: upload status + WiFi (shows IP next to SSID when idle).
    char right[40];
    const char* wifiInd = wifiConnected ? getWifiIndicator() : "";
    if (uploading && uploadTotal > 0) {
      snprintf(right, sizeof(right), "%d/%d %s", uploadCount, uploadTotal, wifiInd);
    } else if (pendingUploads > 0) {
      snprintf(right, sizeof(right), "N%d %s", pendingUploads, wifiInd);
    } else if (wifiConnected) {
      snprintf(right, sizeof(right), "%s %s", wifiInd, WiFi.localIP().toString().c_str());
    } else {
      snprintf(right, sizeof(right), "%s", wifiInd);
    }
    tft.setTextDatum(TR_DATUM);
    tft.drawString(right, SCREEN_WIDTH - 3, BOT_BAR + 7, 2);
    tft.setTextDatum(TL_DATUM);
  }
}

// Main display router
// TFT (VSPI) and SD (HSPI) are on separate buses - no conflicts. Only
// Core 1 ever touches the TFT (Core 0's upload task is display-free), so
// there's no cross-core TFT_eSPI contention to guard against here.

void updateDisplay() {
  // RC fleet panel takes over the screen while this unit is the armed Race
  // Committee — a live, colour-coded table of every peer's distance-to-line
  // and OCS state. (Checked before the boat-local OCS alarm: a stationary
  // committee boat "over" its own line isn't meaningful; it monitors the fleet.)
  if (g_role == ROLE_RC_SIGNAL && g_ocs.armed) {
    g_rcPrePanelShown = false;   // force pre-race panel repaint when we later disarm
    drawRcFleetPanel();
    return;
  }
  // RC, not armed: pre-race fleet-connection roster instead of the nav (COG/SOG)
  // display. The committee boat confirms every boat is connected + fixed here.
  if (g_role == ROLE_RC_SIGNAL) {
    g_rcPanelShown = false;      // OCS panel not shown; force its repaint on next arm
    drawRcPreRacePanel();
    return;
  }
  if (g_rcPanelShown) {  // just left the RC panel — repaint the nav display
    g_rcPanelShown = false;
    d2LayoutDrawn = false;
    d3LayoutDrawn = false;
    lastFullRedraw = 0;
  }

  // OCS alarm takes over the whole screen while this boat is over the line —
  // whether it computed that itself (ocsTick) or was forced by an RC
  // individual recall (ocsForceOver). When you're OCS the only thing that
  // matters is "you're over, come back," so the nav display is hidden and the
  // distance shows how far over you are (how far to dip back). Painted ONCE on
  // entry; only the distance value region is redrawn, on change — no per-tick
  // fillScreen (that would flicker). Big text uses font 4 scaled (fonts 6/7/8
  // are digits-only, can't render "OCS"/"RETURN").
  static bool ocsAlarmShown = false;
  static bool ocsLastInv = false;
  static int  ocsAlarmPrevDm = -1000000;  // last drawn distance, decimetres
  if (g_ocs.armed && g_ocs.over_line) {
    unsigned long now = millis();
    // Blink at ~2 Hz: invert the whole screen every 250 ms — alternate
    // white-on-black and black-on-white (black background, no red). The
    // full-screen repaint only happens on a blink-phase flip (~4×/s) or a
    // distance change, so it's the intended blink, not runaway flicker.
    bool inv = ((now / 250) % 2) == 0;
    uint16_t bg = inv ? TFT_BLACK : TFT_WHITE;
    uint16_t fg = inv ? TFT_WHITE : TFT_BLACK;
    int dm = (int)lroundf(g_ocs.distance_to_line_m * 10.0f);
    bool full = !ocsAlarmShown || (inv != ocsLastInv);  // first entry or blink flip
    if (full || dm != ocsAlarmPrevDm) {
      ocsAlarmShown = true;
      ocsLastInv = inv;
      ocsAlarmPrevDm = dm;
      char buf[24];
      snprintf(buf, sizeof(buf), "%+.1f m", g_ocs.distance_to_line_m);
      tft.setTextColor(fg, bg);
      tft.setTextDatum(MC_DATUM);
      if (full) {
        tft.fillScreen(bg);
        tft.setTextSize(5);
        tft.drawString("OCS", SCREEN_WIDTH/2, 110, 4);
        tft.setTextSize(3);
        tft.drawString("RETURN", SCREEN_WIDTH/2, 245, 4);
        tft.drawString(buf, SCREEN_WIDTH/2, 360, 4);
        tft.setTextSize(1);
      } else {
        // distance changed within the same blink phase — redraw just it
        tft.fillRect(0, 315, SCREEN_WIDTH, 100, bg);
        tft.setTextSize(3);
        tft.drawString(buf, SCREEN_WIDTH/2, 360, 4);
        tft.setTextSize(1);
      }
    }
    return;
  }
  // Just left the alarm — force the nav display to fully repaint over the red.
  if (ocsAlarmShown) {
    ocsAlarmShown = false;
    d2LayoutDrawn = false;
    d3LayoutDrawn = false;
    lastFullRedraw = 0;  // D1's force-redraw lever
  }

  if (displayMode == 1) {
    updateDisplayD1();
  } else if (displayMode == 2) {
    updateDisplayD2();
  } else {
    updateDisplayD3();
  }
}
