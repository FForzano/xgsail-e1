// Config struct definition glue — see config.h. Loads/parses /config.txt
// on SD into the shared `config` instance.
#include "config.h"
#include "v2_types.h"
#include <SD.h>

Config config;

void loadConfig() {
  File f = SD.open("/config.txt", FILE_READ);
  if (!f) { Serial.println("[CFG] No config.txt"); return; }
  Serial.println("[CFG] Loading config.txt");

  // Temp storage for parsing wifi entries
  char tempSSID[64] = "";
  char tempPass[64] = "";
  int currentWifiIdx = -1;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("#") || line.length() == 0) continue;
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String k = line.substring(0, eq); k.trim();
    String v = line.substring(eq + 1); v.trim();

    // Parse wifi1_ssid, wifi2_ssid, etc. (1-indexed in config file)
    if (k.startsWith("wifi") && k.endsWith("_ssid")) {
      int idx = k.substring(4, k.length() - 5).toInt() - 1;  // wifi1 -> index 0
      if (idx >= 0 && idx < MAX_WIFI_NETWORKS) {
        v.toCharArray(config.wifi[idx].ssid, sizeof(config.wifi[idx].ssid));
        if (idx >= config.wifi_count) config.wifi_count = idx + 1;
      }
    }
    else if (k.startsWith("wifi") && k.endsWith("_pass")) {
      int idx = k.substring(4, k.length() - 5).toInt() - 1;
      if (idx >= 0 && idx < MAX_WIFI_NETWORKS) {
        v.toCharArray(config.wifi[idx].pass, sizeof(config.wifi[idx].pass));
      }
    }
    // Also support legacy single wifi_ssid/wifi_pass
    else if (k == "wifi_ssid") {
      v.toCharArray(config.wifi[0].ssid, sizeof(config.wifi[0].ssid));
      if (config.wifi_count == 0) config.wifi_count = 1;
    }
    else if (k == "wifi_pass") {
      v.toCharArray(config.wifi[0].pass, sizeof(config.wifi[0].pass));
    }
    else if (k == "api_base_url") v.toCharArray(config.api_base_url, sizeof(config.api_base_url));
    else if (k == "claim_code") v.toCharArray(config.claim_code, sizeof(config.claim_code));
    else if (k == "boat_id") v.toCharArray(config.boat_id, sizeof(config.boat_id));
    else if (k == "wind_enabled") config.wind_enabled = (v == "true" || v == "1");
    else if (k == "wind_mac") v.toCharArray(config.wind_mac, sizeof(config.wind_mac));
    else if (k == "wind_offset") config.wind_offset = v.toInt();
    // Recording thresholds
    else if (k == "start_speed_knots") config.start_speed_knots = v.toFloat();
    else if (k == "stop_speed_knots") config.stop_speed_knots = v.toFloat();
    else if (k == "start_delay_sec") config.start_delay_sec = v.toInt();
    else if (k == "stop_delay_sec") config.stop_delay_sec = v.toInt();
    // v2.0.0 foundation
    else if (k == "hardware_platform") v.toCharArray(config.hardware_platform, sizeof(config.hardware_platform));
    else if (k == "unit_role")         v.toCharArray(config.unit_role, sizeof(config.unit_role));
    else if (k == "rtk_enabled")       config.rtk_enabled = (v == "1" || v.equalsIgnoreCase("true"));
  }
  f.close();

  // This build is E1-only, so g_hw is always HW_E1 regardless of what
  // hardware_platform says (kept in Config for on-disk/cloud-config
  // compatibility with the wider SailFrames Core fleet).
  g_hw = HW_E1;

  if      (strcasecmp(config.unit_role, "racing_boat")     == 0) g_role = ROLE_RACING_BOAT;
  else if (strcasecmp(config.unit_role, "rc_signal")       == 0) g_role = ROLE_RC_SIGNAL;
  else if (strcasecmp(config.unit_role, "rc_pin")          == 0) g_role = ROLE_RC_PIN;
  else if (strcasecmp(config.unit_role, "mark")            == 0) g_role = ROLE_MARK;
  else if (strcasecmp(config.unit_role, "committee_chase") == 0) g_role = ROLE_COMMITTEE_CHASE;
  else if (strcasecmp(config.unit_role, "spare")           == 0) g_role = ROLE_SPARE;
  else                                                            g_role = ROLE_RACING_BOAT;

  Serial.printf("[CFG] Boat: %s, WiFi networks: %d\n",
    config.boat_id, config.wifi_count);
  for (int i = 0; i < config.wifi_count; i++) {
    Serial.printf("[CFG]   %d: %s\n", i + 1, config.wifi[i].ssid);
  }
  Serial.printf("[CFG] Wind: %s", config.wind_enabled ? "enabled" : "disabled");
  if (strlen(config.wind_mac) > 0) {
    Serial.printf(" (MAC: %s)", config.wind_mac);
  }
  if (config.wind_offset != 0) {
    Serial.printf(" (offset: %d°)", config.wind_offset);
  }
  Serial.println();
  Serial.printf("[CFG] Platform: %s | Role: %s\n", hwName(g_hw), roleName(g_role));
  if (strlen(config.api_base_url) == 0) {
    Serial.println("[CFG] WARNING: api_base_url not set — device-protocol calls will not work");
  }
  Serial.printf("[CFG] Sample rates (firmware-baked): IMU %d ms | GNSS fix %d ms\n",
                IMU_INTERVAL_MS, 1000 / 10);  // GNSS via PQTMCFGFIXRATE,W,100
}
