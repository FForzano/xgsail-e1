#include "v2_types.h"

// v2.0.0 foundation globals. This build is E1-only (hardware_platform is
// always "e1" — see config.h), so g_hw never changes at runtime.
HardwarePlatform g_hw = HW_E1;
UnitRole         g_role = ROLE_RACING_BOAT;
RadioMode        g_radio_mode = MODE_BOOT;
