// SailFrames Firmware v2.0.0 — shared type definitions (Stage 1).
// See SF_FIRMWARE_V2_SPEC.md "Hardware role detection", "Unit role system",
// and "Radio mode state machine" for the contract these types implement.
//
// Lives in a .h (not the main .ino) so the Arduino preprocessor does not
// auto-generate forward declarations that reference these enums before
// their definitions reach the compiler.

#ifndef SAILFRAMES_V2_TYPES_H
#define SAILFRAMES_V2_TYPES_H

#include <string.h>

enum HardwarePlatform {
    HW_E1 = 1,
    HW_B1 = 2
};

enum UnitRole {
    ROLE_RACING_BOAT     = 0,
    ROLE_RC_SIGNAL       = 1,
    ROLE_RC_PIN          = 2,
    ROLE_MARK            = 3,
    ROLE_COMMITTEE_CHASE = 4,
    ROLE_SPARE           = 5
};

enum RadioMode {
    MODE_BOOT,
    MODE_IDLE,
    MODE_DOCK,
    MODE_RACING,
    MODE_RC_ACTIVE
};

static inline const char* hwName(HardwarePlatform p) {
    switch (p) { case HW_E1: return "E1"; case HW_B1: return "B1"; }
    return "?";
}

static inline const char* roleName(UnitRole r) {
    switch (r) {
        case ROLE_RACING_BOAT:     return "racing_boat";
        case ROLE_RC_SIGNAL:       return "rc_signal";
        case ROLE_RC_PIN:          return "rc_pin";
        case ROLE_MARK:            return "mark";
        case ROLE_COMMITTEE_CHASE: return "committee_chase";
        case ROLE_SPARE:           return "spare";
    }
    return "?";
}

static inline const char* radioModeName(RadioMode m) {
    switch (m) {
        case MODE_BOOT:      return "BOOT";
        case MODE_IDLE:      return "IDLE";
        case MODE_DOCK:      return "DOCK";
        case MODE_RACING:    return "RACING";
        case MODE_RC_ACTIVE: return "RC_ACTIVE";
    }
    return "?";
}

// v2.0.0 foundation globals (defined in v2_types.cpp).
extern HardwarePlatform g_hw;
extern UnitRole         g_role;
extern RadioMode        g_radio_mode;

static inline bool roleIsBase()  { return g_role == ROLE_RC_SIGNAL; }
static inline bool roleIsRover() { return g_role != ROLE_RC_SIGNAL; }

#endif
