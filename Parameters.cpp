#include "Parameters.h"
#include <Preferences.h>
#include "FlightState.h"
#include "Controllers.h"
#include "StateEstimator.h"
#include "Config.h"

void dualPrint(const String& msg);
void dualPrintln(const String& msg);

static Preferences prefs;
static bool prefs_open = false;

// Proxy variables for parameters requiring conversion or setter hooks
static int   s_hover_time_s    = 6;
static int   s_controller_type = DEFAULT_CONTROLLER;
static int   s_flight_mode     = MODE_CASCADE;

// =============================================================================
//  PARAMETER TABLE DEFINITION (PX4 Nomenclature)
// =============================================================================
static Parameter paramTable[] = {
  // --- 1. Multicopter Altitude & Takeoff (MPC_...) ---
  { "MPC_THR_HOVER",   PARAM_TYPE_INT,   &flightMgr.takeoff_throttle_target, 1320.0f, 1150.0f, 1600.0f, "Nominal Hover Throttle [us]" },
  { "MPC_RAMP_UP",     PARAM_TYPE_FLOAT, &flightMgr.throttle_ramp_rate,       40.0f,   10.0f,   120.0f, "Climb Ramp Rate [us/s]" },
  { "MPC_RAMP_DN",     PARAM_TYPE_FLOAT, &flightMgr.land_ramp_rate,           35.0f,   10.0f,   100.0f, "Descent Ramp Rate [us/s]" },
  { "MPC_Z_TARGET",    PARAM_TYPE_FLOAT, &altControl.target_height_cm,       45.0f,   15.0f,   75.0f,  "Target Hover Height [cm]" },
  { "MPC_Z_P",         PARAM_TYPE_FLOAT, &altControl.Kp_pos_z,                1.0f,    0.1f,    5.0f,   "Altitude Position P-gain" },
  { "MPC_Z_VEL_P",     PARAM_TYPE_FLOAT, &altControl.Kp_vel_z,                1.8f,    0.2f,    8.0f,   "Altitude Velocity P-gain" },
  { "MPC_Z_VEL_I",     PARAM_TYPE_FLOAT, &altControl.Ki_vel_z,                0.50f,   0.0f,    3.0f,   "Altitude Velocity I-gain" },
  { "MPC_Z_VEL_D",     PARAM_TYPE_FLOAT, &altControl.Kd_vel_z,                0.04f,   0.0f,    0.5f,   "Altitude Velocity D-gain" },
  { "MPC_Z_CLIMB",     PARAM_TYPE_FLOAT, &altControl.max_climb_rate_mps,      0.18f,   0.05f,   0.8f,   "Max Climb Speed [m/s]" },
  { "MPC_Z_DESC",      PARAM_TYPE_FLOAT, &altControl.max_desc_rate_mps,       0.35f,   0.05f,   1.2f,   "Max Descent Speed [m/s]" },
  { "MPC_ALT_HOLD",    PARAM_TYPE_BOOL,  &altControl.alt_hold_enabled,        1.0f,    0.0f,    1.0f,   "Altitude Hold Enable (0/1)" },
  { "AUTO_LAND_EN",    PARAM_TYPE_BOOL,  &flightMgr.auto_landing_enabled,     1.0f,    0.0f,    1.0f,   "Auto Landing Enable (0/1)" },
  { "HOVER_TIME_S",    PARAM_TYPE_INT,   &s_hover_time_s,                     6.0f,    1.0f,    300.0f, "Auto Hover Duration [s]" },

  // --- 2. Multicopter Position / Horizontal GPS (MPC_...) ---
  { "MPC_POS_HOLD",    PARAM_TYPE_BOOL,  &posControl.pos_hold_enabled,        1.0f,    0.0f,    1.0f,   "GPS Position Hold Enable (0/1)" },
  { "MPC_XY_P",        PARAM_TYPE_FLOAT, &posControl.Kp_pos,                  1.2f,    0.1f,    5.0f,   "Horizontal Position P-gain" },
  { "MPC_XY_VEL_P",    PARAM_TYPE_FLOAT, &posControl.Kp_vel,                  3.5f,    0.5f,    10.0f,  "Horizontal Velocity P-gain" },
  { "MPC_XY_VEL_I",    PARAM_TYPE_FLOAT, &posControl.Ki_vel,                  1.0f,    0.0f,    5.0f,   "Horizontal Velocity I-gain" },
  { "MPC_XY_VEL_D",    PARAM_TYPE_FLOAT, &posControl.Kd_vel,                  0.15f,   0.0f,    1.0f,   "Horizontal Velocity D-gain" },
  { "MPC_XY_MAX_VEL",  PARAM_TYPE_FLOAT, &posControl.max_vel_mps,             1.5f,    0.2f,    5.0f,   "Max Return Speed [m/s]" },
  { "MPC_XY_MAX_TILT", PARAM_TYPE_FLOAT, &posControl.max_tilt_deg,            12.0f,   3.0f,    30.0f,  "Max Tilt Angle [deg]" },

  // --- 3. Multicopter Attitude PID (MC_...) ---
  { "MC_ROLL_P",       PARAM_TYPE_FLOAT, &pidController.Kp_angle_roll,        2.0f,    0.1f,    10.0f,  "Cascade Roll Angle P" },
  { "MC_ROLLRATE_P",   PARAM_TYPE_FLOAT, &pidController.Kp_rate_roll,         0.625f,  0.05f,   5.0f,   "Roll Rate P" },
  { "MC_ROLLRATE_I",   PARAM_TYPE_FLOAT, &pidController.Ki_rate_roll,         2.1f,    0.0f,    10.0f,  "Roll Rate I" },
  { "MC_ROLLRATE_D",   PARAM_TYPE_FLOAT, &pidController.Kd_rate_roll,         0.0088f, 0.0f,    0.1f,   "Roll Rate D" },
  { "MC_PITCH_P",      PARAM_TYPE_FLOAT, &pidController.Kp_angle_pitch,       2.0f,    0.1f,    10.0f,  "Cascade Pitch Angle P" },
  { "MC_PITCHRATE_P",  PARAM_TYPE_FLOAT, &pidController.Kp_rate_pitch,        0.625f,  0.05f,   5.0f,   "Pitch Rate P" },
  { "MC_PITCHRATE_I",  PARAM_TYPE_FLOAT, &pidController.Ki_rate_pitch,        2.1f,    0.0f,    10.0f,  "Pitch Rate I" },
  { "MC_PITCHRATE_D",  PARAM_TYPE_FLOAT, &pidController.Kd_rate_pitch,        0.0088f, 0.0f,    0.1f,   "Pitch Rate D" },
  { "MC_YAWRATE_P",    PARAM_TYPE_FLOAT, &pidController.Kp_rate_yaw,          2.0f,    0.1f,    10.0f,  "Yaw Rate P" },

  // --- 4. System & Sensor Configuration (SYS_...) ---
  { "SYS_USE_GPS",     PARAM_TYPE_BOOL,  &droneState.gps_enabled,             (float)(USE_GPS ? 1.0f : 0.0f), 0.0f, 1.0f, "Enable GPS Subsystem (0/1)" },
  { "SYS_USE_FLOW",    PARAM_TYPE_BOOL,  &droneState.flow_enabled,            (float)(USE_OPTICAL_FLOW ? 1.0f : 0.0f), 0.0f, 1.0f, "Enable Matek Optical Flow (0/1)" },
  { "SYS_CTRL_TYPE",   PARAM_TYPE_INT,   &s_controller_type,                  (float)DEFAULT_CONTROLLER, 0.0f, 8.0f, "Active Controller (0=PID..3=MDBCI..8=VIABC)" },
  { "SYS_FLIGHT_MOD",  PARAM_TYPE_INT,   &s_flight_mode,                      0.0f,    0.0f,    2.0f,   "Flight Mode (0=CASCADE, 1=ANGLE, 2=RATE)" },
  { "MAG_DECL_DEG",    PARAM_TYPE_FLOAT, &droneState.mag_declination_deg,     0.0f,    -180.0f, 180.0f, "Magnetic Declination [deg]" },
};

static const int NUM_PARAMS = sizeof(paramTable) / sizeof(paramTable[0]);

int getParameterCount() {
  return NUM_PARAMS;
}

// Find parameter index by name (case-insensitive)
static int findParamIndex(const String& name) {
  for (int i = 0; i < NUM_PARAMS; i++) {
    if (name.equalsIgnoreCase(paramTable[i].name)) return i;
  }
  return -1;
}

// Apply setter hooks to update high-level flight states
static void applyParamHooks(int idx) {
  const char* name = paramTable[idx].name;
  if (strcmp(name, "HOVER_TIME_S") == 0) {
    flightMgr.hover_duration_ms = (unsigned long)s_hover_time_s * 1000;
  } else if (strcmp(name, "SYS_CTRL_TYPE") == 0) {
    selectController((ControllerType)s_controller_type);
  } else if (strcmp(name, "SYS_FLIGHT_MOD") == 0) {
    flightMgr.current_mode = (FlightMode)s_flight_mode;
  } else if (strcmp(name, "MPC_Z_TARGET") == 0) {
    altControl.active_height_sp_cm = altControl.target_height_cm;
  } else if (strcmp(name, "MPC_THR_HOVER") == 0) {
    flightMgr.target_throttle_us = flightMgr.takeoff_throttle_target;
  } else if (strcmp(name, "SYS_USE_GPS") == 0) {
    setGPSEnabled(droneState.gps_enabled);
  } else if (strcmp(name, "SYS_USE_FLOW") == 0) {
    setFlowEnabled(droneState.flow_enabled);
  }
}

// =============================================================================
//  CORE STORAGE IMPLEMENTATION (ESP32 Preferences / NVS Flash)
// =============================================================================
void initParameters() {
  if (!prefs_open) {
    prefs_open = prefs.begin("drone_params", false); // NVS namespace "drone_params"
  }
  loadAllParameters();
}

void loadAllParameters() {
  if (!prefs_open) return;

  int loaded_count = 0;
  for (int i = 0; i < NUM_PARAMS; i++) {
    Parameter& p = paramTable[i];
    if (p.type == PARAM_TYPE_FLOAT) {
      float val = prefs.getFloat(p.name, p.default_val);
      *(float*)p.ptr = constrain(val, p.min_val, p.max_val);
    } else if (p.type == PARAM_TYPE_INT) {
      int val = prefs.getInt(p.name, (int)p.default_val);
      *(int*)p.ptr = constrain(val, (int)p.min_val, (int)p.max_val);
    } else if (p.type == PARAM_TYPE_BOOL) {
      bool val = prefs.getBool(p.name, (p.default_val > 0.5f));
      *(bool*)p.ptr = val;
    }
    applyParamHooks(i);
    if (prefs.isKey(p.name)) loaded_count++;
  }

  dualPrintln(">>> Parameter System initialized. (" + String(loaded_count) + "/" +
              String(NUM_PARAMS) + " loaded from Flash NVS)");
}

void saveAllParameters() {
  if (!prefs_open) return;

  for (int i = 0; i < NUM_PARAMS; i++) {
    Parameter& p = paramTable[i];
    if (p.type == PARAM_TYPE_FLOAT) {
      prefs.putFloat(p.name, *(float*)p.ptr);
    } else if (p.type == PARAM_TYPE_INT) {
      prefs.putInt(p.name, *(int*)p.ptr);
    } else if (p.type == PARAM_TYPE_BOOL) {
      prefs.putBool(p.name, *(bool*)p.ptr);
    }
  }
  dualPrintln(">>> All " + String(NUM_PARAMS) + " parameters committed to Flash.");
}

void resetAllParameters() {
  if (!prefs_open) return;

  prefs.clear(); // Wipe NVS namespace

  for (int i = 0; i < NUM_PARAMS; i++) {
    Parameter& p = paramTable[i];
    if (p.type == PARAM_TYPE_FLOAT) {
      *(float*)p.ptr = p.default_val;
    } else if (p.type == PARAM_TYPE_INT) {
      *(int*)p.ptr = (int)p.default_val;
    } else if (p.type == PARAM_TYPE_BOOL) {
      *(bool*)p.ptr = (p.default_val > 0.5f);
    }
    applyParamHooks(i);
  }
  dualPrintln(">>> All parameters reset to factory defaults.");
}

bool setParameter(const String& name, float val, bool saveToFlash) {
  int idx = findParamIndex(name);
  if (idx < 0) return false;

  Parameter& p = paramTable[idx];
  float c_val = constrain(val, p.min_val, p.max_val);

  if (p.type == PARAM_TYPE_FLOAT) {
    *(float*)p.ptr = c_val;
    if (saveToFlash && prefs_open) prefs.putFloat(p.name, c_val);
  } else if (p.type == PARAM_TYPE_INT) {
    *(int*)p.ptr = (int)round(c_val);
    if (saveToFlash && prefs_open) prefs.putInt(p.name, (int)round(c_val));
  } else if (p.type == PARAM_TYPE_BOOL) {
    bool b_val = (c_val > 0.5f);
    *(bool*)p.ptr = b_val;
    if (saveToFlash && prefs_open) prefs.putBool(p.name, b_val);
  }

  applyParamHooks(idx);
  return true;
}

bool getParameter(const String& name, float &outVal) {
  int idx = findParamIndex(name);
  if (idx < 0) return false;

  Parameter& p = paramTable[idx];
  if (p.type == PARAM_TYPE_FLOAT) {
    outVal = *(float*)p.ptr;
  } else if (p.type == PARAM_TYPE_INT) {
    outVal = (float)*(int*)p.ptr;
  } else if (p.type == PARAM_TYPE_BOOL) {
    outVal = *(bool*)p.ptr ? 1.0f : 0.0f;
  }
  return true;
}

// =============================================================================
//  PRINT / CLI INTERFACE (Pixhawk-Style Formatting)
// =============================================================================
static float getValByIndex(int i) {
  Parameter& p = paramTable[i];
  if (p.type == PARAM_TYPE_FLOAT) return *(float*)p.ptr;
  if (p.type == PARAM_TYPE_INT)   return (float)*(int*)p.ptr;
  return *(bool*)p.ptr ? 1.0f : 0.0f;
}

void printParameterList() {
  dualPrintln("\n============================ SYSTEM PARAMETERS (PX4 / NVS) ============================");
  dualPrintln("NAME             VALUE       DEFAULT     MIN        MAX        DESCRIPTION");
  dualPrintln("----------------------------------------------------------------------------------------");

  for (int i = 0; i < NUM_PARAMS; i++) {
    Parameter& p = paramTable[i];
    float cur = getValByIndex(i);
    bool modified = (fabs(cur - p.default_val) > 0.0001f);

    String nameStr = String(p.name);
    while (nameStr.length() < 17) nameStr += " ";

    String valStr = (p.type == PARAM_TYPE_INT) ? String((int)cur) :
                    ((p.type == PARAM_TYPE_BOOL) ? String((int)cur) : String(cur, (fabs(cur) < 0.01f) ? 4 : 2));
    if (modified) valStr += "*";
    while (valStr.length() < 12) valStr += " ";

    String defStr = (p.type == PARAM_TYPE_INT || p.type == PARAM_TYPE_BOOL) ? String((int)p.default_val) : String(p.default_val, 2);
    while (defStr.length() < 12) defStr += " ";

    String minStr = (p.type == PARAM_TYPE_INT || p.type == PARAM_TYPE_BOOL) ? String((int)p.min_val) : String(p.min_val, 1);
    while (minStr.length() < 11) minStr += " ";

    String maxStr = (p.type == PARAM_TYPE_INT || p.type == PARAM_TYPE_BOOL) ? String((int)p.max_val) : String(p.max_val, 1);
    while (maxStr.length() < 11) maxStr += " ";

    dualPrintln(nameStr + valStr + defStr + minStr + maxStr + String(p.desc));
  }
  dualPrintln("----------------------------------------------------------------------------------------");
  dualPrintln(" * = modified from factory default (stored in Flash NVS: 'drone_params')");
  dualPrintln(" Commands: param set <NAME> <VAL> | param get <NAME> | param reset | param save\n");
}

void printParameter(const String& name) {
  int idx = findParamIndex(name);
  if (idx < 0) {
    dualPrintln("ERR: Parameter '" + name + "' not found. Type 'param show' to view all.");
    return;
  }
  float cur = getValByIndex(idx);
  dualPrintln(String(paramTable[idx].name) + " = " + String(cur, 3) + " (" + String(paramTable[idx].desc) + ")");
}

bool handleParameterCommand(const String& cmd) {
  String s = cmd;
  s.trim();

  if (s.equalsIgnoreCase("param show") || s.equalsIgnoreCase("param list") ||
      s.equalsIgnoreCase("params") || s.equalsIgnoreCase("param")) {
    printParameterList();
    return true;
  }

  if (s.equalsIgnoreCase("param save")) {
    saveAllParameters();
    return true;
  }

  if (s.equalsIgnoreCase("param reset")) {
    resetAllParameters();
    return true;
  }

  if (s.startsWith("param get ")) {
    String pName = s.substring(10);
    pName.trim();
    printParameter(pName);
    return true;
  }

  if (s.startsWith("param set ")) {
    String rest = s.substring(10);
    rest.trim();
    int spaceIdx = rest.indexOf(' ');
    if (spaceIdx < 0) {
      dualPrintln("Usage: param set <NAME> <VALUE>");
      return true;
    }
    String pName = rest.substring(0, spaceIdx);
    pName.trim();
    float val = rest.substring(spaceIdx + 1).toFloat();

    if (setParameter(pName, val, true)) {
      int idx = findParamIndex(pName);
      float newVal = getValByIndex(idx);
      dualPrintln("--> Parameter " + String(paramTable[idx].name) + " set to " + String(newVal, 3) + " (Saved to Flash)");
    } else {
      dualPrintln("ERR: Unknown parameter '" + pName + "'. Type 'param show' to view list.");
    }
    return true;
  }

  return false;
}
