#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <Arduino.h>
#include "Config.h"

// =============================================================================
//  HIGH-LEVEL FLIGHT MANAGER STATE
// =============================================================================
struct FlightManager {
  bool         is_armed;
  bool         sim_mode;
  FlightMode   current_mode;
  FlightStage  flight_stage;

  int          base_throttle_us;
  int          takeoff_throttle_target;
  int          target_throttle_us;
  int          land_throttle_target;
  float        current_throttle_us;
  float        throttle_ramp_rate;     // [us/s] Climb ramp rate (default: THROTTLE_RAMP_RATE)
  float        land_ramp_rate;         // [us/s] Descent ramp rate (default: LAND_RAMP_RATE)

  bool         auto_landing_enabled;
  unsigned long hover_duration_ms;
  unsigned long hover_start_millis;
  unsigned long last_command_millis;

  // Setpoint overrides (e.g. manual pitch/roll offsets)
  float manual_target_roll_deg;
  float manual_target_pitch_deg;
  float manual_target_rate_roll_dps;
  float manual_target_rate_pitch_dps;
  float manual_target_rate_yaw_dps;

  // Output telemetry cache (updated by Core 1 control loop, read by Core 0 comms loop)
  float telemetry_out_roll;
  float telemetry_out_pitch;
  float telemetry_out_yaw;

  // Configurable safety thresholds
  float max_tilt_angle_deg;
  int   timeout_safety_s;
  float max_safe_ceiling_cm;
};

extern FlightManager flightMgr;

void disarm(const char* reason);
const char* getFlightModeName();

#endif // FLIGHT_STATE_H
