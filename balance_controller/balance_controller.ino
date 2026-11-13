// =============================================================================
//  Oqab Modular Flight Controller (ESP32)
//
//  Selectable Attitude Controllers (Config.h default or runtime 'ctrl <name>'):
//    1. PID   : Cascaded Angle P -> Inner Rate PID (or Single Angle / Rate)
//    2. SMC   : Sliding Mode Control with Continuous Boundary Layer
//    3. ADRC  : Linear Active Disturbance Rejection Control (1st/2nd Order ESO)
//    4. MDBCI : Modified Dual Boundary Conditional Integral Backstepping (ISA 2020 Eq. 44)
//    5. MDBCIS: MDBCI + Jerk-Mitigating Switched Controller (ISA 2020 Section 4.2)
//    6. CDI   : Conditional Integration Backstepping (Seshagiri & Khalil 2005)
//    7. CVI   : Conventional Integral Backstepping (Constant lambda_m)
//    8. NOI   : No-Integration Backstepping (lambda = 0)
//    9. VIABC : Variable Integral Adaptive Backstepping (IEEE Access 2024 Paper)
//
//  Hierarchical Cascaded Control Architecture:
//    - Outer XY Position Loop (10 Hz)   : GPS NEO-M8N -> Drone Body Frame Tilt Setpoints
//    - Outer Altitude Loop (50 Hz)      : 3-State EKF (Sharp IR + BMP180 + Accel) -> Throttle Adj
//    - Attitude Loop (250 Hz)           : MPU6050 + GY-87 Compass -> Selected Controller -> Motor Mixer
//
//  Modular Architecture:
//    - Config.h         : Configuration parameters, pins, safety thresholds
//    - FlightState.h    : High-level mission and flight manager state
//    - Sensors.h/.cpp   : Swappable sensor interfaces (IMU, Rangefinder, Baro, Compass, GPS)
//    - StateEstimator.* : Attitude filter, 3-state vertical Kalman filter, GPS Cartesian projection
//    - Controllers.*    : Modular PID, SMC, ADRC & Backstepping controller implementations & outer loops
//    - Motors.h/.cpp    : ESC PWM servo outputs, Quad-X mixer, emergency stop
//    - Comms.h/.cpp     : Serial & Bluetooth command processor and telemetry
// =============================================================================

#include <Wire.h>
#include "Config.h"
#include "FlightState.h"
#include "Sensors.h"
#include "StateEstimator.h"
#include "Controllers.h"
#include "Motors.h"
#include "Comms.h"
#include "Parameters.h"
#include "MissionSystem.h"

FlightManager flightMgr;

// Loop timing records
static unsigned long last_loop_micros     = 0;
static unsigned long last_alt_millis      = 0;
static unsigned long last_gps_loop_millis = 0;

// =============================================================================
//  HELPER FUNCTIONS
// =============================================================================
const char* getFlightModeName() {
  if (flightMgr.current_mode == MODE_POS_HOLD) return "POS_HOLD (Position+Altitude Hold)";
  if (flightMgr.current_mode == MODE_ALT_HOLD) return "ALT_HOLD (Altitude Hold Only)";
  return "MANUAL (Attitude Stabilization)";
}

void disarm(const char* reason) {
  flightMgr.is_armed            = false;
  flightMgr.sim_mode            = false;
  flightMgr.flight_stage        = STAGE_IDLE;
  flightMgr.current_throttle_us = 1000.0f;
  flightMgr.base_throttle_us    = 1000;
  flightMgr.target_throttle_us  = flightMgr.takeoff_throttle_target;

  stopMotors();
  getActiveController()->reset();
  resetPositionControl();
  resetAltitudeControl();
  resetEstimatorStates();
  missionSys.stop(reason);

  dualPrintln("\n!!! DISARMED: " + String(reason));
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  // 0. Prioritize the real-time flight loop task on Core 1
  vTaskPrioritySet(NULL, 15);

  // 1. Arm ESCs IMMEDIATELY at power-on with 1000us idle pulse so they never beep in alarm
  initMotors();

  // 2. Initialize Comms & Serial
  initComms();

  dualPrintln("\n--- Oqab Unified Modular Flight Controller ---");
  dualPrintln("BT Device: " + String(BT_DEVICE_NAME));

  Wire.begin();
  Wire.setClock(400000);

  // Initialize Flight Manager Defaults
  flightMgr.is_armed                = false;
  flightMgr.sim_mode                = false;
  flightMgr.current_mode            = MODE_POS_HOLD;
  flightMgr.flight_stage            = STAGE_IDLE;
  flightMgr.base_throttle_us        = 1000;
  flightMgr.takeoff_throttle_target = 1450; // Nominal hover baseline (auto-latched if MPC_HOVER_LEARN=1)
  flightMgr.target_throttle_us      = 1450;
  flightMgr.land_throttle_target    = 1280;
  flightMgr.min_armed_throttle_us   = MIN_ARMED_PWM;
  flightMgr.stop_triggers_land      = true;
  flightMgr.current_throttle_us     = 1000.0f;
  flightMgr.throttle_ramp_rate      = THROTTLE_RAMP_RATE; // 70.0 us/s (gentle climb ramp)
  flightMgr.land_ramp_rate          = LAND_RAMP_RATE;     // 35.0 us/s (normal landing ramp)
  flightMgr.emerg_land_ramp_rate    = EMERG_LAND_RAMP_RATE; // 100.0 us/s (emergency landing ramp)
  flightMgr.is_emergency_landing    = false;
  flightMgr.auto_hover_calc_enabled = true;               // Auto hover throttle calculation enabled by default
  flightMgr.auto_landing_enabled    = true;
  flightMgr.hover_duration_ms       = 6000; // 6s auto-hover
  flightMgr.last_command_millis     = millis();
  flightMgr.max_tilt_angle_deg      = MAX_TILT_ANGLE_DEG; // 40.0 deg
  flightMgr.timeout_safety_s        = TIMEOUT_SAFETY_MS / 1000; // 30 s
  flightMgr.max_safe_ceiling_cm     = MAX_SAFE_CEILING_CM; // 185.0 cm

  // Initialize State Estimator & Controllers
  initStateEstimator();
  initControllers(1.0f / LOOP_RATE_HZ);

  // Initialize Persistent Parameter System (Loads tuned values from Flash NVS)
  initParameters();

  // Initialize Mission System (loads default takeoff -> (0,0) -> hover -> land plan)
  missionSys.init();

  // Initialize Swappable Sensor Subsystem
  initSensors();

  if (droneState.flow_enabled && flowSensor) {
    if (flowSensor->begin()) {
      dualPrintln(">>> Mateksys Optical Flow (3901-L0X) started (RX:16, TX:17) @ " + String(FLOW_BAUD) + " baud (MSPv2) [ONLINE]");
    } else {
      dualPrintln(">>> WARNING: Mateksys Optical Flow not responding on RX:16, TX:17! [OFFLINE]");
    }
  } else if (droneState.gps_enabled && gpsSensor) {
    if (gpsSensor->begin()) {
      dualPrintln(">>> GPS Serial started (RX:16, TX:17) @ " + String(GPS_BAUD) + " baud (NMEA) [ONLINE]");
    } else {
      dualPrintln(">>> WARNING: GPS not responding on RX:16, TX:17! [OFFLINE - Check wiring/power]");
    }
  } else {
    dualPrintln(">>> Navigation Sensors (GPS / Optical Flow) DISABLED [Pure Inertial Mode active]");
  }

  dualPrintln(">>> ESCs initialized with 1000us idle pulse.");

  // Initialize IMU
  if (!imuSensor || !imuSensor->begin()) {
    dualPrintln("FATAL: IMU sensor not found!");
    while (1) { stopMotors(); delay(100); }
  }

  // Initialize Auxiliary Sensors
  if (compassSensor) compassSensor->begin();
  dualPrintln(">>> Barometer & Sharp IR: DEWIRED / REMOVED (Mateksys 3901-L0X ToF LiDAR active)");

  // Run Sensor Calibration & Set Boot Home
  calibrateAllSensors();

  dualPrintln("\nActive Controller : " + String(getActiveControllerName()));
  dualPrintln("Active Mode       : " + String(getFlightModeName()));
  dualPrintln("Flight Mission    : Climb to " + String(altControl.target_height_cm, 0) +
              "cm -> Hold " + String(flightMgr.hover_duration_ms / 1000) + "s -> Smooth Land");
  dualPrintln("Altitude Control  : PURE MATEKSYS ToF LIDAR MODE (Target: " + String(altControl.target_height_cm, 1) + "cm, Safe Ceil: " + String(MAX_SAFE_CEILING_CM, 1) + "cm)");
  dualPrintln("Key Commands      : 'arm', 'disarm', 'land', 'param show', 'param set <NAME> <VAL>', 'status'");
  dualPrintln("-------------------------------------------------------------------\n");

  // Start FreeRTOS Comms, Telemetry & Sensor Parsing Task pinned to Core 0
  startCommsTask();

  last_loop_micros     = micros();
  last_alt_millis      = millis();
  last_gps_loop_millis = millis();
}

// =============================================================================
//  MAIN DETERMINISTIC LOOP (250 Hz IMU, 50 Hz Altitude, 10 Hz GPS) - Core 1
// =============================================================================
void loop() {
  // Safety Watchdog: Disarm if no keepalive or command received within configurable timeout
  if (flightMgr.is_armed && (millis() - flightMgr.last_command_millis > ((unsigned long)flightMgr.timeout_safety_s * 1000))) {
    disarm("Safety watchdog: link timeout while armed");
  }

  // Deterministic 250 Hz timing (4000 us)
  unsigned long current_micros = micros();
  if (current_micros - last_loop_micros < LOOP_PERIOD_US) {
    taskYIELD();
    return;
  }

  float dt = (current_micros - last_loop_micros) / 1000000.0f;
  last_loop_micros = current_micros;
  if (dt > 0.02f) dt = 0.004f;

  // --- 1. 250 Hz Attitude Estimation ---
  updateAttitudeEstimator(dt);

  // --- 2. 50 Hz Altitude, Horizontal KF & Compass Sub-Loop (every 20ms) ---
  unsigned long current_millis = millis();
  if (current_millis - last_alt_millis >= ALT_LOOP_PERIOD_MS) {
    float dt_alt = (current_millis - last_alt_millis) / 1000.0f;
    last_alt_millis = current_millis;

    updateAltitudeEstimator(dt_alt, droneState.is_airborne, flightMgr.sim_mode);
    updateHorizontalEstimator(dt_alt, droneState.is_airborne, flightMgr.sim_mode);

    // Altitude controller: active in POS_HOLD and ALT_HOLD modes
    bool alt_mode_active = (flightMgr.current_mode == MODE_POS_HOLD || flightMgr.current_mode == MODE_ALT_HOLD);
    computeAltitudeControl(dt_alt, flightMgr.base_throttle_us, flightMgr.flight_stage,
                           droneState.is_airborne, flightMgr.sim_mode, flightMgr.is_armed && alt_mode_active);

    // Horizontal position controller: active strictly in POS_HOLD mode
    bool pos_mode_active = (flightMgr.current_mode == MODE_POS_HOLD);
    computePositionControl(dt_alt, droneState.is_airborne && pos_mode_active, flightMgr.sim_mode && pos_mode_active);
    updateCompassEstimator();

    // Mission System state machine update (50 Hz navigation sequencing)
    if (missionSys.isActive()) {
      missionSys.update(dt_alt);
    }
  }

  // --- 3. 10 Hz GPS Position Sub-Loop (every 100ms) ---
  if (current_millis - last_gps_loop_millis >= GPS_LOOP_PERIOD_MS) {
    float dt_gps = (current_millis - last_gps_loop_millis) / 1000.0f;
    last_gps_loop_millis = current_millis;

    if (droneState.gps_enabled) {
      updatePositionEstimator(dt_gps, flightMgr.is_armed);
    }
  }

  // --- 4. Airborne Detection & Anti-Windup Trigger ---
  // Use a slow EMA to track filtered vertical accel (low-pass against noise)
  droneState.az_filt += 0.05f * (droneState.az_body - droneState.az_filt);

  if (!flightMgr.is_armed) {
    // When disarmed, the drone is physically resting on the ground.
    droneState.is_airborne = false;
    droneState.above_thresh_since_ms = 0;
  } else if (!flightMgr.sim_mode) {
    // Height-based detection: fused altitude is > 5cm above calibrated ground
    bool height_airborne = (droneState.current_height_cm > (droneState.ground_height_cm + 5.0f));

    // ToF LiDAR detection (primary liftoff detector):
    // Ground reading is ~8cm (calibrated in ground_height_cm); increases smoothly as drone lifts off.
    bool lidar_valid = droneState.rf_valid && (!isnan(droneState.raw_height_cm)) &&
                       (droneState.raw_height_cm >= 4.0f && droneState.raw_height_cm <= 200.0f);
    float lidar_dist = (droneState.rf_valid && droneState.z_rf_corrected_m > 0.03f) ?
                       (droneState.z_rf_corrected_m * 100.0f) : droneState.raw_height_cm;
    bool lidar_airborne = lidar_valid && (lidar_dist > (droneState.ground_height_cm + 4.0f));

    // Liftoff requires motors actively commanding genuine takeoff throttle (>= 1260us)
    bool throttle_lift_ready = (flightMgr.base_throttle_us >= 1260);
    bool physical_lift       = (height_airborne || lidar_airborne);

    // Debounce physical liftoff to eliminate transient vibration spikes & prop wash flutter:
    // Requires physical height increase sustained for >= 150 ms before declaring airborne
    static unsigned long lift_candidate_start_ms = 0;
    if (throttle_lift_ready && physical_lift) {
      if (lift_candidate_start_ms == 0) lift_candidate_start_ms = millis();
    } else {
      lift_candidate_start_ms = 0;
    }
    bool confirmed_liftoff = (lift_candidate_start_ms != 0 && (millis() - lift_candidate_start_ms >= 150));

    // Fallback time gate: if throttle reaches TAKEOFF_THROTTLE (~1360us) and holds for 600ms
    if (throttle_lift_ready && flightMgr.base_throttle_us >= TAKEOFF_THROTTLE) {
      if (droneState.above_thresh_since_ms == 0) droneState.above_thresh_since_ms = millis();
    } else {
      droneState.above_thresh_since_ms = 0;
    }
    bool fallback_liftoff = (droneState.above_thresh_since_ms != 0 && (millis() - droneState.above_thresh_since_ms >= 600));

    if (throttle_lift_ready && (confirmed_liftoff || fallback_liftoff)) {
      if (!droneState.is_airborne) {
        droneState.is_airborne = true;
        const char* trigger_src = confirmed_liftoff ? "LIDAR/HEIGHT (Debounced)" : "THROTTLE TIME GATE";
        // PX4/ArduPilot pattern: keep configured/learned MPC_THR_HOVER as nominal baseline
        // instead of latching open-loop ramp overshoot; seed alt_throttle_adj for bumpless handoff.
        flightMgr.takeoff_throttle_target = (int)paramMgr.getParam("MPC_THR_HOVER");
        flightMgr.target_throttle_us      = flightMgr.takeoff_throttle_target;
        altControl.alt_throttle_adj       = constrain(
            flightMgr.current_throttle_us - (float)flightMgr.takeoff_throttle_target, -60.0f, 80.0f);
        altControl.active_height_sp_cm    = max(droneState.current_height_cm, droneState.ground_height_cm);
        dualPrintln(">>> LIFTOFF CONFIRMED via " + String(trigger_src) +
                    " (LiDAR: " + String(lidar_dist, 1) + "cm, H: " + String(droneState.current_height_cm, 1) +
                    "cm). Hover Baseline: " + String(flightMgr.takeoff_throttle_target) + "us");
      }
    } else {
      // Confirm landed only when throttle drops AND sensors show near-ground
      bool landed_by_height = (droneState.current_height_cm <= (droneState.ground_height_cm + 4.0f));
      bool landed_by_lidar  = lidar_valid && (lidar_dist <= (droneState.ground_height_cm + 3.0f));

      if (droneState.is_airborne && (landed_by_height || landed_by_lidar || (flightMgr.base_throttle_us < MIN_ARMED_PWM + 80))) {
        droneState.is_airborne = false;
        flightMgr.is_emergency_landing = false;
        dualPrintln(">>> LANDED (H: " + String(droneState.current_height_cm, 1) +
                    "cm, LiDAR: " + String(lidar_dist, 1) + "cm)");
      }
      droneState.above_thresh_since_ms = 0;
      lift_candidate_start_ms          = 0;
    }
  }

  // --- 5. Tilt & Ceiling Flyaway Protection ---
  if (flightMgr.is_armed && !flightMgr.sim_mode && (fabs(droneState.roll_deg) > flightMgr.max_tilt_angle_deg || fabs(droneState.pitch_deg) > flightMgr.max_tilt_angle_deg)) {
    disarm("CRASH PROTECTION: Excessive tilt angle detected!");
  }

  // Mateksys ToF LiDAR Ceiling Guard: Prevents drone from flying beyond safe ceiling
  static unsigned long ceiling_violation_start_ms = 0;
  bool near_ceiling = (droneState.current_height_cm > flightMgr.max_safe_ceiling_cm || droneState.raw_height_cm >= (flightMgr.max_safe_ceiling_cm - 5.0f));
  if (flightMgr.is_armed && droneState.is_airborne && near_ceiling) {
    if (ceiling_violation_start_ms == 0) ceiling_violation_start_ms = millis();
    if (millis() - ceiling_violation_start_ms >= 200) {
      if (flightMgr.flight_stage != STAGE_LANDING && flightMgr.flight_stage != STAGE_TOUCHDOWN) {
        flightMgr.flight_stage            = STAGE_LANDING;
        flightMgr.is_emergency_landing    = true;
        altControl.active_height_sp_cm    = altControl.target_height_cm;
        flightMgr.takeoff_throttle_target = max(flightMgr.takeoff_throttle_target - 100, MIN_ARMED_PWM + 80);
        dualPrintln(">>> EMERGENCY CEILING INTERVENTION: Height near 180cm! Immediate emergency descent triggered.");
      }
    }
  } else {
    ceiling_violation_start_ms = 0;

  }

  // --- 6. Mission State Machine & Throttle Calculation ---
  float out_roll = 0.0f, out_pitch = 0.0f, out_yaw = 0.0f;

  if (flightMgr.is_armed) {
    bool use_auto_alt = (flightMgr.current_mode == MODE_POS_HOLD || flightMgr.current_mode == MODE_ALT_HOLD);
    if (flightMgr.auto_landing_enabled && use_auto_alt && !flightMgr.sim_mode) {

      // ----------------------------------------------------------------
      // Stage 1: Climb Ramp — smooth ground ramp, stops ramping once airborne
      //   + altitude PID smoothly guides drone to 100cm target height
      // ----------------------------------------------------------------
      if (flightMgr.flight_stage == STAGE_RAMP_UP) {
        if (!droneState.is_airborne) {
          // On ground: ramp base throttle smoothly toward takeoff (uncapped by guess; ramps up to MAX_TAKEOFF_RAMP_PWM)
          float ramp_rate = (flightMgr.throttle_ramp_rate >= 5.0f) ? flightMgr.throttle_ramp_rate : THROTTLE_RAMP_RATE;
          if (flightMgr.current_throttle_us < MAX_TAKEOFF_RAMP_PWM) {
            flightMgr.current_throttle_us += ramp_rate * dt;
            if (flightMgr.current_throttle_us > MAX_TAKEOFF_RAMP_PWM)
              flightMgr.current_throttle_us = MAX_TAKEOFF_RAMP_PWM;
          }
          altControl.active_height_sp_cm = droneState.ground_height_cm;
        } else {
          // Once airborne: STOP open-loop ramping! Altitude PID controls ascent
          flightMgr.current_throttle_us = (float)flightMgr.takeoff_throttle_target;

          if (altControl.alt_hold_enabled) {
            // PX4 PositionControl pattern: command target height directly;
            // outer P-loop + max_climb_rate_mps (MPC_Z_CLIMB) govern climb speed and smooth deceleration.
            altControl.active_height_sp_cm = altControl.target_height_cm;
          }

          // Transition to HOVER only once physical height reaches target (within 5cm)
          bool height_reached = altControl.alt_hold_enabled &&
                                (droneState.current_height_cm >= (altControl.target_height_cm - 5.0f));

          if (height_reached) {
            flightMgr.flight_stage       = STAGE_HOVER;
            flightMgr.hover_start_millis = millis();
            altControl.active_height_sp_cm = altControl.target_height_cm;
            dualPrintln("--> Hover stage reached (" + String(flightMgr.takeoff_throttle_target) +
                        "us). Holding at " + String(altControl.target_height_cm, 1) + "cm for " +
                        String(flightMgr.hover_duration_ms / 1000) + "s...");
          }
        }

      // ----------------------------------------------------------------
      // Stage 2: Stable Hover — altitude PID holds target height & learns hover thrust
      // ----------------------------------------------------------------
      } else if (flightMgr.flight_stage == STAGE_HOVER) {
        altControl.active_height_sp_cm = altControl.target_height_cm;

        // Adaptive In-Flight Hover Learning (PX4/ArduPilot Architecture):
        // Accumulate fractional trim in a float and transfer whole-us steps bumplessly out of vel_i_z.
        static unsigned long hover_stable_start_ms = 0;
        static bool hover_saved_to_nvs = false;
        static float hover_learn_accum = 0.0f;

        if (flightMgr.auto_hover_calc_enabled) {
          float height_err = fabs(droneState.current_height_cm - altControl.target_height_cm);

          if (height_err < 5.0f && fabs(droneState.kf_vz) < 0.12f) {
            if (hover_stable_start_ms == 0) hover_stable_start_ms = millis();

            // Bleed 5% of PID adjustment per second into fractional hover accumulator
            hover_learn_accum += altControl.alt_throttle_adj * 0.05f * dt;
            int delta_us = (int)truncf(hover_learn_accum);
            if (delta_us != 0) {
              int prev_hover = flightMgr.takeoff_throttle_target;
              flightMgr.takeoff_throttle_target = constrain(prev_hover + delta_us, 1150, 1700);
              int applied_us = flightMgr.takeoff_throttle_target - prev_hover;
              hover_learn_accum -= (float)delta_us;

              // Bumpless transfer: subtract applied step from vel_i_z so total thrust stays continuous
              float hover_thrust_us = max(200.0f, (float)(prev_hover - 1000));
              float acc_to_pwm = hover_thrust_us / 9.80665f;
              altControl.vel_i_z = constrain(altControl.vel_i_z - ((float)applied_us / acc_to_pwm), -2.5f, 1.2f);
            }

            // When steady for 2.0s, save learned hover throttle to Flash NVS
            if (!hover_saved_to_nvs && (millis() - hover_stable_start_ms >= 2000)) {
              setParameter("MPC_THR_HOVER", (float)flightMgr.takeoff_throttle_target, true);
              hover_saved_to_nvs = true;
              dualPrintln(">>> Auto-Learned Hover Throttle: " + String(flightMgr.takeoff_throttle_target) +
                          "us (Saved to Flash parameter MPC_THR_HOVER)");
            }
          } else {
            hover_stable_start_ms = 0;
          }
        }

        if (millis() - flightMgr.hover_start_millis >= flightMgr.hover_duration_ms) {
          flightMgr.flight_stage         = STAGE_LANDING;
          flightMgr.is_emergency_landing = false;
          altControl.active_height_sp_cm = droneState.current_height_cm; // Land from current actual height
          flightMgr.target_throttle_us   = flightMgr.land_throttle_target;
          hover_stable_start_ms          = 0;
          hover_saved_to_nvs             = false;
          hover_learn_accum              = 0.0f;
          dualPrintln("--> Hover complete! Beginning controlled descent (" +
                      String(flightMgr.land_ramp_rate, 0) + " us/s) from " +
                      String(droneState.current_height_cm, 1) + "cm...");
        }

      // ----------------------------------------------------------------
      // Stage 3: Smooth Controlled Landing — separate normal vs emergency descent rates
      // ----------------------------------------------------------------
      } else if (flightMgr.flight_stage == STAGE_LANDING) {
        float land_rate = flightMgr.is_emergency_landing ?
                          ((flightMgr.emerg_land_ramp_rate >= 10.0f) ? flightMgr.emerg_land_ramp_rate : EMERG_LAND_RAMP_RATE) :
                          ((flightMgr.land_ramp_rate >= 5.0f) ? flightMgr.land_ramp_rate : LAND_RAMP_RATE);

        if (altControl.alt_hold_enabled) {
          // Emergency descent setpoint drops faster (40 cm/s) vs gentle normal descent (12 cm/s)
          float desc_cmps = flightMgr.is_emergency_landing ? 40.0f : 12.0f;
          altControl.active_height_sp_cm -= desc_cmps * dt;
          // Floor: stop setpoint just above ground so PID doesn't overshoot into earth
          if (altControl.active_height_sp_cm < (droneState.ground_height_cm + 2.0f))
            altControl.active_height_sp_cm = droneState.ground_height_cm + 2.0f;

          // In emergency landing: also ramp down takeoff_throttle_target baseline so throttle responds promptly
          if (flightMgr.is_emergency_landing) {
            if (flightMgr.takeoff_throttle_target > flightMgr.land_throttle_target) {
              flightMgr.takeoff_throttle_target -= (int)(land_rate * dt);
              if (flightMgr.takeoff_throttle_target < flightMgr.land_throttle_target)
                flightMgr.takeoff_throttle_target = flightMgr.land_throttle_target;
            }
          }

          // Trigger touchdown once drone is near ground AND setpoint has descended
          if (droneState.current_height_cm <= (droneState.ground_height_cm + 4.0f) &&
              altControl.active_height_sp_cm <= (droneState.ground_height_cm + 5.0f)) {
            flightMgr.flight_stage = STAGE_TOUCHDOWN;
            dualPrintln("--> Touchdown! (" + String(droneState.current_height_cm, 1) + "cm). Cutting motors...");
          }
        } else {
          // Open-loop descent: ramp throttle down to land target
          if (flightMgr.current_throttle_us > flightMgr.land_throttle_target) {
            flightMgr.current_throttle_us -= land_rate * dt;
            if (flightMgr.current_throttle_us < flightMgr.land_throttle_target)
              flightMgr.current_throttle_us = flightMgr.land_throttle_target;
          }
          if (flightMgr.current_throttle_us <= (flightMgr.land_throttle_target + 8.0f))
            flightMgr.flight_stage = STAGE_TOUCHDOWN;
        }

      // ----------------------------------------------------------------
      // Stage 4: Touchdown — rapidly cut motors and disarm
      // ----------------------------------------------------------------
      } else if (flightMgr.flight_stage == STAGE_TOUCHDOWN) {
        float land_rate = flightMgr.is_emergency_landing ?
                          ((flightMgr.emerg_land_ramp_rate >= 10.0f) ? flightMgr.emerg_land_ramp_rate : EMERG_LAND_RAMP_RATE) :
                          ((flightMgr.land_ramp_rate >= 5.0f) ? flightMgr.land_ramp_rate : LAND_RAMP_RATE);
        int floor_pwm = (flightMgr.min_armed_throttle_us >= 1000) ? flightMgr.min_armed_throttle_us : MIN_ARMED_PWM;
        if (flightMgr.current_throttle_us > floor_pwm) {
          flightMgr.current_throttle_us -= land_rate * dt * 2.0f;
          if (flightMgr.current_throttle_us < floor_pwm)
            flightMgr.current_throttle_us = floor_pwm;
        }
        if (flightMgr.current_throttle_us <= (floor_pwm + 5.0f))
          disarm("Auto-landing complete: Drone touched down safely.");
      }

      // ----------------------------------------------------------------
      // Base Throttle Assembly
      //   HOVER / LANDING (alt_hold): reference throttle + altitude PID adj
      //   RAMP_UP (alt_hold):         ramping throttle + altitude PID adj
      //   Everything else:            raw current_throttle
      // ----------------------------------------------------------------
      int floor_pwm = (flightMgr.min_armed_throttle_us >= 1000) ? flightMgr.min_armed_throttle_us : MIN_ARMED_PWM;
      if (flightMgr.flight_stage == STAGE_HOVER ||
          (flightMgr.flight_stage == STAGE_LANDING && altControl.alt_hold_enabled)) {
        // In hover/landing: PID adj is relative to the latched hover throttle
        flightMgr.base_throttle_us = (int)constrain(
            flightMgr.takeoff_throttle_target + altControl.alt_throttle_adj,
            floor_pwm + 40, MAX_MOTOR_PWM - 80);
      } else if (flightMgr.flight_stage == STAGE_RAMP_UP && altControl.alt_hold_enabled) {
        // During ramp: once airborne, use the latched hover throttle + PID; before airborne, strictly use current_throttle_us
        if (droneState.is_airborne) {
          flightMgr.base_throttle_us = (int)constrain(
              flightMgr.takeoff_throttle_target + altControl.alt_throttle_adj,
              floor_pwm + 40, MAX_MOTOR_PWM - 80);
        } else {
          flightMgr.base_throttle_us = (int)flightMgr.current_throttle_us;
        }
      } else {
        flightMgr.base_throttle_us = (int)flightMgr.current_throttle_us;
      }

    } else {
      // Manual throttle ramp
      float ramp_up = (flightMgr.throttle_ramp_rate >= 5.0f) ? flightMgr.throttle_ramp_rate : THROTTLE_RAMP_RATE;
      float ramp_dn = flightMgr.is_emergency_landing ?
                      ((flightMgr.emerg_land_ramp_rate >= 10.0f) ? flightMgr.emerg_land_ramp_rate : EMERG_LAND_RAMP_RATE) :
                      ((flightMgr.land_ramp_rate >= 5.0f) ? flightMgr.land_ramp_rate : LAND_RAMP_RATE);
      if (flightMgr.current_throttle_us < flightMgr.target_throttle_us) {
        flightMgr.current_throttle_us += ramp_up * dt;
        if (flightMgr.current_throttle_us > flightMgr.target_throttle_us)
          flightMgr.current_throttle_us = flightMgr.target_throttle_us;
      } else if (flightMgr.current_throttle_us > flightMgr.target_throttle_us) {
        flightMgr.current_throttle_us -= ramp_dn * dt;
        if (flightMgr.current_throttle_us < flightMgr.target_throttle_us)
          flightMgr.current_throttle_us = flightMgr.target_throttle_us;
      }
      flightMgr.base_throttle_us = (int)flightMgr.current_throttle_us;
    }

    // --- 7. Closed-Loop Controller Computation & Motor Output ---
    if (flightMgr.base_throttle_us < PID_ACTIVE_THROTTLE) {
      writeMotors(flightMgr.base_throttle_us, flightMgr.base_throttle_us,
                  flightMgr.base_throttle_us, flightMgr.base_throttle_us);
      getActiveController()->reset();
    } else {
      bool run_iterm = flightMgr.sim_mode || droneState.is_airborne;

      // Desired setpoints from Position Control Loop (or manual overrides)
      float tgt_roll  = (flightMgr.current_mode == MODE_POS_HOLD ? posControl.target_roll_deg : 0.0f) + flightMgr.manual_target_roll_deg;
      float tgt_pitch = (flightMgr.current_mode == MODE_POS_HOLD ? posControl.target_pitch_deg : 0.0f) + flightMgr.manual_target_pitch_deg;
      float tgt_yaw   = 0.0f;

      getActiveController()->compute(
        flightMgr.current_mode,
        droneState.roll_deg, droneState.pitch_deg, droneState.yaw_deg,
        droneState.gx_dps, droneState.gy_dps, droneState.gz_dps,
        tgt_roll, tgt_pitch, tgt_yaw,
        flightMgr.manual_target_rate_roll_dps,
        flightMgr.manual_target_rate_pitch_dps,
        flightMgr.manual_target_rate_yaw_dps,
        dt, run_iterm,
        out_roll, out_pitch, out_yaw
      );

      applyMotorMixing(flightMgr.base_throttle_us, out_roll, out_pitch, out_yaw, flightMgr.sim_mode);
    }

    flightMgr.telemetry_out_roll  = out_roll;
    flightMgr.telemetry_out_pitch = out_pitch;
    flightMgr.telemetry_out_yaw   = out_yaw;

  } else {
    stopMotors();
    getActiveController()->reset();
    flightMgr.telemetry_out_roll  = 0.0f;
    flightMgr.telemetry_out_pitch = 0.0f;
    flightMgr.telemetry_out_yaw   = 0.0f;
  }
}
