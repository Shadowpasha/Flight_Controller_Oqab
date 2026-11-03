// =============================================================================
//  F330 Quadcopter Modular Flight Controller (ESP32)
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

FlightManager flightMgr;

// Loop timing records
static unsigned long last_loop_micros     = 0;
static unsigned long last_alt_millis      = 0;
static unsigned long last_gps_loop_millis = 0;

// =============================================================================
//  HELPER FUNCTIONS
// =============================================================================
const char* getFlightModeName() {
  if (flightMgr.current_mode == MODE_CASCADE) return "CASCADE (Angle+Rate)";
  if (flightMgr.current_mode == MODE_ANGLE)   return "ANGLE (Single-Loop)";
  return "RATE (Acro)";
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

  dualPrintln("\n!!! DISARMED: " + String(reason));
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  // 1. Arm ESCs IMMEDIATELY at power-on with 1000us idle pulse so they never beep in alarm
  initMotors();

  // 2. Initialize Comms & Serial
  initComms();

  dualPrintln("\n--- F330 Unified Modular Flight Controller ---");
  dualPrintln("BT Device: " + String(BT_DEVICE_NAME));

  Wire.begin();
  Wire.setClock(400000);

  // Initialize Flight Manager Defaults
  flightMgr.is_armed                = false;
  flightMgr.sim_mode                = false;
  flightMgr.current_mode            = MODE_CASCADE;
  flightMgr.flight_stage            = STAGE_IDLE;
  flightMgr.base_throttle_us        = 1000;
  flightMgr.takeoff_throttle_target = 1380; // Safe initial takeoff baseline (auto-latched to actual liftoff throttle)
  flightMgr.target_throttle_us      = 1380;
  flightMgr.land_throttle_target    = 1280;
  flightMgr.current_throttle_us     = 1000.0f;
  flightMgr.throttle_ramp_rate      = THROTTLE_RAMP_RATE; // 40.0 us/s (gentle climb ramp)
  flightMgr.land_ramp_rate          = LAND_RAMP_RATE;     // 35.0 us/s (gentle landing ramp)
  flightMgr.auto_landing_enabled    = true;
  flightMgr.hover_duration_ms       = 6000; // 6s auto-hover
  flightMgr.last_command_millis     = millis();

  // Initialize State Estimator & Controllers
  initStateEstimator();
  initControllers(1.0f / LOOP_RATE_HZ);

  // Initialize Persistent Parameter System (Loads tuned values from Flash NVS)
  initParameters();

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

  last_loop_micros     = micros();
  last_alt_millis      = millis();
  last_gps_loop_millis = millis();
}

// =============================================================================
//  MAIN DETERMINISTIC LOOP (250 Hz IMU, 50 Hz Altitude, 10 Hz GPS)
// =============================================================================
void loop() {
  // Feed Flow & GPS streams non-blockingly
  if (droneState.flow_enabled && flowSensor) flowSensor->update();
  if (droneState.gps_enabled && gpsSensor) gpsSensor->update();

  // Process incoming commands from Serial and Bluetooth
  handleInputCommands();

  // Safety Watchdog: Disarm if no keepalive or command received for 30s while armed
  if (flightMgr.is_armed && (millis() - flightMgr.last_command_millis > TIMEOUT_SAFETY_MS)) {
    disarm("Safety watchdog: no command received for 30s");
  }

  // Deterministic 250 Hz timing (4000 us)
  unsigned long current_micros = micros();
  if (current_micros - last_loop_micros < LOOP_PERIOD_US) return;

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
    computeAltitudeControl(dt_alt, flightMgr.base_throttle_us, flightMgr.flight_stage,
                           droneState.is_airborne, flightMgr.sim_mode, flightMgr.is_armed);
    computePositionControl(dt_alt, droneState.is_airborne, flightMgr.sim_mode);
    updateCompassEstimator();
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
        // Latch realistic hover baseline throttle (constrained to consistent 1280 - 1420 us range)
        flightMgr.takeoff_throttle_target = constrain((int)flightMgr.current_throttle_us, 1280, 1420);
        flightMgr.target_throttle_us      = flightMgr.takeoff_throttle_target;
        altControl.active_height_sp_cm    = max(droneState.current_height_cm, droneState.ground_height_cm);
        dualPrintln(">>> LIFTOFF CONFIRMED via " + String(trigger_src) +
                    " (LiDAR: " + String(lidar_dist, 1) + "cm, H: " + String(droneState.current_height_cm, 1) +
                    "cm). Latching Hover Thrust: " + String(flightMgr.takeoff_throttle_target) + "us");
      }
    } else {
      // Confirm landed only when throttle drops AND sensors show near-ground
      bool landed_by_height = (droneState.current_height_cm <= (droneState.ground_height_cm + 4.0f));
      bool landed_by_lidar  = lidar_valid && (lidar_dist <= (droneState.ground_height_cm + 3.0f));

      if (droneState.is_airborne && (landed_by_height || landed_by_lidar || (flightMgr.base_throttle_us < MIN_ARMED_PWM + 80))) {
        droneState.is_airborne = false;
        dualPrintln(">>> LANDED (H: " + String(droneState.current_height_cm, 1) +
                    "cm, LiDAR: " + String(lidar_dist, 1) + "cm)");
      }
      droneState.above_thresh_since_ms = 0;
      lift_candidate_start_ms          = 0;
    }
  }

  // --- 5. Tilt & Ceiling Flyaway Protection ---
  if (flightMgr.is_armed && (fabs(droneState.roll_deg) > MAX_TILT_ANGLE_DEG || fabs(droneState.pitch_deg) > MAX_TILT_ANGLE_DEG)) {
    disarm("CRASH PROTECTION: Tilt > 40 deg detected!");
  }

  // Mateksys ToF LiDAR Ceiling Guard: Prevents drone from flying beyond the 185cm ceiling
  static unsigned long ceiling_violation_start_ms = 0;
  bool near_ceiling = (droneState.current_height_cm > MAX_SAFE_CEILING_CM || droneState.raw_height_cm >= 180.0f);
  if (flightMgr.is_armed && droneState.is_airborne && near_ceiling) {
    if (ceiling_violation_start_ms == 0) ceiling_violation_start_ms = millis();
    if (millis() - ceiling_violation_start_ms >= 200) {
      if (flightMgr.flight_stage != STAGE_LANDING && flightMgr.flight_stage != STAGE_TOUCHDOWN) {
        flightMgr.flight_stage            = STAGE_LANDING;
        altControl.active_height_sp_cm    = altControl.target_height_cm;
        flightMgr.takeoff_throttle_target = max(flightMgr.takeoff_throttle_target - 100, MIN_ARMED_PWM + 80);
        dualPrintln(">>> EMERGENCY CEILING INTERVENTION: Height near 180cm! Immediate descent triggered.");
      }
    }
  } else {
    ceiling_violation_start_ms = 0;
  }

  // --- 6. Mission State Machine & Throttle Calculation ---
  float out_roll = 0.0f, out_pitch = 0.0f, out_yaw = 0.0f;

  if (flightMgr.is_armed) {
    if (flightMgr.auto_landing_enabled && !flightMgr.sim_mode) {

      // ----------------------------------------------------------------
      // Stage 1: Climb Ramp — smooth ground ramp, stops ramping once airborne
      //   + altitude PID smoothly guides drone to 45cm target height
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
            // Governed climb rate (default ~18 cm/s) prevents aggressive climb
            float climb_rate_cmps = (altControl.max_climb_rate_mps > 0.05f) ?
                                    (altControl.max_climb_rate_mps * 100.0f) : 18.0f;
            if (altControl.active_height_sp_cm < altControl.target_height_cm) {
              altControl.active_height_sp_cm += climb_rate_cmps * dt;
              if (altControl.active_height_sp_cm > altControl.target_height_cm)
                altControl.active_height_sp_cm = altControl.target_height_cm;
            }
          }

          // Transition to HOVER once near target height (within 4cm) or setpoint reaches target
          bool height_reached = altControl.alt_hold_enabled &&
                                (droneState.current_height_cm >= (altControl.target_height_cm - 4.0f) ||
                                 altControl.active_height_sp_cm >= (altControl.target_height_cm - 1.0f));

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

        // Adaptive In-Flight Hover Learning (PX4 Architecture):
        // When holding steadily within 5cm of target and vertical speed < 12cm/s,
        // automatically absorb steady-state trim into takeoff_throttle_target.
        float height_err = fabs(droneState.current_height_cm - altControl.target_height_cm);
        static unsigned long hover_stable_start_ms = 0;
        static bool hover_saved_to_nvs = false;

        if (height_err < 5.0f && fabs(droneState.kf_vz) < 0.12f) {
          if (hover_stable_start_ms == 0) hover_stable_start_ms = millis();

          // Bleed 5% of PID adjustment per second into hover baseline
          float learn_step = altControl.alt_throttle_adj * 0.05f * dt;
          if (fabs(learn_step) > 0.001f) {
            flightMgr.takeoff_throttle_target = constrain(
                flightMgr.takeoff_throttle_target + (int)round(learn_step),
                1150, 1550);
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

        if (millis() - flightMgr.hover_start_millis >= flightMgr.hover_duration_ms) {
          flightMgr.flight_stage         = STAGE_LANDING;
          altControl.active_height_sp_cm = droneState.current_height_cm; // Land from current actual height
          flightMgr.target_throttle_us   = flightMgr.land_throttle_target;
          hover_saved_to_nvs             = false;
          hover_stable_start_ms          = 0;
          dualPrintln("--> Hover complete! Beginning controlled descent from " +
                      String(droneState.current_height_cm, 1) + "cm...");
        }

      // ----------------------------------------------------------------
      // Stage 3: Smooth Controlled Landing — setpoint descends at 12 cm/s
      // ----------------------------------------------------------------
      } else if (flightMgr.flight_stage == STAGE_LANDING) {
        if (altControl.alt_hold_enabled) {
          altControl.active_height_sp_cm -= 12.0f * dt; // Descend setpoint at 12 cm/s
          // Floor: stop setpoint just above ground so PID doesn't overshoot into earth
          if (altControl.active_height_sp_cm < (droneState.ground_height_cm + 2.0f))
            altControl.active_height_sp_cm = droneState.ground_height_cm + 2.0f;

          // Trigger touchdown once drone is near ground AND setpoint has descended
          if (droneState.current_height_cm <= (droneState.ground_height_cm + 4.0f) &&
              altControl.active_height_sp_cm <= (droneState.ground_height_cm + 5.0f)) {
            flightMgr.flight_stage = STAGE_TOUCHDOWN;
            dualPrintln("--> Touchdown! (" + String(droneState.current_height_cm, 1) + "cm). Cutting motors...");
          }
        } else {
          // Open-loop descent: ramp throttle down to land target
          float land_rate = (flightMgr.land_ramp_rate >= 5.0f) ? flightMgr.land_ramp_rate : LAND_RAMP_RATE;
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
        float land_rate = (flightMgr.land_ramp_rate >= 5.0f) ? flightMgr.land_ramp_rate : LAND_RAMP_RATE;
        if (flightMgr.current_throttle_us > MIN_ARMED_PWM) {
          flightMgr.current_throttle_us -= land_rate * dt * 2.0f;
          if (flightMgr.current_throttle_us < MIN_ARMED_PWM)
            flightMgr.current_throttle_us = MIN_ARMED_PWM;
        }
        if (flightMgr.current_throttle_us <= (MIN_ARMED_PWM + 5.0f))
          disarm("Auto-landing complete: Drone touched down safely.");
      }

      // ----------------------------------------------------------------
      // Base Throttle Assembly
      //   HOVER / LANDING (alt_hold): reference throttle + altitude PID adj
      //   RAMP_UP (alt_hold):         ramping throttle + altitude PID adj
      //   Everything else:            raw current_throttle
      // ----------------------------------------------------------------
      if (flightMgr.flight_stage == STAGE_HOVER ||
          (flightMgr.flight_stage == STAGE_LANDING && altControl.alt_hold_enabled)) {
        // In hover/landing: PID adj is relative to the latched hover throttle
        flightMgr.base_throttle_us = (int)constrain(
            flightMgr.takeoff_throttle_target + altControl.alt_throttle_adj,
            MIN_ARMED_PWM + 40, MAX_MOTOR_PWM - 80);
      } else if (flightMgr.flight_stage == STAGE_RAMP_UP && altControl.alt_hold_enabled) {
        // During ramp: once airborne, use the latched hover throttle + PID; before airborne, strictly use current_throttle_us
        if (droneState.is_airborne) {
          flightMgr.base_throttle_us = (int)constrain(
              flightMgr.takeoff_throttle_target + altControl.alt_throttle_adj,
              MIN_ARMED_PWM + 40, MAX_MOTOR_PWM - 80);
        } else {
          flightMgr.base_throttle_us = (int)flightMgr.current_throttle_us;
        }
      } else {
        flightMgr.base_throttle_us = (int)flightMgr.current_throttle_us;
      }

    } else {
      // Manual throttle ramp
      float ramp_up = (flightMgr.throttle_ramp_rate >= 5.0f) ? flightMgr.throttle_ramp_rate : THROTTLE_RAMP_RATE;
      float ramp_dn = (flightMgr.land_ramp_rate >= 5.0f) ? flightMgr.land_ramp_rate : LAND_RAMP_RATE;
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
      float tgt_roll  = posControl.target_roll_deg  + flightMgr.manual_target_roll_deg;
      float tgt_pitch = posControl.target_pitch_deg + flightMgr.manual_target_pitch_deg;
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

  } else {
    stopMotors();
    getActiveController()->reset();
  }

  // --- 8. Telemetry Transmission: Completely disabled when armed (zero communication delay) ---
  if (!flightMgr.is_armed || flightMgr.sim_mode) {
    static uint8_t telemetry_counter = 0;
    uint8_t telemetry_interval = flightMgr.sim_mode ? 25 : 62;
    if (++telemetry_counter >= telemetry_interval) {
      telemetry_counter = 0;
      sendTelemetry(flightMgr.is_armed, flightMgr.sim_mode, flightMgr.flight_stage,
                    flightMgr.current_mode, flightMgr.base_throttle_us,
                    out_roll, out_pitch, out_yaw);
    }
  }
}
