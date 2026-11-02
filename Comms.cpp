#include "Comms.h"
#include "FlightState.h"
#include "StateEstimator.h"
#include "Controllers.h"
#include "Motors.h"
#include "Parameters.h"

BluetoothSerial SerialBT;

static void btEventCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("\n>>> [BLUETOOTH] Client device connected!");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("\n>>> [BLUETOOTH] Client device disconnected!");
  }
}

static char bt_rx_buffer[64];
static uint8_t bt_rx_index = 0;
static char ser_rx_buffer[64];
static uint8_t ser_rx_index = 0;

void initComms() {
  Serial.begin(115200);
  Serial.setTxBufferSize(512); // Expand TX ring buffer so telemetry prints never stall the 250 Hz loop

  SerialBT.register_callback(btEventCallback);
  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("FATAL: Bluetooth initialization failed!");
    while (1) delay(100);
  }
}

void dualPrint(const String& msg) {
  if (flightMgr.is_armed && !flightMgr.sim_mode) return; // Mute all printing when armed
  Serial.print(msg);
  if (SerialBT.hasClient()) SerialBT.print(msg);
}

void dualPrintln(const String& msg) {
  if (flightMgr.is_armed && !flightMgr.sim_mode) return; // Mute all printing when armed
  Serial.println(msg);
  if (SerialBT.hasClient()) SerialBT.println(msg);
}

void handleInputCommands() {
  while (SerialBT.available() > 0) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (bt_rx_index > 0) {
        bt_rx_buffer[bt_rx_index] = '\0';
        processCommand(bt_rx_buffer);
        bt_rx_index = 0;
      }
    } else if (bt_rx_index < sizeof(bt_rx_buffer) - 1) {
      bt_rx_buffer[bt_rx_index++] = c;
    }
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (ser_rx_index > 0) {
        ser_rx_buffer[ser_rx_index] = '\0';
        processCommand(ser_rx_buffer);
        ser_rx_index = 0;
      }
    } else if (ser_rx_index < sizeof(ser_rx_buffer) - 1) {
      ser_rx_buffer[ser_rx_index++] = c;
    }
  }
}

void processCommand(const char* cmd_raw) {
  String str = String(cmd_raw);
  str.trim();
  if (str.length() == 0) return;

  flightMgr.last_command_millis = millis();

  // --- Pixhawk / PX4 Parameter CLI ('param show', 'param set', 'param get', 'param reset', 'param save') ---
  if (handleParameterCommand(str)) return;

  // --- 1. CONTROLLER SELECTION (PID, SMC, ADRC, MDBCI, MDBCIS, CDI, CVI, NOI, VIABC) ---
  if (str.startsWith("ctrl ") || str.startsWith("controller ")) {
    String cName = str.substring(str.indexOf(' ') + 1);
    cName.trim();
    if (cName.equalsIgnoreCase("pid")) {
      selectController(CONTROLLER_PID);
      dualPrintln("--> Controller switched to: PID");
    } else if (cName.equalsIgnoreCase("smc")) {
      selectController(CONTROLLER_SMC);
      dualPrintln("--> Controller switched to: SMC (Sliding Mode Control)");
    } else if (cName.equalsIgnoreCase("adrc") || cName.equalsIgnoreCase("ladrc")) {
      selectController(CONTROLLER_ADRC);
      dualPrintln("--> Controller switched to: ADRC (Linear Active Disturbance Rejection)");
    } else if (cName.equalsIgnoreCase("mdbci")) {
      selectController(CONTROLLER_MDBCI);
      dualPrintln("--> Controller switched to: MDBCI (Dual Boundary Conditional Integral Backstepping - ISA 2020)");
    } else if (cName.equalsIgnoreCase("mdbcis")) {
      selectController(CONTROLLER_MDBCIS);
      dualPrintln("--> Controller switched to: MDBCIS (MDBCI + Switched Jerk Mitigation - ISA 2020)");
    } else if (cName.equalsIgnoreCase("cdi")) {
      selectController(CONTROLLER_CDI);
      dualPrintln("--> Controller switched to: CDI (Conditional Integration Backstepping - Seshagiri 2005)");
    } else if (cName.equalsIgnoreCase("cvi")) {
      selectController(CONTROLLER_CVI);
      dualPrintln("--> Controller switched to: CVI (Conventional Integral Backstepping)");
    } else if (cName.equalsIgnoreCase("noi")) {
      selectController(CONTROLLER_NOI);
      dualPrintln("--> Controller switched to: NOI (No-Integration Backstepping PD)");
    } else if (cName.equalsIgnoreCase("viabc")) {
      selectController(CONTROLLER_VIABC);
      dualPrintln("--> Controller switched to: VIABC (Adaptive Integral Backstepping - IEEE Access 2024)");
    } else if (cName.equalsIgnoreCase("bs") || cName.equalsIgnoreCase("backstepping")) {
      selectController(CONTROLLER_MDBCI);
      dualPrintln("--> Controller switched to: MDBCI Backstepping (Default)");
    } else {
      dualPrintln("ERR: Unknown controller. Options: pid, smc, adrc, mdbci, mdbcis, cdi, cvi, noi, viabc");
    }

  // --- 2. ARM & DISARM & SIM ---
  } else if (str.equalsIgnoreCase("sim")) {
    if (flightMgr.sim_mode) {
      flightMgr.sim_mode = false;
      disarm("Sim mode off");
      dualPrintln("--> SIM MODE OFF");
    } else {
      flightMgr.sim_mode            = true;
      flightMgr.is_armed            = true;
      flightMgr.flight_stage        = STAGE_IDLE;
      flightMgr.current_throttle_us = (float)(PID_ACTIVE_THROTTLE + 10);
      flightMgr.base_throttle_us    = PID_ACTIVE_THROTTLE + 10;
      flightMgr.target_throttle_us  = PID_ACTIVE_THROTTLE + 10;
      resetPositionControl();
      getActiveController()->reset();
      resetEstimatorStates();

      if (gpsSensor && gpsSensor->hasFix()) {
        droneState.gps_home_lat = gpsSensor->getLatitude();
        droneState.gps_home_lon = gpsSensor->getLongitude();
        droneState.has_gps_home = true;
      }
      dualPrintln("--> SIM MODE ON: Motors stay at IDLE, Controller & GPS loops FULLY ACTIVE");
      dualPrintln("    Active Controller: " + String(getActiveControllerName()));
    }

  } else if (str.equalsIgnoreCase("arm") || str.equalsIgnoreCase("am")) {
    if (flightMgr.is_armed) {
      disarm("Arm toggle stopped");
    } else {
      flightMgr.is_armed            = true;
      flightMgr.flight_stage        = STAGE_RAMP_UP;
      flightMgr.current_throttle_us = MIN_ARMED_PWM;
      flightMgr.base_throttle_us    = MIN_ARMED_PWM;
      flightMgr.target_throttle_us  = flightMgr.takeoff_throttle_target;
      altControl.active_height_sp_cm = droneState.ground_height_cm; // Start at ground clearance; ascends smoothly at 15 cm/s
      resetPositionControl();
      resetAltitudeControl();
      getActiveController()->reset();
      resetEstimatorStates();
      setHomeToArmPoint();

      if (droneState.has_gps_home) {
        dualPrintln("--> GPS FIX OK (Sats: " + String(gpsSensor->getSatellites()) + "). Home re-anchored to ARM point!");
      } else {
        dualPrintln("--> GPS: No solid fix yet. Home set to current physical arming position.");
      }

      dualPrintln("--> ARMED! Controller: " + String(getActiveControllerName()) +
                  " | Mode: " + String(getFlightModeName()) +
                  " | Target Height: " + String(altControl.target_height_cm, 1) + "cm");
    }

  } else if (str.equalsIgnoreCase("disarm") || str.equalsIgnoreCase("s")) {
    disarm("User command");

  } else if (str.equalsIgnoreCase("idle")) {
    flightMgr.is_armed            = true;
    flightMgr.flight_stage        = STAGE_IDLE;
    flightMgr.current_throttle_us = MIN_ARMED_PWM;
    flightMgr.base_throttle_us    = MIN_ARMED_PWM;
    flightMgr.target_throttle_us  = MIN_ARMED_PWM;
    getActiveController()->reset();
    resetEstimatorStates();
    dualPrintln("--> ARMED at IDLE " + String(MIN_ARMED_PWM) + "us (no ramping).");

  } else if (str.equalsIgnoreCase("land")) {
    if (flightMgr.is_armed && !flightMgr.sim_mode) {
      flightMgr.flight_stage         = STAGE_LANDING;
      altControl.active_height_sp_cm = droneState.current_height_cm;
      flightMgr.target_throttle_us   = flightMgr.land_throttle_target;
      dualPrintln("--> LAND command: Initiating smooth controlled descent...");
    }

  // --- 3. FLIGHT MODE (CASCADE, ANGLE, RATE) & CONTROLLER ALIASES ---
  } else if (str.startsWith("mode ") || str.startsWith("m ")) {
    String mStr = str.substring(str.indexOf(' ') + 1);
    mStr.trim();
    if (mStr.equalsIgnoreCase("cascade") || mStr == "0") {
      flightMgr.current_mode = MODE_CASCADE;
      getActiveController()->reset();
      dualPrintln("--> Flight Mode switched to: " + String(getFlightModeName()));
    } else if (mStr.equalsIgnoreCase("angle") || mStr == "1") {
      flightMgr.current_mode = MODE_ANGLE;
      getActiveController()->reset();
      dualPrintln("--> Flight Mode switched to: " + String(getFlightModeName()));
    } else if (mStr.equalsIgnoreCase("rate") || mStr == "2") {
      flightMgr.current_mode = MODE_RATE;
      getActiveController()->reset();
      dualPrintln("--> Flight Mode switched to: " + String(getFlightModeName()));
    } else if (mStr.equalsIgnoreCase("mdbci")) {
      selectController(CONTROLLER_MDBCI);
      dualPrintln("--> Controller switched to: MDBCI Backstepping");
    } else if (mStr.equalsIgnoreCase("mdbcis")) {
      selectController(CONTROLLER_MDBCIS);
      dualPrintln("--> Controller switched to: MDBCIS (Switched Jerk Mitigation)");
    } else if (mStr.equalsIgnoreCase("cdi")) {
      selectController(CONTROLLER_CDI);
      dualPrintln("--> Controller switched to: CDI Backstepping");
    } else if (mStr.equalsIgnoreCase("cvi")) {
      selectController(CONTROLLER_CVI);
      dualPrintln("--> Controller switched to: CVI Backstepping");
    } else if (mStr.equalsIgnoreCase("noi")) {
      selectController(CONTROLLER_NOI);
      dualPrintln("--> Controller switched to: NOI Backstepping");
    } else if (mStr.equalsIgnoreCase("viabc")) {
      selectController(CONTROLLER_VIABC);
      dualPrintln("--> Controller switched to: VIABC Adaptive Backstepping");
    } else if (mStr.equalsIgnoreCase("pid")) {
      selectController(CONTROLLER_PID);
      dualPrintln("--> Controller switched to: PID");
    } else {
      dualPrintln("Usage: mode <cascade|angle|rate> or ctrl <pid|smc|adrc|mdbci|mdbcis|cdi|cvi|noi|viabc>");
    }

  // --- 4. ALTITUDE & MISSION SETPOINTS ---
  } else if (str.startsWith("alt ") || str.startsWith("height ") || str.startsWith("h ")) {
    float h = str.substring(str.indexOf(' ') + 1).toFloat();
    if (h >= 15.0f && h <= 500.0f) {
      setParameter("MPC_Z_TARGET", h, true);
      dualPrintln("Target Hover Height set to: " + String(h, 1) + " cm (Saved to Flash)");
#if USE_SHARP_ONLY_ALTITUDE
      if (h > 65.0f) {
        dualPrintln("WARNING: Target height > 65cm is near Sharp IR 80cm saturation ceiling! Safe range: 35-55cm.");
      }
#else
      if (h > 80.0f) {
        dualPrintln("NOTE: Height > 0.8m active! Utilizing Barometer + Vertical Accel fusion.");
      }
#endif
    } else {
      dualPrintln("ERR: Target height must be between 15 cm and 500 cm");
    }

  } else if (str.startsWith("time ") || str.startsWith("timer ")) {
    int s = str.substring(str.indexOf(' ') + 1).toInt();
    if (s > 0) {
      setParameter("HOVER_TIME_S", (float)s, true);
      setParameter("AUTO_LAND_EN", 1.0f, true);
      dualPrintln("Hover period set to: " + String(s) + "s (Auto-land ENABLED, Saved to Flash)");
    } else {
      setParameter("AUTO_LAND_EN", 0.0f, true);
      dualPrintln("Auto-land DISABLED (Continuous hover mode, Saved to Flash)");
    }

  } else if (str.startsWith("t ") || str.startsWith("takeoff ")) {
    int val = str.substring(str.indexOf(' ') + 1).toInt();
    if (val >= 1150 && val <= MAX_MOTOR_PWM) {
      setParameter("MPC_THR_HOVER", (float)val, true);
      dualPrintln("Nominal Hover Throttle set to: " + String(val) + "us (Saved to Flash)");
    } else {
      dualPrintln("ERR: Throttle must be 1150-" + String(MAX_MOTOR_PWM));
    }

  } else if (str.startsWith("ramp ") || str.startsWith("spool ")) {
    float r = str.substring(str.indexOf(' ') + 1).toFloat();
    if (r >= 10.0f && r <= 150.0f) {
      setParameter("MPC_RAMP_UP", r, true);
      dualPrintln("Climb Ramp Rate set to: " + String(r, 1) + " us/s (Saved to Flash parameter MPC_RAMP_UP)");
    } else {
      dualPrintln("ERR: Ramp rate must be between 10.0 and 150.0 us/s (e.g. 'ramp 35')");
    }

  } else if (str.equalsIgnoreCase("+") || str.equalsIgnoreCase("u") || str.equalsIgnoreCase("up")) {
    if (flightMgr.is_armed) {
      int new_thr = min(flightMgr.takeoff_throttle_target + 20, MAX_MOTOR_PWM - 50);
      setParameter("MPC_THR_HOVER", (float)new_thr, true);
      dualPrintln("Nominal Hover Throttle: " + String(flightMgr.takeoff_throttle_target) + "us (Saved to Flash)");
    }

  } else if (str.equalsIgnoreCase("-") || str.equalsIgnoreCase("d") || str.equalsIgnoreCase("down")) {
    if (flightMgr.is_armed) {
      int new_thr = max(flightMgr.takeoff_throttle_target - 20, MIN_ARMED_PWM + 50);
      setParameter("MPC_THR_HOVER", (float)new_thr, true);
      dualPrintln("Nominal Hover Throttle: " + String(flightMgr.takeoff_throttle_target) + "us (Saved to Flash)");
    }

  // --- 5. HOLD SWITCHES & TUNING ---
  } else if (str.startsWith("pos_hold ")) {
    int val = str.substring(str.indexOf(' ') + 1).toInt();
    setParameter("MPC_POS_HOLD", (float)val, true);
    resetPositionControl();
    dualPrintln("GPS Position Hold: " + String(posControl.pos_hold_enabled ? "ENABLED" : "DISABLED") + " (Saved to Flash)");

  } else if (str.equalsIgnoreCase("rth")) {
    posControl.target_pos_north_m = 0.0f;
    posControl.target_pos_east_m  = 0.0f;
    posControl.setpoint_latched   = true;
    posControl.pos_hold_enabled   = true;
    dualPrintln("--> Return-To-Home (RTH) Target set to Boot/Arm Home (0.0m, 0.0m)");

  } else if (str.startsWith("alt_hold ")) {
    int val = str.substring(str.indexOf(' ') + 1).toInt();
    setParameter("MPC_ALT_HOLD", (float)val, true);
    dualPrintln("Altitude Hold: " + String(altControl.alt_hold_enabled ? "ENABLED" : "DISABLED") + " (Saved to Flash)");

  } else if (str.startsWith("pid_alt ")) {
    float p_pos = 0, p_vel = 0, i_vel = 0, d_vel = 0;
    int n = sscanf(str.c_str(), "pid_alt %f %f %f %f", &p_pos, &p_vel, &i_vel, &d_vel);
    if (n == 4) {
      setParameter("MPC_Z_P", p_pos, true);
      setParameter("MPC_Z_VEL_P", p_vel, true);
      setParameter("MPC_Z_VEL_I", i_vel, true);
      setParameter("MPC_Z_VEL_D", d_vel, true);
      dualPrintln("Altitude Loop updated & saved (PX4 Cascade): Kp_pos_z=" + String(altControl.Kp_pos_z, 2) +
                  " Kp_vel_z=" + String(altControl.Kp_vel_z, 2) +
                  " Ki_vel_z=" + String(altControl.Ki_vel_z, 2) +
                  " Kd_vel_z=" + String(altControl.Kd_vel_z, 3));
    } else if (n == 3) {
      setParameter("MPC_Z_VEL_P", p_pos, true);
      setParameter("MPC_Z_VEL_I", p_vel, true);
      setParameter("MPC_Z_VEL_D", i_vel, true);
      dualPrintln("Altitude Velocity PID updated & saved: Kp=" + String(altControl.Kp_vel_z, 2) +
                  " Ki=" + String(altControl.Ki_vel_z, 2) +
                  " Kd=" + String(altControl.Kd_vel_z, 3));
    } else {
      dualPrintln("Usage: pid_alt <Kp_pos_z> <Kp_vel_z> [Ki_vel_z] [Kd_vel_z]");
    }

  } else if (str.startsWith("pid_pos ")) {
    float p_pos = 0, p_vel = 0, i_vel = 0, d_vel = 0;
    int n = sscanf(str.c_str(), "pid_pos %f %f %f %f", &p_pos, &p_vel, &i_vel, &d_vel);
    if (n >= 2) {
      setParameter("MPC_XY_P", p_pos, true);
      setParameter("MPC_XY_VEL_P", p_vel, true);
      if (n >= 3) setParameter("MPC_XY_VEL_I", i_vel, true);
      if (n >= 4) setParameter("MPC_XY_VEL_D", d_vel, true);
      dualPrintln("Position Loop updated & saved (PX4 Cascade): Kp_pos=" + String(posControl.Kp_pos, 2) +
                  " Kp_vel=" + String(posControl.Kp_vel, 2) +
                  " Ki_vel=" + String(posControl.Ki_vel, 2) +
                  " Kd_vel=" + String(posControl.Kd_vel, 3));
    } else {
      dualPrintln("Usage: pid_pos <Kp_pos> <Kp_vel> [Ki_vel] [Kd_vel]");
    }

  } else if (str.startsWith("pid_r ")) {
    float ka, kp, ki, kd;
    if (sscanf(str.c_str(), "pid_r %f %f %f %f", &ka, &kp, &ki, &kd) == 4) {
      setParameter("MC_ROLL_P", ka, true);
      setParameter("MC_ROLLRATE_P", kp, true);
      setParameter("MC_ROLLRATE_I", ki, true);
      setParameter("MC_ROLLRATE_D", kd, true);
      dualPrintln("Roll PID updated & saved to Flash.");
    }

  } else if (str.startsWith("pid_p ")) {
    float ka, kp, ki, kd;
    if (sscanf(str.c_str(), "pid_p %f %f %f %f", &ka, &kp, &ki, &kd) == 4) {
      setParameter("MC_PITCH_P", ka, true);
      setParameter("MC_PITCHRATE_P", kp, true);
      setParameter("MC_PITCHRATE_I", ki, true);
      setParameter("MC_PITCHRATE_D", kd, true);
      dualPrintln("Pitch PID updated & saved to Flash.");
    }

  } else if (str.startsWith("pid_y ")) {
    float kp;
    if (sscanf(str.c_str(), "pid_y %f", &kp) == 1) {
      setParameter("MC_YAWRATE_P", kp, true);
      dualPrintln("Yaw Rate PID updated & saved to Flash: Kp=" + String(kp, 2));
    }

  // SMC Tuning Commands
  } else if (str.startsWith("smc_r ")) {
    float kp, ksw, lambda, phi;
    if (sscanf(str.c_str(), "smc_r %f %f %f %f", &kp, &ksw, &lambda, &phi) == 4) {
      smcController.Kp_smc_roll     = kp;
      smcController.Ksw_smc_roll    = ksw;
      smcController.lambda_smc_roll = lambda;
      smcController.phi_smc_roll    = phi;
      dualPrintln("Roll SMC updated: Kp=" + String(kp, 2) + " Ksw=" + String(ksw, 1) + " lambda=" + String(lambda, 1) + " phi=" + String(phi, 1));
    }

  } else if (str.startsWith("smc_p ")) {
    float kp, ksw, lambda, phi;
    if (sscanf(str.c_str(), "smc_p %f %f %f %f", &kp, &ksw, &lambda, &phi) == 4) {
      smcController.Kp_smc_pitch     = kp;
      smcController.Ksw_smc_pitch    = ksw;
      smcController.lambda_smc_pitch = lambda;
      smcController.phi_smc_pitch    = phi;
      dualPrintln("Pitch SMC updated: Kp=" + String(kp, 2) + " Ksw=" + String(ksw, 1) + " lambda=" + String(lambda, 1) + " phi=" + String(phi, 1));
    }

  } else if (str.startsWith("smc_y ")) {
    float kp, ksw, phi;
    if (sscanf(str.c_str(), "smc_y %f %f %f", &kp, &ksw, &phi) == 3) {
      smcController.Kp_smc_yaw  = kp;
      smcController.Ksw_smc_yaw = ksw;
      smcController.phi_smc_yaw = phi;
      dualPrintln("Yaw SMC updated: Kp=" + String(kp, 2) + " Ksw=" + String(ksw, 1) + " phi=" + String(phi, 1));
    }

  // ADRC Tuning Commands
  } else if (str.startsWith("adrc_r ")) {
    float wc, wo, b0;
    if (sscanf(str.c_str(), "adrc_r %f %f %f", &wc, &wo, &b0) == 3) {
      adrcController.ladrc_rate_phi.setGains(wc, wo, b0);
      dualPrintln("Roll ADRC Rate updated: wc=" + String(wc, 1) + " wo=" + String(wo, 1) + " b0=" + String(b0, 2));
    }

  } else if (str.startsWith("adrc_p ")) {
    float wc, wo, b0;
    if (sscanf(str.c_str(), "adrc_p %f %f %f", &wc, &wo, &b0) == 3) {
      adrcController.ladrc_rate_theta.setGains(wc, wo, b0);
      dualPrintln("Pitch ADRC Rate updated: wc=" + String(wc, 1) + " wo=" + String(wo, 1) + " b0=" + String(b0, 2));
    }

  // --- Backstepping Tuning Commands (ISA 2020 & IEEE 2024) ---
  } else if (str.startsWith("mdbci_r ")) {
    float c1 = 0, c2 = 0, mu = -1, delta = -1, lr = -1, lm = -1, kl = -1;
    if (sscanf(str.c_str(), "mdbci_r %f %f %f %f %f %f %f", &c1, &c2, &mu, &delta, &lr, &lm, &kl) >= 2) {
      tuneMDBCIRoll(c1, c2, mu, delta, lr, lm, kl);
      dualPrintln("Roll MDBCI updated: c1=" + String(c1, 2) + " c2=" + String(c2, 2) +
                  " mu=" + String(mdbciController.roll_axis.mu, 1) + " delta=" + String(mdbciController.roll_axis.delta, 1));
    } else {
      dualPrintln("Usage: mdbci_r <c1> <c2> [mu] [delta] [lambda_r] [lambda_m] [k_lambda]");
    }

  } else if (str.startsWith("mdbci_p ")) {
    float c1 = 0, c2 = 0, mu = -1, delta = -1, lr = -1, lm = -1, kl = -1;
    if (sscanf(str.c_str(), "mdbci_p %f %f %f %f %f %f %f", &c1, &c2, &mu, &delta, &lr, &lm, &kl) >= 2) {
      tuneMDBCIPitch(c1, c2, mu, delta, lr, lm, kl);
      dualPrintln("Pitch MDBCI updated: c1=" + String(c1, 2) + " c2=" + String(c2, 2) +
                  " mu=" + String(mdbciController.pitch_axis.mu, 1) + " delta=" + String(mdbciController.pitch_axis.delta, 1));
    } else {
      dualPrintln("Usage: mdbci_p <c1> <c2> [mu] [delta] [lambda_r] [lambda_m] [k_lambda]");
    }

  } else if (str.startsWith("mdbci_bounds ")) {
    float mu, delta;
    if (sscanf(str.c_str(), "mdbci_bounds %f %f", &mu, &delta) == 2) {
      if (mu > delta && delta > 0.0f) {
        tuneMDBCIBounds(mu, delta);
        dualPrintln("MDBCI Dual Boundaries updated: mu=" + String(mu, 2) + " deg, delta=" + String(delta, 2) + " deg");
      } else {
        dualPrintln("ERR: Must satisfy mu > delta > 0");
      }
    } else {
      dualPrintln("Usage: mdbci_bounds <mu_deg> <delta_deg>");
    }

  } else if (str.startsWith("viabc_r ")) {
    float c1 = 0, c2 = 0, g0 = -1, g1 = -1, g2 = -1;
    if (sscanf(str.c_str(), "viabc_r %f %f %f %f %f", &c1, &c2, &g0, &g1, &g2) >= 2) {
      tuneVIABCRoll(c1, c2, g0, g1, g2);
      dualPrintln("Roll VIABC updated: c1=" + String(c1, 2) + " c2=" + String(c2, 2) +
                  " Gamma0=" + String(viabcController.roll_axis.Gamma0, 2) +
                  " Gamma1=" + String(viabcController.roll_axis.Gamma1, 2) +
                  " Gamma2=" + String(viabcController.roll_axis.Gamma2, 2));
    } else {
      dualPrintln("Usage: viabc_r <c1> <c2> [Gamma0] [Gamma1] [Gamma2]");
    }

  } else if (str.startsWith("viabc_p ")) {
    float c1 = 0, c2 = 0, g0 = -1, g1 = -1, g2 = -1;
    if (sscanf(str.c_str(), "viabc_p %f %f %f %f %f", &c1, &c2, &g0, &g1, &g2) >= 2) {
      tuneVIABCPitch(c1, c2, g0, g1, g2);
      dualPrintln("Pitch VIABC updated: c1=" + String(c1, 2) + " c2=" + String(c2, 2) +
                  " Gamma0=" + String(viabcController.pitch_axis.Gamma0, 2) +
                  " Gamma1=" + String(viabcController.pitch_axis.Gamma1, 2) +
                  " Gamma2=" + String(viabcController.pitch_axis.Gamma2, 2));
    } else {
      dualPrintln("Usage: viabc_p <c1> <c2> [Gamma0] [Gamma1] [Gamma2]");
    }

  // --- 6. SENSORS & CALIBRATION ---
  } else if (str.equalsIgnoreCase("cal")) {
    if (!flightMgr.is_armed) {
      calibrateAllSensors();
    } else {
      dualPrintln("ERR: Cannot calibrate while armed!");
    }

  } else if (str.equalsIgnoreCase("cal_mag") || str.equalsIgnoreCase("cal_compass")) {
    if (!flightMgr.is_armed) {
      calibrateCompassRoutine();
    } else {
      dualPrintln("ERR: Cannot calibrate compass while armed!");
    }

  } else if (str.startsWith("decl ")) {
    droneState.mag_declination_deg = str.substring(5).toFloat();
    dualPrintln("Magnetic Declination set to: " + String(droneState.mag_declination_deg, 2) + " deg");

  } else if (str.equalsIgnoreCase("cal_esc")) {
    calibrateESCs();

  } else if (str.equalsIgnoreCase("home") || str.equalsIgnoreCase("gps_home")) {
    if (gpsSensor && gpsSensor->hasFix()) {
      droneState.gps_home_lat = gpsSensor->getLatitude();
      droneState.gps_home_lon = gpsSensor->getLongitude();
      droneState.has_gps_home = true;
      dualPrintln("--> GPS Home Reset: (" + String(droneState.gps_home_lat, 6) + ", " + String(droneState.gps_home_lon, 6) + ")");
    } else {
      dualPrintln("ERR: No GPS Fix to set Home!");
    }

  } else if (str.startsWith("gps_test ")) {
    float n = 0.0f, e = 0.0f;
    if (sscanf(str.c_str(), "gps_test %f %f", &n, &e) >= 1) {
      droneState.kf_pos_n     = n;
      droneState.kf_pos_e     = e;
      droneState.pos_north_m  = n;
      droneState.pos_east_m   = e;
      droneState.has_gps_home = true;
      dualPrintln("--> GPS Test Displacement set: North=" + String(n, 2) + "m, East=" + String(e, 2) + "m");
    } else {
      dualPrintln("Usage: gps_test <north_meters> <east_meters>");
    }

  } else if (str.equalsIgnoreCase("gps")) {
    dualPrintln("\n=== GPS DIAGNOSTIC REPORT ===");
    bool connected = gpsSensor && gpsSensor->isConnected();
    bool fix = gpsSensor && gpsSensor->hasFix();
    uint32_t chars = gpsSensor ? gpsSensor->getCharsSeen() : 0;
    uint32_t errs = gpsSensor ? gpsSensor->getChecksumErrors() : 0;
    int sats = gpsSensor ? gpsSensor->getSatellites() : 0;
    float hdop = gpsSensor ? gpsSensor->getHDOP() : 99.9f;

    dualPrintln("  • Serial Bytes : " + String(chars) + " bytes received (Errors: " + String(errs) + ")");
    dualPrintln("  • Link Status  : " + String(connected ? "ONLINE (Receiving NMEA Data)" : "NO DATA (Check Pin 16 / Power)"));
    dualPrintln("  • Fix Status   : " + String(fix ? "3D FIX LOCKED" : (connected ? "SEARCHING FOR SATELLITES" : "NO COMM")));
    dualPrintln("  • Satellites   : " + String(sats));
    dualPrintln("  • HDOP         : " + String(hdop, 2));
    if (fix) {
      dualPrintln("  • Coordinates  : Lat=" + String(gpsSensor->getLatitude(), 6) + ", Lon=" + String(gpsSensor->getLongitude(), 6));
      dualPrintln("  • Altitude     : " + String(gpsSensor->getAltitudeM(), 1) + " m");
      dualPrintln("  • Speed        : " + String(gpsSensor->getSpeedMps(), 2) + " m/s");
    } else if (connected) {
      dualPrintln("  [NOTE] Hardware is communicating properly. The GPS needs 35-90s under clear open sky to acquire lock.");
    } else {
      dualPrintln("  [ERROR] Zero bytes arriving on ESP32 RX Pin 16. Verify: GPS TX -> Pin 16, 5V & GND connected.");
    }
    dualPrintln("==============================\n");

  // --- 7. 3D POSITION & ACCURACY REPORT ---
  } else if (str.equalsIgnoreCase("pos") || str.equalsIgnoreCase("position") ||
             str.equalsIgnoreCase("accuracy") || str.equalsIgnoreCase("acc")) {
    print3DPositionReport();

  // --- 8. STATUS REPORT ---
  } else if (str.equalsIgnoreCase("status")) {
    dualPrintln("=== FLIGHT CONTROLLER STATUS ===");
    dualPrintln("Controller: " + String(getActiveControllerName()) +
                " | Mode: " + String(getFlightModeName()) +
                " | Armed: " + String(flightMgr.is_armed ? "YES" : "NO") +
                " | Stage: " + String(flightMgr.flight_stage));
    const char* zone_str = (droneState.kf_z < 0.50f) ? "LOW (<0.5m: GroundEffect Dampened)" :
                           ((droneState.kf_z <= 0.80f) ? "TRANSITION (0.5m-0.8m: Dynamic R)" : "HIGH (>0.8m: Pure Baro+Acc EKF)");
    dualPrintln("Kalman Filter: Alt=" + String(droneState.kf_z * 100.0f, 1) + "cm | Vz=" + String(droneState.kf_vz * 100.0f, 1) + "cm/s | Zone: " + String(zone_str));
    dualPrintln("Sharp IR: Raw=" + String(droneState.raw_height_cm, 1) + "cm | GroundRef=" + String(droneState.ground_height_cm, 1) + "cm | Valid=" + String(droneState.rf_valid ? "YES" : "NO"));
    if (baroSensor && baroSensor->isAvailable()) {
      dualPrintln("Baro BMP180: Pres=" + String(droneState.baro_pressure_pa / 100.0f, 2) + " hPa | RawAlt=" + String(droneState.baro_raw_alt_cm, 1) + "cm");
    } else {
      dualPrintln("Baro BMP180: OFFLINE (Sharp IR only <= 0.8m)");
    }
    dualPrintln("Altitude (PX4 Cascade): Target=" + String(altControl.target_height_cm, 1) + "cm | Vz=" + String(droneState.kf_vz, 2) + "m/s | Adj=" + String(altControl.alt_throttle_adj, 1) + "us");

    bool gps_online = gpsSensor && gpsSensor->isConnected();
    bool has_gps_fix = gps_online && gpsSensor->hasFix();
    dualPrintln("GPS Hardware: " + String(gps_online ? "ONLINE (NMEA 9600)" : "OFFLINE / DISCONNECTED") +
                " | Status: " + String(has_gps_fix ? "3D FIX" : (gps_online ? "SEARCHING FOR SATS" : "NO COMM")) +
                " | Sats=" + String(gpsSensor ? gpsSensor->getSatellites() : 0) +
                " | HDOP=" + String(gpsSensor ? gpsSensor->getHDOP() : 99.9f, 2) +
                " | Est. Acc: ±" + String(droneState.gps_accuracy_est_m, 1) + "m");
    const char* home_name = (droneState.home_type == DroneState::HOME_ARM) ? "ARM_POINT" : "BOOT_POINT";
    dualPrintln("Dead Reckoning Home: [" + String(home_name) + "] | 2D Dist: " + String(droneState.dist_from_home_2d_m, 2) + "m | 3D Dist: " + String(droneState.dist_from_home_3d_m, 2) + "m");
    dualPrintln("  • DR Pos: dN=" + String(droneState.dr_pos_x_m, 2) + "m | dE=" + String(droneState.dr_pos_y_m, 2) + "m | dAlt=" + String(droneState.dr_pos_z_m, 2) + "m (" + String(droneState.current_height_cm, 1) + "cm AGL)");
    dualPrintln("  • DR Vel: Vn=" + String(droneState.dr_vel_x_mps, 2) + "m/s | Ve=" + String(droneState.dr_vel_y_mps, 2) + "m/s | Vz=" + String(droneState.dr_vel_z_mps, 2) + "m/s (Speed: " + String(droneState.speed_3d_mps, 2) + "m/s)");
    if (droneState.has_gps_home) {
      dualPrintln("  • Raw GPS Disp: North=" + String(droneState.raw_pos_north_m, 2) + "m | East=" + String(droneState.raw_pos_east_m, 2) + "m");
    }
    dualPrintln("Compass: " + String(compassSensor ? compassSensor->getDeviceName() : "OFFLINE") + " | Heading=" + String(droneState.mag_heading_deg, 1) + " deg");
    dualPrintln("Attitude: Roll=" + String(droneState.roll_deg, 2) + " | Pitch=" + String(droneState.pitch_deg, 2) + " | Yaw=" + String(droneState.yaw_deg, 2) + " (True North)");
    dualPrintln("Throttles: Base=" + String(flightMgr.base_throttle_us) + "us | HoverRef=" + String(flightMgr.takeoff_throttle_target) + "us");
    if (getActiveController()->getType() >= CONTROLLER_MDBCI && getActiveController()->getType() <= CONTROLLER_NOI) {
      dualPrintln("Backstepping States -> Roll Lam1: " + String(mdbciController.roll_axis.lambda1, 3) +
                  " Chi: " + String(mdbciController.roll_axis.chi1, 3) +
                  " | Pitch Lam1: " + String(mdbciController.pitch_axis.lambda1, 3) +
                  " Chi: " + String(mdbciController.pitch_axis.chi1, 3));
    } else if (getActiveController()->getType() == CONTROLLER_VIABC) {
      dualPrintln("VIABC States -> Roll Lam1: " + String(viabcController.roll_axis.lambda1, 3) +
                  " Chi: " + String(viabcController.roll_axis.chi1, 3) +
                  " DistEst: " + String(viabcController.roll_axis.tau_D_hat, 2) +
                  " | Pitch Lam1: " + String(viabcController.pitch_axis.lambda1, 3) +
                  " Chi: " + String(viabcController.pitch_axis.chi1, 3) +
                  " DistEst: " + String(viabcController.pitch_axis.tau_D_hat, 2));
    }

  } else if (str.equalsIgnoreCase("ping")) {
    // Keepalive ping
  } else {
    dualPrintln("Unknown cmd: " + str);
  }
}

// =============================================================================
//  DETAILED DEAD RECKONING 3D POSITION, VELOCITY & ACCURACY REPORT
// =============================================================================
void print3DPositionReport() {
  dualPrintln("\n================= DEAD RECKONING 3D STATE & ACCURACY REPORT =================");

  // 1. Home Reference Origin Information
  const char* home_name = (droneState.home_type == DroneState::HOME_ARM) ?
                          "ARM POINT (Re-anchored at launch location)" :
                          "BOOT POINT (Initial reference set at power-on)";
  dualPrintln("[1. HOME REFERENCE ORIGIN]");
  dualPrintln("  • Home Type      : " + String(home_name));
  if (droneState.has_gps_home) {
    dualPrintln("  • Home GPS Coords: Lat=" + String(droneState.gps_home_lat, 6) +
                ", Lon=" + String(droneState.gps_home_lon, 6));
  } else {
    dualPrintln("  • Home GPS Coords: PENDING (Local inertial origin at startup)");
  }
  dualPrintln("  • Home Ground Alt: " + String(droneState.home_alt_baseline_m * 100.0f, 1) + " cm baseline");

  // 2. Dead Reckoning Distance from Home
  dualPrintln("\n[2. DEAD RECKONING DISTANCE FROM HOME]");
  dualPrintln("  • 2D Dist (Home) : " + String(droneState.dist_from_home_2d_m, 3) + " m (Horizontal Euclidean distance)");
  dualPrintln("  • 3D Dist (Home) : " + String(droneState.dist_from_home_3d_m, 3) + " m (Total spatial Euclidean distance)");

  // 3. Dead Reckoning 3D Position Displacement from Home
  dualPrintln("\n[3. DEAD RECKONING 3D POSITION (Displacement from Home)]");
  dualPrintln("  • dN (North Disp): " + String(droneState.dr_pos_x_m, 3) + " m");
  dualPrintln("  • dE (East Disp) : " + String(droneState.dr_pos_y_m, 3) + " m");
  dualPrintln("  • dAlt (Altitude): " + String(droneState.dr_pos_z_m, 3) + " m  (" +
              String(droneState.current_height_cm, 1) + " cm AGL)");

  // 4. Dead Reckoning 3D Velocity
  dualPrintln("\n[4. DEAD RECKONING 3D VELOCITY (Post-Estimator State)]");
  dualPrintln("  • Vn (North Vel) : " + String(droneState.dr_vel_x_mps, 3) + " m/s");
  dualPrintln("  • Ve (East Vel)  : " + String(droneState.dr_vel_y_mps, 3) + " m/s");
  dualPrintln("  • Vz (Climb Rate): " + String(droneState.dr_vel_z_mps, 3) + " m/s (" +
              String(droneState.climb_rate_cms, 1) + " cm/s)");
  dualPrintln("  • 3D Speed Mag   : " + String(droneState.speed_3d_mps, 3) + " m/s (Total velocity magnitude)");

  // 5. Horizontal Accuracy & GPS Sensors
  bool gps_online = gpsSensor && gpsSensor->isConnected();
  bool has_gps_fix = gps_online && gpsSensor->hasFix();
  int sats = gpsSensor ? gpsSensor->getSatellites() : 0;
  float hdop = gpsSensor ? gpsSensor->getHDOP() : 99.9f;
  dualPrintln("\n[5. HORIZONTAL ACCURACY & GPS SENSOR VERIFICATION]");
  dualPrintln("  • Hardware Status: " + String(gps_online ? "ONLINE (NMEA 9600 baud)" : "OFFLINE / DISCONNECTED (Check RX:16 TX:17)"));
  dualPrintln("  • GPS Fix Status : " + String(has_gps_fix ? "3D FIX LOCKED" : (gps_online ? "SEARCHING FOR SATELLITES" : "NO COMM")));
  dualPrintln("  • Satellites     : " + String(sats));
  dualPrintln("  • HDOP           : " + String(hdop, 2) +
              " (Estimated 1-Sigma Horiz Accuracy: ±" + String(droneState.gps_accuracy_est_m, 2) + " m)");
  if (droneState.has_gps_home) {
    if (gpsSensor && gpsSensor->isUpdated()) {
      dualPrintln("  • Current Coords : Lat=" + String(gpsSensor->getLatitude(), 6) +
                  ", Lon=" + String(gpsSensor->getLongitude(), 6));
    }
    dualPrintln("  • Raw GPS Disp   : North=" + String(droneState.raw_pos_north_m, 3) +
                " m, East=" + String(droneState.raw_pos_east_m, 3) + " m");
    float diff_n = droneState.dr_pos_x_m - droneState.raw_pos_north_m;
    float diff_e = droneState.dr_pos_y_m - droneState.raw_pos_east_m;
    dualPrintln("  • DR Fused vs Raw: dNorth=" + String(diff_n, 3) +
                " m, dEast=" + String(diff_e, 3) + " m (KF Filter Tracking Diff)");
    dualPrintln("  • Accel Bias Est : North=" + String(droneState.kf_ba_n, 3) +
                " m/s², East=" + String(droneState.kf_ba_e, 3) + " m/s²");
  } else {
    dualPrintln("  • GPS Home       : GPS Fix pending, using inertial Dead Reckoning from boot origin");
  }

  // 6. Vertical Accuracy & Altitude Sensors
#if USE_SHARP_ONLY_ALTITUDE
  const char* alt_src_str = "Pure Sharp GP2Y0A IR Rangefinder (Single-Sensor Mode, Baro Bypassed)";
#else
  const char* alt_src_str = (droneState.active_alt_source == ALT_SOURCE_SHARP) ?
                            "Sharp GP2Y0A IR Rangefinder (Low-Altitude Precision)" :
                            "BMP180 Barometer + Accel EKF (Medium/High Altitude)";
#endif
  float baro_agl_cm = (droneState.baro_raw_alt_cm - droneState.baro_ground_offset_m * 100.0f);
  if (baro_agl_cm < -500.0f || baro_agl_cm > 10000.0f) baro_agl_cm = 0.0f;
  dualPrintln("\n[6. VERTICAL ACCURACY & ALTITUDE SENSORS VERIFICATION]");
  dualPrintln("  • Active Source  : " + String(alt_src_str));
  dualPrintln("  • Fused Height Z : " + String(droneState.current_height_cm, 1) + " cm  (" +
              String(droneState.kf_z, 2) + " m)");
  dualPrintln("  • Sharp IR Raw   : " + String(droneState.raw_height_cm, 1) + " cm (Tilt-Corrected: " +
              String(droneState.z_rf_corrected_m * 100.0f, 1) + " cm, Valid: " +
              String(droneState.rf_valid ? "YES" : "NO") + ")");
  if (baroSensor && baroSensor->isAvailable()) {
    dualPrintln("  • Baro BMP180    : " + String(baro_agl_cm, 1) + " cm AGL (Pres: " +
                String(droneState.baro_pressure_pa / 100.0f, 2) + " hPa)");
  } else {
    dualPrintln("  • Baro BMP180    : OFFLINE");
  }
  dualPrintln("  • Ground Baseline: " + String(droneState.ground_height_cm, 1) + " cm baseline");
  if (droneState.rf_valid) {
    dualPrintln("  • IR vs Fused Z  : Diff=" + String(droneState.current_height_cm - droneState.raw_height_cm, 1) + " cm");
  }
  dualPrintln("  • Vert Bias Est  : Accel Bias=" + String(droneState.kf_ba, 3) + " m/s²");

  // 7. Attitude & Rates Reference
  dualPrintln("\n[7. ATTITUDE & ORIENTATION REFERENCE]");
  dualPrintln("  • Attitude       : Roll=" + String(droneState.roll_deg, 2) +
              "° | Pitch=" + String(droneState.pitch_deg, 2) +
              "° | Yaw=" + String(droneState.yaw_deg, 2) + "° (True North)");
  dualPrintln("  • Gyro Rates     : gx=" + String(droneState.gx_dps, 1) +
              " dps | gy=" + String(droneState.gy_dps, 1) +
              " dps | gz=" + String(droneState.gz_dps, 1) + " dps");
  dualPrintln("==============================================================================\n");
}

// =============================================================================
//  TELEMETRY OUTPUT (Dead Reckoning Distance from Home, 3D Velocity & Accuracy)
// =============================================================================
void sendTelemetry(bool is_armed, bool sim_mode, FlightStage stage, FlightMode mode,
                   int base_throttle_us, float out_roll, float out_pitch, float out_yaw) {
  if (is_armed && !sim_mode) return; // Mute all telemetry when armed unless in sim mode

  char t_buf[320];
  const char* alt_tag = (droneState.active_alt_source == ALT_SOURCE_SHARP) ? "SHP" : "BAR";
  const char* ctrl_tag = getActiveControllerName();
  const char* mode_tag = (mode == MODE_CASCADE) ? "CAS" : ((mode == MODE_ANGLE) ? "ANG" : "RAT");
  const char* home_tag = getHomeOriginTypeName(); // "BOOT" or "ARM"

  // Flight stage / armed label
  char state_tag[16];
  if (!is_armed && !sim_mode) {
    snprintf(state_tag, sizeof(state_tag), "DISARM");
  } else if (sim_mode) {
    snprintf(state_tag, sizeof(state_tag), "SIM");
  } else {
    const char* stg = ":CLB";
    if (stage == STAGE_HOVER)          stg = ":HOV";
    else if (stage == STAGE_LANDING)   stg = ":LND";
    else if (stage == STAGE_TOUCHDOWN) stg = ":TDN";
    else if (stage == STAGE_IDLE)      stg = ":IDL";
    snprintf(state_tag, sizeof(state_tag), "ARM%s", stg);
  }

  // GPS Fix & accuracy string
  bool is_connected = gpsSensor && gpsSensor->isConnected();
  bool has_fix = is_connected && gpsSensor->hasFix();
  int sats = gpsSensor ? gpsSensor->getSatellites() : 0;
  float hdop = gpsSensor ? gpsSensor->getHDOP() : 99.9f;

  char gps_str[36];
  if (has_fix) {
    snprintf(gps_str, sizeof(gps_str), "3D-FIX(%dS,H:%.1f,~%.1fm)", sats, hdop, droneState.gps_accuracy_est_m);
  } else if (is_connected) {
    snprintf(gps_str, sizeof(gps_str), "SEARCHING(%dS)", sats);
  } else {
    snprintf(gps_str, sizeof(gps_str), "NO-COMM(Check RX:16)");
  }

  // Barometer altitude relative to ground
  float baro_agl_cm = (droneState.baro_raw_alt_cm - droneState.baro_ground_offset_m * 100.0f);
  if (baro_agl_cm < -500.0f || baro_agl_cm > 10000.0f) baro_agl_cm = 0.0f;

  // Structured Dead Reckoning telemetry line:
  // [DEAD-RECKONING] Home:%s | Dist: 2D=%5.2fm, 3D=%5.2fm | Pos: dN=%+6.2fm, dE=%+6.2fm, dAlt=%+5.2fm(%3.0fcm) | Vel: Vn=%+5.2fm/s, Ve=%+5.2fm/s, Vz=%+5.2fm/s (Spd:%.2f) | Acc: GPS[%s] Raw:(%+.2f,%+.2f)m Alt:%s[IR:%.1f,Bar:%.1f]cm | %s [%s|%s]
  snprintf(t_buf, sizeof(t_buf),
           "[DEAD-RECKONING] Home:%s | Dist: 2D=%5.2fm, 3D=%5.2fm | Pos: dN=%+6.2fm, dE=%+6.2fm, dAlt=%+5.2fm(%3.0fcm) | Vel: Vn=%+5.2fm/s, Ve=%+5.2fm/s, Vz=%+5.2fm/s (Spd:%.2f) | Acc: GPS[%s] Raw:(%+.2f,%+.2f)m Alt:%s[IR:%.1f,Bar:%.1f]cm | %s [%s|%s]",
           home_tag,
           droneState.dist_from_home_2d_m,
           droneState.dist_from_home_3d_m,
           droneState.dr_pos_x_m,
           droneState.dr_pos_y_m,
           droneState.dr_pos_z_m,
           droneState.current_height_cm,
           droneState.dr_vel_x_mps,
           droneState.dr_vel_y_mps,
           droneState.dr_vel_z_mps,
           droneState.speed_3d_mps,
           gps_str,
           droneState.raw_pos_north_m,
           droneState.raw_pos_east_m,
           alt_tag,
           droneState.raw_height_cm,
           baro_agl_cm,
           state_tag,
           ctrl_tag,
           mode_tag);

  dualPrintln(t_buf);
}
