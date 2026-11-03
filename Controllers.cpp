#include "Controllers.h"
#include "StateEstimator.h"
#include "FlightState.h"
#include <math.h>

PID_Controller   pidController;
SMC_Controller   smcController;
ADRC_Controller  adrcController;
MDBCI_Controller mdbciController(CONTROLLER_MDBCI, false, 0, "MDBCI");
MDBCI_Controller mdbcisController(CONTROLLER_MDBCIS, true, 0, "MDBCIS");
MDBCI_Controller cdiController(CONTROLLER_CDI, false, 1, "CDI");
MDBCI_Controller cviController(CONTROLLER_CVI, false, 2, "CVI");
MDBCI_Controller noiController(CONTROLLER_NOI, false, 3, "NOI");
VIABC_Controller viabcController;

static AttitudeController* activeController = &pidController;
AltitudeControllerConfig altControl;
PositionControllerConfig posControl;

// =============================================================================
//  1. PID CONTROLLER IMPLEMENTATION
// =============================================================================
PID_Controller::PID_Controller() {
  Kp_angle_roll   = 2.0f;
  Kp_angle_pitch  = 2.0f;

  Kp_rate_roll    = 0.625f;
  Ki_rate_roll    = 2.1f;
  Kd_rate_roll    = 0.0088f;

  Kp_rate_pitch   = 0.625f;
  Ki_rate_pitch   = 2.1f;
  Kd_rate_pitch   = 0.0088f;

  Kp_rate_yaw     = 2.0f;

  Kp_ang_mode_roll  = 2.0f;
  Ki_ang_mode_roll  = 0.5f;
  Kd_ang_mode_roll  = 0.007f;

  Kp_ang_mode_pitch = 2.0f;
  Ki_ang_mode_pitch = 0.5f;
  Kd_ang_mode_pitch = 0.007f;

  Kp_ang_mode_yaw   = 2.0f;

  reset();
}

void PID_Controller::init(float dt) {
  reset();
}

void PID_Controller::reset() {
  roll_rate_iterm   = 0.0f;
  pitch_rate_iterm  = 0.0f;
  yaw_rate_iterm    = 0.0f;
  roll_angle_iterm  = 0.0f;
  pitch_angle_iterm = 0.0f;
  yaw_angle_iterm   = 0.0f;
  last_gx           = 0.0f;
  last_gy           = 0.0f;
  last_gz           = 0.0f;
  d_gyro_roll_filt  = 0.0f;
  d_gyro_pitch_filt = 0.0f;
  first_run         = true;
}

void PID_Controller::compute(FlightMode mode,
                             float roll_deg, float pitch_deg, float yaw_deg,
                             float gx_dps, float gy_dps, float gz_dps,
                             float target_roll, float target_pitch, float target_yaw,
                             float target_rate_r, float target_rate_p, float target_rate_y,
                             float dt, bool run_iterm,
                             float &out_roll, float &out_pitch, float &out_yaw) {
  if (first_run) {
    last_gx = gx_dps;
    last_gy = gy_dps;
    last_gz = gz_dps;
    d_gyro_roll_filt = 0.0f;
    d_gyro_pitch_filt = 0.0f;
    first_run = false;
  }

  if (!run_iterm) {
    roll_rate_iterm   = 0.0f;
    pitch_rate_iterm  = 0.0f;
    roll_angle_iterm  = 0.0f;
    pitch_angle_iterm = 0.0f;
  }
  yaw_rate_iterm  = 0.0f;
  yaw_angle_iterm = 0.0f;

  const float MAX_RATE_ITERM = 150.0f;
  const float MAX_ANG_ITERM  = 100.0f;
  const float D_ALPHA        = 0.25f;

  if (mode == MODE_CASCADE) {
    // Roll Cascade
    float err_ang_roll = target_roll - roll_deg;
    float tgt_rate_r   = constrain(Kp_angle_roll * err_ang_roll, -MAX_TARGET_RATE, MAX_TARGET_RATE);
    float err_rate_r   = tgt_rate_r - gx_dps;

    if (run_iterm) {
      roll_rate_iterm += Ki_rate_roll * err_rate_r * dt;
      if (fabs(err_ang_roll) < 1.5f) roll_rate_iterm *= 0.96f;
      roll_rate_iterm = constrain(roll_rate_iterm, -MAX_RATE_ITERM, MAX_RATE_ITERM);
    }
    float d_raw_roll = (gx_dps - last_gx) / dt;
    d_gyro_roll_filt += D_ALPHA * (d_raw_roll - d_gyro_roll_filt);
    out_roll = (Kp_rate_roll * err_rate_r) + roll_rate_iterm - (Kd_rate_roll * d_gyro_roll_filt);

    // Pitch Cascade
    float err_ang_pitch = target_pitch - pitch_deg;
    float tgt_rate_p    = constrain(Kp_angle_pitch * err_ang_pitch, -MAX_TARGET_RATE, MAX_TARGET_RATE);
    float err_rate_p    = tgt_rate_p - gy_dps;

    if (run_iterm) {
      pitch_rate_iterm += Ki_rate_pitch * err_rate_p * dt;
      if (fabs(err_ang_pitch) < 1.5f) pitch_rate_iterm *= 0.96f;
      pitch_rate_iterm = constrain(pitch_rate_iterm, -MAX_RATE_ITERM, MAX_RATE_ITERM);
    }
    float d_raw_pitch = (gy_dps - last_gy) / dt;
    d_gyro_pitch_filt += D_ALPHA * (d_raw_pitch - d_gyro_pitch_filt);
    out_pitch = (Kp_rate_pitch * err_rate_p) + pitch_rate_iterm - (Kd_rate_pitch * d_gyro_pitch_filt);

    // Yaw Gyro Rate Damping
    float err_rate_y = target_rate_y - gz_dps;
    out_yaw = Kp_rate_yaw * err_rate_y;

  } else if (mode == MODE_ANGLE) {
    float err_ang_roll = target_roll - roll_deg;
    if (run_iterm) {
      roll_angle_iterm += Ki_ang_mode_roll * err_ang_roll * dt;
      roll_angle_iterm = constrain(roll_angle_iterm, -MAX_ANG_ITERM, MAX_ANG_ITERM);
    }
    out_roll = (Kp_ang_mode_roll * err_ang_roll) + roll_angle_iterm - (Kd_ang_mode_roll * gx_dps);

    float err_ang_pitch = target_pitch - pitch_deg;
    if (run_iterm) {
      pitch_angle_iterm += Ki_ang_mode_pitch * err_ang_pitch * dt;
      pitch_angle_iterm = constrain(pitch_angle_iterm, -MAX_ANG_ITERM, MAX_ANG_ITERM);
    }
    out_pitch = (Kp_ang_mode_pitch * err_ang_pitch) + pitch_angle_iterm - (Kd_ang_mode_pitch * gy_dps);

    float err_rate_y = target_rate_y - gz_dps;
    out_yaw = Kp_ang_mode_yaw * err_rate_y;

  } else if (mode == MODE_RATE) {
    float err_rate_r = target_rate_r - gx_dps;
    if (run_iterm) {
      roll_rate_iterm += Ki_rate_roll * err_rate_r * dt;
      roll_rate_iterm = constrain(roll_rate_iterm, -MAX_RATE_ITERM, MAX_RATE_ITERM);
    }
    float d_raw_roll = (gx_dps - last_gx) / dt;
    d_gyro_roll_filt += D_ALPHA * (d_raw_roll - d_gyro_roll_filt);
    out_roll = (Kp_rate_roll * err_rate_r) + roll_rate_iterm - (Kd_rate_roll * d_gyro_roll_filt);

    float err_rate_p = target_rate_p - gy_dps;
    if (run_iterm) {
      pitch_rate_iterm += Ki_rate_pitch * err_rate_p * dt;
      pitch_rate_iterm = constrain(pitch_rate_iterm, -MAX_RATE_ITERM, MAX_RATE_ITERM);
    }
    float d_raw_pitch = (gy_dps - last_gy) / dt;
    d_gyro_pitch_filt += D_ALPHA * (d_raw_pitch - d_gyro_pitch_filt);
    out_pitch = (Kp_rate_pitch * err_rate_p) + pitch_rate_iterm - (Kd_rate_pitch * d_gyro_pitch_filt);

    float err_rate_y = target_rate_y - gz_dps;
    out_yaw = Kp_rate_yaw * err_rate_y;
  }

  last_gx = gx_dps;
  last_gy = gy_dps;
  last_gz = gz_dps;
}

// =============================================================================
//  2. SMC CONTROLLER IMPLEMENTATION
// =============================================================================
SMC_Controller::SMC_Controller() {
  Kp_angle_roll     = 1.8f;
  Kp_angle_pitch    = 1.8f;

  Kp_smc_roll       = 0.50f;
  Ksw_smc_roll      = 28.0f;
  lambda_smc_roll   = 2.5f;
  phi_smc_roll      = 20.0f;

  Kp_smc_pitch      = 0.50f;
  Ksw_smc_pitch     = 28.0f;
  lambda_smc_pitch  = 2.5f;
  phi_smc_pitch     = 20.0f;

  Kp_smc_yaw        = 1.5f;
  Ksw_smc_yaw       = 20.0f;
  phi_smc_yaw       = 20.0f;

  Kp_ang_mode_roll  = 1.8f;
  Ki_ang_mode_roll  = 0.45f;
  Kd_ang_mode_roll  = 0.006f;

  Kp_ang_mode_pitch = 1.8f;
  Ki_ang_mode_pitch = 0.45f;
  Kd_ang_mode_pitch = 0.006f;

  Kp_ang_mode_yaw   = 1.8f;

  reset();
}

void SMC_Controller::init(float dt) {
  reset();
}

void SMC_Controller::reset() {
  roll_smc_integral  = 0.0f;
  pitch_smc_integral = 0.0f;
  yaw_smc_integral   = 0.0f;
  roll_angle_iterm   = 0.0f;
  pitch_angle_iterm  = 0.0f;
  yaw_angle_iterm    = 0.0f;
}

float SMC_Controller::sat_smc(float s, float phi) {
  if (phi <= 0.001f) return (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
  if (s > phi)  return 1.0f;
  if (s < -phi) return -1.0f;
  return s / phi;
}

void SMC_Controller::compute(FlightMode mode,
                             float roll_deg, float pitch_deg, float yaw_deg,
                             float gx_dps, float gy_dps, float gz_dps,
                             float target_roll, float target_pitch, float target_yaw,
                             float target_rate_r, float target_rate_p, float target_rate_y,
                             float dt, bool run_iterm,
                             float &out_roll, float &out_pitch, float &out_yaw) {
  const float MAX_SMC_ITERM = 35.0f;
  const float MAX_ANG_ITERM = 90.0f;

  if (!run_iterm) {
    roll_smc_integral  = 0.0f;
    pitch_smc_integral = 0.0f;
    roll_angle_iterm   = 0.0f;
    pitch_angle_iterm  = 0.0f;
  }
  yaw_smc_integral = 0.0f;

  if (mode == MODE_CASCADE) {
    // 1. Roll Cascade: Outer Angle P -> Inner Rate SMC
    float err_ang_roll = target_roll - roll_deg;
    float tgt_rate_r   = constrain(Kp_angle_roll * err_ang_roll, -220.0f, 220.0f);
    float err_rate_r   = tgt_rate_r - gx_dps;

    if (run_iterm) {
      roll_smc_integral += err_rate_r * dt;
      if (fabs(err_ang_roll) < 1.5f) roll_smc_integral *= 0.96f;
      roll_smc_integral = constrain(roll_smc_integral, -MAX_SMC_ITERM, MAX_SMC_ITERM);
    }
    float s_roll = err_rate_r + (lambda_smc_roll * roll_smc_integral);
    out_roll = (Kp_smc_roll * s_roll) + (Ksw_smc_roll * sat_smc(s_roll, phi_smc_roll));

    // 2. Pitch Cascade: Outer Angle P -> Inner Rate SMC
    float err_ang_pitch = target_pitch - pitch_deg;
    float tgt_rate_p    = constrain(Kp_angle_pitch * err_ang_pitch, -220.0f, 220.0f);
    float err_rate_p    = tgt_rate_p - gy_dps;

    if (run_iterm) {
      pitch_smc_integral += err_rate_p * dt;
      if (fabs(err_ang_pitch) < 1.5f) pitch_smc_integral *= 0.96f;
      pitch_smc_integral = constrain(pitch_smc_integral, -MAX_SMC_ITERM, MAX_SMC_ITERM);
    }
    float s_pitch = err_rate_p + (lambda_smc_pitch * pitch_smc_integral);
    out_pitch = (Kp_smc_pitch * s_pitch) + (Ksw_smc_pitch * sat_smc(s_pitch, phi_smc_pitch));

    // 3. Yaw Rate SMC Damping
    float err_rate_y = target_rate_y - gz_dps;
    float s_yaw      = err_rate_y;
    out_yaw = (Kp_smc_yaw * s_yaw) + (Ksw_smc_yaw * sat_smc(s_yaw, phi_smc_yaw));

  } else if (mode == MODE_ANGLE) {
    float err_ang_roll = target_roll - roll_deg;
    if (run_iterm) {
      roll_angle_iterm += Ki_ang_mode_roll * err_ang_roll * dt;
      roll_angle_iterm = constrain(roll_angle_iterm, -MAX_ANG_ITERM, MAX_ANG_ITERM);
    }
    out_roll = (Kp_ang_mode_roll * err_ang_roll) + roll_angle_iterm - (Kd_ang_mode_roll * gx_dps);

    float err_ang_pitch = target_pitch - pitch_deg;
    if (run_iterm) {
      pitch_angle_iterm += Ki_ang_mode_pitch * err_ang_pitch * dt;
      pitch_angle_iterm = constrain(pitch_angle_iterm, -MAX_ANG_ITERM, MAX_ANG_ITERM);
    }
    out_pitch = (Kp_ang_mode_pitch * err_ang_pitch) + pitch_angle_iterm - (Kd_ang_mode_pitch * gy_dps);

    float err_rate_y = target_rate_y - gz_dps;
    out_yaw = Kp_ang_mode_yaw * err_rate_y;

  } else if (mode == MODE_RATE) {
    float err_rate_r = target_rate_r - gx_dps;
    if (run_iterm) {
      roll_smc_integral += err_rate_r * dt;
      roll_smc_integral = constrain(roll_smc_integral, -MAX_SMC_ITERM, MAX_SMC_ITERM);
    }
    float s_roll = err_rate_r + (lambda_smc_roll * roll_smc_integral);
    out_roll = (Kp_smc_roll * s_roll) + (Ksw_smc_roll * sat_smc(s_roll, phi_smc_roll));

    float err_rate_p = target_rate_p - gy_dps;
    if (run_iterm) {
      pitch_smc_integral += err_rate_p * dt;
      pitch_smc_integral = constrain(pitch_smc_integral, -MAX_SMC_ITERM, MAX_SMC_ITERM);
    }
    float s_pitch = err_rate_p + (lambda_smc_pitch * pitch_smc_integral);
    out_pitch = (Kp_smc_pitch * s_pitch) + (Ksw_smc_pitch * sat_smc(s_pitch, phi_smc_pitch));

    float err_rate_y = target_rate_y - gz_dps;
    float s_yaw      = err_rate_y;
    out_yaw = (Kp_smc_yaw * s_yaw) + (Ksw_smc_yaw * sat_smc(s_yaw, phi_smc_yaw));
  }
}

// =============================================================================
//  3. LADRC CONTROLLER IMPLEMENTATION
// =============================================================================

// --- 1st-Order Discrete LADRC ---
DiscreteLADRC1stOrder::DiscreteLADRC1stOrder(float b0_val, float wc, float wo, float dt_val, float min_u, float max_u) {
  b0    = b0_val;
  dt    = dt_val;
  kp    = wc;
  l1    = 2.0f * wo;
  l2    = wo * wo;
  u_min = min_u;
  u_max = max_u;
  reset();
}

void DiscreteLADRC1stOrder::setGains(float wc, float wo, float b0_val) {
  kp = wc;
  l1 = 2.0f * wo;
  l2 = wo * wo;
  if (b0_val > 0.0f) b0 = b0_val;
}

float DiscreteLADRC1stOrder::update(float y_rate, float ref_rate, bool hold_disturbance) {
  float e1 = ref_rate - z[0];
  float u  = constrain((kp * e1 - z[1]) / b0, u_min, u_max);

  float e_obs = y_rate - z[0];
  float dz0   = z[1] + b0 * u + l1 * e_obs;
  float dz1   = l2 * e_obs;

  z[0] += dz0 * dt;
  if (hold_disturbance) {
    z[1] = 0.0f;
  } else {
    z[1] += dz1 * dt;
  }
  return u;
}

void DiscreteLADRC1stOrder::reset(float y0) {
  z[0] = y0;
  z[1] = 0.0f;
}

// --- 2nd-Order Discrete LADRC ---
DiscreteLADRC2ndOrder::DiscreteLADRC2ndOrder(float b0_val, float wc, float wo, float dt_val,
                                             float min_u, float max_u, float damping_ratio) {
  b0    = b0_val;
  dt    = dt_val;
  kp    = wc * wc;
  kd    = 2.0f * damping_ratio * wc;
  l1    = 3.0f * wo;
  l2    = 3.0f * wo * wo;
  l3    = wo * wo * wo;
  u_min = min_u;
  u_max = max_u;
  reset();
}

void DiscreteLADRC2ndOrder::setGains(float wc, float wo, float b0_val, float damping_ratio) {
  kp = wc * wc;
  kd = 2.0f * damping_ratio * wc;
  l1 = 3.0f * wo;
  l2 = 3.0f * wo * wo;
  l3 = wo * wo * wo;
  if (b0_val > 0.0f) b0 = b0_val;
}

float DiscreteLADRC2ndOrder::update(float y_angle, float ref_angle, float gyro_rate, bool hold_disturbance) {
  float e1 = ref_angle - z[0];
  float rate = (gyro_rate != 0.0f) ? gyro_rate : z[1];
  float u  = constrain((kp * e1 - kd * rate - z[2]) / b0, u_min, u_max);

  float e_obs = y_angle - z[0];
  float dz0   = z[1]      + l1 * e_obs;
  float dz1   = z[2] + b0 * u + l2 * e_obs;
  float dz2   = l3 * e_obs;

  z[0] += dz0 * dt;
  z[1] += dz1 * dt;
  if (hold_disturbance) {
    z[2] = 0.0f;
  } else {
    z[2] += dz2 * dt;
  }
  return u;
}

void DiscreteLADRC2ndOrder::reset(float y0) {
  z[0] = y0;
  z[1] = 0.0f;
  z[2] = 0.0f;
}

// --- ADRC Attitude Controller Wrapper ---
ADRC_Controller::ADRC_Controller()
  : ladrc_rate_phi(1.8f, 15.0f, 40.0f, 0.004f, -MAX_PID_OUT, MAX_PID_OUT),
    ladrc_rate_theta(1.8f, 15.0f, 40.0f, 0.004f, -MAX_PID_OUT, MAX_PID_OUT),
    ladrc_rate_psi(1.0f, 10.0f, 30.0f, 0.004f, -MAX_PID_OUT, MAX_PID_OUT),
    ladrc_ang_phi(1.8f, 6.0f, 20.0f, 0.004f, -MAX_PID_OUT, MAX_PID_OUT),
    ladrc_ang_theta(1.8f, 6.0f, 20.0f, 0.004f, -MAX_PID_OUT, MAX_PID_OUT),
    Kp_cascade_roll(2.0f), Kp_cascade_pitch(2.0f), Kp_rate_yaw(4.0f) {}

void ADRC_Controller::init(float dt) {
  ladrc_rate_phi.dt   = dt;
  ladrc_rate_theta.dt = dt;
  ladrc_rate_psi.dt   = dt;
  ladrc_ang_phi.dt    = dt;
  ladrc_ang_theta.dt  = dt;
  reset();
}

void ADRC_Controller::reset() {
  ladrc_rate_phi.reset();
  ladrc_rate_theta.reset();
  ladrc_rate_psi.reset();
  ladrc_ang_phi.reset();
  ladrc_ang_theta.reset();
}

void ADRC_Controller::compute(FlightMode mode,
                              float roll_deg, float pitch_deg, float yaw_deg,
                              float gx_dps, float gy_dps, float gz_dps,
                              float target_roll, float target_pitch, float target_yaw,
                              float target_rate_r, float target_rate_p, float target_rate_y,
                              float dt, bool run_iterm,
                              float &out_roll, float &out_pitch, float &out_yaw) {
  bool hold_dist = !run_iterm;

  if (mode == MODE_CASCADE) {
    float err_ang_roll  = target_roll - roll_deg;
    float tgt_rate_r    = constrain(Kp_cascade_roll * err_ang_roll, -MAX_TARGET_RATE, MAX_TARGET_RATE);
    out_roll = ladrc_rate_phi.update(gx_dps, tgt_rate_r, hold_dist);

    float err_ang_pitch = target_pitch - pitch_deg;
    float tgt_rate_p    = constrain(Kp_cascade_pitch * err_ang_pitch, -MAX_TARGET_RATE, MAX_TARGET_RATE);
    out_pitch = ladrc_rate_theta.update(gy_dps, tgt_rate_p, hold_dist);

    float err_rate_y = target_rate_y - gz_dps;
    out_yaw = Kp_rate_yaw * err_rate_y;

  } else if (mode == MODE_ANGLE) {
    out_roll  = ladrc_ang_phi.update(roll_deg, target_roll, gx_dps, hold_dist);
    out_pitch = ladrc_ang_theta.update(pitch_deg, target_pitch, gy_dps, hold_dist);

    float err_rate_y = target_rate_y - gz_dps;
    out_yaw = Kp_rate_yaw * err_rate_y;

  } else if (mode == MODE_RATE) {
    out_roll  = ladrc_rate_phi.update(gx_dps, target_rate_r, hold_dist);
    out_pitch = ladrc_rate_theta.update(gy_dps, target_rate_p, hold_dist);
    out_yaw   = ladrc_rate_psi.update(gz_dps, target_rate_y, hold_dist);
  }
}

// =============================================================================
//  4. MODIFIED DUAL BOUNDARY CONDITIONAL INTEGRAL BACKSTEPPING (MDBCI / MDBCIS)
//  Based on ISA Transactions 2020 Paper:
//    - Eq. (44): Dual Boundary Conditional Integral Gain Law lambda(t)
//    - Section 4.2: Switched backstepping for jerking & noise mitigation
// =============================================================================
float MDBCI_Axis::compute(float q1, float q2, float qd, float qv, float qa,
                          float dt, bool run_iterm, bool is_steady_state,
                          bool enable_switching, int forced_mode) {
  if (dt <= 0.0001f || dt > 0.05f) dt = 0.004f;

  // Step 1: Tracking error (Paper Eq. 8)
  float z1 = q1 - qd;
  float abs_z1 = fabs(z1);

  // Position error integral accumulation with anti-windup clamping
  if (run_iterm) {
    chi1 += z1 * dt;
    chi1 = constrain(chi1, -20.0f, 20.0f);
  } else {
    chi1 = 0.0f;
  }

  // -------------------------------------------------------------------------
  // INTEGRAL GAIN LAWS (Paper Section 4.1 & Section 6)
  // -------------------------------------------------------------------------
  if (forced_mode == 3) {
    // NOI: No Integration (lambda = 0)
    lambda1 = 0.0f;
  } else if (forced_mode == 2) {
    // CVI: Conventional Integration (constant lambda = lambda_m)
    lambda1 = run_iterm ? lambda_m : 0.0f;
  } else if (forced_mode == 1) {
    // CDI: Conditional Integration (Seshagiri & Khalil 2005)
    // lambda = lambda_m inside boundary mu, lambda = 0 outside
    if (run_iterm && (abs_z1 <= mu)) {
      lambda1 = lambda_m;
    } else {
      lambda1 = 0.0f;
    }
  } else {
    // MDBCI: Modified Dual Boundary Conditional Integration (Paper Eq. 44)
    if (!run_iterm || (abs_z1 > mu)) {
      // Region 1: |z| > mu > delta -> lambda = 0 (No windup, fast reaching)
      lambda1 = 0.0f;
    } else if (abs_z1 > delta) {
      // Region 2: mu >= |z| > delta -> lambda = lambda_r (Baseline transitional integral)
      lambda1 = lambda_r;
    } else {
      // Region 3: |z| <= delta (Precision inner boundary layer)
      if (!is_steady_state) {
        // Transient phase (t < t_ss):
        // d(lambda)/dt = sgn(z) * k_lambda * |z| + k_r * |qv| = k_lambda * z + k_r * |qv|
        float d_lambda = (k_lambda * z1) + (k_r * fabs(qv));
        lambda1 += d_lambda * dt;
      } else {
        // Steady-state phase (t >= t_ss):
        // d(lambda)/dt = k_lambda * |z| + k_r * |qv|
        float d_lambda = (k_lambda * abs_z1) + (k_r * fabs(qv));
        lambda1 += d_lambda * dt;
      }
      lambda1 = constrain(lambda1, 0.0f, lambda_m);
    }
  }

  // -------------------------------------------------------------------------
  // MDBCIS: HYSTERESIS SWITCHING FOR JERK MITIGATION (Paper Section 4.2)
  // -------------------------------------------------------------------------
  float active_c2 = c2;
  if (enable_switching) {
    if (switched_inner_mode) {
      if (abs_z1 > (delta + hz)) {
        switched_inner_mode = false; // Switch to outer aggressive controller
      }
    } else {
      if (abs_z1 <= delta) {
        switched_inner_mode = true;  // Switch to smooth low-jerk inner controller
      }
    }
    if (switched_inner_mode) {
      active_c2 = c2_switch; // Softened damping reduces motor jerking & noise
    }
  }

  // -------------------------------------------------------------------------
  // LYAPUNOV INTEGRAL BACKSTEPPING CONTROL LAW
  // -------------------------------------------------------------------------
  float term_angle    = -(1.0f + lambda1 + (c1 * active_c2)) * z1;
  float term_rate     = -(c1 + active_c2) * (q2 - qv);
  float term_integral = -(active_c2 * lambda1 * chi1);
  float term_accel    = qa;

  float tau = term_angle + term_rate + term_integral + term_accel;
  return tau;
}

// =============================================================================
//  5. VARIABLE INTEGRAL ADAPTIVE BACKSTEPPING (VIABC - IEEE 2024 Paper)
// =============================================================================
float VIABC_Axis::compute(float q1, float q2, float qd, float qv, float qa,
                          float dt, bool run_iterm, bool is_steady_state) {
  if (dt <= 0.0001f || dt > 0.05f) dt = 0.004f;
  float z1 = q1 - qd;

  if (run_iterm) {
    chi1 += z1 * dt;
    chi1 = constrain(chi1, -20.0f, 20.0f);
    if (fabs(z1) > epsilon) {
      float t_ss = is_steady_state ? 1.0f : 0.0f;
      float d_lambda = Gamma1 * (fabs(z1) + fabs(qv) * t_ss);
      lambda1 += d_lambda * dt;
      lambda1 = constrain(lambda1, 0.0f, lambda_m);
    }
  } else {
    chi1 = 0.0f;
  }

  float alpha1 = -c1 * z1 - lambda1 * chi1 + qv;
  float z2 = q2 - alpha1;

  if (run_iterm) {
    tau_D_hat += (-Gamma2 * z2) * dt;
    tau_D_hat = constrain(tau_D_hat, -40.0f, 40.0f);
  } else {
    tau_D_hat = 0.0f;
  }

  float term_angle    = -(1.0f + lambda1 + (c1 * c2)) * z1;
  float term_rate     = -(c1 + c2) * (q2 - qv);
  float term_integral = -(c2 * lambda1 * chi1);
  float term_accel    = qa;

  return (term_angle + term_rate + term_integral + term_accel + tau_D_hat);
}

// =============================================================================
//  6. BACKSTEPPING ATTITUDE CONTROLLER WRAPPERS
// =============================================================================
MDBCI_Controller::MDBCI_Controller(ControllerType type, bool switching, int mode, const char* name) {
  ctrl_type        = type;
  enable_switching = switching;
  forced_mode      = mode;
  ctrl_name        = name;
  Kp_rate_yaw      = 2.0f;
  init(0.004f);
}

void MDBCI_Controller::init(float dt) {
  roll_axis.init(1.8f, 0.65f, 4.0f, 1.2f, 0.2f, 1.5f, 0.25f, 0.1f);
  pitch_axis.init(1.8f, 0.65f, 4.0f, 1.2f, 0.2f, 1.5f, 0.25f, 0.1f);
  Kp_rate_yaw = 2.0f;
  reset();
}

void MDBCI_Controller::reset() {
  roll_axis.reset();
  pitch_axis.reset();
}

void MDBCI_Controller::compute(FlightMode mode,
                               float roll_deg, float pitch_deg, float yaw_deg,
                               float gx_dps, float gy_dps, float gz_dps,
                               float target_roll, float target_pitch, float target_yaw,
                               float target_rate_r, float target_rate_p, float target_rate_y,
                               float dt, bool run_iterm,
                               float &out_roll, float &out_pitch, float &out_yaw) {
  bool is_steady_state = (flightMgr.flight_stage == STAGE_HOVER) || flightMgr.sim_mode;

  if (mode == MODE_RATE) {
    out_roll  = roll_axis.compute(roll_deg, gx_dps, roll_deg, target_rate_r, 0.0f, dt, run_iterm, is_steady_state, enable_switching, forced_mode);
    out_pitch = pitch_axis.compute(pitch_deg, gy_dps, pitch_deg, target_rate_p, 0.0f, dt, run_iterm, is_steady_state, enable_switching, forced_mode);
  } else {
    out_roll  = roll_axis.compute(roll_deg, gx_dps, target_roll, target_rate_r, 0.0f, dt, run_iterm, is_steady_state, enable_switching, forced_mode);
    out_pitch = pitch_axis.compute(pitch_deg, gy_dps, target_pitch, target_rate_p, 0.0f, dt, run_iterm, is_steady_state, enable_switching, forced_mode);
  }

  // Yaw Gyro Rate Damping
  float err_yaw_rate = target_rate_y - gz_dps;
  out_yaw = Kp_rate_yaw * err_yaw_rate;

  out_roll  = constrain(out_roll,  -MAX_PID_OUT, MAX_PID_OUT);
  out_pitch = constrain(out_pitch, -MAX_PID_OUT, MAX_PID_OUT);
  out_yaw   = constrain(out_yaw,   -MAX_YAW_OUT, MAX_YAW_OUT);
}

// =============================================================================
//  7. VIABC ATTITUDE CONTROLLER WRAPPER
// =============================================================================
VIABC_Controller::VIABC_Controller() {
  Kp_rate_yaw = 2.0f;
  init(0.004f);
}

void VIABC_Controller::init(float dt) {
  roll_axis.init(1.8f, 0.65f, 0.2f, 0.15f, 0.05f);
  pitch_axis.init(1.8f, 0.65f, 0.2f, 0.15f, 0.05f);
  Kp_rate_yaw = 2.0f;
  reset();
}

void VIABC_Controller::reset() {
  roll_axis.reset();
  pitch_axis.reset();
}

void VIABC_Controller::compute(FlightMode mode,
                               float roll_deg, float pitch_deg, float yaw_deg,
                               float gx_dps, float gy_dps, float gz_dps,
                               float target_roll, float target_pitch, float target_yaw,
                               float target_rate_r, float target_rate_p, float target_rate_y,
                               float dt, bool run_iterm,
                               float &out_roll, float &out_pitch, float &out_yaw) {
  bool is_steady_state = (flightMgr.flight_stage == STAGE_HOVER) || flightMgr.sim_mode;

  if (mode == MODE_RATE) {
    out_roll  = roll_axis.compute(roll_deg, gx_dps, roll_deg, target_rate_r, 0.0f, dt, run_iterm, is_steady_state);
    out_pitch = pitch_axis.compute(pitch_deg, gy_dps, pitch_deg, target_rate_p, 0.0f, dt, run_iterm, is_steady_state);
  } else {
    out_roll  = roll_axis.compute(roll_deg, gx_dps, target_roll, target_rate_r, 0.0f, dt, run_iterm, is_steady_state);
    out_pitch = pitch_axis.compute(pitch_deg, gy_dps, target_pitch, target_rate_p, 0.0f, dt, run_iterm, is_steady_state);
  }

  // Yaw Gyro Rate Damping
  float err_yaw_rate = target_rate_y - gz_dps;
  out_yaw = Kp_rate_yaw * err_yaw_rate;

  out_roll  = constrain(out_roll,  -MAX_PID_OUT, MAX_PID_OUT);
  out_pitch = constrain(out_pitch, -MAX_PID_OUT, MAX_PID_OUT);
  out_yaw   = constrain(out_yaw,   -MAX_YAW_OUT, MAX_YAW_OUT);
}

// =============================================================================
//  BACKSTEPPING TUNING HELPERS
// =============================================================================
void tuneMDBCIRoll(float c1, float c2, float mu, float delta, float lr, float lm, float kl) {
  MDBCI_Controller* list[] = { &mdbciController, &mdbcisController, &cdiController, &cviController, &noiController };
  for (int i = 0; i < 5; i++) {
    list[i]->roll_axis.c1 = c1;
    list[i]->roll_axis.c2 = c2;
    if (mu > 0.0f)    list[i]->roll_axis.mu = mu;
    if (delta > 0.0f) list[i]->roll_axis.delta = delta;
    if (lr >= 0.0f)   list[i]->roll_axis.lambda_r = lr;
    if (lm > 0.0f)    list[i]->roll_axis.lambda_m = lm;
    if (kl >= 0.0f)   list[i]->roll_axis.k_lambda = kl;
  }
}

void tuneMDBCIPitch(float c1, float c2, float mu, float delta, float lr, float lm, float kl) {
  MDBCI_Controller* list[] = { &mdbciController, &mdbcisController, &cdiController, &cviController, &noiController };
  for (int i = 0; i < 5; i++) {
    list[i]->pitch_axis.c1 = c1;
    list[i]->pitch_axis.c2 = c2;
    if (mu > 0.0f)    list[i]->pitch_axis.mu = mu;
    if (delta > 0.0f) list[i]->pitch_axis.delta = delta;
    if (lr >= 0.0f)   list[i]->pitch_axis.lambda_r = lr;
    if (lm > 0.0f)    list[i]->pitch_axis.lambda_m = lm;
    if (kl >= 0.0f)   list[i]->pitch_axis.k_lambda = kl;
  }
}

void tuneMDBCIBounds(float mu, float delta) {
  MDBCI_Controller* list[] = { &mdbciController, &mdbcisController, &cdiController, &cviController, &noiController };
  for (int i = 0; i < 5; i++) {
    list[i]->roll_axis.mu = mu;
    list[i]->roll_axis.delta = delta;
    list[i]->pitch_axis.mu = mu;
    list[i]->pitch_axis.delta = delta;
  }
}

void tuneVIABCRoll(float c1, float c2, float g0, float g1, float g2) {
  viabcController.roll_axis.c1 = c1;
  viabcController.roll_axis.c2 = c2;
  if (g0 >= 0.0f) viabcController.roll_axis.Gamma0 = g0;
  if (g1 >= 0.0f) viabcController.roll_axis.Gamma1 = g1;
  if (g2 >= 0.0f) viabcController.roll_axis.Gamma2 = g2;
}

void tuneVIABCPitch(float c1, float c2, float g0, float g1, float g2) {
  viabcController.pitch_axis.c1 = c1;
  viabcController.pitch_axis.c2 = c2;
  if (g0 >= 0.0f) viabcController.pitch_axis.Gamma0 = g0;
  if (g1 >= 0.0f) viabcController.pitch_axis.Gamma1 = g1;
  if (g2 >= 0.0f) viabcController.pitch_axis.Gamma2 = g2;
}

// =============================================================================
//  CONTROLLER MANAGER
// =============================================================================
void initControllers(float dt) {
  pidController.init(dt);
  smcController.init(dt);
  adrcController.init(dt);
  mdbciController.init(dt);
  mdbcisController.init(dt);
  cdiController.init(dt);
  cviController.init(dt);
  noiController.init(dt);
  viabcController.init(dt);

  // Initialize default controller
  selectController(DEFAULT_CONTROLLER);

  // Altitude Controller Defaults (Gentle PX4 Cascade tuned for smooth, controlled ascent)
  altControl.alt_hold_enabled    = true;
  altControl.target_height_cm    = DEFAULT_TARGET_HEIGHT_CM;
  altControl.active_height_sp_cm = DEFAULT_TARGET_HEIGHT_CM;
  altControl.alt_iterm           = 0.0f;
  altControl.alt_throttle_adj    = 0.0f;
  altControl.Kp_pos_z            = 1.0f;   // Position P: 1.0m error -> 1.0 m/s desired velocity
  altControl.Kp_vel_z            = 1.8f;   // Velocity P: ~80 us per 1 m/s error
  altControl.Ki_vel_z            = 0.50f;  // Velocity Integrator: trims steady hover offset cleanly
  altControl.Kd_vel_z            = 0.04f;  // Gentle velocity damping against climb momentum
  altControl.vel_i_z             = 0.0f;
  altControl.last_vz             = 0.0f;
  altControl.max_climb_rate_mps  = 0.25f;  // Governed gentle climb speed: 25 cm/s (guarantees no aggressive ascent)
  altControl.max_desc_rate_mps   = 0.45f;  // Governed max descent speed: 45 cm/s (active braking against overshoot)
  altControl.Kp_alt              = altControl.Kp_vel_z;
  altControl.Ki_alt              = altControl.Ki_vel_z;
  altControl.Kd_alt              = altControl.Kd_vel_z;

  // Horizontal Position Controller Configuration (PX4-style cascade: Pos P -> Vel PID)
  posControl.pos_hold_enabled    = true;
  posControl.setpoint_latched    = false;
  posControl.target_pos_north_m  = 0.0f;
  posControl.target_pos_east_m   = 0.0f;
  posControl.target_roll_deg     = 0.0f;
  posControl.target_pitch_deg    = 0.0f;
  posControl.Kp_pos              = 1.2f;   // 1.0 m error -> 1.2 m/s desired return speed
  posControl.Kp_vel              = 3.5f;   // 1.0 m/s velocity error -> 3.5 deg tilt (PX4 MPC_VEL_P_ACC)
  posControl.Ki_vel              = 1.0f;   // Integrates out steady wind / CG trim offset (PX4 MPC_VEL_I_ACC)
  posControl.Kd_vel              = 0.15f;  // Velocity damping against rapid speed changes (PX4 MPC_VEL_D_ACC)
  posControl.vel_i_fwd           = 0.0f;
  posControl.vel_i_right         = 0.0f;
  posControl.max_i_tilt          = 7.0f;   // Up to 7.0 deg tilt authority for wind rejection and CG trim
  posControl.last_vel_fwd        = 0.0f;
  posControl.last_vel_right      = 0.0f;
  posControl.max_vel_mps         = 1.5f;   // 1.5 m/s horizontal return velocity cap
  posControl.max_tilt_deg        = 12.0f;  // 12.0 deg max tilt limit (matches safe outdoor capability)
  posControl.Kd_pos              = posControl.Kp_vel; // Compatibility mirror
}

void selectController(ControllerType type) {
  switch (type) {
    case CONTROLLER_PID:
      activeController = &pidController;
      break;
    case CONTROLLER_SMC:
      activeController = &smcController;
      break;
    case CONTROLLER_ADRC:
      activeController = &adrcController;
      break;
    case CONTROLLER_MDBCI:
      activeController = &mdbciController;
      break;
    case CONTROLLER_MDBCIS:
      activeController = &mdbcisController;
      break;
    case CONTROLLER_CDI:
      activeController = &cdiController;
      break;
    case CONTROLLER_CVI:
      activeController = &cviController;
      break;
    case CONTROLLER_NOI:
      activeController = &noiController;
      break;
    case CONTROLLER_VIABC:
      activeController = &viabcController;
      break;
    default:
      activeController = &pidController;
      break;
  }
  activeController->reset();
}

AttitudeController* getActiveController() {
  return activeController;
}

const char* getActiveControllerName() {
  return activeController->getName();
}

// =============================================================================
//  OUTER ALTITUDE CONTROLLER (PX4 Multicopter Position Control Architecture - Z Axis)
// =============================================================================
void resetAltitudeControl() {
  altControl.vel_i_z          = 0.0f;
  altControl.last_vz          = 0.0f;
  altControl.alt_iterm        = 0.0f;
  altControl.alt_throttle_adj = 0.0f;
}

void computeAltitudeControl(float dt, int base_throttle_us, FlightStage stage, bool is_airborne, bool sim_mode, bool is_armed) {
  if (!altControl.alt_hold_enabled || !is_armed || sim_mode) {
    resetAltitudeControl();
    return;
  }

  // Only compute altitude control when armed and actively in a flight stage
  if (base_throttle_us < MIN_ARMED_PWM + 20) {
    resetAltitudeControl();
    return;
  }

  // 1. Position Error in meters (PX4 convention: positive = drone below setpoint)
  float pos_z_m    = droneState.current_height_cm * 0.01f;
  float pos_sp_z_m = altControl.active_height_sp_cm * 0.01f;
  float err_z_m    = pos_sp_z_m - pos_z_m;

  // 2. PX4 Outer Loop: Position P -> Desired Vertical Velocity Setpoint [m/s]
  float max_climb = (altControl.max_climb_rate_mps > 0.05f) ? altControl.max_climb_rate_mps : 0.25f;
  float max_desc  = (altControl.max_desc_rate_mps > 0.05f)  ? altControl.max_desc_rate_mps  : 0.45f;
  float tgt_vel_z = constrain(altControl.Kp_pos_z * err_z_m, -max_desc, max_climb);

  // 3. PX4 Inner Loop: Velocity PID Controller (m/s -> m/s^2 desired acceleration)
  // kf_vz is the vertical velocity from the 3-state Kalman filter (m/s)
  float current_vel_z = droneState.kf_vz;
  float err_vel_z     = tgt_vel_z - current_vel_z;

  // Asymmetric Velocity Gain:
  // Upward error (err_vel_z > 0): standard gentle gain (Kp_vel_z = 1.8) -> prevents aggressive climb!
  // Downward braking (err_vel_z < 0): 2.0x higher gain -> actively stops upward surges and halts climbs!
  float kp_vel = (err_vel_z < 0.0f) ? (altControl.Kp_vel_z * 2.0f) : altControl.Kp_vel_z;
  float p_acc  = kp_vel * err_vel_z;

  // Velocity Integrator with Anti-Windup
  if (dt > 0.0001f && dt < 0.1f && is_airborne) {
    if (err_z_m < 0.0f && err_vel_z > 0.0f) {
      // Drone is above target: do not accumulate positive integrator
    } else {
      altControl.vel_i_z += altControl.Ki_vel_z * err_vel_z * dt;
    }
    // If drone is above target, quickly decay any remaining positive integrator
    if (err_z_m < -0.04f && altControl.vel_i_z > 0.0f) {
      altControl.vel_i_z -= 2.0f * dt;
      if (altControl.vel_i_z < 0.0f) altControl.vel_i_z = 0.0f;
    }
    altControl.vel_i_z = constrain(altControl.vel_i_z, -2.5f, 1.2f);
  } else {
    altControl.vel_i_z = 0.0f;
  }
  altControl.alt_iterm = altControl.vel_i_z; // Mirror for telemetry

  // Velocity D-term (Dampens vertical speed surges)
  float d_acc = 0.0f;
  if (altControl.Kd_vel_z > 0.001f && dt > 0.0001f && dt < 0.1f) {
    d_acc = altControl.Kd_vel_z * (current_vel_z - altControl.last_vz) / dt;
    d_acc = constrain(d_acc, -2.5f, 2.5f);
  }
  altControl.last_vz = current_vel_z;

  // Total Desired Vertical Acceleration [m/s^2]
  float acc_sp_z = p_acc + altControl.vel_i_z - d_acc;

  // 4. Map Acceleration [m/s^2] to Throttle PWM Adjustment [us]
  float hover_thrust_us = (float)(base_throttle_us - 1000);
  if (hover_thrust_us < 200.0f) hover_thrust_us = 400.0f;
  float acc_to_pwm = hover_thrust_us / 9.80665f; // ~40-45 us per m/s^2

  float raw_adj = acc_sp_z * acc_to_pwm;
  bool ceiling_emergency = false;

  if (!is_airborne && stage == STAGE_RAMP_UP) {
    // On ground during climb ramp: zero PID intervention; 100% deterministic spool-up via ramp rate
    raw_adj = 0.0f;
    altControl.alt_throttle_adj = 0.0f;
    altControl.vel_i_z          = 0.0f;
    return;
  } else {
    // PX4 Tilt-Thrust Compensation: Boost vertical throttle as tilt increases to maintain vertical lift
    float roll_r   = droneState.roll_deg  * DEG_TO_RAD;
    float pitch_r  = droneState.pitch_deg * DEG_TO_RAD;
    float cos_tilt = cosf(roll_r) * cosf(pitch_r);
    if (cos_tilt < 0.707f) cos_tilt = 0.707f; // Safety clamp at 45 deg tilt
    float tilt_boost = (1.0f / cos_tilt - 1.0f) * hover_thrust_us;

    // Suppress tilt boost if already higher than target to avoid runaway climb
    if (err_z_m < -0.05f) {
      tilt_boost = 0.0f;
    } else if (err_z_m < 0.0f) {
      tilt_boost *= (1.0f + err_z_m / 0.05f);
    }
    raw_adj += constrain(tilt_boost, 0.0f, 60.0f);

    // ASYMMETRIC AUTHORITY:
    // Upward authority capped at +80us -> strictly guarantees drone cannot launch aggressively!
    // Downward authority has full -260us -> ample braking power to hold height and descend!
    raw_adj = constrain(raw_adj, -260.0f, 80.0f);

    // Safety Ceiling Guard: If drone overshoots target or gets close to ToF LiDAR saturation ceiling (>180cm)
    ceiling_emergency = (droneState.current_height_cm > (altControl.target_height_cm + 25.0f) ||
                         droneState.raw_height_cm >= (MAX_SAFE_CEILING_CM - 5.0f));
    if (ceiling_emergency) {
      float ceiling_overshoot_cm = droneState.current_height_cm - altControl.target_height_cm;
      float emergency_cut = constrain(ceiling_overshoot_cm * 5.0f, 70.0f, 220.0f);
      raw_adj = min(raw_adj, -emergency_cut);
    }
  }

  // 5. Responsive Slew Rate Limiter:
  // Upward slew is gentle (+50 us/s) to prevent violent launches (~1.0 us per 20ms step).
  // Downward slew is RAPID (-400 us/s normally, -600 us/s in emergency) for immediate braking!
  float max_step_up   = 50.0f * dt;  // ~1.0 us per 20ms step (smooth, ultra-gentle ascent)
  float max_step_down = ceiling_emergency ? (600.0f * dt) : (400.0f * dt); // fast braking
  if (raw_adj > altControl.alt_throttle_adj + max_step_up)
    altControl.alt_throttle_adj += max_step_up;
  else if (raw_adj < altControl.alt_throttle_adj - max_step_down)
    altControl.alt_throttle_adj -= max_step_down;
  else
    altControl.alt_throttle_adj = raw_adj;
}

// =============================================================================
//  OUTER HORIZONTAL POSITION CONTROLLER (PX4-Style Cascaded Architecture)
// =============================================================================
void resetPositionControl() {
  posControl.setpoint_latched = false;
  posControl.vel_i_fwd        = 0.0f;
  posControl.vel_i_right      = 0.0f;
  posControl.last_vel_fwd     = 0.0f;
  posControl.last_vel_right   = 0.0f;
  posControl.target_roll_deg  = 0.0f;
  posControl.target_pitch_deg = 0.0f;
}

void computePositionControl(float dt, bool is_airborne, bool sim_mode) {
  bool has_pos_ref = droneState.flow_enabled || (droneState.gps_enabled && droneState.has_gps_home);
  bool active = posControl.pos_hold_enabled && has_pos_ref && (sim_mode || is_airborne);
  if (!active) {
    resetPositionControl();
    return;
  }

  // Safety: if GPS is active and fix is stale (>2.0s without update), hold level to avoid runaway
  if (droneState.gps_enabled && !droneState.flow_enabled && !sim_mode && (millis() - droneState.last_gps_fix_millis > 2000)) {
    posControl.target_roll_deg  = 0.0f;
    posControl.target_pitch_deg = 0.0f;
    posControl.vel_i_fwd        = 0.0f;
    posControl.vel_i_right      = 0.0f;
    return;
  }

  // Latch current position when position hold becomes active (hold current location, like PX4 POSCTL)
  if (!posControl.setpoint_latched) {
    posControl.target_pos_north_m = droneState.pos_north_m;
    posControl.target_pos_east_m  = droneState.pos_east_m;
    posControl.vel_i_fwd          = 0.0f;
    posControl.vel_i_right        = 0.0f;
    posControl.last_vel_fwd       = droneState.vel_north_filt;
    posControl.last_vel_right     = droneState.vel_east_filt;
    posControl.setpoint_latched   = true;
  }

  // 1. World NED Position Error
  float err_north = posControl.target_pos_north_m - droneState.pos_north_m;
  float err_east  = posControl.target_pos_east_m  - droneState.pos_east_m;

  // 2. Rotate World NED vectors into Drone Body Frame (Forward / Right)
  float yaw_rad = droneState.yaw_deg * DEG_TO_RAD;
  float cos_y   = cos(yaw_rad);
  float sin_y   = sin(yaw_rad);

  float err_fwd   =  err_north * cos_y + err_east * sin_y;
  float err_right = -err_north * sin_y + err_east * cos_y;

  float vel_fwd   =  droneState.vel_north_filt * cos_y + droneState.vel_east_filt * sin_y;
  float vel_right = -droneState.vel_north_filt * sin_y + droneState.vel_east_filt * cos_y;

  // 3. Outer Loop: Position P -> Desired Horizontal Velocity (with velocity clamp)
  float max_vel = (posControl.max_vel_mps > 0.1f) ? posControl.max_vel_mps : 1.5f;
  float tgt_vel_fwd   = constrain(posControl.Kp_pos * err_fwd,   -max_vel, max_vel);
  float tgt_vel_right = constrain(posControl.Kp_pos * err_right, -max_vel, max_vel);

  // 4. Inner Loop: Velocity PID Controller (PX4 Multicopter Position Control Architecture)
  float err_v_fwd   = tgt_vel_fwd   - vel_fwd;
  float err_v_right = tgt_vel_right - vel_right;

  // Velocity Integrator (rejects steady wind & aerodynamic trim offset)
  // Only accumulates when airborne or in sim mode to prevent ground windup
  if (dt > 0.0001f && dt < 0.1f && (is_airborne || sim_mode)) {
    posControl.vel_i_fwd   += posControl.Ki_vel * err_v_fwd   * dt;
    posControl.vel_i_right += posControl.Ki_vel * err_v_right * dt;

    float max_i = (posControl.max_i_tilt > 0.0f) ? posControl.max_i_tilt : 4.0f;
    posControl.vel_i_fwd   = constrain(posControl.vel_i_fwd,   -max_i, max_i);
    posControl.vel_i_right = constrain(posControl.vel_i_right, -max_i, max_i);
  } else {
    posControl.vel_i_fwd   = 0.0f;
    posControl.vel_i_right = 0.0f;
  }

  // Velocity Damping (Derivative of velocity)
  float d_term_fwd   = 0.0f;
  float d_term_right = 0.0f;
  if (dt > 0.0001f && dt < 0.1f) {
    d_term_fwd   = posControl.Kd_vel * (vel_fwd   - posControl.last_vel_fwd)   / dt;
    d_term_right = posControl.Kd_vel * (vel_right - posControl.last_vel_right) / dt;
    d_term_fwd   = constrain(d_term_fwd,   -3.0f, 3.0f);
    d_term_right = constrain(d_term_right, -3.0f, 3.0f);
  }
  posControl.last_vel_fwd   = vel_fwd;
  posControl.last_vel_right = vel_right;

  // Velocity Error -> Desired Tilt Angles (P + I - D)
  // Positive pitch = nose DOWN = forward acceleration (towards forward setpoint)
  // Positive roll  = right DOWN = rightward acceleration (towards right setpoint)
  float out_pitch = (posControl.Kp_vel * err_v_fwd)   + posControl.vel_i_fwd   - d_term_fwd;
  float out_roll  = (posControl.Kp_vel * err_v_right) + posControl.vel_i_right - d_term_right;

  // Saturated tilt angles to maintain aerodynamic authority and safe headroom
  float max_tilt = (posControl.max_tilt_deg > 1.0f) ? posControl.max_tilt_deg : 12.0f;
  posControl.target_pitch_deg = constrain(out_pitch, -max_tilt, max_tilt);
  posControl.target_roll_deg  = constrain(out_roll,  -max_tilt, max_tilt);
}
