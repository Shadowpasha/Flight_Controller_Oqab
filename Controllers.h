#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <Arduino.h>
#include "Config.h"

// =============================================================================
//  ATTITUDE CONTROLLER BASE CLASS
// =============================================================================
class AttitudeController {
public:
  virtual ~AttitudeController() {}
  virtual void init(float dt) = 0;
  virtual void reset() = 0;
  virtual void compute(FlightMode mode,
                       float roll_deg, float pitch_deg, float yaw_deg,
                       float gx_dps, float gy_dps, float gz_dps,
                       float target_roll, float target_pitch, float target_yaw,
                       float target_rate_r, float target_rate_p, float target_rate_y,
                       float dt, bool run_iterm,
                       float &out_roll, float &out_pitch, float &out_yaw) = 0;
  virtual const char* getName() const = 0;
  virtual ControllerType getType() const = 0;
};

// =============================================================================
//  1. PID CONTROLLER
// =============================================================================
class PID_Controller : public AttitudeController {
public:
  // Outer Cascade Angle Gains
  float Kp_angle_roll;
  float Kp_angle_pitch;

  // Inner Rate Gains
  float Kp_rate_roll, Ki_rate_roll, Kd_rate_roll;
  float Kp_rate_pitch, Ki_rate_pitch, Kd_rate_pitch;
  float Kp_rate_yaw;

  // Single-Loop Angle Mode Gains
  float Kp_ang_mode_roll, Ki_ang_mode_roll, Kd_ang_mode_roll;
  float Kp_ang_mode_pitch, Ki_ang_mode_pitch, Kd_ang_mode_pitch;
  float Kp_ang_mode_yaw;

  // Internal states
  float roll_rate_iterm, pitch_rate_iterm, yaw_rate_iterm;
  float roll_angle_iterm, pitch_angle_iterm, yaw_angle_iterm;
  float last_gx, last_gy, last_gz;
  float d_gyro_roll_filt, d_gyro_pitch_filt;
  bool  first_run;

  PID_Controller();
  void init(float dt) override;
  void reset() override;
  void compute(FlightMode mode,
               float roll_deg, float pitch_deg, float yaw_deg,
               float gx_dps, float gy_dps, float gz_dps,
               float target_roll, float target_pitch, float target_yaw,
               float target_rate_r, float target_rate_p, float target_rate_y,
               float dt, bool run_iterm,
               float &out_roll, float &out_pitch, float &out_yaw) override;
  const char* getName() const override { return "PID"; }
  ControllerType getType() const override { return CONTROLLER_PID; }
};

// =============================================================================
//  2. SLIDING MODE CONTROLLER (SMC)
// =============================================================================
class SMC_Controller : public AttitudeController {
public:
  // Outer Cascade Angle Gains
  float Kp_angle_roll;
  float Kp_angle_pitch;

  // Inner Rate SMC Parameters
  float Kp_smc_roll, Ksw_smc_roll, lambda_smc_roll, phi_smc_roll;
  float Kp_smc_pitch, Ksw_smc_pitch, lambda_smc_pitch, phi_smc_pitch;
  float Kp_smc_yaw, Ksw_smc_yaw, phi_smc_yaw;

  // Sliding surface integral states
  float roll_smc_integral, pitch_smc_integral, yaw_smc_integral;

  // Fallback Angle PID states
  float Kp_ang_mode_roll, Ki_ang_mode_roll, Kd_ang_mode_roll;
  float Kp_ang_mode_pitch, Ki_ang_mode_pitch, Kd_ang_mode_pitch;
  float Kp_ang_mode_yaw;
  float roll_angle_iterm, pitch_angle_iterm, yaw_angle_iterm;

  SMC_Controller();
  void init(float dt) override;
  void reset() override;
  void compute(FlightMode mode,
               float roll_deg, float pitch_deg, float yaw_deg,
               float gx_dps, float gy_dps, float gz_dps,
               float target_roll, float target_pitch, float target_yaw,
               float target_rate_r, float target_rate_p, float target_rate_y,
               float dt, bool run_iterm,
               float &out_roll, float &out_pitch, float &out_yaw) override;
  const char* getName() const override { return "SMC"; }
  ControllerType getType() const override { return CONTROLLER_SMC; }

  static float sat_smc(float s, float phi);
};

// =============================================================================
//  3. LINEAR ACTIVE DISTURBANCE REJECTION CONTROLLER (LADRC)
// =============================================================================

// 1st-Order Discrete LADRC (Rate Controller with ESO)
class DiscreteLADRC1stOrder {
public:
  float b0, kp, l1, l2, dt, u_min, u_max;
  float z[2]; // [rate_est, disturbance_est]

  DiscreteLADRC1stOrder(float b0_val, float wc, float wo, float dt_val, float min_u, float max_u);
  void setGains(float wc, float wo, float b0_val = 0.0f);
  float update(float y_rate, float ref_rate, bool hold_disturbance = false);
  void reset(float y0 = 0.0f);
};

// 2nd-Order Discrete LADRC (Angle Controller with ESO)
class DiscreteLADRC2ndOrder {
public:
  float b0, kp, kd, l1, l2, l3, dt, u_min, u_max;
  float z[3]; // [angle_est, rate_est, disturbance_est]

  DiscreteLADRC2ndOrder(float b0_val, float wc, float wo, float dt_val,
                        float min_u, float max_u, float damping_ratio = 1.2f);
  void setGains(float wc, float wo, float b0_val = 0.0f, float damping_ratio = 1.2f);
  float update(float y_angle, float ref_angle, float gyro_rate = 0.0f, bool hold_disturbance = false);
  void reset(float y0 = 0.0f);
};

class ADRC_Controller : public AttitudeController {
public:
  DiscreteLADRC1stOrder ladrc_rate_phi;
  DiscreteLADRC1stOrder ladrc_rate_theta;
  DiscreteLADRC1stOrder ladrc_rate_psi;

  DiscreteLADRC2ndOrder ladrc_ang_phi;
  DiscreteLADRC2ndOrder ladrc_ang_theta;

  float Kp_cascade_roll;
  float Kp_cascade_pitch;
  float Kp_rate_yaw;

  ADRC_Controller();
  void init(float dt) override;
  void reset() override;
  void compute(FlightMode mode,
               float roll_deg, float pitch_deg, float yaw_deg,
               float gx_dps, float gy_dps, float gz_dps,
               float target_roll, float target_pitch, float target_yaw,
               float target_rate_r, float target_rate_p, float target_rate_y,
               float dt, bool run_iterm,
               float &out_roll, float &out_pitch, float &out_yaw) override;
  const char* getName() const override { return "ADRC"; }
  ControllerType getType() const override { return CONTROLLER_ADRC; }
};

// =============================================================================
//  4. MODIFIED DUAL BOUNDARY CONDITIONAL INTEGRAL BACKSTEPPING (MDBCI / MDBCIS)
//  Based on ISA Transactions 2020 Paper:
//    - Eq. (44): Dual Boundary Conditional Integral Gain Law lambda(t)
//    - Section 4.2: Switched backstepping for jerking & noise mitigation
// =============================================================================
struct MDBCI_Axis {
  // Controller gains
  float c1;        // Stabilizing virtual control bandwidth (rad/s)
  float c2;        // Angular rate error damping gain (1/s)
  float c2_switch; // Softened rate damping inside inner boundary (MDBCIS jerk mitigation)

  // Dual boundary parameters (Paper Eq. 44)
  float mu;        // Outer boundary width (deg, e.g. 4.0 deg)
  float delta;     // Inner boundary width (deg, e.g. 1.2 deg)
  float hz;        // Switching hysteresis margin (deg, e.g. 0.3 deg)

  // Integral gain parameters (Paper Eq. 44)
  float lambda_r;  // Baseline integral gain in transition zone [delta, mu] (e.g. 0.2)
  float lambda_m;  // Maximum allowed integral gain ceiling (e.g. 1.5)
  float k_lambda;  // Integral gain adaptation rate (e.g. 0.25)
  float k_r;       // Reference velocity adaptation gain (e.g. 0.1)

  // Internal states
  float chi1;      // Position error integral: integral(z1 dt)
  float lambda1;   // Dynamic integral gain lambda(t)
  bool  switched_inner_mode; // Flag for MDBCIS hysteresis switching

  void init(float c1_val = 1.8f, float c2_val = 0.65f,
            float mu_val = 4.0f, float delta_val = 1.2f,
            float lr_val = 0.2f, float lm_val = 1.5f,
            float kl_val = 0.25f, float kr_val = 0.1f) {
    c1 = c1_val;
    c2 = c2_val;
    c2_switch = 0.45f; // Softened damping in inner zone to eliminate jerking (MDBCIS)
    mu = mu_val;
    delta = delta_val;
    hz = 0.3f;
    lambda_r = lr_val;
    lambda_m = lm_val;
    k_lambda = kl_val;
    k_r = kr_val;
    switched_inner_mode = false;
    reset();
  }

  void reset() {
    chi1 = 0.0f;
    lambda1 = lambda_r;
    switched_inner_mode = false;
  }

  float compute(float q1, float q2, float qd, float qv, float qa,
                float dt, bool run_iterm, bool is_steady_state,
                bool enable_switching = false, int forced_mode = 0);
};

// =============================================================================
//  5. VARIABLE INTEGRAL ADAPTIVE BACKSTEPPING (VIABC - IEEE Access 2024)
// =============================================================================
struct VIABC_Axis {
  float c1, c2, Gamma0, Gamma1, Gamma2, lambda_m, epsilon;
  float chi1, lambda1, tau_D_hat;

  void init(float c1_val = 1.8f, float c2_val = 0.65f,
            float g0_val = 0.2f, float g1_val = 0.15f, float g2_val = 0.05f) {
    c1 = c1_val; c2 = c2_val;
    Gamma0 = g0_val; Gamma1 = g1_val; Gamma2 = g2_val;
    lambda_m = 2.5f; epsilon = 0.05f;
    reset();
  }

  void reset() {
    chi1 = 0.0f;
    lambda1 = Gamma0;
    tau_D_hat = 0.0f;
  }

  float compute(float q1, float q2, float qd, float qv, float qa,
                float dt, bool run_iterm, bool is_steady_state);
};

// =============================================================================
//  6. BACKSTEPPING ATTITUDE CONTROLLER WRAPPERS (MDBCI, MDBCIS, CDI, CVI, NOI)
// =============================================================================
class MDBCI_Controller : public AttitudeController {
public:
  MDBCI_Axis     roll_axis;
  MDBCI_Axis     pitch_axis;
  float          Kp_rate_yaw;
  ControllerType ctrl_type;
  bool           enable_switching;
  int            forced_mode; // 0=MDBCI, 1=CDI, 2=CVI, 3=NOI
  const char*    ctrl_name;

  MDBCI_Controller(ControllerType type = CONTROLLER_MDBCI, bool switching = false, int mode = 0, const char* name = "MDBCI");
  void init(float dt) override;
  void reset() override;
  void compute(FlightMode mode,
               float roll_deg, float pitch_deg, float yaw_deg,
               float gx_dps, float gy_dps, float gz_dps,
               float target_roll, float target_pitch, float target_yaw,
               float target_rate_r, float target_rate_p, float target_rate_y,
               float dt, bool run_iterm,
               float &out_roll, float &out_pitch, float &out_yaw) override;
  const char* getName() const override { return ctrl_name; }
  ControllerType getType() const override { return ctrl_type; }
};

// =============================================================================
//  7. VIABC ATTITUDE CONTROLLER WRAPPER (IEEE Access 2024)
// =============================================================================
class VIABC_Controller : public AttitudeController {
public:
  VIABC_Axis  roll_axis;
  VIABC_Axis  pitch_axis;
  float       Kp_rate_yaw;

  VIABC_Controller();
  void init(float dt) override;
  void reset() override;
  void compute(FlightMode mode,
               float roll_deg, float pitch_deg, float yaw_deg,
               float gx_dps, float gy_dps, float gz_dps,
               float target_roll, float target_pitch, float target_yaw,
               float target_rate_r, float target_rate_p, float target_rate_y,
               float dt, bool run_iterm,
               float &out_roll, float &out_pitch, float &out_yaw) override;
  const char* getName() const override { return "VIABC"; }
  ControllerType getType() const override { return CONTROLLER_VIABC; }
};

// =============================================================================
//  CONTROLLER MANAGER & OUTER LOOPS
// =============================================================================
extern PID_Controller      pidController;
extern SMC_Controller      smcController;
extern ADRC_Controller     adrcController;
extern MDBCI_Controller    mdbciController;
extern MDBCI_Controller    mdbcisController;
extern MDBCI_Controller    cdiController;
extern MDBCI_Controller    cviController;
extern MDBCI_Controller    noiController;
extern VIABC_Controller    viabcController;

// Tuning Helpers for Backstepping Controllers
void tuneMDBCIRoll(float c1, float c2, float mu = -1, float delta = -1, float lr = -1, float lm = -1, float kl = -1);
void tuneMDBCIPitch(float c1, float c2, float mu = -1, float delta = -1, float lr = -1, float lm = -1, float kl = -1);
void tuneMDBCIBounds(float mu, float delta);
void tuneVIABCRoll(float c1, float c2, float g0 = -1, float g1 = -1, float g2 = -1);
void tuneVIABCPitch(float c1, float c2, float g0 = -1, float g1 = -1, float g2 = -1);

void initControllers(float dt);
void selectController(ControllerType type);
AttitudeController* getActiveController();
const char* getActiveControllerName();

// Outer Altitude Controller (PX4 Multicopter Position Control Architecture - Z Axis)
struct AltitudeControllerConfig {
  bool  alt_hold_enabled;
  float target_height_cm;
  float active_height_sp_cm;
  float alt_iterm;
  float alt_throttle_adj;

  // PX4-Style Cascade Gains: Position P -> Velocity PID
  float Kp_pos_z;            // Position P gain: [m] -> [m/s] (PX4 MPC_Z_P: 1.0)
  float Kp_vel_z;            // Velocity P gain: [m/s] -> [m/s^2] (PX4 MPC_Z_VEL_P_ACC: 4.0)
  float Ki_vel_z;            // Velocity I gain: [m/s*s] -> [m/s^2] (PX4 MPC_Z_VEL_I_ACC: 2.0)
  float Kd_vel_z;            // Velocity D gain (damping): [m/s^2] -> [m/s^2] (PX4 MPC_Z_VEL_D_ACC: 0.15)

  // Integrator & Damping states
  float vel_i_z;
  float last_vz;

  // Kinematic & safety limits
  float max_climb_rate_mps;  // Max upward climb speed (default 0.25 m/s for slow, controlled ascent)
  float max_desc_rate_mps;   // Max downward descent speed (default 0.20 m/s)

  // Backward compatibility aliases
  float Kp_alt;
  float Ki_alt;
  float Kd_alt;
};
extern AltitudeControllerConfig altControl;

void computeAltitudeControl(float dt, int base_throttle_us, FlightStage stage, bool is_airborne, bool sim_mode, bool is_armed);
void resetAltitudeControl();

// Outer Horizontal Position PID Controller (PX4 Multicopter Position Control Architecture)
struct PositionControllerConfig {
  bool  pos_hold_enabled;
  bool  setpoint_latched;
  float target_pos_north_m;
  float target_pos_east_m;
  float target_roll_deg;
  float target_pitch_deg;

  // PX4-Style Cascade Gains: Position P -> Velocity PID
  float Kp_pos;         // Position P gain: [m] -> [m/s] (PX4 MPC_POS_P: ~1.0)
  float Kp_vel;         // Velocity P gain: [m/s] -> [deg] (PX4 MPC_VEL_P_ACC: ~3.5)
  float Ki_vel;         // Velocity I gain: [m/s*s] -> [deg] (PX4 MPC_VEL_I_ACC: ~0.8)
  float Kd_vel;         // Velocity D gain (damping): [m/s^2] -> [deg] (PX4 MPC_VEL_D_ACC: ~0.15)

  // Integrator states (for wind rejection & trim compensation)
  float vel_i_fwd;
  float vel_i_right;
  float max_i_tilt;     // Max integrator authority (deg, default 4.0)

  // Damping states
  float last_vel_fwd;
  float last_vel_right;

  // Kinematic & safety limits
  float max_vel_mps;    // Max horizontal return speed (m/s, default 1.5)
  float max_tilt_deg;   // Max tilt angle (deg, default 12.0)

  // Backward compatibility alias for comms commands
  float Kd_pos;         // Mirrors Kp_vel
};
extern PositionControllerConfig posControl;

void computePositionControl(float dt, bool is_airborne, bool sim_mode);
void resetPositionControl();

#endif // CONTROLLERS_H
