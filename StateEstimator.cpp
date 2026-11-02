#include "StateEstimator.h"
#include <math.h>

void dualPrintln(const String& msg);
void dualPrint(const String& msg);

DroneState droneState;

// --- Complementary Filter Constants ---
static const float ALPHA_ATT = 0.991f;
static const float ALPHA_YAW = 0.995f;

// --- Vertical Kalman Filter Covariance & Noise ---
static float P_00 = 0.05f, P_01 = 0.0f, P_02 = 0.0f;
static float P_10 = 0.0f,  P_11 = 0.05f, P_12 = 0.0f;
static float P_20 = 0.0f,  P_21 = 0.0f, P_22 = 0.02f;

static const float Q_z  = 0.0002f;
static const float Q_vz = 0.0035f;
static const float Q_ba = 0.00003f;

static const float BARO_OFFSET_ALPHA = 0.015f;

// =============================================================================
//  3-STATE 1D KALMAN FILTER FOR HORIZONTAL AXES (NORTH & EAST)
// =============================================================================
struct KalmanFilter1D {
  float pos;
  float vel;
  float ba;
  float P[3][3];
  float q_pos, q_vel, q_ba;

  void init(float q_p, float q_v, float q_b) {
    q_pos = q_p; q_vel = q_v; q_ba = q_b;
    reset();
  }

  void reset(float p0 = 0.0f) {
    pos = p0;
    vel = 0.0f;
    ba  = 0.0f;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        P[i][j] = 0.0f;
    P[0][0] = 0.25f;
    P[1][1] = 0.10f;
    P[2][2] = 0.05f;
  }

  void predict(float a_meas, float dt) {
    if (dt <= 0.0001f || dt > 0.05f) dt = 0.02f;
    float a_corr = a_meas - ba;

    // Sensor noise floor suppression (below 0.015 m/s^2)
    if (fabs(a_corr) < 0.015f) a_corr = 0.0f;

    pos += (vel * dt) + (0.5f * a_corr * dt * dt);
    vel += a_corr * dt;
    vel = constrain(vel, -6.0f, 6.0f);

    float f01 = dt;
    float f02 = -0.5f * dt * dt;
    float f12 = -dt;

    float M00 = P[0][0] + f01 * P[1][0] + f02 * P[2][0];
    float M01 = P[0][1] + f01 * P[1][1] + f02 * P[2][1];
    float M02 = P[0][2] + f01 * P[1][2] + f02 * P[2][2];

    float M10 = P[1][0] + f12 * P[2][0];
    float M11 = P[1][1] + f12 * P[2][1];
    float M12 = P[1][2] + f12 * P[2][2];

    float M20 = P[2][0];
    float M21 = P[2][1];
    float M22 = P[2][2];

    P[0][0] = M00 + M01 * f01 + M02 * f02 + q_pos;
    P[0][1] = M01 + M02 * f12;
    P[0][2] = M02;

    P[1][0] = P[0][1];
    P[1][1] = M11 + M12 * f12 + q_vel;
    P[1][2] = M12;

    P[2][0] = P[0][2];
    P[2][1] = P[1][2];
    P[2][2] = M22 + q_ba;
  }

  void updatePos(float pos_meas, float R) {
    float y = pos_meas - pos;
    float S = P[0][0] + R;
    if (S < 1e-6f) return;

    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float K2 = P[2][0] / S;

    pos += K0 * y;
    vel += K1 * y;
    ba  += K2 * y;
    ba = constrain(ba, -2.5f, 2.5f);

    float p00 = P[0][0], p01 = P[0][1], p02 = P[0][2];

    P[0][0] -= K0 * p00;
    P[0][1] -= K0 * p01;
    P[0][2] -= K0 * p02;

    P[1][0] -= K1 * p00;
    P[1][1] -= K1 * p01;
    P[1][2] -= K1 * p02;

    P[2][0] -= K2 * p00;
    P[2][1] -= K2 * p01;
    P[2][2] -= K2 * p02;

    P[1][0] = P[0][1];
    P[2][0] = P[0][2];
    P[2][1] = P[1][2];
  }

  void updateVel(float vel_meas, float R) {
    float y = vel_meas - vel;
    float S = P[1][1] + R;
    if (S < 1e-6f) return;

    float K0 = P[0][1] / S;
    float K1 = P[1][1] / S;
    float K2 = P[2][1] / S;

    pos += K0 * y;
    vel += K1 * y;
    ba  += K2 * y;
    ba = constrain(ba, -2.5f, 2.5f);

    float p10 = P[1][0], p11 = P[1][1], p12 = P[1][2];

    P[0][0] -= K0 * p10;
    P[0][1] -= K0 * p11;
    P[0][2] -= K0 * p12;

    P[1][0] -= K1 * p10;
    P[1][1] -= K1 * p11;
    P[1][2] -= K1 * p12;

    P[2][0] -= K2 * p10;
    P[2][1] -= K2 * p11;
    P[2][2] -= K2 * p12;

    P[1][0] = P[0][1];
    P[2][0] = P[0][2];
    P[2][1] = P[1][2];
  }
};

static KalmanFilter1D kfNorth;
static KalmanFilter1D kfEast;

// =============================================================================
//  INITIALIZATION & RESET
// =============================================================================
void initStateEstimator() {
  memset(&droneState, 0, sizeof(DroneState));

  droneState.ground_height_cm    = 10.0f;
  droneState.current_height_cm   = 10.0f;
  droneState.kf_z                = 0.10f;
  droneState.az_filt             = 9.81f;
  droneState.active_alt_source   = ALT_SOURCE_SHARP;

  // Horizontal Kalman filter tuning: balanced for MPU6050 and NEO-M8N GPS
  kfNorth.init(0.005f, 0.035f, 0.0005f);
  kfEast.init(0.005f, 0.035f, 0.0005f);

  // Default calibrated compass hard-iron offsets from bench test
  droneState.mag_offset_x        = 53.00f;
  droneState.mag_offset_y        = -1179.00f;
  droneState.mag_offset_z        = 42.50f;
  droneState.mag_declination_deg = 0.0f;

  resetEstimatorStates();
  setHomeToBootPoint();
}

void resetEstimatorStates() {
  droneState.kf_z                 = droneState.ground_height_cm * 0.01f;
  droneState.kf_vz                = 0.0f;
  droneState.kf_ba                = 0.0f;
  P_00 = 0.05f; P_01 = 0.0f; P_02 = 0.0f;
  P_10 = 0.0f;  P_11 = 0.05f; P_12 = 0.0f;
  P_20 = 0.0f;  P_21 = 0.0f; P_22 = 0.02f;

  kfNorth.reset(0.0f);
  kfEast.reset(0.0f);
  droneState.kf_pos_n             = 0.0f;
  droneState.kf_vel_n             = 0.0f;
  droneState.kf_ba_n              = 0.0f;
  droneState.kf_pos_e             = 0.0f;
  droneState.kf_vel_e             = 0.0f;
  droneState.kf_ba_e              = 0.0f;

  droneState.climb_rate_cms       = 0.0f;
  droneState.climb_rate_filt      = 0.0f;
  droneState.current_height_cm    = droneState.ground_height_cm;
  droneState.pos_north_m          = 0.0f;
  droneState.pos_east_m           = 0.0f;
  droneState.dr_pos_x_m           = 0.0f;
  droneState.dr_pos_y_m           = 0.0f;
  droneState.dr_pos_z_m           = 0.0f;
  droneState.dr_vel_x_mps         = 0.0f;
  droneState.dr_vel_y_mps         = 0.0f;
  droneState.dr_vel_z_mps         = 0.0f;
  droneState.raw_pos_north_m      = 0.0f;
  droneState.raw_pos_east_m       = 0.0f;
  droneState.dist_from_home_2d_m  = 0.0f;
  droneState.dist_from_home_3d_m  = 0.0f;
  droneState.dist_2d_m            = 0.0f;
  droneState.dist_3d_m            = 0.0f;
  droneState.speed_3d_mps         = 0.0f;
  droneState.gps_accuracy_est_m   = 99.9f;
  droneState.vel_north_filt       = 0.0f;
  droneState.vel_east_filt        = 0.0f;
  droneState.is_airborne          = false;
  droneState.above_thresh_since_ms = 0;
  droneState.az_filt              = 9.81f;
}

// =============================================================================
//  HOME REFERENCE & DEAD RECKONING ORIGIN MANAGEMENT
// =============================================================================
void setHomeToBootPoint() {
  droneState.home_type           = DroneState::HOME_BOOT;
  droneState.home_is_set         = true;
  bool good_fix                  = (gpsSensor && gpsSensor->hasFix() && 
                                   ((gpsSensor->getSatellites() >= 4 && gpsSensor->getHDOP() <= 4.5f) || 
                                    (gpsSensor->getSatellites() == 0 && gpsSensor->hasFix())));
  droneState.has_gps_home        = good_fix;
  if (droneState.has_gps_home) {
    droneState.gps_home_lat      = gpsSensor->getLatitude();
    droneState.gps_home_lon      = gpsSensor->getLongitude();
  } else {
    droneState.gps_home_lat      = 0.0;
    droneState.gps_home_lon      = 0.0;
  }
  droneState.home_alt_baseline_m = droneState.ground_height_cm * 0.01f;

  kfNorth.reset(0.0f);
  kfEast.reset(0.0f);
  droneState.dr_pos_x_m          = 0.0f;
  droneState.dr_pos_y_m          = 0.0f;
  droneState.dr_pos_z_m          = 0.0f;
  droneState.dr_vel_x_mps        = 0.0f;
  droneState.dr_vel_y_mps        = 0.0f;
  droneState.dr_vel_z_mps        = 0.0f;
  droneState.pos_north_m         = 0.0f;
  droneState.pos_east_m          = 0.0f;
  droneState.vel_north_filt      = 0.0f;
  droneState.vel_east_filt       = 0.0f;
  droneState.raw_pos_north_m     = 0.0f;
  droneState.raw_pos_east_m      = 0.0f;
  droneState.dist_from_home_2d_m = 0.0f;
  droneState.dist_from_home_3d_m = 0.0f;
  droneState.dist_2d_m           = 0.0f;
  droneState.dist_3d_m           = 0.0f;
  droneState.speed_3d_mps        = 0.0f;
  dualPrintln(">>> HOME INITIALIZED TO BOOT POINT (Dead Reckoning Origin at power-on)");
}

void setHomeToArmPoint() {
  droneState.home_type           = DroneState::HOME_ARM;
  droneState.home_is_set         = true;
  if (gpsSensor && gpsSensor->hasFix()) {
    droneState.gps_home_lat      = gpsSensor->getLatitude();
    droneState.gps_home_lon      = gpsSensor->getLongitude();
    droneState.has_gps_home      = true;
  }
  droneState.home_alt_baseline_m = droneState.kf_z;

  kfNorth.reset(0.0f);
  kfEast.reset(0.0f);
  droneState.dr_pos_x_m          = 0.0f;
  droneState.dr_pos_y_m          = 0.0f;
  droneState.dr_pos_z_m          = 0.0f;
  droneState.dr_vel_x_mps        = 0.0f;
  droneState.dr_vel_y_mps        = 0.0f;
  droneState.dr_vel_z_mps        = 0.0f;
  droneState.pos_north_m         = 0.0f;
  droneState.pos_east_m          = 0.0f;
  droneState.vel_north_filt      = 0.0f;
  droneState.vel_east_filt       = 0.0f;
  droneState.raw_pos_north_m     = 0.0f;
  droneState.raw_pos_east_m      = 0.0f;
  droneState.dist_from_home_2d_m = 0.0f;
  droneState.dist_from_home_3d_m = 0.0f;
  droneState.dist_2d_m           = 0.0f;
  droneState.dist_3d_m           = 0.0f;
  droneState.speed_3d_mps        = 0.0f;
  dualPrintln(">>> HOME UPDATED TO ARM POINT! (Dead Reckoning Origin re-anchored at launch location)");
}

const char* getHomeOriginTypeName() {
  if (droneState.home_type == DroneState::HOME_ARM) return "ARM";
  return "BOOT";
}

// =============================================================================
//  SENSOR CALIBRATION (IMU, Ground Clearance, Compass, Barometer P0)
// =============================================================================
void calibrateAllSensors() {
  dualPrintln(">>> Calibrating IMU, Ground Clearance & Barometer Baseline...");
  delay(400);

  float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
  float sum_r = 0.0f, sum_p = 0.0f;
  float sum_dist = 0.0f;
  int valid_dist_count = 0;
  const int SAMPLES = 200;

  for (int i = 0; i < SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    if (imuSensor && imuSensor->read(ax, ay, az, gx, gy, gz)) {
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;

      float r = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
      float p = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
      sum_r += r;
      sum_p += p;
    }

    if (rangeSensor && (i % 4 == 0)) {
      float d = rangeSensor->readDistanceCM();
      if (d >= 5.0f && d <= 80.0f) {
        sum_dist += d;
        valid_dist_count++;
      }
    }
    delay(4);
  }

  droneState.cal_gyro_x       = sum_gx / SAMPLES;
  droneState.cal_gyro_y       = sum_gy / SAMPLES;
  droneState.cal_gyro_z       = sum_gz / SAMPLES;
  droneState.cal_roll_offset  = sum_r  / SAMPLES;
  droneState.cal_pitch_offset = sum_p  / SAMPLES;
  droneState.roll_deg         = 0.0f;
  droneState.pitch_deg        = 0.0f;

  if (valid_dist_count > 0) {
    droneState.ground_height_cm  = sum_dist / (float)valid_dist_count;
  } else {
    droneState.ground_height_cm  = 10.0f;
  }
  // Sanity clamp ground clearance baseline to avoid the <8cm non-linear ambiguity zone
  droneState.ground_height_cm    = constrain(droneState.ground_height_cm, 8.0f, 16.0f);
  droneState.current_height_cm   = droneState.ground_height_cm;
  droneState.kf_z                = droneState.ground_height_cm * 0.01f;

  // Lock True North heading with compass
  droneState.has_compass = compassSensor && compassSensor->isAvailable();
  if (droneState.has_compass) {
    float sum_Xh = 0.0f, sum_Yh = 0.0f;
    for (int i = 0; i < 20; i++) {
      updateCompassEstimator();
      sum_Xh += cos(droneState.mag_heading_deg * DEG_TO_RAD);
      sum_Yh += sin(droneState.mag_heading_deg * DEG_TO_RAD);
      delay(8);
    }
    droneState.yaw_deg = atan2(sum_Yh, sum_Xh) * RAD_TO_DEG;
  } else {
    droneState.yaw_deg = 0.0f;
  }

  // Calibrate Barometer P0
  if (baroSensor && baroSensor->isAvailable()) {
    if (baroSensor->calibrateBaseline(15)) {
      droneState.baro_pressure_pa     = baroSensor->getBaselinePressurePa();
      droneState.baro_raw_alt_cm      = 0.0f;
      droneState.baro_ground_offset_m = 0.0f;
    }
  }

  // Calibrate residual stationary horizontal acceleration offsets (ZUPT baseline)
  float sum_afwd = 0.0f, sum_aright = 0.0f;
  for (int i = 0; i < 60; i++) {
    float ax, ay, az, gx, gy, gz;
    if (imuSensor && imuSensor->read(ax, ay, az, gx, gy, gz)) {
      // At level calibration baseline: theta_r = 0, phi_r = 0
      float theta_r = 0.0f;
      float phi_r   = 0.0f;
      float afwd    = (ax * cos(theta_r)) + (az * sin(theta_r));
      float aright  = (ay * cos(phi_r))   + (az * sin(phi_r));
      sum_afwd   += afwd;
      sum_aright += aright;
    }
    delay(4);
  }
  droneState.cal_accel_fwd_offset   = sum_afwd / 60.0f;
  droneState.cal_accel_right_offset = sum_aright / 60.0f;

  droneState.is_calibrated = true;

  dualPrintln(">>> Auto-Calibration complete!");
  dualPrintln("  Attitude Offsets   -> Roll: " + String(droneState.cal_roll_offset, 2) + " deg, Pitch: " + String(droneState.cal_pitch_offset, 2) + " deg");
  dualPrintln("  Gyro Bias          -> gx: " + String(droneState.cal_gyro_x, 4) + ", gy: " + String(droneState.cal_gyro_y, 4) + ", gz: " + String(droneState.cal_gyro_z, 4));
  dualPrintln("  Accel Horiz Offset -> fwd: " + String(droneState.cal_accel_fwd_offset, 3) + " m/s², right: " + String(droneState.cal_accel_right_offset, 3) + " m/s²");
  dualPrintln("  Ground Clearance   -> " + String(droneState.ground_height_cm, 1) + " cm");
  if (droneState.has_compass) {
    dualPrintln("  True North Heading -> " + String(droneState.yaw_deg, 1) + " deg (" + String(compassSensor->getDeviceName()) + ")");
  }
}

// =============================================================================
//  ATTITUDE ESTIMATOR (Complementary Filter + True North Fusion)
// =============================================================================
void updateAttitudeEstimator(float dt) {
  if (!imuSensor) return;

  float ax, ay, az, gx_rads, gy_rads, gz_rads;
  if (!imuSensor->read(ax, ay, az, gx_rads, gy_rads, gz_rads)) return;

  droneState.ax_body = ax;
  droneState.ay_body = ay;
  droneState.az_body = az;

  // Convert to deg/s with bias subtraction
  droneState.gx_dps = -(gx_rads - droneState.cal_gyro_x) * RAD_TO_DEG;
  droneState.gy_dps =  (gy_rads - droneState.cal_gyro_y) * RAD_TO_DEG;
  droneState.gz_dps =  (gz_rads - droneState.cal_gyro_z) * RAD_TO_DEG;

  float rollAcc  = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
  float pitchAcc = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  droneState.roll_deg  = ALPHA_ATT * (droneState.roll_deg  + droneState.gx_dps * dt) +
                         (1.0f - ALPHA_ATT) * (rollAcc  - droneState.cal_roll_offset);
  droneState.pitch_deg = ALPHA_ATT * (droneState.pitch_deg + droneState.gy_dps * dt) +
                         (1.0f - ALPHA_ATT) * (pitchAcc - droneState.cal_pitch_offset);

  // Yaw Integration with True North Compass Fusion
  if (droneState.has_compass) {
    float yaw_diff = droneState.mag_heading_deg - droneState.yaw_deg;
    while (yaw_diff > 180.0f)  yaw_diff -= 360.0f;
    while (yaw_diff < -180.0f) yaw_diff += 360.0f;
    droneState.yaw_deg += (droneState.gz_dps * dt) + (1.0f - ALPHA_YAW) * yaw_diff;
  } else {
    droneState.yaw_deg += droneState.gz_dps * dt;
  }
  while (droneState.yaw_deg >  180.0f) droneState.yaw_deg -= 360.0f;
  while (droneState.yaw_deg < -180.0f) droneState.yaw_deg += 360.0f;
}

// =============================================================================
//  3-STATE VERTICAL KALMAN FILTER & ALTITUDE FUSION
// =============================================================================
static void kfPredict(float a_z_world, float dt) {
  if (dt <= 0.0001f || dt > 0.05f) dt = 0.02f;

  float a_corr = a_z_world - droneState.kf_ba;

  // 1. Kinematic Propagation
  droneState.kf_z  += (droneState.kf_vz * dt) + (0.5f * a_corr * dt * dt);
  droneState.kf_vz += a_corr * dt;

  // 2. Covariance Propagation
  float f01 = dt;
  float f02 = -0.5f * dt * dt;
  float f12 = -dt;

  float M00 = P_00 + f01 * P_10 + f02 * P_20;
  float M01 = P_01 + f01 * P_11 + f02 * P_21;
  float M02 = P_02 + f01 * P_12 + f02 * P_22;

  float M10 = P_10 + f12 * P_20;
  float M11 = P_11 + f12 * P_21;
  float M12 = P_12 + f12 * P_22;

  float M20 = P_20;
  float M21 = P_21;
  float M22 = P_22;

  P_00 = M00 + M01 * f01 + M02 * f02 + Q_z;
  P_01 = M01 + M02 * f12;
  P_02 = M02;

  P_10 = P_01;
  P_11 = M11 + M12 * f12 + Q_vz;
  P_12 = M12;

  P_20 = P_02;
  P_21 = P_12;
  P_22 = M22 + Q_ba;
}

static void kfUpdate(float z_meas, float R) {
  float y = z_meas - droneState.kf_z;
  float S = P_00 + R;
  if (S < 1e-6f) return;

  float K0 = P_00 / S;
  float K1 = P_10 / S;
  float K2 = P_20 / S;

  droneState.kf_z  += K0 * y;
  droneState.kf_vz += K1 * y;
  droneState.kf_ba += K2 * y;
  droneState.kf_ba = constrain(droneState.kf_ba, -2.0f, 2.0f);

  float p00 = P_00, p01 = P_01, p02 = P_02;

  P_00 -= K0 * p00;
  P_01 -= K0 * p01;
  P_02 -= K0 * p02;

  P_10 -= K1 * p00;
  P_11 -= K1 * p01;
  P_12 -= K1 * p02;

  P_20 -= K2 * p00;
  P_21 -= K2 * p01;
  P_22 -= K2 * p02;

  P_10 = P_01;
  P_20 = P_02;
  P_21 = P_12;
}

void updateAltitudeEstimator(float dt, bool is_airborne, bool sim_mode) {
  // 1. Advance Barometer non-blocking state machine
  if (baroSensor) {
    baroSensor->update();
    droneState.baro_pressure_pa = baroSensor->getPressurePa();
    droneState.baro_raw_alt_cm  = baroSensor->getRawAltitudeCm();
  }

  // 2. Kinematic Prediction (World vertical acceleration)
  float phi_r   = droneState.roll_deg  * DEG_TO_RAD;
  float theta_r = droneState.pitch_deg * DEG_TO_RAD;
  float a_z_world = (droneState.ax_body * sin(theta_r)) -
                    (droneState.ay_body * sin(phi_r) * cos(theta_r)) +
                    (droneState.az_body * cos(phi_r) * cos(theta_r)) - 9.80665f;

  if (is_airborne || sim_mode) {
    kfPredict(a_z_world, dt);
  } else {
    // When on ground before airborne confirmation:
    // Allow kf_z to track the tilt-corrected IR rangefinder if valid,
    // otherwise fallback to ground baseline (~10cm).
    float ground_z = (droneState.rf_valid && droneState.z_rf_corrected_m >= 0.05f) ?
                     droneState.z_rf_corrected_m : (droneState.ground_height_cm * 0.01f);
    droneState.kf_z  = ground_z;
    droneState.kf_vz = 0.0f;
    droneState.kf_ba = 0.0f;
    // Keep Kalman error covariance primed for smooth handoff to kfPredict upon liftoff
    P_00 = 0.01f;
    P_11 = 0.05f;
    P_22 = 0.01f;
  }

  // 3. Rangefinder reading & tilt compensation
  if (rangeSensor) {
    float raw_cm = rangeSensor->readDistanceCM();
    if (!isnan(raw_cm) && raw_cm > 0.0f) {
      droneState.raw_height_cm = raw_cm;
    }
  }

  float z_raw_m = droneState.raw_height_cm * 0.01f;
  droneState.z_rf_corrected_m = z_raw_m * cos(phi_r) * cos(theta_r);

  // Rangefinder is physically valid whenever it is within its 0.8m sensing envelope
  // and tilt angle is within normal flying limits (beam pointing toward the ground)
  bool tilt_ok = (fabs(droneState.roll_deg) <= 30.0f) && (fabs(droneState.pitch_deg) <= 30.0f);
  droneState.rf_valid = tilt_ok && !isnan(droneState.raw_height_cm) &&
                        (droneState.z_rf_corrected_m >= 0.05f) &&
                        (droneState.z_rf_corrected_m <= 0.80f);

  float z_baro = droneState.baro_raw_alt_cm * 0.01f;
  bool has_baro = baroSensor && baroSensor->isAvailable();

  // Rangefinder Handoff & Continuous Datum Tracking (kept updated in background)
  if (has_baro && droneState.rf_valid && droneState.kf_z < 0.65f) {
    droneState.baro_ground_offset_m = (1.0f - BARO_OFFSET_ALPHA) * droneState.baro_ground_offset_m +
                                      BARO_OFFSET_ALPHA * (z_baro - droneState.z_rf_corrected_m);
  }

  // 4. Measurement update
#if USE_SHARP_ONLY_ALTITUDE
  // PURE SHARP IR RANGEFINDER MODE:
  // Completely immune to propeller downwash, air buffeting, and baro drift.
  // Directly updates the vertical Kalman filter using tilt-compensated distance.
  if (droneState.rf_valid) {
    float R_meas = (droneState.raw_height_cm >= 70.0f) ? 0.03f : 0.005f;
    kfUpdate(droneState.z_rf_corrected_m, R_meas);
    droneState.active_alt_source = ALT_SOURCE_SHARP;
  }
#else
  // Measurement update based on altitude zone
  if (droneState.kf_z < 0.50f) {
    if (droneState.rf_valid) {
      kfUpdate(droneState.z_rf_corrected_m, 0.005f);
      droneState.active_alt_source = ALT_SOURCE_SHARP;
    } else if (has_baro) {
      kfUpdate(z_baro - droneState.baro_ground_offset_m, 0.60f);
      droneState.active_alt_source = ALT_SOURCE_BARO_ACC;
    }
  } else if (droneState.kf_z <= 0.80f) {
    float t = constrain((droneState.kf_z - 0.50f) / 0.30f, 0.0f, 1.0f);
    float R_rf   = 0.005f + 0.12f * (t * t);
    float R_baro = 0.15f  + 0.45f * (1.0f - t);

    if (droneState.rf_valid) {
      kfUpdate(droneState.z_rf_corrected_m, R_rf);
    }
    if (has_baro) {
      kfUpdate(z_baro - droneState.baro_ground_offset_m, R_baro);
    }
    droneState.active_alt_source = (t < 0.5f && droneState.rf_valid) ? ALT_SOURCE_SHARP : ALT_SOURCE_BARO_ACC;
  } else {
    if (has_baro) {
      kfUpdate(z_baro - droneState.baro_ground_offset_m, 0.15f);
      droneState.active_alt_source = ALT_SOURCE_BARO_ACC;
    } else if (droneState.rf_valid) {
      kfUpdate(droneState.z_rf_corrected_m, 0.05f);
      droneState.active_alt_source = ALT_SOURCE_SHARP;
    }
  }
#endif

  droneState.current_height_cm   = droneState.kf_z * 100.0f;
  droneState.climb_rate_cms      = droneState.kf_vz * 100.0f;
  droneState.climb_rate_filt     = droneState.kf_vz * 100.0f;
  droneState.dr_pos_z_m          = droneState.kf_z - droneState.home_alt_baseline_m;
  droneState.dr_vel_z_mps        = droneState.kf_vz;
  droneState.dist_from_home_3d_m = sqrtf(droneState.dist_from_home_2d_m * droneState.dist_from_home_2d_m +
                                         droneState.dr_pos_z_m * droneState.dr_pos_z_m);
  droneState.dist_3d_m           = droneState.dist_from_home_3d_m;
  droneState.speed_3d_mps        = sqrtf(droneState.vel_north_filt * droneState.vel_north_filt +
                                         droneState.vel_east_filt * droneState.vel_east_filt +
                                         droneState.kf_vz * droneState.kf_vz);
}

// =============================================================================
//  COMPASS HEADING ESTIMATOR (3D Tilt-Compensated)
// =============================================================================
void updateCompassEstimator() {
  if (!compassSensor || !compassSensor->isAvailable()) return;

  float raw_x, raw_y, raw_z;
  if (!compassSensor->readRaw(raw_x, raw_y, raw_z)) return;

  float mx = raw_x - droneState.mag_offset_x;
  float my = raw_y - droneState.mag_offset_y;
  float mz = raw_z - droneState.mag_offset_z;

  float phi   = (droneState.roll_deg  + droneState.cal_roll_offset)  * DEG_TO_RAD;
  float theta = (droneState.pitch_deg + droneState.cal_pitch_offset) * DEG_TO_RAD;

  float cos_phi   = cos(phi);
  float sin_phi   = sin(phi);
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

  // 3D Horizontal Projection
  float Xh = mx * cos_theta + my * sin_phi * sin_theta + mz * cos_phi * sin_theta;
  float Yh = my * cos_phi - mz * sin_phi;

  float heading = atan2(Yh, Xh) * RAD_TO_DEG;
  heading += droneState.mag_declination_deg;

  while (heading > 180.0f)  heading -= 360.0f;
  while (heading < -180.0f) heading += 360.0f;

  droneState.mag_heading_deg = heading;
}

void calibrateCompassRoutine() {
  if (!compassSensor || !compassSensor->isAvailable()) {
    dualPrintln("ERR: Compass not available for calibration!");
    return;
  }

  dualPrintln(">>> STARTING GY-87 COMPASS CALIBRATION (12 seconds)...");
  dualPrintln(">>> Slowly rotate the drone in a 3D figure-8 pattern NOW!");
  delay(1000);

  float min_x = 99999.0f, max_x = -99999.0f;
  float min_y = 99999.0f, max_y = -99999.0f;
  float min_z = 99999.0f, max_z = -99999.0f;

  unsigned long start = millis();
  while (millis() - start < 12000) {
    float rx, ry, rz;
    if (compassSensor->readRaw(rx, ry, rz)) {
      if (rx < min_x) min_x = rx; if (rx > max_x) max_x = rx;
      if (ry < min_y) min_y = ry; if (ry > max_y) max_y = ry;
      if (rz < min_z) min_z = rz; if (rz > max_z) max_z = rz;
    }
    delay(20);
  }

  droneState.mag_offset_x = (max_x + min_x) / 2.0f;
  droneState.mag_offset_y = (max_y + min_y) / 2.0f;
  droneState.mag_offset_z = (max_z + min_z) / 2.0f;

  dualPrintln(">>> GY-87 Compass Calibration COMPLETE!");
  dualPrintln("  Offsets -> X: " + String(droneState.mag_offset_x, 1) +
              ", Y: " + String(droneState.mag_offset_y, 1) +
              ", Z: " + String(droneState.mag_offset_z, 1));
}

// =============================================================================
//  50 Hz HORIZONTAL INERTIAL ESTIMATOR (IMU Accelerometer Prediction)
// =============================================================================
void updateHorizontalEstimator(float dt, bool is_airborne, bool sim_mode) {
  // Tilt-compensated Earth-frame horizontal acceleration
  float phi_r   = droneState.roll_deg  * DEG_TO_RAD;
  float theta_r = droneState.pitch_deg * DEG_TO_RAD;
  float psi_r   = droneState.yaw_deg   * DEG_TO_RAD;

  // Corrected body-oriented horizontal plane accelerations (physics-based):
  // Pitching nose down (theta > 0) directs rotor thrust forward -> positive a_fwd
  // Rolling right (phi > 0) directs rotor thrust rightward -> positive a_right
  float a_fwd_horiz   = (droneState.ax_body * cos(theta_r)) + (droneState.az_body * sin(theta_r)) - droneState.cal_accel_fwd_offset;
  float a_right_horiz = (droneState.ay_body * cos(phi_r))   + (droneState.az_body * sin(phi_r))   - droneState.cal_accel_right_offset;

  // Rotate body-oriented horizontal plane into Earth NED frame (North, East)
  float cos_psi = cos(psi_r);
  float sin_psi = sin(psi_r);
  float a_north =  a_fwd_horiz * cos_psi - a_right_horiz * sin_psi;
  float a_east  =  a_fwd_horiz * sin_psi + a_right_horiz * cos_psi;

  droneState.accel_north_world = a_north;
  droneState.accel_east_world  = a_east;

  if (is_airborne) {
    kfNorth.predict(a_north, dt);
    kfEast.predict(a_east, dt);
  } else if (sim_mode) {
    // In sim mode on the bench: predict, but apply gentle damping when IMU is still
    kfNorth.predict(a_north, dt);
    kfEast.predict(a_east, dt);
    if (fabs(a_north) < 0.10f) kfNorth.vel *= 0.98f;
    if (fabs(a_east)  < 0.10f) kfEast.vel  *= 0.98f;
  } else {
    // When on ground or disarmed: hold velocity and accel bias strictly at 0.0 to prevent runaway integration
    kfNorth.vel = 0.0f;
    kfNorth.ba  = 0.0f;
    kfEast.vel  = 0.0f;
    kfEast.ba   = 0.0f;

    if (droneState.home_type == DroneState::HOME_ARM) {
      // Armed on the ground at launch pad: firmly locked at origin (0.0m) until liftoff
      kfNorth.pos = 0.0f;
      kfEast.pos  = 0.0f;
    } else if (droneState.has_gps_home) {
      kfNorth.pos = droneState.raw_pos_north_m;
      kfEast.pos  = droneState.raw_pos_east_m;
    } else {
      kfNorth.pos = 0.0f;
      kfEast.pos  = 0.0f;
    }
  }

  // Sync fused estimates into public state
  droneState.kf_pos_n       = kfNorth.pos;
  droneState.kf_vel_n       = kfNorth.vel;
  droneState.kf_ba_n        = kfNorth.ba;

  droneState.kf_pos_e       = kfEast.pos;
  droneState.kf_vel_e       = kfEast.vel;
  droneState.kf_ba_e        = kfEast.ba;

  droneState.pos_north_m    = kfNorth.pos;
  droneState.pos_east_m     = kfEast.pos;
  droneState.vel_north_filt = kfNorth.vel;
  droneState.vel_east_filt  = kfEast.vel;

  // Sync Dead Reckoning outputs relative to Home
  droneState.dr_pos_x_m     = droneState.pos_north_m;
  droneState.dr_pos_y_m     = droneState.pos_east_m;
  droneState.dr_pos_z_m     = droneState.kf_z - droneState.home_alt_baseline_m;
  droneState.dr_vel_x_mps   = droneState.vel_north_filt;
  droneState.dr_vel_y_mps   = droneState.vel_east_filt;
  droneState.dr_vel_z_mps   = droneState.kf_vz;

  droneState.dist_from_home_2d_m = sqrtf(droneState.dr_pos_x_m * droneState.dr_pos_x_m +
                                         droneState.dr_pos_y_m * droneState.dr_pos_y_m);
  droneState.dist_from_home_3d_m = sqrtf(droneState.dist_from_home_2d_m * droneState.dist_from_home_2d_m +
                                         droneState.dr_pos_z_m * droneState.dr_pos_z_m);
  droneState.dist_2d_m           = droneState.dist_from_home_2d_m;
  droneState.dist_3d_m           = droneState.dist_from_home_3d_m;
  droneState.speed_3d_mps        = sqrtf(droneState.dr_vel_x_mps * droneState.dr_vel_x_mps +
                                         droneState.dr_vel_y_mps * droneState.dr_vel_y_mps +
                                         droneState.dr_vel_z_mps * droneState.dr_vel_z_mps);
}

// =============================================================================
//  GPS POSITION & VELOCITY ESTIMATOR (Measurement Updates)
// =============================================================================
void updatePositionEstimator(float dt, bool is_armed) {
  if (!gpsSensor) return;

  if (gpsSensor->hasFix()) {
    droneState.last_gps_fix_millis = millis();
    droneState.gps_accuracy_est_m  = gpsSensor->getHDOP() * 2.5f;

    int sats = gpsSensor->getSatellites();
    float hdop = gpsSensor->getHDOP();
    bool good_lock = (sats >= 4 && hdop <= 4.5f) || (sats == 0 && gpsSensor->hasFix());

    // If home is currently at boot point and has no GPS coordinate yet, anchor once lock is solid
    if (!droneState.has_gps_home) {
      if (good_lock) {
        droneState.gps_home_lat     = gpsSensor->getLatitude();
        droneState.gps_home_lon     = gpsSensor->getLongitude();
        droneState.has_gps_home     = true;
        kfNorth.reset(0.0f);
        kfEast.reset(0.0f);
        droneState.raw_pos_north_m  = 0.0f;
        droneState.raw_pos_east_m   = 0.0f;
        droneState.pos_north_m      = 0.0f;
        droneState.pos_east_m       = 0.0f;
        droneState.dr_pos_x_m       = 0.0f;
        droneState.dr_pos_y_m       = 0.0f;
        droneState.vel_north_filt   = 0.0f;
        droneState.vel_east_filt    = 0.0f;
        droneState.dist_from_home_2d_m = 0.0f;
        droneState.dist_from_home_3d_m = droneState.dr_pos_z_m;
        droneState.dist_2d_m        = 0.0f;
        droneState.dist_3d_m        = droneState.dr_pos_z_m;
        dualPrintln(">>> BOOT HOME ANCHORED TO SETTLED GPS: Lat=" + String(droneState.gps_home_lat, 6) +
                    " Lon=" + String(droneState.gps_home_lon, 6) +
                    " (Sats: " + String(gpsSensor->getSatellites()) +
                    ", HDOP: " + String(gpsSensor->getHDOP(), 2) + ")");
        return;
      }
    }

    if (droneState.has_gps_home && gpsSensor->isUpdated()) {
      double dLat = gpsSensor->getLatitude() - droneState.gps_home_lat;
      double dLon = gpsSensor->getLongitude() - droneState.gps_home_lon;

      float raw_pos_n = (float)(dLat * 111139.0);
      float raw_pos_e = (float)(dLon * 111139.0 * cos(droneState.gps_home_lat * 0.017453292519943295));

      // Smooth continuous GPS displacement
      droneState.raw_pos_north_m = raw_pos_n;
      droneState.raw_pos_east_m  = raw_pos_e;

      float hdop = gpsSensor->getHDOP();
      if (hdop <= 4.0f && gpsSensor->getSatellites() >= 5) {
        // Continuous variance scaling with HDOP without artificial step-deadbands (PX4 standard)
        float r_pos = constrain(hdop * hdop * 0.4f, 0.4f, 15.0f);

        if (droneState.is_airborne) {
          kfNorth.updatePos(raw_pos_n, r_pos);
          kfEast.updatePos(raw_pos_e, r_pos);
        } else if (is_armed) {
          // Armed on the ground at launch pad: firmly lock at origin (0.0m) until liftoff
          kfNorth.pos = 0.0f;
          kfEast.pos  = 0.0f;
          kfNorth.vel = 0.0f;
          kfEast.vel  = 0.0f;
        } else {
          // Disarmed on bench/ground: filter GPS displacement to eliminate stationary wandering
          kfNorth.pos += 0.15f * (raw_pos_n - kfNorth.pos);
          kfEast.pos  += 0.15f * (raw_pos_e - kfEast.pos);
        }

        // Doppler velocity update from GPS if available
        if (gpsSensor->hasSpeedFix()) {
          float spd = gpsSensor->getSpeedMps();
          float crs = gpsSensor->getCourseDeg() * DEG_TO_RAD;
          float raw_vn = (spd > 0.05f) ? (spd * cos(crs)) : 0.0f;
          float raw_ve = (spd > 0.05f) ? (spd * sin(crs)) : 0.0f;
          float r_vel  = 0.08f;

          if (droneState.is_airborne) {
            kfNorth.updateVel(raw_vn, r_vel);
            kfEast.updateVel(raw_ve, r_vel);
          } else {
            kfNorth.vel = raw_vn;
            kfEast.vel  = raw_ve;
          }
        }

          // Sync fused estimates into public state
          droneState.pos_north_m    = kfNorth.pos;
          droneState.pos_east_m     = kfEast.pos;
          droneState.vel_north_filt = kfNorth.vel;
          droneState.vel_east_filt  = kfEast.vel;

          // Sync Dead Reckoning outputs relative to Home
          droneState.dr_pos_x_m     = droneState.pos_north_m;
          droneState.dr_pos_y_m     = droneState.pos_east_m;
          droneState.dr_pos_z_m     = droneState.kf_z - droneState.home_alt_baseline_m;
          droneState.dr_vel_x_mps   = droneState.vel_north_filt;
          droneState.dr_vel_y_mps   = droneState.vel_east_filt;
          droneState.dr_vel_z_mps   = droneState.kf_vz;

          droneState.dist_from_home_2d_m = sqrtf(droneState.dr_pos_x_m * droneState.dr_pos_x_m +
                                                 droneState.dr_pos_y_m * droneState.dr_pos_y_m);
          droneState.dist_from_home_3d_m = sqrtf(droneState.dist_from_home_2d_m * droneState.dist_from_home_2d_m +
                                                 droneState.dr_pos_z_m * droneState.dr_pos_z_m);
          droneState.dist_2d_m           = droneState.dist_from_home_2d_m;
          droneState.dist_3d_m           = droneState.dist_from_home_3d_m;
          droneState.speed_3d_mps        = sqrtf(droneState.dr_vel_x_mps * droneState.dr_vel_x_mps +
                                                 droneState.dr_vel_y_mps * droneState.dr_vel_y_mps +
                                                 droneState.dr_vel_z_mps * droneState.dr_vel_z_mps);
        }
      }
  } else {
    droneState.gps_accuracy_est_m = 99.9f;
  }
}
