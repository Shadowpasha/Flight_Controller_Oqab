#include "StateEstimator.h"
#include "Controllers.h"
#include <math.h>

void dualPrintln(const String& msg);
void dualPrint(const String& msg);

DroneState droneState;

// --- Dynamic Estimator Configuration & Noise Parameters ---
EstimatorConfig estConfig = {
  0.991f,   // alpha_att (attitude complementary filter gain)
  0.995f,   // alpha_yaw (yaw complementary filter gain)
  0.0002f,  // q_z (vertical position process noise)
  0.0035f,  // q_vz (vertical velocity process noise)
  0.00003f, // q_ba (vertical accel bias process noise)
  0.0025f,  // r_lidar (ToF LiDAR measurement covariance)
  0.001f,   // q_horiz_pos
  0.008f,   // q_horiz_vel
  0.0001f,  // q_horiz_ba
  0.02f,    // r_flow (optical flow base velocity measurement noise)
  35.0f     // flow_q_thresh (minimum surface tracking quality threshold)
};

// --- Vertical Kalman Filter Covariance ---
static float P_00 = 0.05f, P_01 = 0.0f, P_02 = 0.0f;
static float P_10 = 0.0f,  P_11 = 0.05f, P_12 = 0.0f;
static float P_20 = 0.0f,  P_21 = 0.0f, P_22 = 0.02f;

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
  droneState.gps_enabled         = USE_GPS;
  droneState.flow_enabled        = USE_OPTICAL_FLOW;
  droneState.flow_rot_deg        = FLOW_DEFAULT_ROT_DEG;

  droneState.ground_height_cm    = 10.0f;
  droneState.current_height_cm   = 10.0f;
  droneState.kf_z                = 0.10f;
  droneState.az_filt             = 9.81f;
  droneState.active_alt_source   = ALT_SOURCE_LIDAR;

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
  droneState.flow_valid           = false;
  droneState.flow_vel_body_x      = 0.0f;
  droneState.flow_vel_body_y      = 0.0f;
  droneState.flow_vel_north_mps   = 0.0f;
  droneState.flow_vel_east_mps    = 0.0f;
  droneState.vel_north_filt       = 0.0f;
  droneState.vel_east_filt        = 0.0f;
  droneState.is_airborne          = false;
  droneState.above_thresh_since_ms = 0;
  droneState.az_filt              = 9.81f;
}

void setFlowEnabled(bool enable) {
  droneState.flow_enabled = enable;
  if (!enable) {
    droneState.flow_valid         = false;
    droneState.flow_vel_body_x    = 0.0f;
    droneState.flow_vel_body_y    = 0.0f;
    droneState.flow_vel_north_mps = 0.0f;
    droneState.flow_vel_east_mps  = 0.0f;
    if (flowSensor) {
      flowSensor->end();
    }
    resetPositionControl();
    dualPrintln("---> Mateksys Optical Flow DISABLED");
  } else {
    if (flowSensor) {
      flowSensor->begin();
    }
    dualPrintln("---> Mateksys Optical Flow ENABLED on Serial2 (RX:16, TX:17) @ 115200 baud");
  }
}

void setGPSEnabled(bool enable) {
  droneState.gps_enabled = enable;
  if (!enable) {
    droneState.has_gps_home       = false;
    droneState.gps_home_lat       = 0.0;
    droneState.gps_home_lon       = 0.0;
    droneState.raw_pos_north_m    = 0.0f;
    droneState.raw_pos_east_m     = 0.0f;
    droneState.gps_accuracy_est_m = 99.9f;
    if (gpsSensor) {
      gpsSensor->end();
    }
    resetPositionControl();
    dualPrintln("---> GPS Subsystem DISABLED: All GPS navigation and fusion stopped");
  } else {
    if (gpsSensor) {
      gpsSensor->begin();
    }
    dualPrintln("---> GPS Subsystem ENABLED: Listening on Serial2 (RX:16, TX:17)");
  }
}

// =============================================================================
//  HOME REFERENCE & DEAD RECKONING ORIGIN MANAGEMENT
// =============================================================================
void setHomeToBootPoint() {
  droneState.home_type           = DroneState::HOME_BOOT;
  droneState.home_is_set         = true;
  bool good_fix                  = (droneState.gps_enabled && gpsSensor && gpsSensor->hasFix() && 
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
  if (droneState.gps_enabled && gpsSensor && gpsSensor->hasFix()) {
    droneState.gps_home_lat      = gpsSensor->getLatitude();
    droneState.gps_home_lon      = gpsSensor->getLongitude();
    droneState.has_gps_home      = true;
  } else {
    droneState.gps_home_lat      = 0.0;
    droneState.gps_home_lon      = 0.0;
    droneState.has_gps_home      = false;
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
  dualPrintln(">>> Calibrating IMU, Mateksys ToF LiDAR Ground Clearance & Gyro Bias...");
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

    // Feed and sample Mateksys ToF LiDAR distance
    if (droneState.flow_enabled && flowSensor) {
      flowSensor->update();
      if (flowSensor->hasLidar()) {
        float d_cm = flowSensor->getLidarDistanceM() * 100.0f;
        if (d_cm >= 3.0f && d_cm <= 40.0f) {
          sum_dist += d_cm;
          valid_dist_count++;
        }
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
    droneState.ground_height_cm = sum_dist / (float)valid_dist_count;
  } else {
    droneState.ground_height_cm = 8.0f; // Default landing gear height baseline
  }
  droneState.ground_height_cm    = constrain(droneState.ground_height_cm, 3.0f, 25.0f);
  droneState.current_height_cm   = droneState.ground_height_cm;
  droneState.raw_height_cm       = droneState.ground_height_cm;
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

  // Barometer completely disabled - set ground datum to zero
  droneState.baro_pressure_pa     = 101325.0f;
  droneState.baro_raw_alt_cm      = 0.0f;
  droneState.baro_ground_offset_m = 0.0f;

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

  droneState.roll_deg  = estConfig.alpha_att * (droneState.roll_deg  + droneState.gx_dps * dt) +
                         (1.0f - estConfig.alpha_att) * (rollAcc  - droneState.cal_roll_offset);
  droneState.pitch_deg = estConfig.alpha_att * (droneState.pitch_deg + droneState.gy_dps * dt) +
                         (1.0f - estConfig.alpha_att) * (pitchAcc - droneState.cal_pitch_offset);

  // Yaw Integration with True North Compass Fusion
  if (droneState.has_compass) {
    float yaw_diff = droneState.mag_heading_deg - droneState.yaw_deg;
    while (yaw_diff > 180.0f)  yaw_diff -= 360.0f;
    while (yaw_diff < -180.0f) yaw_diff += 360.0f;
    droneState.yaw_deg += (droneState.gz_dps * dt) + (1.0f - estConfig.alpha_yaw) * yaw_diff;
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

  P_00 = M00 + M01 * f01 + M02 * f02 + estConfig.q_z;
  P_01 = M01 + M02 * f12;
  P_02 = M02;

  P_10 = P_01;
  P_11 = M11 + M12 * f12 + estConfig.q_vz;
  P_12 = M12;

  P_20 = P_02;
  P_21 = P_12;
  P_22 = M22 + estConfig.q_ba;
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
  // 1. Kinematic Prediction (World vertical acceleration)
  float phi_r   = droneState.roll_deg  * DEG_TO_RAD;
  float theta_r = droneState.pitch_deg * DEG_TO_RAD;
  float a_z_world = (droneState.ax_body * sin(theta_r)) -
                    (droneState.ay_body * sin(phi_r) * cos(theta_r)) +
                    (droneState.az_body * cos(phi_r) * cos(theta_r)) - 9.80665f;

  if (is_airborne || sim_mode) {
    kfPredict(a_z_world, dt);
  } else {
    // When on ground before airborne confirmation:
    // Allow kf_z to track the tilt-corrected LiDAR if valid,
    // otherwise fallback to ground baseline (~8cm).
    float ground_z = (droneState.rf_valid && droneState.z_rf_corrected_m >= 0.03f) ?
                     droneState.z_rf_corrected_m : (droneState.ground_height_cm * 0.01f);
    droneState.kf_z  = ground_z;
    droneState.kf_vz = 0.0f;
    droneState.kf_ba = 0.0f;
    // Keep Kalman error covariance primed for smooth handoff to kfPredict upon liftoff
    P_00 = 0.01f;
    P_11 = 0.05f;
    P_22 = 0.01f;
  }

  // 2. Mateksys ToF LiDAR Reading & Tilt Compensation
  // Barometer and Sharp IR are completely removed.
  bool lidar_active = droneState.flow_enabled && flowSensor && flowSensor->hasLidar() &&
                      (flowSensor->getLidarQuality() > 30) && (flowSensor->getLidarDistanceM() >= 0.03f);

  if (lidar_active) {
    droneState.raw_height_cm = flowSensor->getLidarDistanceM() * 100.0f;
  }

  float z_raw_m = droneState.raw_height_cm * 0.01f;
  droneState.z_rf_corrected_m = z_raw_m * cos(phi_r) * cos(theta_r);

  // Rangefinder is valid up to 2.00m within +/-35 deg tilt
  bool tilt_ok = (fabs(droneState.roll_deg) <= 35.0f) && (fabs(droneState.pitch_deg) <= 35.0f);
  droneState.rf_valid = lidar_active && tilt_ok && !isnan(droneState.raw_height_cm) &&
                        (droneState.z_rf_corrected_m >= 0.03f) &&
                        (droneState.z_rf_corrected_m <= 2.00f);

  // 3. Vertical Kalman Filter Measurement Update
  // Pure ToF LiDAR altitude mode: high precision (R=0.0025 m^2), 0-200 cm, immune to prop wash.
  if (droneState.rf_valid) {
    kfUpdate(droneState.z_rf_corrected_m, estConfig.r_lidar);
    droneState.active_alt_source = ALT_SOURCE_LIDAR;
  }

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

  // Optical Flow Measurement Update (High-rate 30-50 Hz velocity fusion)
  if (droneState.flow_enabled && flowSensor && flowSensor->isUpdated()) {
    updateOpticalFlowEstimator(flowSensor->getDeltaTimeSec(), is_airborne);
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
//  OPTICAL FLOW HORIZONTAL VELOCITY & POSITION ESTIMATOR
// =============================================================================
void updateOpticalFlowEstimator(float dt, bool is_airborne) {
  if (!flowSensor) return;

  uint8_t q     = flowSensor->getQuality();
  int32_t raw_x = flowSensor->getMotionX();
  int32_t raw_y = flowSensor->getMotionY();

  droneState.flow_quality      = q;
  droneState.flow_raw_x        = raw_x;
  droneState.flow_raw_y        = raw_y;
  droneState.flow_packets_seen = flowSensor->getPacketCount();

  if (flowSensor->hasLidar()) {
    droneState.flow_lidar_dist_m  = flowSensor->getLidarDistanceM();
    droneState.flow_lidar_quality = flowSensor->getLidarQuality();
  }

  // Discard measurements with poor tracking confidence
  if (q < (uint8_t)estConfig.flow_q_thresh) {
    droneState.flow_valid = false;
    return;
  }

  if (dt <= 0.001f || dt > 0.15f) dt = 0.02f;

  // 1. Determine current height above ground Z (m)
  // Prefer Sharp IR / fused altitude (or onboard Matek LiDAR if valid)
  float z_m = droneState.kf_z;
  if (flowSensor->hasLidar() && flowSensor->getLidarQuality() > 50 && flowSensor->getLidarDistanceM() >= 0.06f) {
    z_m = flowSensor->getLidarDistanceM();
  } else if (droneState.rf_valid && droneState.z_rf_corrected_m >= 0.06f) {
    z_m = droneState.z_rf_corrected_m;
  }
  z_m = constrain(z_m, FLOW_MIN_HEIGHT_M, FLOW_MAX_HEIGHT_M);

  // If drone is resting on ground, ground velocity is zero
  if (!is_airborne && z_m <= 0.08f) {
    droneState.flow_valid = true;
    droneState.flow_vel_body_x = 0.0f;
    droneState.flow_vel_body_y = 0.0f;
    droneState.flow_vel_north_mps = 0.0f;
    droneState.flow_vel_east_mps  = 0.0f;
    kfNorth.vel = 0.0f;
    kfEast.vel  = 0.0f;
    return;
  }

  // 1. Transform raw sensor counts into Drone Body Frame (Forward / Right) based on mounting yaw rotation
  // If mounted at rotation theta (deg) clockwise: apply inverse rotation R(-theta) to project into body frame
  float rot_rad = droneState.flow_rot_deg * DEG_TO_RAD;
  float cos_r   = cos(rot_rad);
  float sin_r   = sin(rot_rad);
  float count_body_fwd   =  (float)raw_x * cos_r - (float)raw_y * sin_r;
  float count_body_right =  (float)raw_x * sin_r + (float)raw_y * cos_r;

  // 2. Optical flow angular rates (rad/s) in Body Frame
  float omega_flow_x = (count_body_fwd   / dt) * FLOW_RAD_PER_COUNT;
  float omega_flow_y = (count_body_right / dt) * FLOW_RAD_PER_COUNT;

  // 3. Gyro derotation: subtract body angular rates (rad/s)
  // Pitch down (gy > 0) creates apparent backward motion along X
  // Roll right (gx > 0) creates apparent leftward motion along Y
  float gyro_pitch = droneState.gy_dps * DEG_TO_RAD;
  float gyro_roll  = droneState.gx_dps * DEG_TO_RAD;

  // Linear ground velocity in Body Frame (m/s)
  // Body X = Forward, Body Y = Right
  float v_body_fwd   = (omega_flow_x - gyro_pitch) * z_m;
  float v_body_right = (omega_flow_y - gyro_roll)  * z_m;

  v_body_fwd   = constrain(v_body_fwd,   -3.5f, 3.5f);
  v_body_right = constrain(v_body_right, -3.5f, 3.5f);

  droneState.flow_vel_body_x = v_body_fwd;
  droneState.flow_vel_body_y = v_body_right;

  // 4. Rotate Body Velocity into World NED Frame (North, East)
  float psi = droneState.yaw_deg * DEG_TO_RAD;
  float cos_p = cos(psi);
  float sin_p = sin(psi);

  float v_north =  v_body_fwd * cos_p - v_body_right * sin_p;
  float v_east  =  v_body_fwd * sin_p + v_body_right * cos_p;

  droneState.flow_vel_north_mps = v_north;
  droneState.flow_vel_east_mps  = v_east;
  droneState.flow_valid         = true;

  // 5. Measurement update on Horizontal 3-State Kalman Filter
  // Variance scales inversely with surface tracking quality
  float r_flow = constrain((estConfig.r_flow * 2.0f) / ((float)q / 255.0f), 0.015f, 0.40f);

  if (is_airborne) {
    kfNorth.updateVel(v_north, r_flow);
    kfEast.updateVel(v_east, r_flow);
  }
}

// =============================================================================
//  GPS POSITION & VELOCITY ESTIMATOR (Measurement Updates)
// =============================================================================
void updatePositionEstimator(float dt, bool is_armed) {
  if (!droneState.gps_enabled || !gpsSensor) {
    droneState.gps_accuracy_est_m = 99.9f;
    return;
  }

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
