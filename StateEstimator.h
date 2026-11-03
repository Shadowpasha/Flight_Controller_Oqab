#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"

// =============================================================================
//  ESTIMATED DRONE STATE & NAVIGATION DATA
// =============================================================================

struct DroneState {
  // --- Attitude (Degrees & deg/s) ---
  float roll_deg;
  float pitch_deg;
  float yaw_deg; // True North referenced

  float gx_dps;
  float gy_dps;
  float gz_dps;

  float ax_body;
  float ay_body;
  float az_body;

  // --- Altitude & Vertical Velocity ---
  float current_height_cm;  // Fused altitude AGL (cm)
  float raw_height_cm;      // Instantaneous Sharp IR (cm)
  float climb_rate_cms;     // Vertical velocity (cm/s)
  float climb_rate_filt;    // Filtered vertical velocity (cm/s)
  float ground_height_cm;   // Ground clearance baseline (cm)
  AltSource active_alt_source;

  // --- 3-State Vertical Kalman Filter States ---
  float kf_z;   // Altitude (meters)
  float kf_vz;  // Vertical velocity (m/s)
  float kf_ba;  // Vertical accel bias (m/s^2)

  // --- Rangefinder & Baro Fusion Internals ---
  float z_rf_corrected_m;
  bool  rf_valid;
  float baro_ground_offset_m;
  float baro_raw_alt_cm;
  float baro_pressure_pa;

  // --- Compass ---
  bool  has_compass;
  float mag_heading_deg;
  float mag_declination_deg;
  float mag_offset_x;
  float mag_offset_y;
  float mag_offset_z;

  // --- GPS & Optical Flow Subsystem Toggles ---
  bool   gps_enabled;
  bool   flow_enabled;
  bool   flow_valid;
  uint8_t flow_quality;
  int32_t flow_raw_x;
  int32_t flow_raw_y;
  float   flow_vel_body_x;      // Body forward velocity from flow (m/s)
  float   flow_vel_body_y;      // Body rightward velocity from flow (m/s)
  float   flow_vel_north_mps;   // Earth-frame North velocity from flow (m/s)
  float   flow_vel_east_mps;    // Earth-frame East velocity from flow (m/s)
  float   flow_lidar_dist_m;    // Matek onboard LiDAR height (m)
  uint8_t flow_lidar_quality;
  uint32_t flow_packets_seen;

  // --- Home Reference & Dead Reckoning Origin ---
  enum HomeOriginType { HOME_BOOT = 0, HOME_ARM = 1 };
  HomeOriginType home_type;
  bool   home_is_set;
  bool   has_gps_home;
  double gps_home_lat;
  double gps_home_lon;
  float  home_alt_baseline_m;

  // --- Dead Reckoning 3D Position Displacement from Home (m) ---
  float  dr_pos_x_m;        // North displacement from Home (m)
  float  dr_pos_y_m;        // East displacement from Home (m)
  float  dr_pos_z_m;        // Altitude displacement above Home ground (m)
  float  pos_north_m;       // Fused North position (m)
  float  pos_east_m;        // Fused East position (m)

  // --- Dead Reckoning 3D Velocity (m/s) ---
  float  dr_vel_x_mps;      // North velocity (m/s)
  float  dr_vel_y_mps;      // East velocity (m/s)
  float  dr_vel_z_mps;      // Vertical velocity (m/s)
  float  vel_north_filt;    // Filtered North velocity (m/s)
  float  vel_east_filt;     // Filtered East velocity (m/s)
  unsigned long last_gps_fix_millis;

  // --- 3-State Horizontal Kalman Filter States (North & East) ---
  float  accel_north_world; // World-frame horizontal accel (m/s^2)
  float  accel_east_world;
  float  kf_pos_n;          // Fused North position (m)
  float  kf_vel_n;          // Fused North velocity (m/s)
  float  kf_ba_n;           // Estimated North accel bias (m/s^2)
  float  kf_pos_e;          // Fused East position (m)
  float  kf_vel_e;          // Fused East velocity (m/s)
  float  kf_ba_e;           // Estimated East accel bias (m/s^2)

  // --- Dead Reckoning Distance & Accuracy ---
  float  dist_from_home_2d_m; // 2D horizontal distance from Home: sqrt(dx^2 + dy^2)
  float  dist_from_home_3d_m; // 3D spatial distance from Home: sqrt(dx^2 + dy^2 + dz^2)
  float  dist_2d_m;           // (alias)
  float  dist_3d_m;           // (alias)
  float  speed_3d_mps;        // Total 3D velocity magnitude (m/s)
  float  raw_pos_north_m;     // Instantaneous GPS North displacement from Home (m)
  float  raw_pos_east_m;      // Instantaneous GPS East displacement from Home (m)
  float  gps_accuracy_est_m;  // Estimated horizontal accuracy (m) based on HDOP

  // --- Calibration Offsets ---
  float cal_gyro_x;
  float cal_gyro_y;
  float cal_gyro_z;
  float cal_roll_offset;
  float cal_pitch_offset;
  float cal_accel_fwd_offset;
  float cal_accel_right_offset;
  bool  is_calibrated;

  // --- Airborne Detection ---
  bool  is_airborne;
  float az_filt;
  unsigned long above_thresh_since_ms;
};

extern DroneState droneState;

// =============================================================================
//  ESTIMATOR FUNCTIONS
// =============================================================================
void initStateEstimator();
void calibrateAllSensors();
void setHomeToBootPoint();
void setHomeToArmPoint();
const char* getHomeOriginTypeName();
void updateAttitudeEstimator(float dt);
void updateAltitudeEstimator(float dt, bool is_airborne, bool sim_mode);
void updateHorizontalEstimator(float dt, bool is_airborne, bool sim_mode);
void updatePositionEstimator(float dt, bool is_armed);
void updateCompassEstimator();
void calibrateCompassRoutine();
void resetEstimatorStates();
void setGPSEnabled(bool enable);
void setFlowEnabled(bool enable);
void updateOpticalFlowEstimator(float dt, bool is_airborne);

#endif // STATE_ESTIMATOR_H
