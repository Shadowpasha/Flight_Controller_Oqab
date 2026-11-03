#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
//  SYSTEM CONFIGURATION & TUNING CONSTANTS
// =============================================================================

// --- Controller Options ---
enum ControllerType {
  CONTROLLER_PID = 0,    // Standard Cascaded / Angle / Rate PID
  CONTROLLER_SMC = 1,    // Sliding Mode Control (SMC with boundary layer)
  CONTROLLER_ADRC = 2,   // Linear Active Disturbance Rejection Control (LADRC)
  CONTROLLER_MDBCI = 3,  // Modified Dual Boundary Conditional Integration Backstepping (ISA 2020 Eq. 44)
  CONTROLLER_MDBCIS = 4, // MDBCI + Jerk-Mitigating Switched Controller (ISA 2020 Section 4.2)
  CONTROLLER_CDI = 5,    // Conditional Integration Backstepping (Seshagiri & Khalil 2005)
  CONTROLLER_CVI = 6,    // Conventional Integral Backstepping (Constant lambda_m)
  CONTROLLER_NOI = 7,    // No-Integration Backstepping (lambda = 0)
  CONTROLLER_VIABC = 8,  // Variable Integral Adaptive Backstepping (IEEE Access 2024)
  CONTROLLER_BACKSTEPPING = CONTROLLER_MDBCI // Alias for convenience
};-

// Default active controller at boot (can also be switched at runtime via 'ctrl pid|smc|adrc|mdbci|mdbcis|cdi|cvi|noi|viabc')
#define DEFAULT_CONTROLLER CONTROLLER_PID

// --- Flight Modes ---
enum FlightMode {
  MODE_CASCADE = 0, // Outer Angle P -> Inner Rate Loop (Default)
  MODE_ANGLE   = 1, // Single-loop Angle Loop
  MODE_RATE    = 2  // Single-loop Rate Loop (Acro Mode)
};

// --- Auto-Flight Mission Stages ---
enum FlightStage {
  STAGE_IDLE = 0,
  STAGE_RAMP_UP,
  STAGE_HOVER,
  STAGE_LANDING,
  STAGE_TOUCHDOWN
};

// --- Active Altitude Sensor Source ---
enum AltSource {
  ALT_SOURCE_LIDAR = 0,
  ALT_SOURCE_SHARP,
  ALT_SOURCE_BARO_ACC
};

// --- Hardware Pin Definitions ---
// ESC Pins (Quad-X configuration)
#define ESC1_PIN 14 // Front Right (M1, CCW)
#define ESC2_PIN 27 // Rear Right  (M2, CW)
#define ESC3_PIN 26 // Rear Left   (M3, CCW)
#define ESC4_PIN 25 // Front Left  (M4, CW)
#define ESC_FREQ 250 // 250 Hz PWM refresh rate

// Distance & Compass / Baro Pins
#define SHARP_IR_PIN 34 // ADC1_CH6 analog input

// Sensor Serial2 Pins (Shared UART port for GPS or Matek Optical Flow)
#define SENSOR_UART_RX_PIN  16
#define SENSOR_UART_TX_PIN  17

// GPS Subsystem Toggle (Legacy)
#define USE_GPS             false // Set false to disable GPS entirely
#define GPS_RX_PIN          SENSOR_UART_RX_PIN
#define GPS_TX_PIN          SENSOR_UART_TX_PIN
#define GPS_BAUD            9600  // 9600 baud for NMEA

// Optical Flow Subsystem (Mateksys 3901-L0X / 3901-L1X via MSPv2)
#define USE_OPTICAL_FLOW    true   // Master toggle: Mateksys Optical Flow as primary horizontal estimator
#define FLOW_RX_PIN         SENSOR_UART_RX_PIN
#define FLOW_TX_PIN         SENSOR_UART_TX_PIN
#define FLOW_BAUD           115200 // Matek 3901 MSPv2 default baud rate
#define FLOW_RAD_PER_COUNT  0.0038f // PMW3901 42 deg FOV angular scale (rad/count)
#define FLOW_QUALITY_THRESH 35     // Minimum tracking quality threshold (0-255)
#define FLOW_MIN_HEIGHT_M   0.05f  // Minimum ground distance (5cm) for flow calculation
#define FLOW_MAX_HEIGHT_M   3.00f  // Maximum reliable range (3.0m)

// Bluetooth Device Name
#define BT_DEVICE_NAME "F330_Flight_Controller"

// --- Loop Timing ---
#define LOOP_RATE_HZ         250
#define LOOP_PERIOD_US       4000     // 4000 us = 250 Hz
#define ALT_LOOP_PERIOD_MS   20       // 20 ms = 50 Hz
#define GPS_LOOP_PERIOD_MS   100      // 100 ms = 10 Hz

// --- Throttle & Safety Limits ---
#define MIN_ARMED_PWM        1080     // Idle spin floor
#define MAX_MOTOR_PWM        1920     // Ceiling giving headroom for differential control
#define PID_ACTIVE_THROTTLE  1200     // Closed loop starts early during takeoff
#define TAKEOFF_THROTTLE     1360     // Throttle threshold for airborne detection (~1360us)
#define MAX_TAKEOFF_RAMP_PWM 1500     // Upper safety ceiling for automatic liftoff climb ramp
#define THROTTLE_RAMP_RATE   40.0f    // Slow, ultra-smooth climb ramp rate (~40 us/s; ~6.5s from idle to liftoff)
#define LAND_RAMP_RATE       35.0f    // Gentle landing descent rate (~35 us/s)
#define MAX_TILT_ANGLE_DEG   40.0f    // Instant crash disarm threshold
#define TIMEOUT_SAFETY_MS    30000    // 30s safety watchdog without serial command

// --- Pure ToF LiDAR Altitude Control Config ---
#define USE_LIDAR_ONLY_ALTITUDE   true   // Lock vertical state estimation exclusively to Mateksys ToF LiDAR (0-200cm, immune to downwash/baro drift)
#define USE_SHARP_ONLY_ALTITUDE   false  // Legacy Sharp GP2Y0A analog IR rangefinder (dewired)
#define DEFAULT_TARGET_HEIGHT_CM  45.0f  // 45 cm hover height
#define MAX_SAFE_CEILING_CM       185.0f // Safety ceiling before Matek 3901-L0X VL53 ToF saturation (200cm) triggering emergency descent

// Output Saturation Caps
#define MAX_PID_OUT          350.0f   // Max differential PWM for roll/pitch
#define MAX_YAW_OUT          200.0f   // Max yaw differential PWM
#define MAX_TARGET_RATE      400.0f   // Max target rate in deg/s

#endif // CONFIG_H
