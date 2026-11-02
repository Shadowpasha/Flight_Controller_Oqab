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
  ALT_SOURCE_SHARP = 0,
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

// GPS Serial2 Pins
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD   9600 // Standard 9600 baud for NEO-6M / NEO-7M / NEO-M8N NMEA

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

// --- Pure Sharp IR Altitude Control Config ---
#define USE_SHARP_ONLY_ALTITUDE   true   // Lock altitude estimation to high-speed Sharp IR (avoids baro downwash noise)
#define DEFAULT_TARGET_HEIGHT_CM  45.0f  // 45 cm hover height (centered cleanly in Sharp 10-60cm linear zone)
#define MAX_SAFE_CEILING_CM       72.0f  // Ceil threshold before Sharp saturation (80cm) triggering emergency descent

// Output Saturation Caps
#define MAX_PID_OUT          350.0f   // Max differential PWM for roll/pitch
#define MAX_YAW_OUT          200.0f   // Max yaw differential PWM
#define MAX_TARGET_RATE      400.0f   // Max target rate in deg/s

#endif // CONFIG_H
