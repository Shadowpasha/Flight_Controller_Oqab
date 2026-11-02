#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "Config.h"

// =============================================================================
//  MOTOR & ESC HARDWARE INTERFACE
// =============================================================================

void initMotors();
void writeMotors(int m1, int m2, int m3, int m4);
void stopMotors();
void applyMotorMixing(int base_throttle_us, float out_roll, float out_pitch, float out_yaw, bool sim_mode);
void calibrateESCs();

#endif // MOTORS_H
