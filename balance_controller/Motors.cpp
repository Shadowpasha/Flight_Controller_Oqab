#include "Motors.h"

void dualPrintln(const String& msg);
void disarm(const char* reason);

static Servo esc1, esc2, esc3, esc4;

void initMotors() {
  // 1. Force signal pins LOW as output first to prevent floating noise spikes
  pinMode(ESC1_PIN, OUTPUT); digitalWrite(ESC1_PIN, LOW);
  pinMode(ESC2_PIN, OUTPUT); digitalWrite(ESC2_PIN, LOW);
  pinMode(ESC3_PIN, OUTPUT); digitalWrite(ESC3_PIN, LOW);
  pinMode(ESC4_PIN, OUTPUT); digitalWrite(ESC4_PIN, LOW);
  delay(10);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(ESC_FREQ);
  esc2.setPeriodHertz(ESC_FREQ);
  esc3.setPeriodHertz(ESC_FREQ);
  esc4.setPeriodHertz(ESC_FREQ);

  // Attach and immediately override ESP32Servo's default 1500us with 1000us idle
  esc1.attach(ESC1_PIN, 1000, 2000);
  esc1.writeMicroseconds(1000);

  esc2.attach(ESC2_PIN, 1000, 2000);
  esc2.writeMicroseconds(1000);

  esc3.attach(ESC3_PIN, 1000, 2000);
  esc3.writeMicroseconds(1000);

  esc4.attach(ESC4_PIN, 1000, 2000);
  esc4.writeMicroseconds(1000);

  stopMotors();
}

void writeMotors(int m1, int m2, int m3, int m4) {
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

void stopMotors() {
  writeMotors(1000, 1000, 1000, 1000);
}

void applyMotorMixing(int base_throttle_us, float out_roll, float out_pitch, float out_yaw, bool sim_mode) {
  out_roll  = constrain(out_roll,  -MAX_PID_OUT, MAX_PID_OUT);
  out_pitch = constrain(out_pitch, -MAX_PID_OUT, MAX_PID_OUT);
  out_yaw   = constrain(out_yaw,   -MAX_YAW_OUT, MAX_YAW_OUT);

  // --- Quad-X Motor Mixer ---
  // M1 Front Right (CCW)
  // M2 Rear Right  (CW)
  // M3 Rear Left   (CCW)
  // M4 Front Left  (CW)
  int pwm1 = base_throttle_us - (int)out_roll - (int)out_pitch - (int)out_yaw;
  int pwm2 = base_throttle_us - (int)out_roll + (int)out_pitch + (int)out_yaw;
  int pwm3 = base_throttle_us + (int)out_roll + (int)out_pitch - (int)out_yaw;
  int pwm4 = base_throttle_us + (int)out_roll - (int)out_pitch + (int)out_yaw;

  pwm1 = constrain(pwm1, MIN_ARMED_PWM, MAX_MOTOR_PWM);
  pwm2 = constrain(pwm2, MIN_ARMED_PWM, MAX_MOTOR_PWM);
  pwm3 = constrain(pwm3, MIN_ARMED_PWM, MAX_MOTOR_PWM);
  pwm4 = constrain(pwm4, MIN_ARMED_PWM, MAX_MOTOR_PWM);

  if (sim_mode) {
    stopMotors();
  } else {
    writeMotors(pwm1, pwm2, pwm3, pwm4);
  }
}

void calibrateESCs() {
  disarm("ESC Calibration Mode");
  dualPrintln(">>> MAX THROTTLE (2000us) - Connect battery");
  writeMotors(2000, 2000, 2000, 2000);
  delay(4000);
  dualPrintln(">>> MIN THROTTLE (1000us)...");
  stopMotors();
  delay(4000);
  dualPrintln(">>> ESC Calibration complete.\n");
}
