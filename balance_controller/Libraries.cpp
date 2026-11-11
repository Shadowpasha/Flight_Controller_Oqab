// =============================================================================
//  Local Library Unity Compilation Unit
//  Compiles all vendor libraries located in 'libraries/' in a single file.
// =============================================================================

#include "libraries/Adafruit_BusIO_Register.cpp"
#include "libraries/Adafruit_GenericDevice.cpp"
#include "libraries/Adafruit_I2CDevice.cpp"
#include "libraries/Adafruit_SPIDevice.cpp"
#include "libraries/Adafruit_Sensor.cpp"
#include "libraries/Adafruit_MPU6050.cpp"
#include "libraries/Adafruit_QMC5883P.cpp"
#include "libraries/ESP32_SharpIR.cpp"
#include "libraries/TinyGPS.cpp"
#include "libraries/ESP32PWM.cpp"

// Disambiguate file-scope TAG symbol between ESP32PWM and ESP32Servo
#define TAG SERVO_TAG
#include "libraries/ESP32Servo.cpp"
#undef TAG

