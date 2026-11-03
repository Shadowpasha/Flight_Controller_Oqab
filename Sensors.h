#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_QMC5883P.h>
#include <ESP32_SharpIR.h>
#include <TinyGPS++.h>
#include "Config.h"

// =============================================================================
//  MODULAR SENSOR INTERFACES (Enables swapping any sensor in the future)
// =============================================================================

// --- 1. IMU Interface ---
class IMUInterface {
public:
  virtual ~IMUInterface() {}
  virtual bool begin() = 0;
  // Reads accelerometer (m/s^2) and raw gyroscope (rad/s) in Drone Body Frame:
  // Forward = +X, Right = +Y, Up = +Z
  virtual bool read(float &ax, float &ay, float &az, float &gx_rads, float &gy_rads, float &gz_rads) = 0;
};

// --- 2. Rangefinder Interface (Sharp IR, ToF, LiDAR, Ultrasonic) ---
class RangefinderInterface {
public:
  virtual ~RangefinderInterface() {}
  virtual bool begin() = 0;
  virtual float readDistanceCM() = 0;
};

// --- 3. Barometer Interface (BMP180, BMP280, BMP388, MS5611) ---
class BarometerInterface {
public:
  virtual ~BarometerInterface() {}
  virtual bool begin() = 0;
  virtual void end() {}
  virtual void update() = 0; // Non-blocking state machine update
  virtual bool isAvailable() const = 0;
  virtual float getPressurePa() const = 0;
  virtual float getRawAltitudeCm() const = 0;
  virtual float getBaselinePressurePa() const = 0;
  virtual void setBaselinePressurePa(float p0) = 0;
  virtual bool calibrateBaseline(int samples = 15) { return false; }
};

// --- 4. Magnetometer / Compass Interface (QMC5883P, QMC5883L, HMC5883L, IST8310) ---
class CompassInterface {
public:
  virtual ~CompassInterface() {}
  virtual bool begin() = 0;
  virtual bool isAvailable() const = 0;
  virtual const char* getDeviceName() const = 0;
  virtual bool readRaw(float &mx, float &my, float &mz) = 0;
};

// --- 6. Optical Flow Interface (Mateksys 3901-L0X / 3901-L1X via MSPv2) ---
class OpticalFlowInterface {
public:
  virtual ~OpticalFlowInterface() {}
  virtual bool begin() = 0;
  virtual void end() {}
  virtual void update() = 0;
  virtual bool isUpdated() = 0;
  virtual bool isConnected() const = 0;
  virtual uint8_t getQuality() const = 0;
  virtual int32_t getMotionX() const = 0;
  virtual int32_t getMotionY() const = 0;
  virtual float getDeltaTimeSec() const = 0;
  virtual bool hasLidar() const { return false; }
  virtual uint8_t getLidarQuality() const { return 0; }
  virtual float getLidarDistanceM() const { return 0.0f; }
  virtual uint32_t getPacketCount() const { return 0; }
  virtual uint32_t getChecksumErrors() const { return 0; }
};

// --- 5. GPS Interface (NEO-M8N, NEO-6M, Beitian, etc.) ---
class GPSInterface {
public:
  virtual ~GPSInterface() {}
  virtual bool begin() = 0;
  virtual void update() = 0;
  virtual bool hasFix() = 0;
  virtual bool isUpdated() = 0;
  virtual int getSatellites() = 0;
  virtual float getHDOP() = 0;
  virtual double getLatitude() = 0;
  virtual double getLongitude() = 0;
  virtual float getSpeedMps() = 0;
  virtual float getCourseDeg() = 0;
  virtual bool hasSpeedFix() = 0;
  virtual float getAltitudeM() { return 0.0f; }
  virtual bool hasAltitudeFix() { return false; }
  virtual bool isConnected() const { return false; }
  virtual uint32_t getCharsSeen() const { return 0; }
  virtual uint32_t getSentencesWithFix() const { return 0; }
  virtual uint32_t getChecksumErrors() const { return 0; }
};

// =============================================================================
//  CONCRETE SENSOR DRIVER IMPLEMENTATIONS
// =============================================================================

// --- Concrete IMU: MPU6050 ---
class MPU6050_IMU : public IMUInterface {
public:
  Adafruit_MPU6050 mpu;
  bool begin() override;
  bool read(float &ax, float &ay, float &az, float &gx_rads, float &gy_rads, float &gz_rads) override;
};

// --- Concrete Rangefinder: Sharp GP2Y0A IR ---
class SharpIR_Rangefinder : public RangefinderInterface {
private:
  int pin;
  ESP32_SharpIR sharp;
public:
  SharpIR_Rangefinder(int adc_pin);
  bool begin() override;
  float readDistanceCM() override;
};

// --- Concrete Barometer: BMP180 (I2C 0x77 via MPU6050 Bypass) ---
class BMP180_Barometer : public BarometerInterface {
private:
  bool available;
  enum BaroState { IDLE, WAIT_TEMP, WAIT_PRESS } state;
  unsigned long timer;
  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6, ut;
  int32_t b5;
  uint32_t up;
  float pressure_pa;
  float baseline_p0_pa;
  float raw_alt_cm;

public:
  BMP180_Barometer();
  bool begin() override;
  void end() override {
    gpsSerial.end();
  }
  void update() override;
  bool isAvailable() const override { return available; }
  float getPressurePa() const override { return pressure_pa; }
  float getRawAltitudeCm() const override { return raw_alt_cm; }
  float getBaselinePressurePa() const override { return baseline_p0_pa; }
  void setBaselinePressurePa(float p0) override { baseline_p0_pa = p0; }
  bool calibrateBaseline(int samples = 15) override;
  bool readSync(float* press_pa); // Calibration use
};

// --- Concrete Compass: GY-87 (QMC5883P / HMC5883L) ---
class GY87_Compass : public CompassInterface {
public:
  enum MagChipType { MAG_NONE = 0, MAG_HMC5883L, MAG_QMC5883P };
private:
  MagChipType chip_type;
  Adafruit_QMC5883P qmc;
  bool available;

public:
  GY87_Compass();
  bool begin() override;
  bool isAvailable() const override { return available; }
  const char* getDeviceName() const override;
  bool readRaw(float &mx, float &my, float &mz) override;
};

// --- Concrete GPS: NMEA GPS (NEO-6M / NEO-7M / NEO-M8N / BN-220) via TinyGPS++ ---
class NEO_M8N_GPS : public GPSInterface {
private:
  TinyGPSPlus gps;
  HardwareSerial gpsSerial;
  int rx_pin;
  int tx_pin;
  uint32_t baud;

  unsigned long last_char_time_ms;
  uint32_t total_chars_seen;
  bool new_data_available;

public:
  NEO_M8N_GPS(int rx, int tx, uint32_t baud_rate);
  bool begin() override;
  void update() override;
  bool isConnected() const override {
    return (millis() - last_char_time_ms < 3000) && (total_chars_seen > 10);
  }
  bool hasFix() override {
    // True GNSS fix determined by TinyGPS++ valid coordinates fresh within 3s
    return gps.location.isValid() && (gps.location.age() < 3000);
  }
  bool isUpdated() override {
    if (new_data_available || (gps.location.isValid() && gps.location.age() < 250)) {
      new_data_available = false;
      return true;
    }
    return false;
  }
  int getSatellites() override {
    return gps.satellites.isValid() ? gps.satellites.value() : 0;
  }
  float getHDOP() override {
    return gps.hdop.isValid() ? (gps.hdop.value() / 100.0f) : 99.9f;
  }
  double getLatitude() override {
    return gps.location.isValid() ? gps.location.lat() : 0.0;
  }
  double getLongitude() override {
    return gps.location.isValid() ? gps.location.lng() : 0.0;
  }
  float getSpeedMps() override {
    return gps.speed.isValid() ? (float)gps.speed.mps() : 0.0f;
  }
  float getCourseDeg() override {
    return gps.course.isValid() ? (float)gps.course.deg() : 0.0f;
  }
  bool hasSpeedFix() override {
    return (gps.speed.isValid() && hasFix());
  }
  float getAltitudeM() override {
    return gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
  }
  bool hasAltitudeFix() override {
    return (gps.altitude.isValid() && hasFix());
  }
  uint32_t getCharsSeen() const override { return total_chars_seen; }
  uint32_t getSentencesWithFix() const override { return const_cast<TinyGPSPlus&>(gps).sentencesWithFix(); }
  uint32_t getChecksumErrors() const override { return const_cast<TinyGPSPlus&>(gps).failedChecksum(); }
  TinyGPSPlus& getRawGPS() { return gps; }
};

// --- Concrete Optical Flow: Mateksys 3901-L0X / 3901-L1X (MSPv2 Protocol) ---
class Matek3901_Flow : public OpticalFlowInterface {
private:
  HardwareSerial flowSerial;
  int rx_pin;
  int tx_pin;
  uint32_t baud;

  enum ParserState {
    P_IDLE,
    P_HEADER_X,
    P_HEADER_DIR,
    P_FLAG,
    P_CMD_LOW,
    P_CMD_HIGH,
    P_SIZE_LOW,
    P_SIZE_HIGH,
    P_PAYLOAD,
    P_CRC
  };

  ParserState parse_state;
  uint8_t  msg_flag;
  uint16_t msg_cmd;
  uint16_t msg_size;
  uint16_t payload_idx;
  uint8_t  payload[32];
  uint8_t  calculated_crc;

  uint8_t  quality;
  int32_t  motion_x;
  int32_t  motion_y;
  uint8_t  lidar_quality;
  int32_t  distance_mm;

  unsigned long last_flow_time_ms;
  unsigned long last_packet_time_ms;
  float    dt_sec;
  bool     new_data_available;
  uint32_t total_packets;
  uint32_t crc_errors;

  void parseByte(uint8_t b);
  void dispatchMessage();

public:
  Matek3901_Flow(int rx, int tx, uint32_t baud_rate);
  bool begin() override;
  void end() override;
  void update() override;
  bool isUpdated() override;
  bool isConnected() const override { return (millis() - last_packet_time_ms < 1000) && (total_packets > 0); }
  uint8_t getQuality() const override { return quality; }
  int32_t getMotionX() const override { return motion_x; }
  int32_t getMotionY() const override { return motion_y; }
  float getDeltaTimeSec() const override { return dt_sec; }
  bool hasLidar() const override { return (millis() - last_packet_time_ms < 1000) && (distance_mm > 0); }
  uint8_t getLidarQuality() const override { return lidar_quality; }
  float getLidarDistanceM() const override { return (distance_mm > 0) ? (distance_mm * 0.001f) : 0.0f; }
  uint32_t getPacketCount() const override { return total_packets; }
  uint32_t getChecksumErrors() const override { return crc_errors; }
};

// =============================================================================
//  GLOBAL SENSOR ACCESSORS
// =============================================================================
extern IMUInterface*         imuSensor;
extern RangefinderInterface* rangeSensor;
extern BarometerInterface*   baroSensor;
extern CompassInterface*     compassSensor;
extern GPSInterface*         gpsSensor;
extern OpticalFlowInterface* flowSensor;

void initSensors();

#endif // SENSORS_H
