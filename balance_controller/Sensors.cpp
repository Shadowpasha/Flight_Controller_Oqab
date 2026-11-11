#include "Sensors.h"

// Forward declaration of dualPrintln for logging
void dualPrintln(const String& msg);

// Global Sensor Pointers
IMUInterface*         imuSensor     = nullptr;
RangefinderInterface* rangeSensor   = nullptr;
BarometerInterface*   baroSensor    = nullptr;
CompassInterface*     compassSensor = nullptr;
GPSInterface*         gpsSensor     = nullptr;

// Static sensor instances
static MPU6050_IMU         static_mpu;
static SharpIR_Rangefinder static_sharp(SHARP_IR_PIN);
static BMP180_Barometer    static_bmp;
static GY87_Compass        static_compass;
static NEO_M8N_GPS         static_gps(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

// =============================================================================
//  MPU6050 IMU IMPLEMENTATION
// =============================================================================
bool MPU6050_IMU::begin() {
  if (!mpu.begin()) {
    return false;
  }
  mpu.setI2CBypass(true);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  delay(100);
  return true;
}

bool MPU6050_IMU::read(float &ax, float &ay, float &az, float &gx_rads, float &gy_rads, float &gz_rads) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float raw_ax = a.acceleration.x;
  float raw_ay = a.acceleration.y;
  float raw_az = a.acceleration.z;
  float raw_gx = g.gyro.x;
  float raw_gy = g.gyro.y;
  float raw_gz = g.gyro.z;

  switch (orientation) {
    case 1:
      // ROTATION_NONE (Standard FRD/NED: Fwd=+X, Right=+Y, Up=+Z)
      ax = raw_ax;  ay = raw_ay;  az = raw_az;
      gx_rads = raw_gx;  gy_rads = raw_gy;  gz_rads = raw_gz;
      break;

    case 2:
      // ROTATION_YAW_90 CW (Fwd=-Y, Right=+X, Up=+Z)
      ax = -raw_ay; ay = raw_ax;  az = raw_az;
      gx_rads = -raw_gy; gy_rads = raw_gx;  gz_rads = raw_gz;
      break;

    case 3:
      // ROTATION_YAW_180 CW (Fwd=-X, Right=-Y, Up=+Z)
      ax = -raw_ax; ay = -raw_ay; az = raw_az;
      gx_rads = -raw_gx; gy_rads = -raw_gy; gz_rads = raw_gz;
      break;

    case 4:
      // ROTATION_YAW_270 CW (Fwd=+Y, Right=-X, Up=+Z)
      ax = raw_ay;  ay = -raw_ax; az = raw_az;
      gx_rads = raw_gy;  gy_rads = -raw_gx; gz_rads = raw_gz;
      break;

    case 5:
      // ROTATION_PITCH_180 (Upside down: Fwd=+X, Right=-Y, Up=-Z)
      ax = raw_ax;  ay = -raw_ay; az = -raw_az;
      gx_rads = raw_gx;  gy_rads = -raw_gy; gz_rads = -raw_gz;
      break;

    case 0:
    default:
      // ROTATION_DEFAULT (Current physical hardware baseline - 100% backward compatible):
      // Forward (X_body) = +Y_mpu
      // Right   (Y_body) = +X_mpu
      // Up      (Z_body) = +Z_mpu
      ax = raw_ay;
      ay = raw_ax;
      az = raw_az;
      gx_rads = raw_gy;
      gy_rads = raw_gx;
      gz_rads = raw_gz;
      break;
  }

  return true;
}

// =============================================================================
//  SHARP IR RANGEFINDER IMPLEMENTATION
// =============================================================================
SharpIR_Rangefinder::SharpIR_Rangefinder(int adc_pin)
  : pin(adc_pin), sharp(adc_pin) {}

bool SharpIR_Rangefinder::begin() {
  pinMode(pin, INPUT);
#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
#endif
  return true;
}

float SharpIR_Rangefinder::readDistanceCM() {
  // Fast non-blocking ADC read (~30us, eliminating 50ms delay(2)*25 from library)
  int rawADC = analogRead(pin);
  if (rawADC < 100) return 80.0f;

  static float filt_adc = -1.0f;
  if (filt_adc < 0) filt_adc = (float)rawADC;
  else filt_adc += 0.35f * ((float)rawADC - filt_adc);

  // Calibrated Sharp GP2Y0A power regression: jarak = (ADC / 22581.6)^(1 / -0.9184)
  // 1 / -0.9184 = -1.08885
  float d_cm = powf(filt_adc / 22581.6f, -1.08885f);
  return constrain(d_cm, 6.0f, 80.0f);
}

// =============================================================================
//  BMP180 BAROMETER IMPLEMENTATION
// =============================================================================
BMP180_Barometer::BMP180_Barometer()
  : available(false), state(IDLE), timer(0),
    pressure_pa(101325.0f), baseline_p0_pa(101325.0f), raw_alt_cm(0.0f) {}

bool BMP180_Barometer::begin() {
  Wire.beginTransmission(0x77);
  Wire.write(0xD0);
  if (Wire.endTransmission() != 0) {
    available = false;
    return false;
  }

  Wire.requestFrom(0x77, 1);
  if (Wire.available()) {
    Wire.read(); // Chip ID (0x55)
  }

  // Read 22 bytes of calibration coefficients (0xAA to 0xBF)
  Wire.beginTransmission(0x77);
  Wire.write(0xAA);
  Wire.endTransmission();

  Wire.requestFrom(0x77, 22);
  if (Wire.available() < 22) {
    available = false;
    return false;
  }

  ac1 = (Wire.read() << 8) | Wire.read();
  ac2 = (Wire.read() << 8) | Wire.read();
  ac3 = (Wire.read() << 8) | Wire.read();
  ac4 = (Wire.read() << 8) | Wire.read();
  ac5 = (Wire.read() << 8) | Wire.read();
  ac6 = (Wire.read() << 8) | Wire.read();
  b1  = (Wire.read() << 8) | Wire.read();
  b2  = (Wire.read() << 8) | Wire.read();
  mb  = (Wire.read() << 8) | Wire.read();
  mc  = (Wire.read() << 8) | Wire.read();
  md  = (Wire.read() << 8) | Wire.read();

  available = true;
  return true;
}

bool BMP180_Barometer::readSync(float* press_pa) {
  if (!available) return false;

  // 1. Trigger Temperature
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x77);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(0x77, 2);
  if (Wire.available() < 2) return false;
  uint16_t ut_val = (Wire.read() << 8) | Wire.read();

  int32_t x1 = ((int32_t)ut_val - (int32_t)ac6) * (int32_t)ac5 >> 15;
  int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
  int32_t b5_val = x1 + x2;

  // 2. Trigger Pressure (oss = 1: 7.5ms)
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x34 + (1 << 6));
  Wire.endTransmission();
  delay(8);

  Wire.beginTransmission(0x77);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(0x77, 3);
  if (Wire.available() < 3) return false;
  uint32_t up_val = (((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | (uint32_t)Wire.read()) >> 7;

  int32_t b6 = b5_val - 4000;
  x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((int32_t)ac1 * 4 + x3) << 1) + 2) >> 2;
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up_val - b3) * (50000 >> 1);
  int32_t p;
  if (b7 < 0x80000000) {
    p = (b7 << 1) / b4;
  } else {
    p = (b7 / b4) << 1;
  }
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);

  *press_pa = (float)p;
  return true;
}

bool BMP180_Barometer::calibrateBaseline(int samples) {
  if (!available) return false;
  float sum_press = 0.0f;
  int p_count = 0;
  for (int i = 0; i < samples; i++) {
    float sample_p = 0.0f;
    if (readSync(&sample_p)) {
      sum_press += sample_p;
      p_count++;
    }
    delay(15);
  }
  if (p_count > 0) {
    baseline_p0_pa = sum_press / (float)p_count;
    pressure_pa    = baseline_p0_pa;
    raw_alt_cm     = 0.0f;
    return true;
  }
  return false;
}

void BMP180_Barometer::update() {
  if (!available) return;

  unsigned long now = millis();

  switch (state) {
    case IDLE:
      // Start Temperature conversion
      Wire.beginTransmission(0x77);
      Wire.write(0xF4);
      Wire.write(0x2E);
      Wire.endTransmission();
      timer = now;
      state = WAIT_TEMP;
      break;

    case WAIT_TEMP:
      if (now - timer >= 5) {
        Wire.beginTransmission(0x77);
        Wire.write(0xF6);
        Wire.endTransmission();
        Wire.requestFrom(0x77, 2);
        if (Wire.available() >= 2) {
          ut = (Wire.read() << 8) | Wire.read();
          int32_t x1 = ((int32_t)ut - (int32_t)ac6) * (int32_t)ac5 >> 15;
          int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
          b5 = x1 + x2;
        }
        // Start Pressure conversion
        Wire.beginTransmission(0x77);
        Wire.write(0xF4);
        Wire.write(0x34 + (1 << 6));
        Wire.endTransmission();
        timer = now;
        state = WAIT_PRESS;
      }
      break;

    case WAIT_PRESS:
      if (now - timer >= 8) {
        Wire.beginTransmission(0x77);
        Wire.write(0xF6);
        Wire.endTransmission();
        Wire.requestFrom(0x77, 3);
        if (Wire.available() >= 3) {
          up = (((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | (uint32_t)Wire.read()) >> 7;

          int32_t b6 = b5 - 4000;
          int32_t x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
          int32_t x2 = (ac2 * b6) >> 11;
          int32_t x3 = x1 + x2;
          int32_t b3 = ((((int32_t)ac1 * 4 + x3) << 1) + 2) >> 2;
          x1 = (ac3 * b6) >> 13;
          x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
          x3 = ((x1 + x2) + 2) >> 2;
          uint32_t b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
          uint32_t b7 = ((uint32_t)up - b3) * (50000 >> 1);
          int32_t p;
          if (b7 < 0x80000000) {
            p = (b7 << 1) / b4;
          } else {
            p = (b7 / b4) << 1;
          }
          x1 = (p >> 8) * (p >> 8);
          x1 = (x1 * 3038) >> 16;
          x2 = (-7357 * p) >> 16;
          p = p + ((x1 + x2 + 3791) >> 4);

          pressure_pa = (float)p;

          if (baseline_p0_pa > 80000.0f) {
            float raw_alt_m = 44330.0f * (1.0f - powf(pressure_pa / baseline_p0_pa, 0.1902949f));
            raw_alt_cm = raw_alt_m * 100.0f;
          }
        }
        state = IDLE;
      }
      break;
  }
}

// =============================================================================
//  GY-87 COMPASS IMPLEMENTATION
// =============================================================================
GY87_Compass::GY87_Compass()
  : chip_type(MAG_NONE), available(false) {}

bool GY87_Compass::begin() {
  // Configure MPU6050 AUX I2C bypass
  Wire.beginTransmission(0x68);
  Wire.write(0x6A); // USER_CTRL
  Wire.write(0x00); // Disable I2C Master mode
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x37); // INT_PIN_CFG
  Wire.write(0x02); // Enable I2C Bypass multiplexer
  Wire.endTransmission();
  delay(15);

  // 1. Try QMC5883P @ 0x2C
  if (qmc.begin(0x2C, &Wire)) {
    chip_type = MAG_QMC5883P;
    available = true;
    qmc.setMode(QMC5883P_MODE_NORMAL);
    qmc.setODR(QMC5883P_ODR_100HZ);
    qmc.setOSR(QMC5883P_OSR_4);
    qmc.setRange(QMC5883P_RANGE_8G);
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    return true;
  }

  // 2. Try HMC5883L @ 0x1E
  Wire.beginTransmission(0x1E);
  if (Wire.endTransmission() == 0) {
    chip_type = MAG_HMC5883L;
    available = true;
    Wire.beginTransmission(0x1E);
    Wire.write(0x00);
    Wire.write(0x78);
    Wire.endTransmission();

    Wire.beginTransmission(0x1E);
    Wire.write(0x01);
    Wire.write(0x20);
    Wire.endTransmission();

    Wire.beginTransmission(0x1E);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();
    return true;
  }

  chip_type = MAG_NONE;
  available = false;
  return false;
}

const char* GY87_Compass::getDeviceName() const {
  if (chip_type == MAG_QMC5883P) return "QMC5883P (0x2C)";
  if (chip_type == MAG_HMC5883L) return "HMC5883L (0x1E)";
  return "OFFLINE";
}

bool GY87_Compass::readRaw(float &mx, float &my, float &mz) {
  if (!available) return false;

  if (chip_type == MAG_QMC5883P) {
    int16_t x, y, z;
    if (qmc.getRawMagnetic(&x, &y, &z)) {
      mx = (float)x;
      my = (float)y;
      mz = (float)z;
      return true;
    }
  } else if (chip_type == MAG_HMC5883L) {
    Wire.beginTransmission(0x1E);
    Wire.write(0x03);
    Wire.endTransmission();

    Wire.requestFrom(0x1E, 6);
    if (Wire.available() >= 6) {
      int16_t x = (Wire.read() << 8) | Wire.read();
      int16_t z = (Wire.read() << 8) | Wire.read();
      int16_t y = (Wire.read() << 8) | Wire.read();
      mx = (float)x;
      my = (float)y;
      mz = (float)z;
      return true;
    }
  }
  return false;
}

// =============================================================================
//  NMEA GPS IMPLEMENTATION (NEO-6M / NEO-7M / NEO-M8N / BN-220 via TinyGPS++)
// =============================================================================

NEO_M8N_GPS::NEO_M8N_GPS(int rx, int tx, uint32_t baud_rate)
  : gpsSerial(2), rx_pin(rx), tx_pin(tx), baud(baud_rate),
    last_char_time_ms(0), total_chars_seen(0), new_data_available(false) {}

bool NEO_M8N_GPS::begin() {
  gpsSerial.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  delay(10);
  return true;
}

void NEO_M8N_GPS::update() {
  // Non-blocking drain of hardware UART FIFO into TinyGPS++
  while (gpsSerial.available() > 0) {
    char c = (char)gpsSerial.read();
    last_char_time_ms = millis();
    total_chars_seen++;
    if (gps.encode(c)) {
      new_data_available = true;
    }
  }
}

// =============================================================================
//  MATEKSYS 3901-L0X / 3901-L1X OPTICAL FLOW & LIDAR (MSPV2 DRIVER)
// =============================================================================
static uint8_t flow_crc8_dvb_s2(uint8_t crc, uint8_t ch) {
  crc ^= ch;
  for (int i = 0; i < 8; ++i) {
    if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
    else crc = (crc << 1);
  }
  return crc;
}

Matek3901_Flow::Matek3901_Flow(int rx, int tx, uint32_t baud_rate)
  : flowSerial(2), rx_pin(rx), tx_pin(tx), baud(baud_rate),
    parse_state(P_IDLE), msg_flag(0), msg_cmd(0), msg_size(0), payload_idx(0),
    calculated_crc(0), quality(0), motion_x(0), motion_y(0), lidar_quality(0),
    distance_mm(0), last_flow_time_us(0), last_packet_time_ms(0), dt_sec(0.02f),
    new_data_available(false), total_packets(0), crc_errors(0) {}

bool Matek3901_Flow::begin() {
  flowSerial.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  delay(10);
  parse_state = P_IDLE;
  last_flow_time_us = micros();
  return true;
}

void Matek3901_Flow::end() {
  flowSerial.end();
}

void Matek3901_Flow::dispatchMessage() {
  total_packets++;
  last_packet_time_ms = millis();

  if (msg_cmd == 0x1F02 && msg_size >= 9) { // MSP2_SENSOR_OPTIC_FLOW
    quality  = payload[0];
    motion_x = (int32_t)((uint32_t)payload[1] | ((uint32_t)payload[2] << 8) | ((uint32_t)payload[3] << 16) | ((uint32_t)payload[4] << 24));
    motion_y = (int32_t)((uint32_t)payload[5] | ((uint32_t)payload[6] << 8) | ((uint32_t)payload[7] << 16) | ((uint32_t)payload[8] << 24));
    
    unsigned long now = micros();
    if (last_flow_time_us > 0 && now > last_flow_time_us) {
      dt_sec = (float)(now - last_flow_time_us) * 1e-6f;
      if (dt_sec < 0.005f || dt_sec > 0.10f) dt_sec = 0.02f;
    } else {
      dt_sec = 0.02f;
    }
    last_flow_time_us = now;
    new_data_available = true;

  } else if (msg_cmd == 0x1F01 && msg_size >= 5) { // MSP2_SENSOR_RANGEFINDER
    lidar_quality = payload[0];
    distance_mm   = (int32_t)((uint32_t)payload[1] | ((uint32_t)payload[2] << 8) | ((uint32_t)payload[3] << 16) | ((uint32_t)payload[4] << 24));
  }
}

void Matek3901_Flow::parseByte(uint8_t b) {
  switch (parse_state) {
    case P_IDLE:
      if (b == '$') parse_state = P_HEADER_X;
      break;
    case P_HEADER_X:
      parse_state = (b == 'X') ? P_HEADER_DIR : P_IDLE;
      break;
    case P_HEADER_DIR:
      if (b == '<') {
        parse_state = P_FLAG;
        calculated_crc = 0;
      } else {
        parse_state = P_IDLE;
      }
      break;
    case P_FLAG:
      msg_flag = b;
      calculated_crc = flow_crc8_dvb_s2(calculated_crc, b);
      parse_state = P_CMD_LOW;
      break;
    case P_CMD_LOW:
      msg_cmd = b;
      calculated_crc = flow_crc8_dvb_s2(calculated_crc, b);
      parse_state = P_CMD_HIGH;
      break;
    case P_CMD_HIGH:
      msg_cmd |= ((uint16_t)b << 8);
      calculated_crc = flow_crc8_dvb_s2(calculated_crc, b);
      parse_state = P_SIZE_LOW;
      break;
    case P_SIZE_LOW:
      msg_size = b;
      calculated_crc = flow_crc8_dvb_s2(calculated_crc, b);
      parse_state = P_SIZE_HIGH;
      break;
    case P_SIZE_HIGH:
      msg_size |= ((uint16_t)b << 8);
      calculated_crc = flow_crc8_dvb_s2(calculated_crc, b);
      payload_idx = 0;
      if (msg_size > sizeof(payload)) parse_state = P_IDLE;
      else if (msg_size == 0) parse_state = P_CRC;
      else parse_state = P_PAYLOAD;
      break;
    case P_PAYLOAD:
      payload[payload_idx++] = b;
      calculated_crc = flow_crc8_dvb_s2(calculated_crc, b);
      if (payload_idx >= msg_size) parse_state = P_CRC;
      break;
    case P_CRC:
      if (calculated_crc == b) dispatchMessage();
      else crc_errors++;
      parse_state = P_IDLE;
      break;
  }
}

void Matek3901_Flow::update() {
  while (flowSerial.available() > 0) {
    parseByte((uint8_t)flowSerial.read());
  }
}

bool Matek3901_Flow::isUpdated() {
  if (new_data_available) {
    new_data_available = false;
    return true;
  }
  return false;
}

static Matek3901_Flow static_flow(FLOW_RX_PIN, FLOW_TX_PIN, FLOW_BAUD);
OpticalFlowInterface* flowSensor = &static_flow;

// =============================================================================
//  SENSOR INITIALIZER
// =============================================================================
void initSensors() {
  imuSensor     = &static_mpu;
  rangeSensor   = &static_sharp;
  baroSensor    = &static_bmp;
  compassSensor = &static_compass;
  gpsSensor     = &static_gps;
  flowSensor    = &static_flow;
}
