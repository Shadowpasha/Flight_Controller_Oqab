/*
 * Mateksys Optical Flow & LiDAR (3901-L0X / 3901-L1X) Test Sketch for ESP32
 * 
 * Hardware Wiring:
 *   ESP32 Pin 16 (RX2)  <---  Matek Sensor TX
 *   ESP32 Pin 17 (TX2)  --->  Matek Sensor RX
 *   ESP32 5V            <---  Matek Sensor 5V (4.5V to 5.5V)
 *   ESP32 GND           <---  Matek Sensor GND
 * 
 * Protocol: MSPv2 (MultiWii Serial Protocol Version 2) @ 115200 baud
 * Frame format:
 *   ['$', 'X', '<', flag, cmd_low, cmd_high, size_low, size_high, payload..., crc8]
 * 
 * Sensor Message IDs:
 *   0x1F01 (MSP2_SENSOR_RANGEFINDER): 5 bytes (quality: uint8, distance_mm: int32)
 *   0x1F02 (MSP2_SENSOR_OPTIC_FLOW) : 9 bytes (quality: uint8, motion_x: int32, motion_y: int32)
 * Checksum: CRC-8-DVB-S2 (poly 0xD5)
 */

#include <Arduino.h>

#define FLOW_RX_PIN 16
#define FLOW_TX_PIN 17
#define FLOW_BAUD   115200

#define MSP2_SENSOR_RANGEFINDER 0x1F01
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02

HardwareSerial flowSerial(2);

// Optical Flow and Rangefinder State
struct OpticalFlowData {
  uint8_t  flow_quality;
  int32_t  motion_x;
  int32_t  motion_y;
  int64_t  accumulated_x;
  int64_t  accumulated_y;
  uint32_t flow_packet_count;
  uint32_t last_flow_time_ms;

  uint8_t  lidar_quality;
  int32_t  distance_mm;
  float    distance_cm;
  uint32_t lidar_packet_count;
  uint32_t last_lidar_time_ms;

  uint32_t crc_errors;
  uint32_t total_bytes_seen;
};

OpticalFlowData sensorData;

// CRC-8-DVB-S2 (Polynomial 0xD5) for MSPv2
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t ch) {
  crc ^= ch;
  for (int i = 0; i < 8; ++i) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc = (crc << 1);
    }
  }
  return crc;
}

// MSPv2 Parser State Machine
enum MSPv2State {
  STATE_IDLE,
  STATE_HEADER_X,
  STATE_HEADER_DIR,
  STATE_FLAG,
  STATE_CMD_LOW,
  STATE_CMD_HIGH,
  STATE_SIZE_LOW,
  STATE_SIZE_HIGH,
  STATE_PAYLOAD,
  STATE_CRC
};

MSPv2State parse_state = STATE_IDLE;
uint8_t    msg_flag = 0;
uint16_t   msg_cmd = 0;
uint16_t   msg_size = 0;
uint16_t   payload_idx = 0;
uint8_t    payload_buffer[64];
uint8_t    calculated_crc = 0;

void processMsp2Message(uint16_t cmd, const uint8_t* payload, uint16_t size) {
  if (cmd == MSP2_SENSOR_OPTIC_FLOW && size >= 9) {
    sensorData.flow_quality = payload[0];
    sensorData.motion_x = (int32_t)((uint32_t)payload[1] | ((uint32_t)payload[2] << 8) | ((uint32_t)payload[3] << 16) | ((uint32_t)payload[4] << 24));
    sensorData.motion_y = (int32_t)((uint32_t)payload[5] | ((uint32_t)payload[6] << 8) | ((uint32_t)payload[7] << 16) | ((uint32_t)payload[8] << 24));
    sensorData.accumulated_x += sensorData.motion_x;
    sensorData.accumulated_y += sensorData.motion_y;
    sensorData.flow_packet_count++;
    sensorData.last_flow_time_ms = millis();
  } else if (cmd == MSP2_SENSOR_RANGEFINDER && size >= 5) {
    sensorData.lidar_quality = payload[0];
    sensorData.distance_mm = (int32_t)((uint32_t)payload[1] | ((uint32_t)payload[2] << 8) | ((uint32_t)payload[3] << 16) | ((uint32_t)payload[4] << 24));
    sensorData.distance_cm = (sensorData.distance_mm > 0) ? (sensorData.distance_mm * 0.1f) : 0.0f;
    sensorData.lidar_packet_count++;
    sensorData.last_lidar_time_ms = millis();
  }
}

void parseIncomingByte(uint8_t b) {
  sensorData.total_bytes_seen++;

  switch (parse_state) {
    case STATE_IDLE:
      if (b == '$') {
        parse_state = STATE_HEADER_X;
      }
      break;

    case STATE_HEADER_X:
      if (b == 'X') {
        parse_state = STATE_HEADER_DIR;
      } else {
        parse_state = STATE_IDLE;
      }
      break;

    case STATE_HEADER_DIR:
      if (b == '<' || b == '>') {
        parse_state = STATE_FLAG;
        calculated_crc = 0;
      } else {
        parse_state = (b == '$') ? STATE_HEADER_X : STATE_IDLE;
      }
      break;

    case STATE_FLAG:
      msg_flag = b;
      calculated_crc = crc8_dvb_s2(calculated_crc, b);
      parse_state = STATE_CMD_LOW;
      break;

    case STATE_CMD_LOW:
      msg_cmd = b;
      calculated_crc = crc8_dvb_s2(calculated_crc, b);
      parse_state = STATE_CMD_HIGH;
      break;

    case STATE_CMD_HIGH:
      msg_cmd |= ((uint16_t)b << 8);
      calculated_crc = crc8_dvb_s2(calculated_crc, b);
      parse_state = STATE_SIZE_LOW;
      break;

    case STATE_SIZE_LOW:
      msg_size = b;
      calculated_crc = crc8_dvb_s2(calculated_crc, b);
      parse_state = STATE_SIZE_HIGH;
      break;

    case STATE_SIZE_HIGH:
      msg_size |= ((uint16_t)b << 8);
      calculated_crc = crc8_dvb_s2(calculated_crc, b);
      payload_idx = 0;
      if (msg_size > sizeof(payload_buffer)) {
        parse_state = STATE_IDLE;
      } else if (msg_size == 0) {
        parse_state = STATE_CRC;
      } else {
        parse_state = STATE_PAYLOAD;
      }
      break;

    case STATE_PAYLOAD:
      payload_buffer[payload_idx++] = b;
      calculated_crc = crc8_dvb_s2(calculated_crc, b);
      if (payload_idx >= msg_size) {
        parse_state = STATE_CRC;
      }
      break;

    case STATE_CRC:
      if (calculated_crc == b) {
        processMsp2Message(msg_cmd, payload_buffer, msg_size);
      } else {
        sensorData.crc_errors++;
      }
      parse_state = STATE_IDLE;
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================================");
  Serial.println("  Mateksys Optical Flow & LiDAR (3901-L0X) Diagnostic");
  Serial.println("========================================================");
  Serial.println("Serial2 initializing on RX: GPIO 16, TX: GPIO 17 @ 115200 baud");

  flowSerial.begin(FLOW_BAUD, SERIAL_8N1, FLOW_RX_PIN, FLOW_TX_PIN);

  memset(&sensorData, 0, sizeof(OpticalFlowData));
}

unsigned long last_print_ms = 0;
uint32_t last_flow_pkts = 0;
uint32_t last_lidar_pkts = 0;

void loop() {
  // Non-blocking drain of hardware serial FIFO
  while (flowSerial.available() > 0) {
    uint8_t c = (uint8_t)flowSerial.read();
    parseIncomingByte(c);
  }

  // Print formatted report at 10 Hz (every 100 ms)
  unsigned long now = millis();
  if (now - last_print_ms >= 100) {
    float dt = (now - last_print_ms) / 1000.0f;
    last_print_ms = now;

    float flow_hz = (sensorData.flow_packet_count - last_flow_pkts) / dt;
    float lidar_hz = (sensorData.lidar_packet_count - last_lidar_pkts) / dt;
    last_flow_pkts = sensorData.flow_packet_count;
    last_lidar_pkts = sensorData.lidar_packet_count;

    bool flow_alive = (now - sensorData.last_flow_time_ms < 1000) && (sensorData.flow_packet_count > 0);
    bool lidar_alive = (now - sensorData.last_lidar_time_ms < 1000) && (sensorData.lidar_packet_count > 0);

    Serial.printf("[FLOW] Stat:%s (%4.1fHz) | Q:%3u | dX:%+5d dY:%+5d | AccX:%+7lld AccY:%+7lld || [LIDAR] Stat:%s (%4.1fHz) | Q:%3u | Dist:%5.1fcm (%4dmm) || RxBytes:%lu CrcErr:%lu\n",
                  flow_alive ? "OK" : "NO_DATA",
                  flow_hz,
                  sensorData.flow_quality,
                  sensorData.motion_x,
                  sensorData.motion_y,
                  sensorData.accumulated_x,
                  sensorData.accumulated_y,
                  lidar_alive ? "OK" : "NO_DATA",
                  lidar_hz,
                  sensorData.lidar_quality,
                  sensorData.distance_cm,
                  sensorData.distance_mm,
                  sensorData.total_bytes_seen,
                  sensorData.crc_errors);
  }
}
