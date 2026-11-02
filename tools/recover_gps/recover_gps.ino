/*******************************************************************************
 * GPS Factory Reset & NMEA Recovery Tool for ESP32
 * 
 * Purpose:
 *   If a previous u-blox command silenced ASCII NMEA output (UBX only) or changed
 *   the baud rate and saved it to the module's battery-backed RAM (BBR) or EEPROM,
 *   this script completely clears all saved settings, restores factory defaults,
 *   forces UART1 to 9600 baud, enables standard NMEA output, and saves it.
 * 
 * Hardware Wiring:
 *   ESP32 Pin 16 (RX2) <---> GPS Module TX
 *   ESP32 Pin 17 (TX2) <---> GPS Module RX
 *   ESP32 5V (or VIN)  <---> GPS Module VCC
 *   ESP32 GND          <---> GPS Module GND
 ******************************************************************************/

#include <Arduino.h>

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

HardwareSerial gpsSerial(2);

// 1. UBX-CFG-CFG: Factory Reset (Clear all configurations, reload defaults from ROM)
const uint8_t UBX_FACTORY_RESET[] = {
  0xB5, 0x62,                         // Sync chars
  0x06, 0x09,                         // Class: CFG, ID: CFG
  0x0D, 0x00,                         // Length: 13 bytes
  0xFF, 0xFF, 0x00, 0x00,             // clearMask (clear all sections)
  0x00, 0x00, 0x00, 0x00,             // saveMask (do not save current)
  0xFF, 0xFF, 0x00, 0x00,             // loadMask (reload all from ROM)
  0x17,                               // deviceMask (devBBR, devFlash, devEEPROM, devSpiFlash)
  0x2F, 0xAE                          // Checksum CK_A, CK_B
};

// 2. UBX-CFG-PRT: Force UART1 to 9600 Baud, 8N1, Input: UBX+NMEA, Output: UBX+NMEA
const uint8_t UBX_FORCE_UART1_9600_NMEA[] = {
  0xB5, 0x62,                         // Sync chars
  0x06, 0x00,                         // Class: CFG, ID: PRT
  0x14, 0x00,                         // Length: 20 bytes
  0x01, 0x00, 0x00, 0x00,             // portID = 1 (UART1)
  0xD0, 0x08, 0x00, 0x00,             // mode = 0x08D0 (8N1)
  0x80, 0x25, 0x00, 0x00,             // baudRate = 9600 (0x00002580)
  0x07, 0x00,                         // inProtoMask = 0x0007 (UBX + NMEA + RTCM)
  0x03, 0x00,                         // outProtoMask = 0x0003 (UBX + NMEA enabled!)
  0x00, 0x00, 0x00, 0x00,             // flags & reserved
  0xA2, 0xB5                          // Checksum CK_A, CK_B
};

// 3. UBX-CFG-CFG: Save 9600 Baud & NMEA Configuration to BBR & Flash
const uint8_t UBX_SAVE_CONFIG[] = {
  0xB5, 0x62,
  0x06, 0x09,
  0x0D, 0x00,
  0x00, 0x00, 0x00, 0x00,             // clearMask
  0xFF, 0xFF, 0x00, 0x00,             // saveMask (save all current settings)
  0x00, 0x00, 0x00, 0x00,             // loadMask
  0x17,                               // deviceMask (all storage)
  0x31, 0xBF                          // Checksum CK_A, CK_B
};

const uint32_t bauds_to_sweep[] = { 115200, 57600, 38400, 19200, 9600, 4800 };
const int num_bauds = sizeof(bauds_to_sweep) / sizeof(bauds_to_sweep[0]);

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=======================================================");
  Serial.println("       U-BLOX GPS RECOVERY & FACTORY RESET TOOL        ");
  Serial.println("=======================================================");
  Serial.println("Sweeping all baud rates to restore factory NMEA 9600 baud...");

  for (int i = 0; i < num_bauds; i++) {
    uint32_t b = bauds_to_sweep[i];
    Serial.printf(">>> Blasting Factory Reset & NMEA Enable at %u baud...\n", b);

    gpsSerial.begin(b, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(100);

    // Send Factory Reset
    gpsSerial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET));
    gpsSerial.flush();
    delay(100);

    // Send Force 9600 + NMEA Enable
    gpsSerial.write(UBX_FORCE_UART1_9600_NMEA, sizeof(UBX_FORCE_UART1_9600_NMEA));
    gpsSerial.flush();
    delay(100);

    // Send Save to Flash / BBR
    gpsSerial.write(UBX_SAVE_CONFIG, sizeof(UBX_SAVE_CONFIG));
    gpsSerial.flush();
    delay(100);
  }

  // Now settle permanently on 9600 baud
  Serial.println("\n>>> Opening Serial2 permanently at 9600 baud (Standard NMEA)...");
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(200);

  // Send NMEA enable one more time at 9600
  gpsSerial.write(UBX_FORCE_UART1_9600_NMEA, sizeof(UBX_FORCE_UART1_9600_NMEA));
  gpsSerial.flush();
  delay(50);
  gpsSerial.write(UBX_SAVE_CONFIG, sizeof(UBX_SAVE_CONFIG));
  gpsSerial.flush();

  Serial.println("\n=======================================================");
  Serial.println("LIVE GPS RAW STREAM PREVIEW (Pins: RX=16, TX=17):");
  Serial.println("If recovery succeeded, you should see $GNGGA / $GNRMC below:");
  Serial.println("=======================================================\n");
}

unsigned long last_stat_ms = 0;
uint32_t total_bytes = 0;
uint32_t dollar_count = 0;

void loop() {
  while (gpsSerial.available() > 0) {
    char c = (char)gpsSerial.read();
    total_bytes++;
    if (c == '$') dollar_count++;
    Serial.write(c);
  }

  // Every 5 seconds, print status if no characters are arriving
  if (millis() - last_stat_ms >= 5000) {
    last_stat_ms = millis();
    if (total_bytes == 0) {
      Serial.println("\n[WAITING]: No serial bytes received yet...");
      Serial.println("  1. Verify GPS TX is wired to ESP32 Pin 16.");
      Serial.println("  2. Verify Red Power LED on GPS board is ON.");
      Serial.println("  3. Verify common GND is connected.");
    } else {
      Serial.printf("\n[STATUS]: Total Bytes: %u | Valid NMEA Sentences ($): %u\n", total_bytes, dollar_count);
    }
  }
}

