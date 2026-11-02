/*******************************************************************************
 * Standalone Bluetooth Serial (SPP) Diagnostic for ESP32
 * 
 * Target Device Name: "F330_Flight_Controller"
 * 
 * CRITICAL REQUIREMENTS & TROUBLESHOOTING:
 * 1. Android Phones:
 *    - Go to Android Settings -> Bluetooth -> Scan.
 *    - Pair with "F330_Flight_Controller" (PIN is usually not needed or 1234 / 0000).
 *    - Install "Serial Bluetooth Terminal" app from Google Play Store.
 *    - Open the app -> Menu (three lines) -> Devices -> Bluetooth Classic.
 *    - Select "F330_Flight_Controller" -> Tap the connect icon.
 * 
 * 2. Apple iOS (iPhones / iPads):
 *    - IMPORTANT: iPhones DO NOT support Bluetooth Classic SPP (Serial Port Profile)!
 *    - Apple restricts Classic Bluetooth SPP to MFi-certified hardware.
 *    - If you are trying to connect an iPhone, it will NOT work with BluetoothSerial.
 *      Use an Android phone, tablet, or PC/laptop with Bluetooth instead.
 * 
 * 3. Windows PC / Laptop:
 *    - Windows Settings -> Bluetooth & Devices -> Add device -> Bluetooth.
 *    - Pair with "F330_Flight_Controller".
 *    - Open Device Manager -> Ports (COM & LPT) to find the assigned Standard Serial COM port.
 *    - Open Arduino Serial Monitor or PuTTY on that COM port at 115200 baud.
 * 
 * 4. ESP32 Board Compatibility:
 *    - Standard ESP32 (WROOM-32 / WROVER) HAS Classic Bluetooth.
 *    - ESP32-S2, ESP32-S3, and ESP32-C3 DO NOT support Classic Bluetooth SPP!
 ******************************************************************************/

#include <Arduino.h>
#include "BluetoothSerial.h"

#define BT_DEVICE_NAME "F330_Flight_Controller"

BluetoothSerial SerialBT;
bool client_connected = false;

// Event callback to detect connection and disconnection in real-time
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    client_connected = true;
    Serial.println("\n*******************************************************");
    Serial.println(">>> [SUCCESS] BLUETOOTH CLIENT CONNECTED TO ESP32! <<<");
    Serial.println("*******************************************************\n");
    SerialBT.println("\n[CONNECTED] Welcome to ESP32 Flight Controller Bluetooth Console!");
    SerialBT.println("Type 'ping' or any text to test communication.\n");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    client_connected = false;
    Serial.println("\n>>> [INFO] Bluetooth client disconnected.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=======================================================");
  Serial.println("      ESP32 BLUETOOTH SERIAL (SPP) TEST SKETCH        ");
  Serial.println("=======================================================");
  Serial.printf("Broadcasting Bluetooth Device Name: \"%s\"\n", BT_DEVICE_NAME);

  // Register connection listener callback before starting
  SerialBT.register_callback(btCallback);

  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("\n>>> [FATAL ERROR]: SerialBT.begin() failed!");
    Serial.println("    Possible reasons:");
    Serial.println("    1. Are you using an ESP32-S3 or ESP32-C3? They do not have Classic Bluetooth!");
    Serial.println("    2. Insufficient DRAM for Bluetooth stack.");
    while (1) {
      delay(500);
    }
  }

  Serial.println(">>> Bluetooth Serial started successfully!");
  Serial.println(">>> ESP32 is now discoverable. Follow pairing instructions above.\n");
}

unsigned long last_heartbeat = 0;

void loop() {
  // Bridge Bluetooth RX -> USB Serial TX
  while (SerialBT.available() > 0) {
    char c = (char)SerialBT.read();
    Serial.write(c);

    // Echo back to the phone for instant feedback
    if (c == '\n') {
      SerialBT.println("[ESP32 ACK] Command received OK");
    }
  }

  // Bridge USB Serial RX -> Bluetooth TX
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    SerialBT.write(c);
  }

  // Periodic status heartbeat on Serial Monitor every 3 seconds
  if (millis() - last_heartbeat >= 3000) {
    last_heartbeat = millis();
    if (SerialBT.hasClient()) {
      Serial.println("[BT Status] Client active & connected.");
    } else {
      Serial.printf("[BT Status] Waiting for connection... (Device name: \"%s\")\n", BT_DEVICE_NAME);
    }
  }
}

