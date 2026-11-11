#include "ESP32_SharpIR.h"

ESP32_SharpIR::ESP32_SharpIR(uint8_t pin) {
  _pin = pin;
  pinMode(_pin, INPUT);

  #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);// 0–4095
  #endif
}

// =======================
// Median Filter
// =======================
int ESP32_SharpIR::medianRead(int samples) {
  const int MAX_SAMPLES = 50;
  int buf[MAX_SAMPLES];

  if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;

  for (int i = 0; i < samples; i++) {
    buf[i] = analogRead(_pin);
    delay(2);
  }

  // sorting (lebih ringan dari bubble kalau mau nanti kita upgrade)
  for (int i = 0; i < samples - 1; i++) {
    for (int j = i + 1; j < samples; j++) {
      if (buf[i] > buf[j]) {
        int t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
      }
    }
  }

  return buf[samples / 2];
}

// =======================
// Convert to distance (CM)
// =======================
float ESP32_SharpIR::readCM() {
  int medianRaw = medianRead(25);

  // Regresi Pangkat: ADC = 22581.6 * jarak^(-0.9184), R² = 0.9997
  // Dibalik: jarak = (ADC / 22581.6)^(1 / -0.9184)
  float A = 22581.6;
  float B = -0.9184;

  float d_float = pow((float)medianRaw / A, 1.0 / B);

  float distance_cm = d_float;

  if (distance_cm < 10) distance_cm = 10;
  if (distance_cm > 80) distance_cm = 80;

  return distance_cm;
}

float ESP32_SharpIR::readCMv2() {
  int medianRaw = medianRead(25);

  // Regresi Inverse: ADC = 29103.3 / (jarak + 0.89) + 49.57, R² = 0.9997
  // Dibalik: jarak = 29103.3 / (ADC - 49.57) - 0.89
  float A = 29103.3;
  float B = 0.89;
  float C = 49.57;

  float d_float = A / ((float)medianRaw - C) - B;

  float distance_cm = d_float;

  if (distance_cm < 10) distance_cm = 10;
  if (distance_cm > 80) distance_cm = 80;

  return distance_cm;
}
