#ifndef ESP32_SharpIR_h
#define ESP32_SharpIR_h

#include <Arduino.h>

class ESP32_SharpIR {
  public:
    ESP32_SharpIR(uint8_t pin);
    float readCM();
    float readCMv2();
  private:
    uint8_t _pin;
    int medianRead(int samples);
};

#endif
