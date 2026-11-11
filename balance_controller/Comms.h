#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include "BluetoothSerial.h"
#include "Config.h"

extern BluetoothSerial SerialBT;

void initComms();
void startCommsTask();
void dualPrint(const String& msg);
void dualPrintln(const String& msg);
void handleInputCommands();
void processCommand(const char* cmd_raw);
void sendTelemetry(bool is_armed, bool sim_mode, FlightStage stage, FlightMode mode,
                   int base_throttle_us, float out_roll, float out_pitch, float out_yaw);
void print3DPositionReport();

#endif // COMMS_H
