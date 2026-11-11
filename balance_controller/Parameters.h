#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

// =============================================================================
//  PIXHAWK / PX4-STYLE PERSISTENT PARAMETER SYSTEM
//  Stores all flight, altitude, position, and attitude parameters in ESP32 Flash (NVS)
// =============================================================================

enum ParamType {
  PARAM_TYPE_FLOAT,
  PARAM_TYPE_INT,
  PARAM_TYPE_BOOL
};

struct Parameter {
  const char* name;
  ParamType   type;
  void*       ptr;
  float       default_val;
  float       min_val;
  float       max_val;
  const char* desc;
};

// Core Parameter Management
void initParameters();
void loadAllParameters();
void saveAllParameters();
void resetAllParameters();

// Get / Set by name (case-insensitive)
bool setParameter(const String& name, float val, bool saveToFlash = true);
bool getParameter(const String& name, float &outVal);

// CLI / Terminal Interfaces
void printParameterList();
void printParameter(const String& name);
bool handleParameterCommand(const String& cmd);
int  getParameterCount();

#endif // PARAMETERS_H
