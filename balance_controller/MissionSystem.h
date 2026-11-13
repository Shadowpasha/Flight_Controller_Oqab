#ifndef MISSION_SYSTEM_H
#define MISSION_SYSTEM_H

#include <Arduino.h>
#include "Config.h"

// =============================================================================
//  MISSION ITEM / COMMAND DEFINITIONS
// =============================================================================
enum MissionCmdType {
  MISSION_CMD_TAKEOFF   = 0, // Param1: target_alt_cm, Param2: climb_rate_cmps
  MISSION_CMD_WAYPOINT  = 1, // Param1: north_m, Param2: east_m, Param3: alt_cm, Param4: speed_mps
  MISSION_CMD_DELAY     = 2, // Param1: duration_s
  MISSION_CMD_LAND      = 3  // Param1: descent_rate_cmps
};

struct MissionItem {
  MissionCmdType type;
  float p1; // Param 1 (e.g. alt_cm, north_m, duration_s, desc_rate)
  float p2; // Param 2 (e.g. climb_rate, east_m)
  float p3; // Param 3 (e.g. target_alt_cm for waypoint)
  float p4; // Param 4 (e.g. speed_mps)
};

#define MAX_MISSION_ITEMS 16

// =============================================================================
//  MISSION SYSTEM INTERFACE
// =============================================================================
class MissionSystem {
public:
  MissionSystem();

  void init();
  void clear();
  bool addItem(MissionCmdType type, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f, float p4 = 0.0f);
  void loadDefaultMission();

  void start();
  void stop(const char* reason = "Mission Stopped");
  void reset();

  // Executed periodically in control loop (e.g. 50Hz or 250Hz)
  void update(float dt);

  // Status & Telemetry
  bool isActive() const { return _active; }
  bool isCompleted() const { return _completed; }
  uint8_t getCurrentIndex() const { return _currentIndex; }
  uint8_t getItemCount() const { return _count; }
  const MissionItem* getItem(uint8_t index) const;

  // Print mission overview to serial/BT
  void printMission();

private:
  MissionItem _items[MAX_MISSION_ITEMS];
  uint8_t     _count;
  uint8_t     _currentIndex;
  bool        _active;
  bool        _completed;

  // Execution states for current active item
  unsigned long _stepStartMs;
  bool          _stepInitialized;
  float         _startNorth;
  float         _startEast;
  float         _startAlt;

  void advanceItem();
};

extern MissionSystem missionSys;

#endif // MISSION_SYSTEM_H

