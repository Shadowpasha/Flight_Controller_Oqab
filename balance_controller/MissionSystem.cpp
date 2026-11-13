#include "MissionSystem.h"
#include "FlightState.h"
#include "Controllers.h"
#include "StateEstimator.h"

void dualPrint(const String& msg);
void dualPrintln(const String& msg);

MissionSystem missionSys;

MissionSystem::MissionSystem()
  : _count(0), _currentIndex(0), _active(false), _completed(false),
    _stepStartMs(0), _stepInitialized(false), _startNorth(0.0f), _startEast(0.0f), _startAlt(0.0f) {
}

void MissionSystem::init() {
  clear();
  loadDefaultMission();
}

void MissionSystem::clear() {
  _count = 0;
  _currentIndex = 0;
  _active = false;
  _completed = false;
  _stepInitialized = false;
}

bool MissionSystem::addItem(MissionCmdType type, float p1, float p2, float p3, float p4) {
  if (_count >= MAX_MISSION_ITEMS) return false;
  _items[_count].type = type;
  _items[_count].p1   = p1;
  _items[_count].p2   = p2;
  _items[_count].p3   = p3;
  _items[_count].p4   = p4;
  _count++;
  return true;
}

// Default mission:
// 1. Takeoff to target hover height (e.g. 100 cm)
// 2. Waypoint (0, 0) at target altitude (hold origin)
// 3. Delay (hover for default hover duration, e.g. 6.0s)
// 4. Land gently
void MissionSystem::loadDefaultMission() {
  clear();
  float default_alt = (altControl.target_height_cm > 15.0f) ? altControl.target_height_cm : DEFAULT_TARGET_HEIGHT_CM;
  float default_hover_s = (float)(flightMgr.hover_duration_ms / 1000.0f);
  if (default_hover_s <= 0.0f) default_hover_s = 6.0f;

  addItem(MISSION_CMD_TAKEOFF, default_alt, 18.0f);                      // Takeoff to 100cm at 18 cm/s climb
  addItem(MISSION_CMD_WAYPOINT, 0.0f, 0.0f, default_alt, 0.5f);         // Nav to (0.0m, 0.0m) at target alt
  addItem(MISSION_CMD_DELAY, default_hover_s);                           // Hold / hover delay
  addItem(MISSION_CMD_LAND, 12.0f);                                      // Controlled landing descent
}

const MissionItem* MissionSystem::getItem(uint8_t index) const {
  if (index < _count) return &_items[index];
  return nullptr;
}

void MissionSystem::start() {
  if (_count == 0) {
    dualPrintln("ERR: Mission is empty! Load items first.");
    return;
  }
  _currentIndex    = 0;
  _active          = true;
  _completed       = false;
  _stepInitialized = false;
  _stepStartMs     = millis();
  dualPrintln(">>> MISSION STARTED with " + String(_count) + " items.");
}

void MissionSystem::stop(const char* reason) {
  if (_active) {
    _active = false;
    dualPrintln(">>> MISSION PAUSED / STOPPED: " + String(reason));
  }
}

void MissionSystem::reset() {
  _currentIndex    = 0;
  _active          = false;
  _completed       = false;
  _stepInitialized = false;
}

void MissionSystem::advanceItem() {
  _currentIndex++;
  _stepInitialized = false;
  _stepStartMs     = millis();

  if (_currentIndex >= _count) {
    _active    = false;
    _completed = true;
    dualPrintln(">>> MISSION COMPLETE: All sequence blocks finished.");
  } else {
    dualPrintln("--> Mission advancing to Step " + String(_currentIndex + 1) + "/" + String(_count));
  }
}

void MissionSystem::update(float dt) {
  if (!_active || _completed || _currentIndex >= _count) return;
  if (!flightMgr.is_armed && !flightMgr.sim_mode) {
    stop("Disarmed during mission");
    return;
  }

  const MissionItem& item = _items[_currentIndex];

  switch (item.type) {
    // -------------------------------------------------------------------------
    // 1. TAKEOFF
    // -------------------------------------------------------------------------
    case MISSION_CMD_TAKEOFF: {
      float target_alt = (item.p1 > 10.0f) ? item.p1 : altControl.target_height_cm;
      float climb_rate = (item.p2 > 5.0f) ? item.p2 : 18.0f; // cm/s

      if (!_stepInitialized) {
        _stepInitialized = true;
        _stepStartMs     = millis();
        flightMgr.flight_stage = STAGE_RAMP_UP;
        altControl.active_height_sp_cm = droneState.ground_height_cm;
        altControl.target_height_cm    = target_alt;
        dualPrintln("--> [Mission 1/" + String(_count) + "] TAKEOFF Target Alt: " +
                    String(target_alt, 1) + "cm | Climb: " + String(climb_rate, 1) + "cm/s");
      }

      // Check if airborne & altitude reached
      bool altitude_reached = droneState.is_airborne &&
                              (droneState.current_height_cm >= (target_alt - 4.0f) ||
                               altControl.active_height_sp_cm >= (target_alt - 1.0f));

      if (altitude_reached) {
        flightMgr.flight_stage         = STAGE_HOVER;
        altControl.active_height_sp_cm = target_alt;
        advanceItem();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // 2. WAYPOINT (North, East, Alt)
    // -------------------------------------------------------------------------
    case MISSION_CMD_WAYPOINT: {
      float tgt_n   = item.p1;
      float tgt_e   = item.p2;
      float tgt_alt = (item.p3 > 10.0f) ? item.p3 : altControl.target_height_cm;

      if (!_stepInitialized) {
        _stepInitialized = true;
        _stepStartMs     = millis();
        flightMgr.flight_stage         = STAGE_HOVER;
        altControl.active_height_sp_cm = tgt_alt;
        altControl.target_height_cm    = tgt_alt;

        // Command horizontal position controller
        posControl.target_pos_north_m  = tgt_n;
        posControl.target_pos_east_m   = tgt_e;
        posControl.setpoint_latched    = true;

        dualPrintln("--> [Mission " + String(_currentIndex + 1) + "/" + String(_count) +
                    "] WAYPOINT: North=" + String(tgt_n, 2) + "m, East=" + String(tgt_e, 2) +
                    "m, Alt=" + String(tgt_alt, 1) + "cm");
      }

      // Calculate distance to waypoint target
      float err_n = tgt_n - droneState.pos_north_m;
      float err_e = tgt_e - droneState.pos_east_m;
      float err_dist = sqrtf(err_n * err_n + err_e * err_e);
      float err_alt  = fabs(droneState.current_height_cm - tgt_alt);

      // Reached if within 0.15m horizontal error and 6cm vertical error, or timeout 30s
      if ((err_dist < 0.15f && err_alt < 6.0f) || (millis() - _stepStartMs > 30000)) {
        advanceItem();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // 3. DELAY / HOVER HOLD
    // -------------------------------------------------------------------------
    case MISSION_CMD_DELAY: {
      unsigned long duration_ms = (unsigned long)(item.p1 * 1000.0f);
      if (!_stepInitialized) {
        _stepInitialized = true;
        _stepStartMs     = millis();
        flightMgr.flight_stage = STAGE_HOVER;
        dualPrintln("--> [Mission " + String(_currentIndex + 1) + "/" + String(_count) +
                    "] DELAY / HOVER for " + String(item.p1, 1) + "s");
      }

      if (millis() - _stepStartMs >= duration_ms) {
        advanceItem();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // 4. LAND
    // -------------------------------------------------------------------------
    case MISSION_CMD_LAND: {
      if (!_stepInitialized) {
        _stepInitialized = true;
        _stepStartMs     = millis();
        flightMgr.flight_stage         = STAGE_LANDING;
        altControl.active_height_sp_cm = droneState.current_height_cm;
        flightMgr.target_throttle_us   = flightMgr.land_throttle_target;
        dualPrintln("--> [Mission " + String(_currentIndex + 1) + "/" + String(_count) +
                    "] LANDING: Initiating controlled descent...");
      }

      // Landing completion is detected by touchdown transition in flight state machine
      if (flightMgr.flight_stage == STAGE_TOUCHDOWN || !flightMgr.is_armed) {
        advanceItem();
      }
      break;
    }

    default:
      advanceItem();
      break;
  }
}

void MissionSystem::printMission() {
  dualPrintln("================ CURRENT MISSION PLAN (" + String(_count) + " items) ================");
  for (uint8_t i = 0; i < _count; i++) {
    String line = " [" + String(i + 1) + "] ";
    switch (_items[i].type) {
      case MISSION_CMD_TAKEOFF:
        line += "TAKEOFF  -> Target Alt: " + String(_items[i].p1, 1) + " cm, Climb: " + String(_items[i].p2, 1) + " cm/s";
        break;
      case MISSION_CMD_WAYPOINT:
        line += "WAYPOINT -> N: " + String(_items[i].p1, 2) + "m, E: " + String(_items[i].p2, 2) + "m, Alt: " + String(_items[i].p3, 1) + "cm";
        break;
      case MISSION_CMD_DELAY:
        line += "DELAY    -> " + String(_items[i].p1, 1) + " s";
        break;
      case MISSION_CMD_LAND:
        line += "LAND     -> Descent: " + String(_items[i].p1, 1) + " cm/s";
        break;
    }
    if (_active && _currentIndex == i) {
      line += "  <-- [ACTIVE]";
    }
    dualPrintln(line);
  }
  dualPrintln("Status: " + String(_active ? "RUNNING" : (_completed ? "COMPLETED" : "IDLE")));
  dualPrintln("================================================================");
}

