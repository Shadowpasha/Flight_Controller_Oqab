export interface DroneTelemetry {
  homeOrigin: string;
  dist2d: number;
  dist3d: number;
  posNorth: number;
  posEast: number;
  altDiff: number;
  heightCm: number;
  velNorth: number;
  velEast: number;
  velDown: number;
  speed3d: number;
  gpsStatus: string;
  rawNorth: number;
  rawEast: number;
  altSource: string;
  lidarCm: number;
  stateTag: string;
  controller: string;
  mode: string;
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
  outRoll: number;
  outPitch: number;
  outYaw: number;
  flowQuality: number;
  flowSpeed: number;
  flowRotDeg: number;
  isArmed: boolean;
  isAirborne: boolean;
  isSimMode: boolean;
  timestamp: number;
}

export interface FlightParameter {
  name: string;
  type: 'float' | 'int' | 'bool';
  value: number;
  defaultValue: number;
  min: number;
  max: number;
  description: string;
  category: 'Sensors & Flow' | 'Attitude Control' | 'Altitude & Hover' | 'Navigation';
}

export type ConnectionState = 'disconnected' | 'connecting' | 'connected';

export type MissionBlockType = 'TAKEOFF' | 'WAYPOINT' | 'DELAY' | 'LAND';

export interface MissionBlock {
  id: string;
  type: MissionBlockType;
  title: string;
  // Parameters
  altCm?: number;
  climbRateCmps?: number;
  northM?: number;
  eastM?: number;
  speedMps?: number;
  durationS?: number;
  descRateCmps?: number;
}

export interface FlowLivePoint {
  time: number;
  quality: number;
  vFwd: number;
  vRight: number;
  ofX: number; // Optical flow forward rate in dps
  ofY: number; // Optical flow rightward rate in dps
  gyroX: number; // Roll rate in dps (around X axis)
  gyroY: number; // Pitch rate in dps (around Y axis)
  gyroZ: number; // Yaw rate in dps (around Z axis)
  heightM: number;
  rawX: number;
  rawY: number;
}

