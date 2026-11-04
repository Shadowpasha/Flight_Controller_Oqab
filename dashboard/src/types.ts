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
  flowQuality: number;
  flowSpeed: number;
  flowRotDeg: number;
  isArmed: boolean;
  isAirborne: boolean;
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

export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'simulated';

