import React from 'react';
import { ArrowUp, Navigation, Compass, Activity } from 'lucide-react';
import { DroneTelemetry } from '../types';

interface Props {
  telemetry: DroneTelemetry;
}

export const TelemetryCards: React.FC<Props> = ({ telemetry }) => {
  const isArmed = telemetry.isArmed;

  return (
    <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
      {/* 1. Altitude & Climb Rate */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
        <div className="flex items-center justify-between text-[11px] font-mono text-neutral-400">
          <span className="uppercase">Fused Alt</span>
          <ArrowUp className="w-3.5 h-3.5 text-neutral-500" />
        </div>
        <div className="my-1">
          <span className="text-2xl font-bold font-mono text-white">
            {telemetry.heightCm.toFixed(1)}
          </span>
          <span className="text-xs font-mono text-neutral-400 ml-1">cm</span>
        </div>
        <div className="text-[11px] font-mono flex items-center justify-between text-neutral-400">
          <span>Climb:</span>
          <span className={`font-semibold ${Math.abs(telemetry.velDown) > 0.05 ? 'text-red-500' : 'text-neutral-300'}`}>
            {telemetry.velDown >= 0 ? '+' : ''}{telemetry.velDown.toFixed(2)} m/s
          </span>
        </div>
      </div>

      {/* 2. Dead Reckoning Distance */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
        <div className="flex items-center justify-between text-[11px] font-mono text-neutral-400">
          <span className="uppercase">Distance</span>
          <Navigation className="w-3.5 h-3.5 text-neutral-500" />
        </div>
        <div className="my-1">
          <span className="text-2xl font-bold font-mono text-white">
            {telemetry.dist2d.toFixed(2)}
          </span>
          <span className="text-xs font-mono text-neutral-400 ml-1">m (2D)</span>
        </div>
        <div className="text-[11px] font-mono flex items-center justify-between text-neutral-400">
          <span>3D Dist:</span>
          <span className="text-neutral-300 font-semibold">{telemetry.dist3d.toFixed(2)} m</span>
        </div>
      </div>

      {/* 3. Horizontal Displacement */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
        <div className="flex items-center justify-between text-[11px] font-mono text-neutral-400">
          <span className="uppercase">Position</span>
          <Compass className="w-3.5 h-3.5 text-neutral-500" />
        </div>
        <div className="my-1 font-mono text-xs font-semibold flex items-center justify-between text-neutral-200">
          <div>
            <span className="text-neutral-400">N: </span>
            <span>{telemetry.posNorth >= 0 ? '+' : ''}{telemetry.posNorth.toFixed(2)}m</span>
          </div>
          <div>
            <span className="text-neutral-400">E: </span>
            <span>{telemetry.posEast >= 0 ? '+' : ''}{telemetry.posEast.toFixed(2)}m</span>
          </div>
        </div>
        <div className="text-[11px] font-mono flex items-center justify-between text-neutral-400">
          <span>Speed 3D:</span>
          <span className="text-neutral-300 font-semibold">{telemetry.speed3d.toFixed(2)} m/s</span>
        </div>
      </div>

      {/* 4. Flight State & Controller */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
        <div className="flex items-center justify-between text-[11px] font-mono text-neutral-400">
          <span className="uppercase">State</span>
          <Activity className="w-3.5 h-3.5 text-neutral-500" />
        </div>
        <div className="my-1 flex items-center gap-2">
          <span className={`text-xs font-bold font-mono px-2 py-0.5 rounded border ${
            isArmed
              ? 'bg-red-600 text-white border-red-600'
              : 'bg-neutral-900 text-neutral-400 border-neutral-800'
          }`}>
            {telemetry.stateTag}
          </span>
          <span className="text-xs font-mono font-bold text-neutral-200">
            {telemetry.controller}
          </span>
        </div>
        <div className="text-[11px] font-mono text-neutral-400 flex items-center justify-between">
          <span>Mode: {telemetry.mode}</span>
          <span>Home: {telemetry.homeOrigin}</span>
        </div>
      </div>
    </div>
  );
};
