import React from 'react';
import { Compass, Eye, Check, RefreshCw } from 'lucide-react';
import { serialService } from '../services/serialService';

interface OpticalFlowProps {
  quality: number;
  flowSpeed: number;
  flowRotDeg: number;
  lidarCm: number;
  velNorth: number;
  velEast: number;
  isArmed: boolean;
}

export const OpticalFlowPanel: React.FC<OpticalFlowProps> = ({
  quality,
  flowSpeed,
  flowRotDeg,
  lidarCm,
  velNorth,
  velEast,
  isArmed,
}) => {
  const rotationOptions = [
    { deg: 0, label: '0° (Forward)' },
    { deg: 90, label: '90° (Right)' },
    { deg: 180, label: '180° (Rear)' },
    { deg: 270, label: '270° (Left)' },
  ];

  const handleSetRotation = (deg: number) => {
    serialService.sendCommand(`flow rot ${deg}`);
  };

  const isTrackingLocked = quality >= 35;

  return (
    <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col gap-4">
      <div className="flex items-center justify-between border-b border-neutral-800/80 pb-3">
        <div className="flex items-center gap-2">
          <Eye className="w-4 h-4 text-red-600" />
          <h2 className="text-xs font-mono font-semibold uppercase tracking-wider text-neutral-200">
            Mateksys 3901-L0X Optical Flow
          </h2>
        </div>
        <span className="text-[11px] font-mono px-2 py-0.5 rounded bg-neutral-900 text-neutral-400 border border-neutral-800">
          MSPv2 115200
        </span>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Quality & Stats */}
        <div className="flex flex-col gap-3">
          <div className="bg-black p-3.5 rounded border border-neutral-800">
            <div className="flex justify-between items-center mb-1.5 font-mono text-xs">
              <span className="text-neutral-400">Tracking Quality</span>
              <span className="font-semibold text-neutral-100">{quality} / 255</span>
            </div>
            <div className="w-full bg-neutral-900 h-2 rounded overflow-hidden">
              <div
                className={`h-full transition-all duration-300 ${isTrackingLocked ? 'bg-red-600' : 'bg-neutral-600'}`}
                style={{ width: `${Math.min(100, (quality / 255) * 100)}%` }}
              />
            </div>
            <div className="mt-2 flex items-center justify-between font-mono text-[11px]">
              <span className={isTrackingLocked ? 'text-red-500 font-semibold' : 'text-neutral-500'}>
                {isTrackingLocked ? 'TRACKING LOCKED' : 'POOR TEXTURE'}
              </span>
              <span className="text-neutral-500">Threshold: 35</span>
            </div>
          </div>

          <div className="grid grid-cols-2 gap-3">
            <div className="bg-black p-3 rounded border border-neutral-800 flex flex-col">
              <span className="text-[11px] font-mono text-neutral-400">ToF LiDAR Height</span>
              <span className="text-xl font-bold font-mono text-neutral-100 mt-1">
                {lidarCm.toFixed(1)} <span className="text-xs font-normal text-neutral-500">cm</span>
              </span>
              <span className="text-[10px] font-mono text-neutral-600 mt-0.5">Range: 4 - 200cm</span>
            </div>

            <div className="bg-black p-3 rounded border border-neutral-800 flex flex-col">
              <span className="text-[11px] font-mono text-neutral-400">Ground Speed</span>
              <span className="text-xl font-bold font-mono text-neutral-100 mt-1">
                {flowSpeed.toFixed(2)} <span className="text-xs font-normal text-neutral-500">m/s</span>
              </span>
              <span className="text-[10px] font-mono text-neutral-600 mt-0.5">
                Vx: {velNorth.toFixed(2)} | Vy: {velEast.toFixed(2)}
              </span>
            </div>
          </div>
        </div>

        {/* Mounting Yaw Angle Adjustment */}
        <div className="bg-black p-3.5 rounded border border-neutral-800 flex flex-col justify-between">
          <div>
            <div className="flex items-center justify-between mb-2">
              <div className="flex items-center gap-1.5 font-mono text-xs">
                <Compass className="w-3.5 h-3.5 text-red-600" />
                <span className="text-neutral-300 font-semibold">Mounting Yaw Angle</span>
              </div>
              <span className="text-xs font-mono font-bold text-red-500 bg-neutral-900 px-2 py-0.5 rounded border border-neutral-800">
                {flowRotDeg.toFixed(0)}°
              </span>
            </div>
            <p className="text-[11px] text-neutral-400 mb-3 leading-snug">
              Set clockwise rotation angle relative to drone nose.
            </p>

            <div className="grid grid-cols-2 gap-2">
              {rotationOptions.map((opt) => {
                const isActive = Math.abs(flowRotDeg - opt.deg) < 1.0;
                return (
                  <button
                    key={opt.deg}
                    onClick={() => handleSetRotation(opt.deg)}
                    disabled={isArmed}
                    className={`px-3 py-2 text-xs font-mono rounded transition-colors text-left flex items-center justify-between border ${
                      isActive
                        ? 'bg-red-600 text-white border-red-600 font-bold'
                        : 'bg-neutral-900 text-neutral-400 border-neutral-800 hover:bg-neutral-800 hover:text-neutral-200'
                    } ${isArmed ? 'opacity-40 cursor-not-allowed' : 'cursor-pointer'}`}
                  >
                    <span>{opt.label}</span>
                    {isActive && <Check className="w-3.5 h-3.5 text-white" />}
                  </button>
                );
              })}
            </div>
          </div>

          <div className="mt-3 pt-3 border-t border-neutral-800/80 flex items-center justify-between text-[11px] font-mono text-neutral-400">
            <span>Flash: <code className="text-neutral-200">SENS_FLOW_ROT</code></span>
            <button
              onClick={() => serialService.sendCommand('flow toggle')}
              disabled={isArmed}
              className="text-neutral-300 hover:text-red-500 underline underline-offset-2 cursor-pointer"
            >
              Toggle
            </button>
          </div>
        </div>
      </div>

      {/* Auto-Calibration Action */}
      <div className="bg-neutral-950 p-3 rounded border border-neutral-800 flex flex-wrap items-center justify-between gap-3 font-mono text-xs">
        <div className="flex flex-col gap-0.5">
          <span className="text-neutral-200 font-semibold">Optical Flow Derotation Auto-Calibration</span>
          <span className="text-[11px] text-neutral-500">
            Hold drone ~1m over floor & rock in pitch/roll to calibrate rad/count scale factor (saves to Flash).
          </span>
        </div>
        <button
          onClick={() => serialService.sendCommand('cal flow')}
          disabled={isArmed}
          className={`px-3 py-1.5 bg-red-600 hover:bg-red-700 text-white rounded text-xs font-bold transition flex items-center gap-1.5 ${
            isArmed ? 'opacity-40 cursor-not-allowed' : 'cursor-pointer'
          }`}
          title="Start 10s automatic optical flow derotation calibration"
        >
          <RefreshCw className="w-3.5 h-3.5" />
          <span>Calibrate Derotation</span>
        </button>
      </div>
    </div>
  );
};
