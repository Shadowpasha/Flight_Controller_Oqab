import React, { useState } from 'react';
import { Activity, Sliders, RefreshCw, RotateCcw, Save, Check, Eye, Compass, Layers } from 'lucide-react';
import { serialService } from '../services/serialService';
import { DroneTelemetry } from '../types';

interface Props {
  telemetry: DroneTelemetry;
  isArmed: boolean;
}

export const StateEstimationPanel: React.FC<Props> = ({ telemetry, isArmed }) => {
  const [params, setParams] = useState([
    {
      name: 'EKF_Q_Z',
      value: 0.0002,
      defaultValue: 0.0002,
      min: 0.00001,
      max: 0.005,
      step: 0.00005,
      description: 'Vertical Kalman filter position process noise Q (higher = trust model less)',
    },
    {
      name: 'EKF_Q_VZ',
      value: 0.0035,
      defaultValue: 0.0035,
      min: 0.0001,
      max: 0.02,
      step: 0.0002,
      description: 'Vertical Kalman filter velocity process noise Q (higher = tracks fast velocity changes)',
    },
    {
      name: 'EKF_Q_BA',
      value: 0.00003,
      defaultValue: 0.00003,
      min: 0.000001,
      max: 0.0005,
      step: 0.000005,
      description: 'Vertical accelerometer bias drift learning rate Q',
    },
    {
      name: 'EKF_R_LIDAR',
      value: 0.0025,
      defaultValue: 0.0025,
      min: 0.0001,
      max: 0.02,
      step: 0.0002,
      description: 'ToF LiDAR measurement covariance R (lower = trust LiDAR distance more)',
    },
    {
      name: 'ATT_ALPHA',
      value: 0.991,
      defaultValue: 0.991,
      min: 0.90,
      max: 0.999,
      step: 0.001,
      description: 'Attitude complementary filter alpha (gyro weight vs accelerometer gravity)',
    },
    {
      name: 'YAW_ALPHA',
      value: 0.995,
      defaultValue: 0.995,
      min: 0.90,
      max: 0.999,
      step: 0.001,
      description: 'Yaw complementary filter alpha (gyro weight vs compass true North)',
    },
    {
      name: 'EKF_R_FLOW',
      value: 0.02,
      defaultValue: 0.02,
      min: 0.001,
      max: 0.15,
      step: 0.002,
      description: 'Optical flow velocity measurement covariance R for horizontal Kalman filters',
    },
    {
      name: 'FLOW_Q_THRESH',
      value: 35,
      defaultValue: 35,
      min: 10,
      max: 120,
      step: 1,
      description: 'Minimum surface tracking quality threshold before fusing optical flow',
    },
    {
      name: 'MAG_DECL_DEG',
      value: 0.0,
      defaultValue: 0.0,
      min: -180.0,
      max: 180.0,
      step: 0.5,
      description: 'Geographic magnetic declination offset [deg]',
    },
  ]);

  const [saveSuccess, setSaveSuccess] = useState(false);

  const handleValueChange = (name: string, val: number) => {
    setParams((prev) =>
      prev.map((p) => (p.name === name ? { ...p, value: val } : p))
    );
    serialService.sendCommand(`param set ${name} ${val}`);
  };

  const handleSaveToFlash = () => {
    serialService.sendCommand('param save');
    setSaveSuccess(true);
    setTimeout(() => setSaveSuccess(false), 2000);
  };

  const handleReload = () => {
    serialService.sendCommand('param show');
  };

  const handleResetDefaults = () => {
    if (confirm('Reset State Estimator parameters to defaults?')) {
      serialService.sendCommand('param reset');
      setParams((prev) => prev.map((p) => ({ ...p, value: p.defaultValue })));
    }
  };

  return (
    <div className="flex flex-col gap-4">
      {/* Real-time State Monitor Bar */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">3-State Vertical EKF</span>
          <div className="my-1">
            <span className="text-xl font-bold font-mono text-white">
              {(telemetry.heightCm / 100).toFixed(3)}
            </span>
            <span className="text-xs font-mono text-neutral-500 ml-1">m (kf_z)</span>
          </div>
          <span className="text-[10px] font-mono text-neutral-400">
            Vz: {telemetry.velDown.toFixed(3)} m/s
          </span>
        </div>

        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">LiDAR Tilt-Corrected</span>
          <div className="my-1">
            <span className="text-xl font-bold font-mono text-white">
              {telemetry.lidarCm.toFixed(1)}
            </span>
            <span className="text-xs font-mono text-neutral-500 ml-1">cm</span>
          </div>
          <span className="text-[10px] font-mono text-neutral-400">
            Source: {telemetry.altSource} [Valid: {telemetry.lidarCm >= 4.0 ? 'YES' : 'NO'}]
          </span>
        </div>

        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">Flow Earth Velocity</span>
          <div className="my-1 font-mono text-sm font-bold text-white flex justify-between">
            <span>Vx: {telemetry.velNorth.toFixed(2)}m/s</span>
            <span>Vy: {telemetry.velEast.toFixed(2)}m/s</span>
          </div>
          <span className="text-[10px] font-mono text-neutral-400">
            Quality: {telemetry.flowQuality} / 255
          </span>
        </div>

        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col justify-between">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">Attitude Filter (Madgwick)</span>
          <div className="my-1 font-mono text-xs font-bold text-white flex justify-between">
            <span>R: {telemetry.rollDeg.toFixed(1)}°</span>
            <span>P: {telemetry.pitchDeg.toFixed(1)}°</span>
            <span>Y: {telemetry.yawDeg.toFixed(0)}°</span>
          </div>
          <span className="text-[10px] font-mono text-neutral-400">
            Alpha Att: 0.991 | Yaw: 0.995
          </span>
        </div>
      </div>

      {/* Estimator Configuration Cards */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col gap-4">
        <div className="flex flex-wrap items-center justify-between gap-3 border-b border-neutral-800/80 pb-3 font-mono text-xs">
          <div className="flex items-center gap-2">
            <Layers className="w-4 h-4 text-red-600" />
            <h2 className="font-semibold uppercase tracking-wider text-neutral-200">
              State Estimator & Kalman Filter Parameters
            </h2>
          </div>

          <div className="flex items-center gap-2">
            <button
              onClick={handleReload}
              className="flex items-center gap-1.5 px-3 py-1.5 bg-neutral-900 text-neutral-300 rounded hover:bg-neutral-800 transition border border-neutral-800 cursor-pointer"
            >
              <RefreshCw className="w-3.5 h-3.5" />
              <span>Reload</span>
            </button>

            <button
              onClick={handleResetDefaults}
              disabled={isArmed}
              className="flex items-center gap-1.5 px-3 py-1.5 bg-neutral-900 text-neutral-400 rounded hover:bg-neutral-800 hover:text-red-500 transition border border-neutral-800 cursor-pointer disabled:opacity-40"
            >
              <RotateCcw className="w-3.5 h-3.5" />
              <span>Defaults</span>
            </button>

            <button
              onClick={handleSaveToFlash}
              disabled={isArmed}
              className={`flex items-center gap-1.5 px-3.5 py-1.5 font-bold rounded transition border cursor-pointer ${
                saveSuccess
                  ? 'bg-neutral-100 text-black border-white'
                  : 'bg-red-600 hover:bg-red-700 text-white border-red-600'
              } disabled:opacity-40`}
            >
              {saveSuccess ? <Check className="w-3.5 h-3.5" /> : <Save className="w-3.5 h-3.5" />}
              <span>{saveSuccess ? 'COMMITTED' : 'SAVE TO FLASH'}</span>
            </button>
          </div>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-3.5 max-h-[520px] overflow-y-auto pr-1">
          {params.map((p) => {
            const step = p.step || (p.max - p.min) / 100;
            return (
              <div
                key={p.name}
                className="bg-black p-4 rounded-lg border border-neutral-800 flex flex-col justify-between overflow-hidden min-w-0"
              >
                <div className="mb-2">
                  <div className="flex justify-between items-start gap-2 mb-1">
                    <span className="font-mono text-xs font-bold text-neutral-200 truncate min-w-0">
                      {p.name}
                    </span>
                    <span className="shrink-0 font-mono text-xs font-bold text-red-500 bg-neutral-900 px-2.5 py-0.5 rounded border border-neutral-800">
                      {p.value < 0.01 ? p.value.toFixed(5) : p.value < 0.1 ? p.value.toFixed(4) : p.value.toFixed(3)}
                    </span>
                  </div>
                  <p className="text-[11px] text-neutral-400 leading-tight">
                    {p.description}
                  </p>
                </div>

                <div className="mt-2 pt-2 border-t border-neutral-900 flex flex-col gap-1 w-full">
                  <input
                    type="range"
                    min={p.min}
                    max={p.max}
                    step={step}
                    value={p.value}
                    disabled={isArmed}
                    onChange={(e) => handleValueChange(p.name, parseFloat(e.target.value))}
                    className="w-full accent-red-600 cursor-pointer disabled:opacity-40"
                  />
                  <div className="flex justify-between items-center text-[10px] font-mono text-neutral-500 px-0.5">
                    <span>Min: {p.min}</span>
                    <span>Default: {p.defaultValue}</span>
                    <span>Max: {p.max}</span>
                  </div>
                </div>
              </div>
            );
          })}
        </div>
      </div>
    </div>
  );
};

