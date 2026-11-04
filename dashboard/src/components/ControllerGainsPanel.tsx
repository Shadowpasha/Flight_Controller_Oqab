import React, { useState } from 'react';
import { Cpu, Sliders, RefreshCw, RotateCcw, Save, Check, ShieldCheck } from 'lucide-react';
import { serialService } from '../services/serialService';

interface Props {
  activeController: string;
  isArmed: boolean;
}

interface GainParam {
  name: string;
  value: number;
  defaultValue: number;
  min: number;
  max: number;
  step?: number;
  description: string;
  group: 'Attitude PID' | 'Sliding Mode Control' | 'Backstepping / MDBCI' | 'Altitude PID' | 'Position PID (GPS/Flow)';
}

export const ControllerGainsPanel: React.FC<Props> = ({ activeController, isArmed }) => {
  const [params, setParams] = useState<GainParam[]>([
    // --- Attitude PID ---
    {
      name: 'MC_ROLL_P',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Cascade outer loop roll angle proportional gain [deg -> deg/s]',
      group: 'Attitude PID',
    },
    {
      name: 'MC_ROLLRATE_P',
      value: 0.625,
      defaultValue: 0.625,
      min: 0.05,
      max: 5.0,
      step: 0.025,
      description: 'Inner rate roll proportional response gain',
      group: 'Attitude PID',
    },
    {
      name: 'MC_ROLLRATE_I',
      value: 2.1,
      defaultValue: 2.1,
      min: 0.0,
      max: 10.0,
      step: 0.1,
      description: 'Inner rate roll integral windup rejection gain',
      group: 'Attitude PID',
    },
    {
      name: 'MC_ROLLRATE_D',
      value: 0.0088,
      defaultValue: 0.0088,
      min: 0.0,
      max: 0.10,
      step: 0.0005,
      description: 'Inner rate roll derivative gyro damping gain',
      group: 'Attitude PID',
    },
    {
      name: 'MC_PITCH_P',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Cascade outer loop pitch angle proportional gain [deg -> deg/s]',
      group: 'Attitude PID',
    },
    {
      name: 'MC_PITCHRATE_P',
      value: 0.625,
      defaultValue: 0.625,
      min: 0.05,
      max: 5.0,
      step: 0.025,
      description: 'Inner rate pitch proportional response gain',
      group: 'Attitude PID',
    },
    {
      name: 'MC_PITCHRATE_I',
      value: 2.1,
      defaultValue: 2.1,
      min: 0.0,
      max: 10.0,
      step: 0.1,
      description: 'Inner rate pitch integral windup rejection gain',
      group: 'Attitude PID',
    },
    {
      name: 'MC_PITCHRATE_D',
      value: 0.0088,
      defaultValue: 0.0088,
      min: 0.0,
      max: 0.10,
      step: 0.0005,
      description: 'Inner rate pitch derivative gyro damping gain',
      group: 'Attitude PID',
    },
    {
      name: 'MC_YAWRATE_P',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Yaw axis rate proportional gain',
      group: 'Attitude PID',
    },

    // --- Sliding Mode Control (SMC) ---
    {
      name: 'SMC_KP_ROLL',
      value: 1.2,
      defaultValue: 1.2,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC roll reaching law proportional gain',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_KSW_ROLL',
      value: 0.8,
      defaultValue: 0.8,
      min: 0.05,
      max: 5.0,
      step: 0.05,
      description: 'SMC roll discontinuous switching gain (robust disturbance rejection)',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_LAMBDA_R',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC roll sliding surface slope lambda',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_PHI_ROLL',
      value: 0.3,
      defaultValue: 0.3,
      min: 0.05,
      max: 2.0,
      step: 0.05,
      description: 'SMC roll boundary layer thickness (chattering mitigation)',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_KP_PITCH',
      value: 1.2,
      defaultValue: 1.2,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC pitch reaching law proportional gain',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_KSW_PITCH',
      value: 0.8,
      defaultValue: 0.8,
      min: 0.05,
      max: 5.0,
      step: 0.05,
      description: 'SMC pitch discontinuous switching gain',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_LAMBDA_P',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC pitch sliding surface slope lambda',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_PHI_PITCH',
      value: 0.3,
      defaultValue: 0.3,
      min: 0.05,
      max: 2.0,
      step: 0.05,
      description: 'SMC pitch boundary layer thickness',
      group: 'Sliding Mode Control',
    },
    {
      name: 'SMC_KP_YAW',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC yaw axis proportional gain',
      group: 'Sliding Mode Control',
    },

    // --- Backstepping / MDBCI ---
    {
      name: 'BS_C1_ROLL',
      value: 1.8,
      defaultValue: 1.8,
      min: 0.2,
      max: 10.0,
      step: 0.1,
      description: 'MDBCI virtual control stabilizing bandwidth c1 (attitude error convergence rate)',
      group: 'Backstepping / MDBCI',
    },
    {
      name: 'BS_C2_ROLL',
      value: 0.65,
      defaultValue: 0.65,
      min: 0.05,
      max: 5.0,
      step: 0.05,
      description: 'MDBCI rate error damping gain c2',
      group: 'Backstepping / MDBCI',
    },
    {
      name: 'BS_C1_PITCH',
      value: 1.8,
      defaultValue: 1.8,
      min: 0.2,
      max: 10.0,
      step: 0.1,
      description: 'MDBCI pitch virtual control stabilizing bandwidth c1',
      group: 'Backstepping / MDBCI',
    },
    {
      name: 'BS_C2_PITCH',
      value: 0.65,
      defaultValue: 0.65,
      min: 0.05,
      max: 5.0,
      step: 0.05,
      description: 'MDBCI pitch rate error damping gain c2',
      group: 'Backstepping / MDBCI',
    },
    {
      name: 'BS_LAMBDA_M',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 5.0,
      step: 0.1,
      description: 'MDBCI dual-boundary conditional integral ceiling lambda_m',
      group: 'Backstepping / MDBCI',
    },

    // --- Altitude PID ---
    {
      name: 'MPC_Z_P',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.1,
      max: 5.0,
      step: 0.1,
      description: 'Altitude position proportional gain [cm error -> target climb rate]',
      group: 'Altitude PID',
    },
    {
      name: 'MPC_Z_VEL_P',
      value: 1.8,
      defaultValue: 1.8,
      min: 0.2,
      max: 8.0,
      step: 0.1,
      description: 'Altitude velocity proportional damping gain [m/s error -> throttle adj us]',
      group: 'Altitude PID',
    },
    {
      name: 'MPC_Z_VEL_I',
      value: 0.50,
      defaultValue: 0.50,
      min: 0.0,
      max: 3.0,
      step: 0.05,
      description: 'Altitude velocity integral trim gain',
      group: 'Altitude PID',
    },
    {
      name: 'MPC_Z_VEL_D',
      value: 0.04,
      defaultValue: 0.04,
      min: 0.0,
      max: 0.5,
      step: 0.01,
      description: 'Altitude velocity derivative acceleration damping',
      group: 'Altitude PID',
    },

    // --- Position PID (GPS/Flow) ---
    {
      name: 'MPC_XY_P',
      value: 1.2,
      defaultValue: 1.2,
      min: 0.1,
      max: 5.0,
      step: 0.1,
      description: 'Outer horizontal position proportional gain [m -> target m/s]',
      group: 'Position PID (GPS/Flow)',
    },
    {
      name: 'MPC_XY_VEL_P',
      value: 3.5,
      defaultValue: 3.5,
      min: 0.5,
      max: 10.0,
      step: 0.1,
      description: 'Inner horizontal velocity proportional gain [m/s -> target tilt deg]',
      group: 'Position PID (GPS/Flow)',
    },
    {
      name: 'MPC_XY_VEL_I',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.0,
      max: 5.0,
      step: 0.1,
      description: 'Horizontal velocity integral wind rejection gain',
      group: 'Position PID (GPS/Flow)',
    },
    {
      name: 'MPC_XY_VEL_D',
      value: 0.15,
      defaultValue: 0.15,
      min: 0.0,
      max: 1.0,
      step: 0.02,
      description: 'Horizontal velocity derivative damping gain',
      group: 'Position PID (GPS/Flow)',
    },
  ]);

  const [activeGroup, setActiveGroup] = useState<string>('Attitude PID');
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
    if (confirm(`Reset ${activeGroup} gains to defaults?`)) {
      serialService.sendCommand('param reset');
      setParams((prev) => prev.map((p) => ({ ...p, value: p.defaultValue })));
    }
  };

  const groups = [
    'Attitude PID',
    'Sliding Mode Control',
    'Backstepping / MDBCI',
    'Altitude PID',
    'Position PID (GPS/Flow)',
  ];

  const filteredParams = params.filter((p) => p.group === activeGroup);

  return (
    <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col gap-4">
      {/* Top Controller Header */}
      <div className="flex flex-wrap items-center justify-between gap-3 border-b border-neutral-800/80 pb-3 font-mono text-xs">
        <div className="flex items-center gap-2">
          <Cpu className="w-4 h-4 text-red-600" />
          <h2 className="font-semibold uppercase tracking-wider text-neutral-200">
            Controller Feedback Gains
          </h2>
          <span className="text-[10px] bg-neutral-900 text-red-500 px-2 py-0.5 rounded border border-neutral-800">
            Active: {activeController}
          </span>
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

      {/* Sub-Group Navigation Tabs */}
      <div className="flex flex-wrap gap-1.5 font-mono text-xs">
        {groups.map((grp) => (
          <button
            key={grp}
            onClick={() => setActiveGroup(grp)}
            className={`px-3 py-1.5 rounded transition cursor-pointer border ${
              activeGroup === grp
                ? 'bg-red-600 text-white border-red-600 font-semibold'
                : 'bg-neutral-900 text-neutral-400 border-neutral-800 hover:text-neutral-200'
            }`}
          >
            {grp}
          </button>
        ))}
      </div>

      {/* Gain Parameter Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-3.5 max-h-[500px] overflow-y-auto pr-1">
        {filteredParams.map((p) => {
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
                    {p.step && p.step < 0.001 ? p.value.toFixed(4) : p.step && p.step < 0.05 ? p.value.toFixed(3) : p.value.toFixed(2)}
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
  );
};

