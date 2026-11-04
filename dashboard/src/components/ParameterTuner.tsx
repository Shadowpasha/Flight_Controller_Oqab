import React, { useState } from 'react';
import { Sliders, Save, RefreshCw, RotateCcw, Check, Compass, ShieldAlert, Cpu } from 'lucide-react';
import { serialService } from '../services/serialService';

export interface TunableParam {
  name: string;
  type: 'float' | 'int' | 'bool' | 'enum';
  value: number;
  defaultValue: number;
  min: number;
  max: number;
  step?: number;
  description: string;
  category: 'IMU & Sensors' | 'Safety & Failsafe' | 'Altitude & Hover' | 'Attitude Control' | 'Position (GPS/Flow)';
  options?: { value: number; label: string }[];
}

interface Props {
  isArmed: boolean;
}

export const ParameterTuner: React.FC<Props> = ({ isArmed }) => {
  const [params, setParams] = useState<TunableParam[]>([
    // --- 1. IMU & Sensors ---
    {
      name: 'SENS_BOARD_ROT',
      type: 'enum',
      value: 0,
      defaultValue: 0,
      min: 0,
      max: 5,
      description: 'IMU Board Mounting Orientation (0=Default baseline for current drone)',
      category: 'IMU & Sensors',
      options: [
        { value: 0, label: '0: Default (+Y Fwd, +X Right, +Z Up)' },
        { value: 1, label: '1: None (+X Fwd, +Y Right, +Z Up)' },
        { value: 2, label: '2: Yaw 90° CW' },
        { value: 3, label: '3: Yaw 180° CW' },
        { value: 4, label: '4: Yaw 270° CW' },
        { value: 5, label: '5: Pitch 180° (Inverted)' },
      ],
    },
    {
      name: 'SENS_FLOW_ROT',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: 0.0,
      max: 360.0,
      step: 1.0,
      description: 'Optical flow mounting yaw rotation clockwise [deg]',
      category: 'IMU & Sensors',
    },
    {
      name: 'SYS_USE_FLOW',
      type: 'bool',
      value: 1,
      defaultValue: 1,
      min: 0,
      max: 1,
      description: 'Enable Mateksys 3901-L0X Optical Flow Subsystem',
      category: 'IMU & Sensors',
    },
    {
      name: 'SYS_USE_GPS',
      type: 'bool',
      value: 0,
      defaultValue: 0,
      min: 0,
      max: 1,
      description: 'Enable GPS Navigation Subsystem',
      category: 'IMU & Sensors',
    },
    {
      name: 'MAG_DECL_DEG',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -180.0,
      max: 180.0,
      step: 0.5,
      description: 'Magnetic Declination angle at your geographic location [deg]',
      category: 'IMU & Sensors',
    },

    // --- 2. Safety & Failsafe ---
    {
      name: 'SAFE_MAX_TILT',
      type: 'float',
      value: 40.0,
      defaultValue: 40.0,
      min: 15.0,
      max: 60.0,
      step: 1.0,
      description: 'Crash Protection: Instant motor shutdown if tilt exceeds limit [deg]',
      category: 'Safety & Failsafe',
    },
    {
      name: 'SAFE_TIMEOUT_S',
      type: 'int',
      value: 30,
      defaultValue: 30,
      min: 5,
      max: 120,
      step: 1,
      description: 'Communication Watchdog: Disarm if no keepalive packet received [s]',
      category: 'Safety & Failsafe',
    },
    {
      name: 'SAFE_CEIL_CM',
      type: 'float',
      value: 185.0,
      defaultValue: 185.0,
      min: 50.0,
      max: 200.0,
      step: 5.0,
      description: 'Ceiling Guard: Trigger controlled descent before ToF saturation [cm]',
      category: 'Safety & Failsafe',
    },

    // --- 3. Altitude & Takeoff ---
    {
      name: 'MPC_THR_HOVER',
      type: 'int',
      value: 1380,
      defaultValue: 1380,
      min: 1150,
      max: 1600,
      step: 5,
      description: 'Nominal hover throttle baseline pulse [us]',
      category: 'Altitude & Hover',
    },
    {
      name: 'MPC_Z_TARGET',
      type: 'float',
      value: 45.0,
      defaultValue: 45.0,
      min: 15.0,
      max: 150.0,
      step: 1.0,
      description: 'Autonomous mission hover altitude target above ground [cm]',
      category: 'Altitude & Hover',
    },
    {
      name: 'MPC_Z_CLIMB',
      type: 'float',
      value: 0.18,
      defaultValue: 0.18,
      min: 0.05,
      max: 0.80,
      step: 0.01,
      description: 'Maximum autonomous climb rate [m/s]',
      category: 'Altitude & Hover',
    },
    {
      name: 'MPC_Z_DESC',
      type: 'float',
      value: 0.35,
      defaultValue: 0.35,
      min: 0.05,
      max: 1.20,
      step: 0.01,
      description: 'Maximum autonomous descent rate [m/s]',
      category: 'Altitude & Hover',
    },
    {
      name: 'MPC_RAMP_UP',
      type: 'float',
      value: 40.0,
      defaultValue: 40.0,
      min: 10.0,
      max: 120.0,
      step: 5.0,
      description: 'Smooth takeoff ground ramp speed [us/s]',
      category: 'Altitude & Hover',
    },
    {
      name: 'MPC_RAMP_DN',
      type: 'float',
      value: 35.0,
      defaultValue: 35.0,
      min: 10.0,
      max: 100.0,
      step: 5.0,
      description: 'Landing throttle descent ramp speed [us/s]',
      category: 'Altitude & Hover',
    },
    {
      name: 'MPC_Z_P',
      type: 'float',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.1,
      max: 5.0,
      step: 0.1,
      description: 'Altitude position proportional gain',
      category: 'Altitude & Hover',
    },
    {
      name: 'MPC_Z_VEL_P',
      type: 'float',
      value: 1.8,
      defaultValue: 1.8,
      min: 0.2,
      max: 8.0,
      step: 0.1,
      description: 'Altitude velocity proportional damping gain',
      category: 'Altitude & Hover',
    },

    // --- 4. Attitude Control ---
    {
      name: 'MC_ROLL_P',
      type: 'float',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Cascade roll angle outer proportional gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_ROLLRATE_P',
      type: 'float',
      value: 0.625,
      defaultValue: 0.625,
      min: 0.05,
      max: 5.0,
      step: 0.025,
      description: 'Inner rate roll proportional gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_ROLLRATE_I',
      type: 'float',
      value: 2.1,
      defaultValue: 2.1,
      min: 0.0,
      max: 10.0,
      step: 0.1,
      description: 'Inner rate roll integral gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_ROLLRATE_D',
      type: 'float',
      value: 0.0088,
      defaultValue: 0.0088,
      min: 0.0,
      max: 0.10,
      step: 0.0005,
      description: 'Inner rate roll derivative damping gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCH_P',
      type: 'float',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Cascade pitch angle outer proportional gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCHRATE_P',
      type: 'float',
      value: 0.625,
      defaultValue: 0.625,
      min: 0.05,
      max: 5.0,
      step: 0.025,
      description: 'Inner rate pitch proportional gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCHRATE_I',
      type: 'float',
      value: 2.1,
      defaultValue: 2.1,
      min: 0.0,
      max: 10.0,
      step: 0.1,
      description: 'Inner rate pitch integral gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCHRATE_D',
      type: 'float',
      value: 0.0088,
      defaultValue: 0.0088,
      min: 0.0,
      max: 0.10,
      step: 0.0005,
      description: 'Inner rate pitch derivative damping gain',
      category: 'Attitude Control',
    },
    {
      name: 'MC_YAWRATE_P',
      type: 'float',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Yaw rate proportional gain',
      category: 'Attitude Control',
    },

    // --- 5. Position (GPS/Flow) ---
    {
      name: 'MPC_XY_P',
      type: 'float',
      value: 1.2,
      defaultValue: 1.2,
      min: 0.1,
      max: 5.0,
      step: 0.1,
      description: 'Horizontal position proportional tracking gain',
      category: 'Position (GPS/Flow)',
    },
    {
      name: 'MPC_XY_VEL_P',
      type: 'float',
      value: 3.5,
      defaultValue: 3.5,
      min: 0.5,
      max: 10.0,
      step: 0.1,
      description: 'Horizontal velocity proportional damping gain',
      category: 'Position (GPS/Flow)',
    },
    {
      name: 'MPC_XY_MAX_VEL',
      type: 'float',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.2,
      max: 5.0,
      step: 0.1,
      description: 'Maximum horizontal autonomous speed [m/s]',
      category: 'Position (GPS/Flow)',
    },
    {
      name: 'MPC_XY_MAX_TILT',
      type: 'float',
      value: 12.0,
      defaultValue: 12.0,
      min: 3.0,
      max: 30.0,
      step: 1.0,
      description: 'Maximum horizontal position controller tilt angle [deg]',
      category: 'Position (GPS/Flow)',
    },
  ]);

  const [activeCategory, setActiveCategory] = useState<string>('All');
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
    if (confirm('Reset all parameters to factory defaults?')) {
      serialService.sendCommand('param reset');
      setParams((prev) => prev.map((p) => ({ ...p, value: p.defaultValue })));
    }
  };

  const categories = [
    'All',
    'IMU & Sensors',
    'Safety & Failsafe',
    'Altitude & Hover',
    'Attitude Control',
    'Position (GPS/Flow)',
  ];

  const filteredParams = activeCategory === 'All'
    ? params
    : params.filter((p) => p.category === activeCategory);

  const formatDisplayValue = (p: TunableParam): string => {
    if (p.type === 'int' || p.type === 'enum') return p.value.toFixed(0);
    if (p.type === 'bool') return p.value > 0.5 ? 'ON' : 'OFF';
    if (p.step && p.step < 0.01) return p.value.toFixed(4);
    if (p.step && p.step < 0.1) return p.value.toFixed(3);
    return p.value.toFixed(1);
  };

  return (
    <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col gap-4">
      {/* Header Bar */}
      <div className="flex flex-wrap items-center justify-between gap-3 border-b border-neutral-800/80 pb-3">
        <div className="flex items-center gap-2">
          <Sliders className="w-4 h-4 text-red-600" />
          <h2 className="text-xs font-mono font-semibold uppercase tracking-wider text-neutral-200">
            System Parameters ({params.length} Tunable Items)
          </h2>
        </div>

        <div className="flex items-center gap-2 font-mono text-xs">
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

      {/* Category Tabs */}
      <div className="flex flex-wrap gap-1.5 font-mono text-xs">
        {categories.map((cat) => (
          <button
            key={cat}
            onClick={() => setActiveCategory(cat)}
            className={`px-3 py-1.5 rounded transition cursor-pointer border ${
              activeCategory === cat
                ? 'bg-red-600 text-white border-red-600 font-semibold'
                : 'bg-neutral-900 text-neutral-400 border-neutral-800 hover:text-neutral-200'
            }`}
          >
            {cat}
          </button>
        ))}
      </div>

      {/* Parameter Cards Grid with no-overflow containment */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-3.5 max-h-[500px] overflow-y-auto pr-1">
        {filteredParams.map((p) => {
          const step = p.step || (p.type === 'int' || p.type === 'enum' ? 1 : (p.max - p.min) / 100);

          return (
            <div
              key={p.name}
              className="bg-black p-4 rounded-lg border border-neutral-800 flex flex-col justify-between overflow-hidden min-w-0"
            >
              {/* Header row with Title and Value Badge */}
              <div className="mb-2">
                <div className="flex justify-between items-start gap-2 mb-1">
                  <span className="font-mono text-xs font-bold text-neutral-200 truncate min-w-0">
                    {p.name}
                  </span>
                  <span className="shrink-0 font-mono text-xs font-bold text-red-500 bg-neutral-900 px-2.5 py-0.5 rounded border border-neutral-800">
                    {formatDisplayValue(p)}
                  </span>
                </div>
                <p className="text-[11px] text-neutral-400 leading-tight">
                  {p.description}
                </p>
              </div>

              {/* Control Area */}
              <div className="mt-2 pt-2 border-t border-neutral-900">
                {p.type === 'enum' && p.options ? (
                  /* Dropdown for Enums (like SENS_BOARD_ROT) */
                  <select
                    value={p.value}
                    disabled={isArmed}
                    onChange={(e) => handleValueChange(p.name, parseInt(e.target.value, 10))}
                    className="w-full bg-neutral-900 text-neutral-200 font-mono text-xs px-2.5 py-1.5 rounded border border-neutral-800 outline-none cursor-pointer focus:border-red-600 disabled:opacity-40"
                  >
                    {p.options.map((opt) => (
                      <option key={opt.value} value={opt.value} className="bg-neutral-950 text-neutral-200">
                        {opt.label}
                      </option>
                    ))}
                  </select>
                ) : p.type === 'bool' ? (
                  /* Toggle button for Booleans */
                  <div className="flex items-center justify-between">
                    <span className="text-[11px] font-mono text-neutral-500">Status</span>
                    <button
                      disabled={isArmed}
                      onClick={() => handleValueChange(p.name, p.value > 0.5 ? 0 : 1)}
                      className={`px-4 py-1 text-xs font-mono font-bold rounded transition border cursor-pointer ${
                        p.value > 0.5
                          ? 'bg-red-600 text-white border-red-600'
                          : 'bg-neutral-900 text-neutral-500 border-neutral-800 hover:text-neutral-300'
                      } disabled:opacity-40`}
                    >
                      {p.value > 0.5 ? 'ENABLED' : 'DISABLED'}
                    </button>
                  </div>
                ) : (
                  /* Range Slider for Continuous Floats and Ints */
                  <div className="flex flex-col gap-1 w-full">
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
                )}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};
