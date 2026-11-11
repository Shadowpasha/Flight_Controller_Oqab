import React, { useState, useEffect } from 'react';
import {
  Compass,
  Target,
  Gauge,
  Eye,
  RefreshCw,
  CheckCircle2,
  AlertTriangle,
  Layers,
  Sparkles,
  ShieldAlert,
} from 'lucide-react';
import { serialService } from '../services/serialService';
import { DroneTelemetry } from '../types';
import { FlowDerotationChart } from './FlowDerotationChart';

interface SensorsCalibrationProps {
  telemetry: DroneTelemetry;
  isArmed: boolean;
}

export const SensorsCalibrationPanel: React.FC<SensorsCalibrationProps> = ({
  telemetry,
  isArmed,
}) => {
  const [activeCal, setActiveCal] = useState<string | null>(null);
  const [calLogs, setCalLogs] = useState<string[]>([]);
  const [lastCompleted, setLastCompleted] = useState<string | null>(null);

  useEffect(() => {
    const unsub = serialService.onLine((line) => {
      // Capture calibration lines
      if (
        line.includes('CALIBRATION') ||
        line.includes('Auto-Calibration') ||
        line.includes('Offsets') ||
        line.includes('Bias') ||
        line.includes('COMPLETE') ||
        line.includes('SUCCESSFUL') ||
        line.includes('ERR: Cannot calibrate') ||
        line.includes('Correlation') ||
        line.includes('rocking in progress') ||
        line.includes('Samples') ||
        line.includes('Saved to Flash')
      ) {
        setCalLogs((prev) => [...prev.slice(-15), line]);

        if (line.includes('COMPLETE') || line.includes('complete') || line.includes('SUCCESSFUL')) {
          setActiveCal(null);
          setLastCompleted(line);
        } else if (line.includes('ERR:')) {
          setActiveCal(null);
        }
      }
    });
    return unsub;
  }, []);

  const runCalibration = (cmd: string, name: string) => {
    if (isArmed) {
      alert('Cannot calibrate while motors are armed!');
      return;
    }
    setActiveCal(name);
    setCalLogs([`>>> Sent '${cmd}' command to flight controller...`]);
    serialService.sendCommand(cmd);
  };

  return (
    <div className="flex flex-col gap-4">
      {/* Top Banner */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-wrap items-center justify-between gap-4">
        <div className="flex items-center gap-3">
          <div className="w-10 h-10 rounded bg-black border border-red-600 flex items-center justify-center">
            <Compass className="w-5 h-5 text-red-600" />
          </div>
          <div>
            <div className="flex items-center gap-2">
              <h2 className="text-sm font-mono font-bold uppercase tracking-wider text-white">
                Sensors & Hardware Calibration
              </h2>
              <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-neutral-900 text-red-500 border border-neutral-800">
                Flash NVS Persistent
              </span>
            </div>
            <p className="text-xs font-mono text-neutral-400 mt-0.5">
              Calibrate attitude horizon, gyro zero-rate bias, 3D compass, and optical flow derotation.
            </p>
          </div>
        </div>

        {isArmed ? (
          <div className="flex items-center gap-2 px-3 py-1.5 rounded bg-red-950/60 border border-red-700/80 text-red-400 text-xs font-mono font-semibold">
            <ShieldAlert className="w-4 h-4 text-red-500" />
            <span>DISARM REQUIRED FOR CALIBRATION</span>
          </div>
        ) : (
          <button
            onClick={() => runCalibration('cal all', 'Full Sensor Suite')}
            disabled={activeCal !== null}
            className="px-4 py-2 bg-red-600 hover:bg-red-700 text-white rounded text-xs font-mono font-bold transition flex items-center gap-2 cursor-pointer disabled:opacity-40"
          >
            <RefreshCw className={`w-3.5 h-3.5 ${activeCal ? 'animate-spin' : ''}`} />
            <span>RUN FULL AUTO-CALIBRATION</span>
          </button>
        )}
      </div>

      {/* Live Sensor State Strip */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">IMU Attitude</span>
          <div className="my-1 font-mono text-xs font-bold text-white flex justify-between">
            <span>R: {telemetry.rollDeg >= 0 ? '+' : ''}{telemetry.rollDeg.toFixed(1)}°</span>
            <span>P: {telemetry.pitchDeg >= 0 ? '+' : ''}{telemetry.pitchDeg.toFixed(1)}°</span>
          </div>
          <span className="text-[10px] font-mono text-neutral-500">MPU-6050 6-DoF @ 250 Hz</span>
        </div>

        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">True North Heading</span>
          <span className="my-1 font-mono text-sm font-bold text-white">
            {telemetry.yawDeg.toFixed(0)}° <span className="text-xs font-normal text-neutral-500">Azimuth</span>
          </span>
          <span className="text-[10px] font-mono text-neutral-500">QMC5883P / GY-87 Compass</span>
        </div>

        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">Ground Altitude</span>
          <span className="my-1 font-mono text-sm font-bold text-white">
            {telemetry.heightCm.toFixed(1)} <span className="text-xs font-normal text-neutral-500">cm AGL</span>
          </span>
          <span className="text-[10px] font-mono text-neutral-500">Source: {telemetry.altSource}</span>
        </div>

        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-3.5 flex flex-col">
          <span className="text-[11px] font-mono text-neutral-400 uppercase">Optical Flow Status</span>
          <div className="my-1 font-mono text-xs font-bold text-white flex justify-between">
            <span>Q: {telemetry.flowQuality} / 255</span>
            <span>Rot: {telemetry.flowRotDeg.toFixed(0)}°</span>
          </div>
          <span className="text-[10px] font-mono text-neutral-500">Mateksys 3901-L0X MSPv2</span>
        </div>
      </div>

      {/* Main Calibration Cards Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Card 1: Level Accelerometer Calibration */}
        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col justify-between">
          <div className="flex flex-col gap-2">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2 font-mono text-xs font-bold text-white uppercase">
                <Target className="w-4 h-4 text-red-600" />
                <span>Level Horizon Calibration</span>
              </div>
              <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-neutral-900 text-neutral-400 border border-neutral-800">
                cal level
              </span>
            </div>

            <p className="text-xs text-neutral-400 leading-relaxed mt-1">
              Place the drone on a flat, leveled workbench and keep it still for 3 seconds. Measures accelerometer
              gravity vectors to establish zero-degree physical roll and pitch offsets. Saved level offsets persist
              across boots so slanted takeoff surfaces do not skew your flight horizon.
            </p>

            <div className="mt-3 p-2.5 rounded bg-black border border-neutral-800/80 font-mono text-[11px] text-neutral-300 flex items-center justify-between">
              <span className="text-neutral-500">Target Parameters:</span>
              <span className="text-neutral-200">CAL_ROLL_OFF, CAL_PITCH_OFF, CAL_ACC_X_OFF</span>
            </div>
          </div>

          <div className="mt-4 pt-3 border-t border-neutral-800/80 flex items-center justify-between">
            <span className="text-xs font-mono text-neutral-500">
              Live: R {telemetry.rollDeg >= 0 ? '+' : ''}{telemetry.rollDeg.toFixed(1)}° | P {telemetry.pitchDeg >= 0 ? '+' : ''}{telemetry.pitchDeg.toFixed(1)}°
            </span>
            <button
              onClick={() => runCalibration('cal level', 'Level Horizon')}
              disabled={isArmed || activeCal !== null}
              className={`px-3.5 py-1.5 rounded text-xs font-mono font-bold transition flex items-center gap-1.5 ${
                activeCal === 'Level Horizon'
                  ? 'bg-neutral-800 text-neutral-300 border border-neutral-700'
                  : 'bg-red-600 hover:bg-red-700 text-white cursor-pointer'
              } disabled:opacity-40`}
            >
              {activeCal === 'Level Horizon' ? (
                <>
                  <RefreshCw className="w-3.5 h-3.5 animate-spin" />
                  <span>Calibrating...</span>
                </>
              ) : (
                <span>Calibrate Level</span>
              )}
            </button>
          </div>
        </div>

        {/* Card 2: Gyroscope Bias Calibration */}
        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col justify-between">
          <div className="flex flex-col gap-2">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2 font-mono text-xs font-bold text-white uppercase">
                <Gauge className="w-4 h-4 text-red-600" />
                <span>Gyroscope Bias Zeroing</span>
              </div>
              <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-neutral-900 text-neutral-400 border border-neutral-800">
                cal gyro
              </span>
            </div>

            <p className="text-xs text-neutral-400 leading-relaxed mt-1">
              Rest the drone on a stable surface with no vibration or movement. Averages 300 samples across the roll,
              pitch, and yaw angular rate gyros to subtract sensor zero-rate bias offsets. Prevents attitude angle drift
              and ensures crisp rate-damping response in inner flight loops.
            </p>

            <div className="mt-3 p-2.5 rounded bg-black border border-neutral-800/80 font-mono text-[11px] text-neutral-300 flex items-center justify-between">
              <span className="text-neutral-500">Target Parameters:</span>
              <span className="text-neutral-200">CAL_GYRO_X, CAL_GYRO_Y, CAL_GYRO_Z</span>
            </div>
          </div>

          <div className="mt-4 pt-3 border-t border-neutral-800/80 flex items-center justify-between">
            <span className="text-xs font-mono text-neutral-500">Duration: ~2.0 seconds</span>
            <button
              onClick={() => runCalibration('cal gyro', 'Gyro Bias')}
              disabled={isArmed || activeCal !== null}
              className={`px-3.5 py-1.5 rounded text-xs font-mono font-bold transition flex items-center gap-1.5 ${
                activeCal === 'Gyro Bias'
                  ? 'bg-neutral-800 text-neutral-300 border border-neutral-700'
                  : 'bg-red-600 hover:bg-red-700 text-white cursor-pointer'
              } disabled:opacity-40`}
            >
              {activeCal === 'Gyro Bias' ? (
                <>
                  <RefreshCw className="w-3.5 h-3.5 animate-spin" />
                  <span>Calibrating...</span>
                </>
              ) : (
                <span>Calibrate Gyro</span>
              )}
            </button>
          </div>
        </div>

        {/* Card 3: 3D Compass Magnetometer Calibration */}
        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col justify-between">
          <div className="flex flex-col gap-2">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2 font-mono text-xs font-bold text-white uppercase">
                <Compass className="w-4 h-4 text-red-600" />
                <span>3D Compass Magnetometer</span>
              </div>
              <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-neutral-900 text-neutral-400 border border-neutral-800">
                cal mag
              </span>
            </div>

            <p className="text-xs text-neutral-400 leading-relaxed mt-1">
              Start the 12-second routine and slowly rotate the drone in a 3D figure-8 pattern through all orientations
              away from metal objects or computer screens. Calculates hard-iron magnetic center coordinates (X, Y, Z) and
              saves them permanently to Flash to prevent heading toilet-bowling.
            </p>

            <div className="mt-3 p-2.5 rounded bg-black border border-neutral-800/80 font-mono text-[11px] text-neutral-300 flex items-center justify-between">
              <span className="text-neutral-500">Target Parameters:</span>
              <span className="text-neutral-200">CAL_MAG_X, CAL_MAG_Y, CAL_MAG_Z, MAG_DECL_DEG</span>
            </div>
          </div>

          <div className="mt-4 pt-3 border-t border-neutral-800/80 flex items-center justify-between">
            <span className="text-xs font-mono text-neutral-500">
              Current Heading: {telemetry.yawDeg.toFixed(0)}°
            </span>
            <button
              onClick={() => runCalibration('cal mag', '3D Compass')}
              disabled={isArmed || activeCal !== null}
              className={`px-3.5 py-1.5 rounded text-xs font-mono font-bold transition flex items-center gap-1.5 ${
                activeCal === '3D Compass'
                  ? 'bg-neutral-800 text-neutral-300 border border-neutral-700'
                  : 'bg-red-600 hover:bg-red-700 text-white cursor-pointer'
              } disabled:opacity-40`}
            >
              {activeCal === '3D Compass' ? (
                <>
                  <RefreshCw className="w-3.5 h-3.5 animate-spin" />
                  <span>Rotate 3D...</span>
                </>
              ) : (
                <span>Calibrate Compass</span>
              )}
            </button>
          </div>
        </div>

        {/* Card 4: Optical Flow Derotation Auto-Calibration */}
        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col justify-between">
          <div className="flex flex-col gap-2">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2 font-mono text-xs font-bold text-white uppercase">
                <Eye className="w-4 h-4 text-red-600" />
                <span>Optical Flow Derotation</span>
              </div>
              <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-neutral-900 text-neutral-400 border border-neutral-800">
                cal flow
              </span>
            </div>

            <p className="text-xs text-neutral-400 leading-relaxed mt-1">
              Hold the drone approximately 1.0 meter above a textured floor and gently rock it in roll and pitch for
              10 seconds without translating horizontally. Solves for the exact physical scale factor (rad/count), checks
              mounting correlation direction, and commits the result directly to Flash NVS.
            </p>

            <div className="mt-3 p-2.5 rounded bg-black border border-neutral-800/80 font-mono text-[11px] text-neutral-300 flex items-center justify-between">
              <span className="text-neutral-500">Target Parameters:</span>
              <span className="text-neutral-200">FLOW_RAD_SCALE, SENS_FLOW_ROT</span>
            </div>
          </div>

          <div className="mt-4 pt-3 border-t border-neutral-800/80 flex items-center justify-between">
            <span className="text-xs font-mono text-neutral-500">
              Quality: {telemetry.flowQuality}/255
            </span>
            <button
              onClick={() => runCalibration('cal flow', 'Optical Flow Derotation')}
              disabled={isArmed || activeCal !== null}
              className={`px-3.5 py-1.5 rounded text-xs font-mono font-bold transition flex items-center gap-1.5 ${
                activeCal === 'Optical Flow Derotation'
                  ? 'bg-neutral-800 text-neutral-300 border border-neutral-700'
                  : 'bg-red-600 hover:bg-red-700 text-white cursor-pointer'
              } disabled:opacity-40`}
            >
              {activeCal === 'Optical Flow Derotation' ? (
                <>
                  <RefreshCw className="w-3.5 h-3.5 animate-spin" />
                  <span>Rocking (10s)...</span>
                </>
              ) : (
                <span>Calibrate Flow</span>
              )}
            </button>
          </div>
        </div>
      </div>

      {/* Optical Flow Derotation Visual Analysis Graph */}
      <FlowDerotationChart isArmed={isArmed} />

      {/* Live Calibration Console Feedback Box */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-4 flex flex-col gap-2">
        <div className="flex items-center justify-between border-b border-neutral-800/80 pb-2">
          <div className="flex items-center gap-2">
            <Sparkles className="w-3.5 h-3.5 text-red-600" />
            <span className="text-xs font-mono font-semibold uppercase tracking-wider text-neutral-300">
              Calibration Terminal Feedback
            </span>
          </div>
          {lastCompleted && (
            <div className="flex items-center gap-1.5 text-emerald-400 font-mono text-xs">
              <CheckCircle2 className="w-3.5 h-3.5" />
              <span>Operation Complete</span>
            </div>
          )}
        </div>

        <div className="bg-black rounded p-3 font-mono text-xs space-y-1 max-h-48 overflow-y-auto border border-neutral-900 select-text">
          {calLogs.length === 0 ? (
            <div className="text-neutral-600 py-3 text-center">
              Click any calibration button above to begin. Real-time feedback and Flash commit confirmations appear here.
            </div>
          ) : (
            calLogs.map((log, idx) => (
              <div
                key={idx}
                className={
                  log.includes('COMPLETE') || log.includes('SUCCESSFUL')
                    ? 'text-emerald-400 font-semibold'
                    : log.includes('ERR:')
                    ? 'text-red-400 font-bold'
                    : log.includes('>>>')
                    ? 'text-neutral-200 font-semibold'
                    : 'text-neutral-400'
                }
              >
                {log}
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
};

