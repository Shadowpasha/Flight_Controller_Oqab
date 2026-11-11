import React, { useState, useEffect } from 'react';
import {
  Cable,
  Flame,
  Sliders,
  Terminal as TerminalIcon,
  Eye,
  Activity,
  Radio,
  Layers,
  Cpu,
  Compass,
} from 'lucide-react';
import oqabLogo from './assets/oqab-logo.png';
import { serialService } from './services/serialService';
import { DroneTelemetry, ConnectionState } from './types';
import { AttitudeVisualizer } from './components/AttitudeVisualizer';
import { TelemetryCards } from './components/TelemetryCards';
import { OpticalFlowPanel } from './components/OpticalFlowPanel';
import { ParameterTuner } from './components/ParameterTuner';
import { ConsoleTerminal } from './components/ConsoleTerminal';
import { LiveTelemetryChart } from './components/LiveTelemetryChart';
import { StateEstimationPanel } from './components/StateEstimationPanel';
import { ControllerGainsPanel } from './components/ControllerGainsPanel';
import { SensorsCalibrationPanel } from './components/SensorsCalibrationPanel';
import { FlowDerotationChart } from './components/FlowDerotationChart';

export const App: React.FC = () => {
  const [status, setStatus] = useState<ConnectionState>('disconnected');
  const [telemetry, setTelemetry] = useState<DroneTelemetry>(serialService.getLatestTelemetry());
  const [activeTab, setActiveTab] = useState<'overview' | 'sensors' | 'state_est' | 'ctrl_gains' | 'flow' | 'params' | 'terminal'>('overview');

  useEffect(() => {
    const unsubStatus = serialService.onStatus(setStatus);
    const unsubTelem = serialService.onTelemetry(setTelemetry);
    return () => {
      unsubStatus();
      unsubTelem();
    };
  }, []);

  useEffect(() => {
    if (activeTab === 'flow' && status === 'connected') {
      serialService.sendCommand('param get SENS_FLOW_ROT');
    }
  }, [activeTab, status]);

  const isSimMode = telemetry.isSimMode || telemetry.stateTag === 'SIM';

  const handleConnectToggle = () => {
    if (status === 'connected') {
      serialService.disconnect();
    } else {
      serialService.connect();
    }
  };

  const handleSimToggle = () => {
    serialService.sendCommand('sim');
  };

  const handleArmToggle = () => {
    if (telemetry.isArmed) {
      serialService.sendCommand('disarm');
    } else {
      if (confirm('CAUTION: Are you sure you want to ARM the drone? Motors will spool up!')) {
        serialService.sendCommand('arm');
      }
    }
  };

  const handleControllerSwitch = (ctrl: string) => {
    serialService.sendCommand(`ctrl ${ctrl}`);
  };

  const controllers = ['PID', 'SMC', 'ADRC', 'MDBCI', 'MDBCIS', 'CDI', 'CVI', 'NOI', 'VIABC'];

  return (
    <div className="min-h-screen bg-[#050505] text-neutral-100 flex flex-col font-sans">
      {/* Top Navbar */}
      <header className="bg-[#0a0a0a] border-b border-neutral-800 px-6 py-3 flex items-center justify-between sticky top-0 z-50">
        <div className="flex items-center gap-3">
          <div className="w-9 h-9 rounded-lg bg-neutral-950 border border-red-600/50 p-1 flex items-center justify-center shadow-[0_0_12px_rgba(220,38,38,0.25)]">
            <img src={oqabLogo} alt="Oqab Logo" className="w-full h-full object-contain filter drop-shadow-[0_0_3px_rgba(220,38,38,0.4)]" />
          </div>
          <div>
            <div className="flex items-center gap-2">
              <h1 className="text-xs font-mono font-bold tracking-widest uppercase text-white">
                OQAB GROUND CONTROL
              </h1>
              <span className="text-[10px] font-mono px-1.5 py-0.2 rounded bg-neutral-900 text-red-500 border border-neutral-800">
                AUTOPILOT
              </span>
            </div>
            <p className="text-[11px] font-mono text-neutral-400">ESP32 Dual-Core Avionics</p>
          </div>
        </div>

        {/* Center Controller Selector */}
        <div className="hidden lg:flex items-center gap-2 bg-black px-3 py-1.5 rounded border border-neutral-800 text-xs font-mono">
          <span className="text-neutral-400">ACTIVE CONTROLLER:</span>
          <select
            value={telemetry.controller}
            onChange={(e) => handleControllerSwitch(e.target.value)}
            disabled={telemetry.isArmed}
            className="bg-transparent font-bold text-red-500 outline-none cursor-pointer disabled:opacity-40"
          >
            {controllers.map((c) => (
              <option key={c} value={c} className="bg-neutral-900 text-white">
                {c}
              </option>
            ))}
          </select>
        </div>

        {/* Right Action Controls */}
        <div className="flex items-center gap-2.5 font-mono text-xs">
          <button
            onClick={handleSimToggle}
            disabled={status !== 'connected'}
            className={`px-3 py-1.5 rounded font-semibold flex items-center gap-1.5 transition border cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed ${
              isSimMode
                ? 'bg-amber-500/20 text-amber-400 border-amber-500 animate-pulse'
                : 'bg-neutral-900 text-neutral-400 hover:text-white border-neutral-800'
            }`}
            title={status !== 'connected' ? 'Connect serial link to send sim command' : 'Toggle hardware simulation mode (Motors stay at IDLE, loops active)'}
          >
            <Radio className="w-3.5 h-3.5 text-amber-500" />
            <span>{isSimMode ? 'SIM ACTIVE' : 'SIM MODE'}</span>
          </button>

          <button
            onClick={handleConnectToggle}
            className={`px-3.5 py-1.5 rounded font-semibold flex items-center gap-1.5 transition border cursor-pointer ${
              status === 'connected'
                ? 'bg-neutral-900 text-red-500 border-red-600/50 hover:bg-red-950/40 hover:text-red-400'
                : status === 'connecting'
                ? 'bg-neutral-900 text-neutral-300 border-neutral-700'
                : 'bg-neutral-900 hover:bg-neutral-800 text-white border-neutral-700'
            }`}
          >
            <Cable className="w-3.5 h-3.5" />
            <span>
              {status === 'connected'
                ? 'DISCONNECT'
                : status === 'connecting'
                ? 'CONNECTING...'
                : 'CONNECT LINK'}
            </span>
          </button>

          <button
            onClick={handleArmToggle}
            disabled={status !== 'connected'}
            className={`px-4 py-1.5 rounded font-bold uppercase tracking-wider flex items-center gap-1.5 transition border cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed ${
              telemetry.isArmed && !isSimMode
                ? 'bg-red-600 hover:bg-red-700 text-white border-red-600 animate-pulse'
                : isSimMode
                ? 'bg-amber-600/30 text-amber-400 border-amber-500'
                : 'bg-neutral-900 hover:bg-neutral-800 text-neutral-300 border-neutral-800'
            }`}
          >
            <Flame className="w-3.5 h-3.5 text-red-500" />
            <span>{telemetry.isArmed ? (isSimMode ? 'SIM RUNNING' : 'DISARM') : 'ARM'}</span>
          </button>
        </div>
      </header>

      {/* Main Container */}
      <main className="flex-1 max-w-7xl w-full mx-auto p-5 flex flex-col gap-4">
        {/* Navigation Tabs */}
        <div className="flex flex-wrap items-center justify-between border-b border-neutral-800/80 pb-2 gap-2">
          <div className="flex flex-wrap gap-1.5 font-mono text-xs">
            {[
              { id: 'overview', label: 'Overview', icon: Activity },
              { id: 'sensors', label: 'Sensors & Cal', icon: Compass },
              { id: 'state_est', label: 'State Estimation', icon: Layers },
              { id: 'ctrl_gains', label: 'Controller Gains', icon: Cpu },
              { id: 'flow', label: 'Optical Flow', icon: Eye },
              { id: 'params', label: 'All Parameters', icon: Sliders },
              { id: 'terminal', label: 'Console', icon: TerminalIcon },
            ].map((tab) => {
              const Icon = tab.icon;
              const isActive = activeTab === tab.id;
              return (
                <button
                  key={tab.id}
                  onClick={() => setActiveTab(tab.id as any)}
                  className={`flex items-center gap-2 px-3.5 py-1.5 rounded transition border cursor-pointer ${
                    isActive
                      ? 'bg-red-600 text-white border-red-600 font-bold'
                      : 'bg-neutral-900 text-neutral-400 border-neutral-800 hover:text-white'
                  }`}
                >
                  <Icon className="w-3.5 h-3.5" />
                  <span>{tab.label}</span>
                </button>
              );
            })}
          </div>

          <div className="flex items-center gap-2 text-xs font-mono text-neutral-400">
            <div
              className={`w-2 h-2 rounded-full ${
                status === 'connected'
                  ? isSimMode
                    ? 'bg-amber-400 animate-pulse'
                    : 'bg-red-600 animate-pulse'
                  : 'bg-neutral-600'
              }`}
            />
            <span>
              {status === 'connected'
                ? isSimMode
                  ? 'SIMULATION'
                  : 'LINK ACTIVE'
                : status === 'connecting'
                ? 'CONNECTING...'
                : 'OFFLINE'}
            </span>
          </div>
        </div>

        {/* Global Live Telemetry Stat Cards */}
        <TelemetryCards telemetry={telemetry} />

        {/* Active Tab View */}
        {activeTab === 'overview' && (
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 flex-1">
            <div className="lg:col-span-7 flex flex-col gap-4">
              <AttitudeVisualizer
                roll={telemetry.rollDeg}
                pitch={telemetry.pitchDeg}
                yaw={telemetry.yawDeg}
              />
            </div>
            <div className="lg:col-span-5 flex flex-col gap-4">
              <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col gap-4">
                <div className="flex items-center justify-between border-b border-neutral-800/80 pb-3 font-mono">
                  <div className="flex items-center gap-2">
                    <Radio className="w-4 h-4 text-red-600" />
                    <h2 className="text-xs font-semibold uppercase tracking-wider text-neutral-200">
                      Flight Dynamics & Navigation
                    </h2>
                  </div>
                  <span
                    className={`text-[11px] px-2 py-0.5 rounded border ${
                      telemetry.isArmed
                        ? 'bg-red-950 text-red-400 border-red-800 font-bold'
                        : 'bg-neutral-900 text-neutral-400 border-neutral-800'
                    }`}
                  >
                    {telemetry.stateTag}
                  </span>
                </div>

                <div className="grid grid-cols-2 gap-3 font-mono">
                  <div className="bg-black p-3 rounded border border-neutral-800 flex flex-col">
                    <span className="text-[11px] text-neutral-400">Position North (X)</span>
                    <span className="text-xl font-bold text-neutral-100 mt-1">
                      {telemetry.posNorth > 0 ? '+' : ''}
                      {telemetry.posNorth.toFixed(2)} <span className="text-xs text-neutral-500">m</span>
                    </span>
                    <span className="text-[10px] text-neutral-500 mt-0.5">
                      Vel: {telemetry.velNorth > 0 ? '+' : ''}
                      {telemetry.velNorth.toFixed(2)} m/s
                    </span>
                  </div>

                  <div className="bg-black p-3 rounded border border-neutral-800 flex flex-col">
                    <span className="text-[11px] text-neutral-400">Position East (Y)</span>
                    <span className="text-xl font-bold text-neutral-100 mt-1">
                      {telemetry.posEast > 0 ? '+' : ''}
                      {telemetry.posEast.toFixed(2)} <span className="text-xs text-neutral-500">m</span>
                    </span>
                    <span className="text-[10px] text-neutral-500 mt-0.5">
                      Vel: {telemetry.velEast > 0 ? '+' : ''}
                      {telemetry.velEast.toFixed(2)} m/s
                    </span>
                  </div>

                  <div className="bg-black p-3 rounded border border-neutral-800 flex flex-col">
                    <span className="text-[11px] text-neutral-400">Altitude AGL</span>
                    <span className="text-xl font-bold text-neutral-100 mt-1">
                      {telemetry.heightCm.toFixed(1)} <span className="text-xs text-neutral-500">cm</span>
                    </span>
                    <span className="text-[10px] text-neutral-500 mt-0.5">
                      Vz: {telemetry.velDown > 0 ? '+' : ''}
                      {telemetry.velDown.toFixed(2)} m/s
                    </span>
                  </div>

                  <div className="bg-black p-3 rounded border border-neutral-800 flex flex-col">
                    <span className="text-[11px] text-neutral-400">Distance From Home</span>
                    <span className="text-xl font-bold text-neutral-100 mt-1">
                      {telemetry.dist2d.toFixed(2)} <span className="text-xs text-neutral-500">m</span>
                    </span>
                    <span className="text-[10px] text-neutral-500 mt-0.5">Origin: {telemetry.homeOrigin}</span>
                  </div>
                </div>
              </div>

              <LiveTelemetryChart telemetry={telemetry} />
            </div>
          </div>
        )}

        {activeTab === 'sensors' && (
          <SensorsCalibrationPanel telemetry={telemetry} isArmed={telemetry.isArmed} />
        )}

        {activeTab === 'state_est' && (
          <StateEstimationPanel telemetry={telemetry} isArmed={telemetry.isArmed} />
        )}

        {activeTab === 'ctrl_gains' && (
          <ControllerGainsPanel activeController={telemetry.controller} isArmed={telemetry.isArmed} />
        )}

        {activeTab === 'flow' && (
          <div className="flex flex-col gap-4">
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
              <OpticalFlowPanel
                quality={telemetry.flowQuality}
                flowSpeed={telemetry.flowSpeed}
                flowRotDeg={telemetry.flowRotDeg}
                lidarCm={telemetry.lidarCm}
                velNorth={telemetry.velNorth}
                velEast={telemetry.velEast}
                isArmed={telemetry.isArmed}
              />
              <LiveTelemetryChart telemetry={telemetry} />
            </div>
            <FlowDerotationChart isArmed={telemetry.isArmed} />
          </div>
        )}

        {activeTab === 'params' && (
          <ParameterTuner isArmed={telemetry.isArmed} />
        )}

        {activeTab === 'terminal' && (
          <ConsoleTerminal />
        )}
      </main>

      {/* Footer Status Bar */}
      <footer className="bg-[#0a0a0a] border-t border-neutral-800 px-6 py-2.5 text-[11px] font-mono text-neutral-400 flex items-center justify-between">
        <div className="flex items-center gap-4">
          <span>Target: Oqab Flight Controller</span>
          <span>Baud: 115200 8N1</span>
          <span>Core 0: Comms & Parsing (100 Hz)</span>
          <span>Core 1: Control Loop (250 Hz)</span>
        </div>
        <div>
          <span>Mateksys 3901-L0X ToF LiDAR</span>
        </div>
      </footer>
    </div>
  );
};
