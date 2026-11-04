import React, { useState, useEffect } from 'react';
import {
  Plane,
  Cable,
  Flame,
  Sliders,
  Terminal as TerminalIcon,
  Eye,
  Activity,
  Sparkles,
  Layers,
  Cpu,
} from 'lucide-react';
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

export const App: React.FC = () => {
  const [status, setStatus] = useState<ConnectionState>('disconnected');
  const [telemetry, setTelemetry] = useState<DroneTelemetry>(serialService.getLatestTelemetry());
  const [activeTab, setActiveTab] = useState<'overview' | 'state_est' | 'ctrl_gains' | 'flow' | 'params' | 'terminal'>('overview');

  useEffect(() => {
    const unsubStatus = serialService.onStatus(setStatus);
    const unsubTelem = serialService.onTelemetry(setTelemetry);
    return () => {
      unsubStatus();
      unsubTelem();
    };
  }, []);

  const handleConnectToggle = () => {
    if (status === 'connected' || status === 'simulated') {
      serialService.disconnect();
    } else {
      serialService.connect();
    }
  };

  const handleDemoToggle = () => {
    if (status === 'simulated') {
      serialService.stopSimulation();
    } else {
      serialService.startSimulation();
    }
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
          <div className="w-8 h-8 rounded bg-black border border-red-600 flex items-center justify-center">
            <Plane className="w-4 h-4 text-red-600 transform -rotate-45" />
          </div>
          <div>
            <div className="flex items-center gap-2">
              <h1 className="text-xs font-mono font-bold tracking-widest uppercase text-white">
                OQAB GROUND CONTROL
              </h1>
              <span className="text-[10px] font-mono px-1.5 py-0.2 rounded bg-neutral-900 text-red-500 border border-neutral-800">
                F330
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
            onClick={handleDemoToggle}
            className={`px-3 py-1.5 rounded font-semibold flex items-center gap-1.5 transition border cursor-pointer ${
              status === 'simulated'
                ? 'bg-red-600/20 text-red-400 border-red-600'
                : 'bg-neutral-900 text-neutral-400 hover:text-white border-neutral-800'
            }`}
          >
            <Sparkles className="w-3.5 h-3.5" />
            <span>{status === 'simulated' ? 'SIM RUNNING' : 'DEMO MODE'}</span>
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
                : status === 'simulated'
                ? 'STOP DEMO'
                : 'CONNECT LINK'}
            </span>
          </button>

          <button
            onClick={handleArmToggle}
            className={`px-4 py-1.5 rounded font-bold uppercase tracking-wider flex items-center gap-1.5 transition border cursor-pointer ${
              telemetry.isArmed
                ? 'bg-red-600 hover:bg-red-700 text-white border-red-600 animate-pulse'
                : 'bg-neutral-900 hover:bg-neutral-800 text-neutral-300 border-neutral-800'
            }`}
          >
            <Flame className="w-3.5 h-3.5 text-red-500" />
            <span>{telemetry.isArmed ? 'DISARM' : 'ARM'}</span>
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
                  ? 'bg-red-600 animate-pulse'
                  : status === 'simulated'
                  ? 'bg-white'
                  : 'bg-neutral-600'
              }`}
            />
            <span>
              {status === 'connected'
                ? 'LINK ACTIVE'
                : status === 'simulated'
                ? 'EMULATION'
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
          </div>
        )}

        {activeTab === 'state_est' && (
          <StateEstimationPanel telemetry={telemetry} isArmed={telemetry.isArmed} />
        )}

        {activeTab === 'ctrl_gains' && (
          <ControllerGainsPanel activeController={telemetry.controller} isArmed={telemetry.isArmed} />
        )}

        {activeTab === 'flow' && (
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
          <span>Target: ESP32 F330</span>
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
