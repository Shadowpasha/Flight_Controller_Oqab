import React, { useState, useEffect, useRef } from 'react';
import { Eye, Play, Square, RefreshCw, CheckCircle2, AlertTriangle, Info } from 'lucide-react';
import { serialService } from '../services/serialService';
import { FlowLivePoint } from '../types';

interface FlowDerotationChartProps {
  isArmed?: boolean;
}

export const FlowDerotationChart: React.FC<FlowDerotationChartProps> = ({ isArmed = false }) => {
  const [activeAxis, setActiveAxis] = useState<'X' | 'Y'>('X');
  const [isStreaming, setIsStreaming] = useState<boolean>(false);
  const [latestPoint, setLatestPoint] = useState<FlowLivePoint | null>(null);
  const [latencyMs, setLatencyMs] = useState<number>(28);

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const bufferRef = useRef<FlowLivePoint[]>([]);

  useEffect(() => {
    const unsub = serialService.onFlowLive((point) => {
      setLatestPoint(point);
      setIsStreaming(true);

      const buf = bufferRef.current;
      buf.push(point);
      if (buf.length > 150) buf.shift();
    });

    const unsubLine = serialService.onLine((line) => {
      if (line.includes('Flow Live Stream: STARTED')) {
        setIsStreaming(true);
      } else if (line.includes('Flow Live Stream: STOPPED')) {
        setIsStreaming(false);
      }
    });

    return () => {
      unsub();
      unsubLine();
    };
  }, []);

  const toggleStreaming = () => {
    serialService.sendCommand('flow stream');
    setIsStreaming(!isStreaming);
  };

  const handleSetRotation = (deg: number) => {
    serialService.sendCommand(`flow rot ${deg}`);
  };

  const handleRunCalFlow = () => {
    serialService.sendCommand('cal flow');
  };

  const handleSetLatency = (val: number) => {
    setLatencyMs(val);
    serialService.sendCommand(`param set FLOW_DELAY_MS ${val}`);
  };

  // Canvas drawing loop
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let animId: number;
    const render = () => {
      animId = requestAnimationFrame(render);
      const w = canvas.width;
      const h = canvas.height;
      const pts = bufferRef.current;

      // Pure dark background
      ctx.fillStyle = '#050505';
      ctx.fillRect(0, 0, w, h);

      // Horizontal grid lines
      ctx.strokeStyle = '#1a1a1a';
      ctx.lineWidth = 1;
      const midY = h / 2;

      // Grid intervals (-60, -30, 0, +30, +60 dps)
      const maxDps = 70.0;
      [-60, -40, -20, 20, 40, 60].forEach((val) => {
        const y = midY - (val / maxDps) * (midY - 20);
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();

        ctx.fillStyle = '#525252';
        ctx.font = '9px monospace';
        ctx.fillText(`${val > 0 ? '+' : ''}${val}°/s`, 8, y - 3);
      });

      // Zero reference line
      ctx.strokeStyle = '#333333';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, midY);
      ctx.lineTo(w, midY);
      ctx.stroke();

      ctx.fillStyle = '#737373';
      ctx.font = '10px monospace';
      ctx.fillText('0°/s', 8, midY - 4);

      if (pts.length < 2) {
        ctx.fillStyle = '#525252';
        ctx.font = '12px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(
          isStreaming ? 'Waiting for live flow stream data...' : "Click 'Start Live Stream' to begin live derotation graph",
          w / 2,
          midY
        );
        ctx.textAlign = 'left';
        return;
      }

      const step = w / 150;
      const startX = w - pts.length * step;

      // 1. Draw Optical Flow Angular Rate (Cyan #06b6d4)
      ctx.strokeStyle = '#06b6d4';
      ctx.lineWidth = 2;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const x = startX + i * step;
        const rate = activeAxis === 'X' ? p.ofX : p.ofY;
        const clampedRate = Math.max(-maxDps, Math.min(maxDps, rate));
        const y = midY - (clampedRate / maxDps) * (midY - 20);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      // 2. Draw Gyro Rate (Amber #f59e0b)
      // Note: X-axis (Forward flow) pairs with Gyro Y (Pitch). Y-axis (Right flow) pairs with Gyro X (Roll).
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const x = startX + i * step;
        const rate = activeAxis === 'X' ? p.gyroY : p.gyroX;
        const clampedRate = Math.max(-maxDps, Math.min(maxDps, rate));
        const y = midY - (clampedRate / maxDps) * (midY - 20);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      // 3. Draw Residual Derotated Velocity (Red #ef4444) scaled to fit visually (1 m/s = 30 dps visual)
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const x = startX + i * step;
        const vel = activeAxis === 'X' ? p.vFwd : p.vRight;
        const equivDps = vel * 30.0;
        const clampedRate = Math.max(-maxDps, Math.min(maxDps, equivDps));
        const y = midY - (clampedRate / maxDps) * (midY - 20);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    };

    render();
    return () => cancelAnimationFrame(animId);
  }, [activeAxis, isStreaming]);

  // Real-time analysis of the current axis
  const ofRate = latestPoint ? (activeAxis === 'X' ? latestPoint.ofX : latestPoint.ofY) : 0;
  const gyroRate = latestPoint ? (activeAxis === 'X' ? latestPoint.gyroY : latestPoint.gyroX) : 0;
  const residualVel = latestPoint ? (activeAxis === 'X' ? latestPoint.vFwd : latestPoint.vRight) : 0;

  const motionMagnitude = Math.abs(gyroRate);
  const isMoving = motionMagnitude >= 10.0;
  const sameSign = (ofRate > 0 && gyroRate > 0) || (ofRate < 0 && gyroRate < 0);
  const ratio = gyroRate !== 0 ? Math.abs(ofRate / gyroRate) : 1.0;

  let diagnosticState: 'idle' | 'canceling' | 'inverted' | 'scale_mismatch' = 'idle';
  if (isMoving) {
    if (!sameSign) {
      diagnosticState = 'inverted';
    } else if (ratio < 0.65 || ratio > 1.45) {
      diagnosticState = 'scale_mismatch';
    } else {
      diagnosticState = 'canceling';
    }
  }

  return (
    <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col gap-4">
      {/* Header & Controls */}
      <div className="flex flex-wrap items-center justify-between border-b border-neutral-800/80 pb-3 gap-3">
        <div className="flex items-center gap-2">
          <Eye className="w-4 h-4 text-red-600" />
          <h2 className="text-xs font-mono font-bold uppercase tracking-wider text-white">
            Optical Flow Derotation Live Inspection
          </h2>
        </div>

        <div className="flex items-center gap-2">
          <button
            onClick={toggleStreaming}
            className={`px-3 py-1.5 rounded text-xs font-mono font-semibold flex items-center gap-1.5 transition cursor-pointer border ${
              isStreaming
                ? 'bg-neutral-900 border-red-700 text-red-400 hover:bg-neutral-800'
                : 'bg-red-600 border-red-600 text-white hover:bg-red-700'
            }`}
          >
            {isStreaming ? (
              <>
                <Square className="w-3.5 h-3.5 fill-current" />
                <span>Stop Stream</span>
              </>
            ) : (
              <>
                <Play className="w-3.5 h-3.5 fill-current" />
                <span>Start Live Stream</span>
              </>
            )}
          </button>
        </div>
      </div>

      {/* Axis Selection Tabs */}
      <div className="flex items-center justify-between gap-3 border-b border-neutral-800/60 pb-2">
        <div className="flex gap-2">
          <button
            onClick={() => setActiveAxis('X')}
            className={`px-4 py-1.5 rounded font-mono text-xs transition cursor-pointer border ${
              activeAxis === 'X'
                ? 'bg-red-600 text-white border-red-600 font-bold shadow-[0_0_10px_rgba(220,38,38,0.3)]'
                : 'bg-neutral-900 text-neutral-400 border-neutral-800 hover:text-white'
            }`}
          >
            Tab 1: X-Axis (Forward Flow & Pitch Gyro)
          </button>

          <button
            onClick={() => setActiveAxis('Y')}
            className={`px-4 py-1.5 rounded font-mono text-xs transition cursor-pointer border ${
              activeAxis === 'Y'
                ? 'bg-red-600 text-white border-red-600 font-bold shadow-[0_0_10px_rgba(220,38,38,0.3)]'
                : 'bg-neutral-900 text-neutral-400 border-neutral-800 hover:text-white'
            }`}
          >
            Tab 2: Y-Axis (Rightward Flow & Roll Gyro)
          </button>
        </div>

        <span className="text-[11px] font-mono text-neutral-500 hidden sm:inline">
          {activeAxis === 'X' ? 'Forward Flow vs Gyro Y (Pitch)' : 'Right Flow vs Gyro X (Roll)'}
        </span>
      </div>

      {/* Chart Canvas */}
      <div className="relative w-full h-56 bg-black rounded border border-neutral-800 overflow-hidden">
        <canvas ref={canvasRef} width={640} height={224} className="w-full h-full block" />

        {/* Live Legend Floating Overlay */}
        <div className="absolute top-2 right-2 bg-black/85 backdrop-blur border border-neutral-800/80 rounded px-3 py-2 flex flex-col gap-1.5 text-[11px] font-mono shadow-lg">
          <div className="flex items-center justify-between gap-4">
            <span className="flex items-center gap-1.5 text-cyan-400 font-semibold">
              <span className="w-2.5 h-0.5 bg-cyan-400 rounded-full inline-block" />
              Optical Flow Rate
            </span>
            <span className="text-neutral-200">
              {ofRate > 0 ? '+' : ''}
              {ofRate.toFixed(1)}°/s
            </span>
          </div>

          <div className="flex items-center justify-between gap-4">
            <span className="flex items-center gap-1.5 text-amber-400 font-semibold">
              <span className="w-2.5 h-0.5 bg-amber-400 rounded-full inline-block" />
              {activeAxis === 'X' ? 'Pitch Rate (Gyro Y)' : 'Roll Rate (Gyro X)'}
            </span>
            <span className="text-neutral-200">
              {gyroRate > 0 ? '+' : ''}
              {gyroRate.toFixed(1)}°/s
            </span>
          </div>

          <div className="flex items-center justify-between gap-4">
            <span className="flex items-center gap-1.5 text-red-500 font-semibold">
              <span className="w-2.5 h-0.5 bg-red-500 rounded-full inline-block" />
              Residual Velocity
            </span>
            <span className="text-neutral-200">
              {residualVel > 0 ? '+' : ''}
              {residualVel.toFixed(2)} m/s
            </span>
          </div>
        </div>
      </div>

      {/* Real-Time Diagnostic Evaluation Box */}
      <div className="p-3.5 rounded border font-mono text-xs flex items-start gap-3 bg-neutral-950 border-neutral-800">
        {diagnosticState === 'canceling' && (
          <>
            <CheckCircle2 className="w-5 h-5 text-emerald-400 flex-shrink-0 mt-0.5" />
            <div className="flex flex-col gap-0.5">
              <span className="text-emerald-400 font-bold">EXCELLENT DEROTATION MATCH</span>
              <span className="text-neutral-400 text-[11px]">
                Optical flow rate and gyro angular rate track in the same direction and magnitude. Residual velocity is
                effectively canceled.
              </span>
            </div>
          </>
        )}

        {diagnosticState === 'inverted' && (
          <>
            <AlertTriangle className="w-5 h-5 text-red-500 flex-shrink-0 mt-0.5" />
            <div className="flex flex-col gap-0.5">
              <span className="text-red-400 font-bold">DIRECTION INVERTED (OPPOSITE SIGNS)</span>
              <span className="text-neutral-300 text-[11px]">
                Optical flow is moving opposite to the gyro rate. Derotation is currently DOUBLING the apparent velocity
                instead of subtracting it. Check your mounting rotation angle or invert this axis in Config.h.
              </span>
            </div>
          </>
        )}

        {diagnosticState === 'scale_mismatch' && (
          <>
            <AlertTriangle className="w-5 h-5 text-amber-400 flex-shrink-0 mt-0.5" />
            <div className="flex flex-col gap-0.5">
              <span className="text-amber-400 font-bold">SCALE FACTOR MISMATCH</span>
              <span className="text-neutral-300 text-[11px]">
                The directions match, but peak heights differ (Ratio: {ratio.toFixed(2)}x). Run 'Calibrate Flow' below to
                automatically tune rad/count.
              </span>
            </div>
          </>
        )}

        {diagnosticState === 'idle' && (
          <>
            <Info className="w-5 h-5 text-neutral-400 flex-shrink-0 mt-0.5" />
            <div className="flex flex-col gap-0.5">
              <span className="text-neutral-300 font-semibold">STATIONARY BENCH TEST</span>
              <span className="text-neutral-400 text-[11px]">
                Hold your drone at ~0.8m height over a textured surface and gently rock it in {activeAxis === 'X' ? 'PITCH (nose up & down)' : 'ROLL (left & right)'}. Watch the Cyan and Amber lines overlap.
              </span>
            </div>
          </>
        )}
      </div>

      {/* Quick Calibration, Latency & Rotation Action Bar */}
      <div className="flex flex-wrap items-center justify-between gap-3 pt-2 border-t border-neutral-800/80 text-xs font-mono">
        <div className="flex flex-wrap items-center gap-4">
          <div className="flex items-center gap-1.5">
            <span className="text-neutral-400">Delay:</span>
            {[0, 16, 24, 28, 32, 40].map((ms) => (
              <button
                key={ms}
                onClick={() => handleSetLatency(ms)}
                disabled={isArmed}
                className={`px-2 py-0.5 rounded text-[11px] border transition cursor-pointer ${
                  latencyMs === ms
                    ? 'bg-amber-500 text-black border-amber-400 font-bold'
                    : 'bg-neutral-900 border-neutral-800 text-neutral-300 hover:text-white'
                } disabled:opacity-40`}
              >
                {ms}ms
              </button>
            ))}
          </div>

          <div className="flex items-center gap-1.5">
            <span className="text-neutral-400">Rot:</span>
            {[0, 90, 180, 270].map((deg) => (
              <button
                key={deg}
                onClick={() => handleSetRotation(deg)}
                disabled={isArmed}
                className="px-2 py-0.5 rounded text-[11px] bg-neutral-900 border border-neutral-800 text-neutral-300 hover:text-white hover:border-red-600 transition cursor-pointer disabled:opacity-40"
              >
                {deg}°
              </button>
            ))}
          </div>
        </div>

        <button
          onClick={handleRunCalFlow}
          disabled={isArmed}
          className={`px-3 py-1.5 bg-red-600 hover:bg-red-700 text-white rounded font-bold transition flex items-center gap-1.5 ${
            isArmed ? 'opacity-40 cursor-not-allowed' : 'cursor-pointer'
          }`}
        >
          <RefreshCw className="w-3.5 h-3.5" />
          <span>Calibrate Flow Scale (10s)</span>
        </button>
      </div>
    </div>
  );
};

