import React, { useEffect, useRef } from 'react';
import { LineChart } from 'lucide-react';
import { DroneTelemetry } from '../types';

interface Props {
  telemetry: DroneTelemetry;
}

interface DataPoint {
  time: number;
  height: number;
  vz: number;
  speed: number;
  quality: number;
}

export const LiveTelemetryChart: React.FC<Props> = ({ telemetry }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const dataRef = useRef<DataPoint[]>([]);

  useEffect(() => {
    const pts = dataRef.current;
    pts.push({
      time: telemetry.timestamp,
      height: telemetry.heightCm,
      vz: telemetry.velDown,
      speed: telemetry.speed3d,
      quality: telemetry.flowQuality,
    });
    if (pts.length > 120) pts.shift();
  }, [telemetry]);

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
      const pts = dataRef.current;

      // Pure black background
      ctx.fillStyle = '#000000';
      ctx.fillRect(0, 0, w, h);

      // Clean neutral grid lines
      ctx.strokeStyle = '#171717';
      ctx.lineWidth = 1;
      for (let y = 30; y < h; y += 40) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }

      if (pts.length < 2) return;

      const step = w / 120;
      const startX = w - pts.length * step;

      // 1. Altitude Curve in iconic Red
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 2;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const x = startX + i * step;
        const normH = Math.min(1.0, Math.max(0, p.height / 100.0));
        const y = h - 20 - normH * (h - 40);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      // 2. 3D Speed Curve in White
      ctx.strokeStyle = '#e5e5e5';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const x = startX + i * step;
        const normSpd = Math.min(1.0, Math.max(0, p.speed / 1.5));
        const y = h - 20 - normSpd * (h - 40);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      // 3. Vertical Velocity Vz in Mid-Gray
      const midY = h / 2;
      ctx.strokeStyle = '#737373';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const x = startX + i * step;
        const y = midY - p.vz * 80;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    };

    render();

    return () => cancelAnimationFrame(animId);
  }, []);

  return (
    <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-4 flex flex-col">
      <div className="flex items-center justify-between border-b border-neutral-800/80 pb-2 mb-3 font-mono">
        <div className="flex items-center gap-2">
          <LineChart className="w-3.5 h-3.5 text-red-600" />
          <span className="text-xs uppercase tracking-wider text-neutral-300 font-semibold">
            Telemetry Scope
          </span>
        </div>
        <div className="flex items-center gap-4 text-[11px]">
          <div className="flex items-center gap-1.5">
            <div className="w-2.5 h-1 rounded bg-red-600" />
            <span className="text-neutral-300">Altitude</span>
          </div>
          <div className="flex items-center gap-1.5">
            <div className="w-2.5 h-1 rounded bg-neutral-200" />
            <span className="text-neutral-300">Speed 3D</span>
          </div>
          <div className="flex items-center gap-1.5">
            <div className="w-2.5 h-1 rounded bg-neutral-500" />
            <span className="text-neutral-400">Vz Rate</span>
          </div>
        </div>
      </div>

      <div className="relative w-full h-[180px] rounded overflow-hidden border border-neutral-900 bg-black">
        <canvas ref={canvasRef} width={800} height={180} className="w-full h-full block" />
      </div>
    </div>
  );
};
