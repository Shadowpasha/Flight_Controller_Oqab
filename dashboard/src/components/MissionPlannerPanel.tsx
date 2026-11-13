import React, { useState } from 'react';
import {
  Play,
  Square,
  Upload,
  RotateCcw,
  Plus,
  Trash2,
  ArrowDown,
  PlaneTakeoff,
  PlaneLanding,
  Navigation,
  Clock,
  ChevronUp,
  ChevronDown,
  Sliders,
  CheckCircle2,
  ListFilter,
} from 'lucide-react';
import { MissionBlock, MissionBlockType } from '../types';
import { serialService } from '../services/serialService';

const DEFAULT_BLOCKS: MissionBlock[] = [
  {
    id: 'b-takeoff',
    type: 'TAKEOFF',
    title: 'Takeoff & Climb',
    altCm: 100,
    climbRateCmps: 18,
  },
  {
    id: 'b-wp-origin',
    type: 'WAYPOINT',
    title: 'Nav to Origin (0, 0)',
    northM: 0,
    eastM: 0,
    altCm: 100,
    speedMps: 0.5,
  },
  {
    id: 'b-delay',
    type: 'DELAY',
    title: 'Stationary Hover',
    durationS: 6.0,
  },
  {
    id: 'b-land',
    type: 'LAND',
    title: 'Controlled Landing',
    descRateCmps: 12,
  },
];

export const MissionPlannerPanel: React.FC = () => {
  const [blocks, setBlocks] = useState<MissionBlock[]>(DEFAULT_BLOCKS);
  const [selectedBlockId, setSelectedBlockId] = useState<string | null>(null);
  const [draggedIndex, setDraggedIndex] = useState<number | null>(null);
  const [uploadStatus, setUploadStatus] = useState<string>('');

  const addBlock = (type: MissionBlockType) => {
    const newId = `b-${Date.now()}-${Math.floor(Math.random() * 1000)}`;
    let newBlock: MissionBlock;
    if (type === 'TAKEOFF') {
      newBlock = {
        id: newId,
        type: 'TAKEOFF',
        title: 'Takeoff Block',
        altCm: 100,
        climbRateCmps: 18,
      };
    } else if (type === 'WAYPOINT') {
      newBlock = {
        id: newId,
        type: 'WAYPOINT',
        title: 'Waypoint Target',
        northM: 0,
        eastM: 0,
        altCm: 100,
        speedMps: 0.5,
      };
    } else if (type === 'DELAY') {
      newBlock = {
        id: newId,
        type: 'DELAY',
        title: 'Hover Delay',
        durationS: 5.0,
      };
    } else {
      newBlock = {
        id: newId,
        type: 'LAND',
        title: 'Landing Block',
        descRateCmps: 12,
      };
    }
    setBlocks((prev) => [...prev, newBlock]);
    setSelectedBlockId(newId);
  };

  const removeBlock = (id: string) => {
    setBlocks((prev) => prev.filter((b) => b.id !== id));
    if (selectedBlockId === id) setSelectedBlockId(null);
  };

  const moveBlock = (index: number, direction: 'up' | 'down') => {
    const newIndex = direction === 'up' ? index - 1 : index + 1;
    if (newIndex < 0 || newIndex >= blocks.length) return;
    setBlocks((prev) => {
      const copy = [...prev];
      const [removed] = copy.splice(index, 1);
      copy.splice(newIndex, 0, removed);
      return copy;
    });
  };

  // Drag and drop handlers
  const handleDragStart = (index: number) => {
    setDraggedIndex(index);
  };

  const handleDragOver = (e: React.DragEvent, index: number) => {
    e.preventDefault();
    if (draggedIndex === null || draggedIndex === index) return;
    setBlocks((prev) => {
      const copy = [...prev];
      const [dragged] = copy.splice(draggedIndex, 1);
      copy.splice(index, 0, dragged);
      return copy;
    });
    setDraggedIndex(index);
  };

  const handleDragEnd = () => {
    setDraggedIndex(null);
  };

  const updateSelectedParam = (key: keyof MissionBlock, val: number | string) => {
    if (!selectedBlockId) return;
    setBlocks((prev) =>
      prev.map((b) => (b.id === selectedBlockId ? { ...b, [key]: val } : b))
    );
  };

  const handleResetToDefault = () => {
    setBlocks(DEFAULT_BLOCKS);
    setSelectedBlockId(null);
    setUploadStatus('Reset to default mission plan');
    setTimeout(() => setUploadStatus(''), 3000);
  };

  const handleUploadToDrone = async () => {
    try {
      setUploadStatus('Uploading mission...');
      // 1. Clear existing mission
      serialService.sendCommand('mission clear');
      await new Promise((r) => setTimeout(r, 60));

      // 2. Add each block sequentially
      for (const b of blocks) {
        if (b.type === 'TAKEOFF') {
          const alt = b.altCm ?? 100;
          const rate = b.climbRateCmps ?? 18;
          serialService.sendCommand(`mission add TAKEOFF ${alt} ${rate}`);
        } else if (b.type === 'WAYPOINT') {
          const n = b.northM ?? 0;
          const e = b.eastM ?? 0;
          const alt = b.altCm ?? 100;
          const spd = b.speedMps ?? 0.5;
          serialService.sendCommand(`mission add WAYPOINT ${n} ${e} ${alt} ${spd}`);
        } else if (b.type === 'DELAY') {
          const dur = b.durationS ?? 5.0;
          serialService.sendCommand(`mission add DELAY ${dur}`);
        } else if (b.type === 'LAND') {
          const rate = b.descRateCmps ?? 12;
          serialService.sendCommand(`mission add LAND ${rate}`);
        }
        await new Promise((r) => setTimeout(r, 60));
      }

      serialService.sendCommand('mission show');
      setUploadStatus(`Uploaded ${blocks.length} blocks to flight controller!`);
      setTimeout(() => setUploadStatus(''), 4000);
    } catch (err) {
      setUploadStatus('Error uploading mission');
    }
  };

  const handleStartMission = () => {
    serialService.sendCommand('mission start');
  };

  const handleStopMission = () => {
    serialService.sendCommand('mission stop');
  };

  const selectedBlock = blocks.find((b) => b.id === selectedBlockId);

  const getBlockIcon = (type: MissionBlockType) => {
    switch (type) {
      case 'TAKEOFF':
        return <PlaneTakeoff className="w-4 h-4 text-emerald-400" />;
      case 'WAYPOINT':
        return <Navigation className="w-4 h-4 text-cyan-400" />;
      case 'DELAY':
        return <Clock className="w-4 h-4 text-amber-400" />;
      case 'LAND':
        return <PlaneLanding className="w-4 h-4 text-red-400" />;
    }
  };

  const getBlockColor = (type: MissionBlockType) => {
    switch (type) {
      case 'TAKEOFF':
        return 'border-emerald-500/50 bg-emerald-950/20 hover:border-emerald-500';
      case 'WAYPOINT':
        return 'border-cyan-500/50 bg-cyan-950/20 hover:border-cyan-500';
      case 'DELAY':
        return 'border-amber-500/50 bg-amber-950/20 hover:border-amber-500';
      case 'LAND':
        return 'border-red-500/50 bg-red-950/20 hover:border-red-500';
    }
  };

  return (
    <div className="flex flex-col gap-4 font-sans text-white">
      {/* Top Action Bar */}
      <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-4 flex flex-wrap items-center justify-between gap-3">
        <div className="flex items-center gap-3">
          <div className="p-2 rounded bg-neutral-900 border border-neutral-800 text-red-500">
            <ListFilter className="w-5 h-5" />
          </div>
          <div>
            <h2 className="text-sm font-bold font-mono tracking-wide uppercase">
              Mission Flowchart Architecture
            </h2>
            <p className="text-xs text-neutral-400 font-mono">
              Drag, order, and tune sequence blocks. Synchronizes directly with ESP32 mission engine.
            </p>
          </div>
        </div>

        <div className="flex items-center gap-2">
          {uploadStatus && (
            <span className="text-xs font-mono text-emerald-400 flex items-center gap-1.5 mr-2">
              <CheckCircle2 className="w-3.5 h-3.5" />
              {uploadStatus}
            </span>
          )}

          <button
            onClick={handleResetToDefault}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded bg-neutral-900 hover:bg-neutral-800 border border-neutral-700 text-xs font-mono font-medium transition"
          >
            <RotateCcw className="w-3.5 h-3.5 text-neutral-400" />
            Default Mission
          </button>

          <button
            onClick={handleUploadToDrone}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded bg-blue-600 hover:bg-blue-500 text-white text-xs font-mono font-semibold transition shadow-[0_0_10px_rgba(37,99,235,0.3)]"
          >
            <Upload className="w-3.5 h-3.5" />
            Sync Mission to Drone
          </button>

          <button
            onClick={handleStartMission}
            className="flex items-center gap-1.5 px-3.5 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white text-xs font-mono font-bold transition shadow-[0_0_10px_rgba(16,185,129,0.3)]"
          >
            <Play className="w-3.5 h-3.5 fill-current" />
            Start Mission
          </button>

          <button
            onClick={handleStopMission}
            className="flex items-center gap-1.5 px-3.5 py-1.5 rounded bg-red-600 hover:bg-red-500 text-white text-xs font-mono font-bold transition shadow-[0_0_10px_rgba(220,38,38,0.3)]"
          >
            <Square className="w-3.5 h-3.5 fill-current" />
            Abort
          </button>
        </div>
      </div>

      {/* Main Grid: Flow Chart Visualizer (Left) & Parameters / Block Inspector (Right) */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
        {/* Left 2 Cols: Flowchart Diagram & Sequence Canvas */}
        <div className="lg:col-span-2 bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col justify-between">
          <div>
            <div className="flex items-center justify-between mb-4 border-b border-neutral-800/80 pb-3">
              <span className="text-xs font-mono font-semibold uppercase text-neutral-300">
                Mission Execution Sequence ({blocks.length} Blocks)
              </span>
              <div className="flex items-center gap-1.5">
                <span className="text-[11px] font-mono text-neutral-500 mr-1">Add Step:</span>
                <button
                  onClick={() => addBlock('TAKEOFF')}
                  className="px-2 py-1 text-[11px] font-mono rounded bg-emerald-950/40 text-emerald-400 border border-emerald-800 hover:bg-emerald-900/40"
                >
                  + Takeoff
                </button>
                <button
                  onClick={() => addBlock('WAYPOINT')}
                  className="px-2 py-1 text-[11px] font-mono rounded bg-cyan-950/40 text-cyan-400 border border-cyan-800 hover:bg-cyan-900/40"
                >
                  + Waypoint
                </button>
                <button
                  onClick={() => addBlock('DELAY')}
                  className="px-2 py-1 text-[11px] font-mono rounded bg-amber-950/40 text-amber-400 border border-amber-800 hover:bg-amber-900/40"
                >
                  + Delay
                </button>
                <button
                  onClick={() => addBlock('LAND')}
                  className="px-2 py-1 text-[11px] font-mono rounded bg-red-950/40 text-red-400 border border-red-800 hover:bg-red-900/40"
                >
                  + Land
                </button>
              </div>
            </div>

            {/* Sequence Flow */}
            <div className="flex flex-col items-center gap-2 py-2">
              {blocks.map((block, idx) => {
                const isSelected = selectedBlockId === block.id;
                return (
                  <React.Fragment key={block.id}>
                    <div
                      draggable
                      onDragStart={() => handleDragStart(idx)}
                      onDragOver={(e) => handleDragOver(e, idx)}
                      onDragEnd={handleDragEnd}
                      onClick={() => setSelectedBlockId(block.id)}
                      className={`w-full max-w-xl p-3.5 rounded-lg border cursor-pointer transition flex items-center justify-between shadow-sm select-none ${
                        isSelected
                          ? 'border-white ring-2 ring-blue-500/50 bg-neutral-900'
                          : getBlockColor(block.type)
                      }`}
                    >
                      <div className="flex items-center gap-3">
                        <span className="w-6 h-6 rounded-full bg-neutral-900 border border-neutral-700 flex items-center justify-center font-mono text-xs font-bold text-neutral-300">
                          {idx + 1}
                        </span>
                        <div className="p-1.5 rounded bg-black/60 border border-neutral-800">
                          {getBlockIcon(block.type)}
                        </div>
                        <div>
                          <div className="font-mono text-xs font-bold text-neutral-100 flex items-center gap-2">
                            <span>{block.title}</span>
                            <span className="text-[10px] uppercase font-mono px-1.5 py-0.2 rounded bg-black/70 text-neutral-400 border border-neutral-800">
                              {block.type}
                            </span>
                          </div>
                          <div className="text-[11px] font-mono text-neutral-400 mt-0.5">
                            {block.type === 'TAKEOFF' && (
                              <span>
                                Target Alt: <strong className="text-white">{block.altCm}cm</strong> | Climb: <strong className="text-white">{block.climbRateCmps}cm/s</strong>
                              </span>
                            )}
                            {block.type === 'WAYPOINT' && (
                              <span>
                                Target: North=<strong className="text-white">{block.northM}m</strong>, East=<strong className="text-white">{block.eastM}m</strong>, Alt=<strong className="text-white">{block.altCm}cm</strong>
                              </span>
                            )}
                            {block.type === 'DELAY' && (
                              <span>
                                Duration: <strong className="text-white">{block.durationS} seconds</strong>
                              </span>
                            )}
                            {block.type === 'LAND' && (
                              <span>
                                Descent Rate: <strong className="text-white">{block.descRateCmps} cm/s</strong>
                              </span>
                            )}
                          </div>
                        </div>
                      </div>

                      {/* Controls: Up, Down, Delete */}
                      <div className="flex items-center gap-1" onClick={(e) => e.stopPropagation()}>
                        <button
                          onClick={() => moveBlock(idx, 'up')}
                          disabled={idx === 0}
                          className="p-1 rounded hover:bg-neutral-800 text-neutral-400 disabled:opacity-20"
                          title="Move up"
                        >
                          <ChevronUp className="w-4 h-4" />
                        </button>
                        <button
                          onClick={() => moveBlock(idx, 'down')}
                          disabled={idx === blocks.length - 1}
                          className="p-1 rounded hover:bg-neutral-800 text-neutral-400 disabled:opacity-20"
                          title="Move down"
                        >
                          <ChevronDown className="w-4 h-4" />
                        </button>
                        <button
                          onClick={() => removeBlock(block.id)}
                          className="p-1 rounded hover:bg-red-950 text-neutral-400 hover:text-red-400 ml-1"
                          title="Delete block"
                        >
                          <Trash2 className="w-4 h-4" />
                        </button>
                      </div>
                    </div>

                    {/* Flow arrow connecting next block */}
                    {idx < blocks.length - 1 && (
                      <div className="flex flex-col items-center my-0.5">
                        <div className="w-0.5 h-3 bg-neutral-700" />
                        <ArrowDown className="w-3.5 h-3.5 text-neutral-500 -mt-1" />
                      </div>
                    )}
                  </React.Fragment>
                );
              })}
            </div>
          </div>
        </div>

        {/* Right Col: Block Inspector & Parameters */}
        <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col justify-between">
          <div>
            <div className="flex items-center gap-2 border-b border-neutral-800 pb-3 mb-4">
              <Sliders className="w-4 h-4 text-blue-400" />
              <h3 className="text-xs font-mono font-bold uppercase tracking-wider text-neutral-200">
                Block Parameter Inspector
              </h3>
            </div>

            {selectedBlock ? (
              <div className="space-y-4">
                <div>
                  <label className="block text-[11px] font-mono text-neutral-400 mb-1">Block Title</label>
                  <input
                    type="text"
                    value={selectedBlock.title}
                    onChange={(e) => updateSelectedParam('title', e.target.value)}
                    className="w-full bg-neutral-900 border border-neutral-700 rounded px-3 py-1.5 text-xs font-mono text-white focus:outline-none focus:border-blue-500"
                  />
                </div>

                <div className="text-[11px] font-mono px-2.5 py-1 rounded bg-neutral-950 border border-neutral-800 text-neutral-400">
                  Type: <strong className="text-white">{selectedBlock.type}</strong>
                </div>

                {/* Specific parameters depending on block type */}
                {selectedBlock.type === 'TAKEOFF' && (
                  <>
                    <div>
                      <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                        <span>Target Altitude (cm):</span>
                        <span className="font-bold text-white">{selectedBlock.altCm} cm</span>
                      </div>
                      <input
                        type="range"
                        min="20"
                        max="180"
                        step="5"
                        value={selectedBlock.altCm ?? 100}
                        onChange={(e) => updateSelectedParam('altCm', parseFloat(e.target.value))}
                        className="w-full accent-emerald-500"
                      />
                    </div>
                    <div>
                      <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                        <span>Climb Rate (cm/s):</span>
                        <span className="font-bold text-white">{selectedBlock.climbRateCmps} cm/s</span>
                      </div>
                      <input
                        type="range"
                        min="5"
                        max="50"
                        step="1"
                        value={selectedBlock.climbRateCmps ?? 18}
                        onChange={(e) => updateSelectedParam('climbRateCmps', parseFloat(e.target.value))}
                        className="w-full accent-emerald-500"
                      />
                    </div>
                  </>
                )}

                {selectedBlock.type === 'WAYPOINT' && (
                  <>
                    <div>
                      <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                        <span>Target North Displacement (m):</span>
                        <span className="font-bold text-white">{(selectedBlock.northM ?? 0).toFixed(2)} m</span>
                      </div>
                      <input
                        type="range"
                        min="-5.0"
                        max="5.0"
                        step="0.1"
                        value={selectedBlock.northM ?? 0}
                        onChange={(e) => updateSelectedParam('northM', parseFloat(e.target.value))}
                        className="w-full accent-cyan-500"
                      />
                    </div>
                    <div>
                      <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                        <span>Target East Displacement (m):</span>
                        <span className="font-bold text-white">{(selectedBlock.eastM ?? 0).toFixed(2)} m</span>
                      </div>
                      <input
                        type="range"
                        min="-5.0"
                        max="5.0"
                        step="0.1"
                        value={selectedBlock.eastM ?? 0}
                        onChange={(e) => updateSelectedParam('eastM', parseFloat(e.target.value))}
                        className="w-full accent-cyan-500"
                      />
                    </div>
                    <div>
                      <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                        <span>Waypoint Altitude (cm):</span>
                        <span className="font-bold text-white">{selectedBlock.altCm} cm</span>
                      </div>
                      <input
                        type="range"
                        min="20"
                        max="180"
                        step="5"
                        value={selectedBlock.altCm ?? 100}
                        onChange={(e) => updateSelectedParam('altCm', parseFloat(e.target.value))}
                        className="w-full accent-cyan-500"
                      />
                    </div>
                    <div>
                      <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                        <span>Transit Speed (m/s):</span>
                        <span className="font-bold text-white">{selectedBlock.speedMps} m/s</span>
                      </div>
                      <input
                        type="range"
                        min="0.1"
                        max="2.0"
                        step="0.1"
                        value={selectedBlock.speedMps ?? 0.5}
                        onChange={(e) => updateSelectedParam('speedMps', parseFloat(e.target.value))}
                        className="w-full accent-cyan-500"
                      />
                    </div>
                  </>
                )}

                {selectedBlock.type === 'DELAY' && (
                  <div>
                    <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                      <span>Hover Duration (seconds):</span>
                      <span className="font-bold text-white">{(selectedBlock.durationS ?? 5.0).toFixed(1)} s</span>
                    </div>
                    <input
                      type="range"
                      min="1.0"
                      max="60.0"
                      step="0.5"
                      value={selectedBlock.durationS ?? 5.0}
                      onChange={(e) => updateSelectedParam('durationS', parseFloat(e.target.value))}
                      className="w-full accent-amber-500"
                    />
                  </div>
                )}

                {selectedBlock.type === 'LAND' && (
                  <div>
                    <div className="flex justify-between text-xs font-mono text-neutral-300 mb-1">
                      <span>Descent Rate (cm/s):</span>
                      <span className="font-bold text-white">{selectedBlock.descRateCmps} cm/s</span>
                    </div>
                    <input
                      type="range"
                      min="5"
                      max="35"
                      step="1"
                      value={selectedBlock.descRateCmps ?? 12}
                      onChange={(e) => updateSelectedParam('descRateCmps', parseFloat(e.target.value))}
                      className="w-full accent-red-500"
                    />
                  </div>
                )}
              </div>
            ) : (
              <div className="p-8 text-center text-xs font-mono text-neutral-500 border border-dashed border-neutral-800 rounded">
                Click any sequence box on the left to inspect and configure its parameters.
              </div>
            )}
          </div>

          <div className="mt-6 pt-4 border-t border-neutral-800 text-[11px] font-mono text-neutral-400">
            <p className="mb-2"><strong>Tip:</strong> The default sequence is calibrated for indoor flight testing using Mateksys optical flow and ToF LiDAR.</p>
          </div>
        </div>
      </div>
    </div>
  );
};

