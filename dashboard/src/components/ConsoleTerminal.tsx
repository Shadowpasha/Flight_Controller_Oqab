import React, { useState, useEffect, useRef } from 'react';
import { Terminal, Send, Trash2, ArrowDownCircle } from 'lucide-react';
import { serialService } from '../services/serialService';

export const ConsoleTerminal: React.FC = () => {
  const [logs, setLogs] = useState<string[]>([]);
  const [inputCmd, setInputCmd] = useState('');
  const [history, setHistory] = useState<string[]>([]);
  const [historyIdx, setHistoryIdx] = useState<number>(-1);
  const [autoScroll, setAutoScroll] = useState(true);
  const bottomRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const unsub = serialService.onLine((line) => {
      setLogs((prev) => {
        const next = [...prev, line];
        if (next.length > 500) next.shift();
        return next;
      });
    });
    return unsub;
  }, []);

  useEffect(() => {
    if (autoScroll) {
      bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs, autoScroll]);

  const handleSend = () => {
    const trimmed = inputCmd.trim();
    if (!trimmed) return;
    serialService.sendCommand(trimmed);
    setHistory((prev) => [...prev, trimmed]);
    setHistoryIdx(-1);
    setInputCmd('');
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleSend();
    } else if (e.key === 'ArrowUp') {
      if (history.length > 0) {
        const nextIdx = historyIdx === -1 ? history.length - 1 : Math.max(0, historyIdx - 1);
        setHistoryIdx(nextIdx);
        setInputCmd(history[nextIdx]);
      }
    } else if (e.key === 'ArrowDown') {
      if (history.length > 0 && historyIdx !== -1) {
        const nextIdx = historyIdx + 1;
        if (nextIdx >= history.length) {
          setHistoryIdx(-1);
          setInputCmd('');
        } else {
          setHistoryIdx(nextIdx);
          setInputCmd(history[nextIdx]);
        }
      }
    }
  };

  const getLineStyle = (line: string) => {
    if (line.startsWith('>')) return 'text-red-500 font-semibold';
    if (line.includes('!!!') || line.includes('FATAL') || line.includes('WARNING')) return 'text-red-400 font-bold';
    if (line.startsWith('>>>') || line.startsWith('-->')) return 'text-neutral-200 font-semibold';
    if (line.includes('[DEAD-RECKONING]')) return 'text-neutral-500 text-[11px]';
    return 'text-neutral-300';
  };

  const quickCommands = ['status', 'param show', 'param get SENS_FLOW_ROT', 'flow toggle', 'land'];

  return (
    <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-4 flex flex-col h-[520px]">
      <div className="flex items-center justify-between border-b border-neutral-800/80 pb-2.5 mb-2.5">
        <div className="flex items-center gap-2">
          <Terminal className="w-4 h-4 text-red-600" />
          <span className="text-xs font-mono font-semibold uppercase tracking-wider text-neutral-300">
            Serial Terminal
          </span>
          <span className="text-[10px] text-neutral-600 font-mono">({logs.length})</span>
        </div>

        <div className="flex items-center gap-1.5">
          <button
            onClick={() => setAutoScroll(!autoScroll)}
            className={`p-1 rounded text-xs transition cursor-pointer border ${
              autoScroll ? 'text-red-500 bg-neutral-900 border-neutral-800' : 'text-neutral-600 border-transparent hover:text-neutral-400'
            }`}
            title="Toggle Autoscroll"
          >
            <ArrowDownCircle className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={() => setLogs([])}
            className="p-1 rounded text-neutral-500 hover:text-red-500 hover:bg-neutral-900 transition cursor-pointer"
            title="Clear Console"
          >
            <Trash2 className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>

      {/* Terminal Log Area */}
      <div className="flex-1 bg-black rounded p-3 overflow-y-auto font-mono text-xs space-y-1 select-text border border-neutral-900">
        {logs.length === 0 ? (
          <div className="text-neutral-600 text-center py-10 font-mono text-xs">
            Console ready. Connect via Serial/Bluetooth or activate Demo Mode.
          </div>
        ) : (
          logs.map((l, idx) => (
            <div key={idx} className={`leading-relaxed ${getLineStyle(l)}`}>
              {l}
            </div>
          ))
        )}
        <div ref={bottomRef} />
      </div>

      {/* Quick Command Pills */}
      <div className="flex items-center gap-1.5 mt-2.5 overflow-x-auto pb-1 text-[11px] font-mono">
        <span className="text-neutral-600 uppercase text-[10px] font-semibold">Quick:</span>
        {quickCommands.map((cmd) => (
          <button
            key={cmd}
            onClick={() => serialService.sendCommand(cmd)}
            className="px-2 py-0.5 rounded bg-neutral-900 hover:bg-neutral-800 text-neutral-300 transition border border-neutral-800 cursor-pointer"
          >
            {cmd}
          </button>
        ))}
      </div>

      {/* Input Bar */}
      <div className="flex items-center gap-2 mt-2">
        <div className="relative flex-1">
          <span className="absolute left-3 top-2 text-xs font-mono text-red-600">{'>'}</span>
          <input
            type="text"
            value={inputCmd}
            onChange={(e) => setInputCmd(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Type command ('param set <NAME> <VAL>', 'status', 'arm')..."
            className="w-full bg-black text-neutral-100 font-mono text-xs pl-7 pr-3 py-2 rounded border border-neutral-800 focus:outline-none focus:border-red-600 transition"
          />
        </div>
        <button
          onClick={handleSend}
          className="px-3.5 py-2 bg-red-600 hover:bg-red-700 text-white rounded text-xs font-mono font-bold flex items-center gap-1.5 transition cursor-pointer"
        >
          <Send className="w-3.5 h-3.5" />
          <span>SEND</span>
        </button>
      </div>
    </div>
  );
};
