import { DroneTelemetry, ConnectionState } from '../types';

type LineCallback = (line: string) => void;
type TelemetryCallback = (data: DroneTelemetry) => void;
type StatusCallback = (status: ConnectionState) => void;

class SerialService {
  private port: any = null;
  private reader: any = null;
  private writer: any = null;
  private status: ConnectionState = 'disconnected';
  private lineListeners: LineCallback[] = [];
  private telemetryListeners: TelemetryCallback[] = [];
  private statusListeners: StatusCallback[] = [];
  private simInterval: any = null;
  private simTime: number = 0;

  // Telemetry state cache
  private latestTelemetry: DroneTelemetry = {
    homeOrigin: 'BOOT',
    dist2d: 0.0,
    dist3d: 0.0,
    posNorth: 0.0,
    posEast: 0.0,
    altDiff: 0.0,
    heightCm: 10.0,
    velNorth: 0.0,
    velEast: 0.0,
    velDown: 0.0,
    speed3d: 0.0,
    gpsStatus: 'FLOW(Q:0,Spd:0.00)',
    rawNorth: 0.0,
    rawEast: 0.0,
    altSource: 'LID',
    lidarCm: 10.0,
    stateTag: 'DISARM',
    controller: 'MDBCI',
    mode: 'CAS',
    rollDeg: 0.0,
    pitchDeg: 0.0,
    yawDeg: 0.0,
    flowQuality: 0,
    flowSpeed: 0.0,
    flowRotDeg: 0.0,
    isArmed: false,
    isAirborne: false,
    timestamp: Date.now(),
  };

  public getStatus(): ConnectionState {
    return this.status;
  }

  public getLatestTelemetry(): DroneTelemetry {
    return { ...this.latestTelemetry };
  }

  public onLine(cb: LineCallback) {
    this.lineListeners.push(cb);
    return () => {
      this.lineListeners = this.lineListeners.filter((l) => l !== cb);
    };
  }

  public onTelemetry(cb: TelemetryCallback) {
    this.telemetryListeners.push(cb);
    return () => {
      this.telemetryListeners = this.telemetryListeners.filter((l) => l !== cb);
    };
  }

  public onStatus(cb: StatusCallback) {
    this.statusListeners.push(cb);
    return () => {
      this.statusListeners = this.statusListeners.filter((l) => l !== cb);
    };
  }

  private setStatus(s: ConnectionState) {
    this.status = s;
    this.statusListeners.forEach((cb) => cb(s));
  }

  public async connect(): Promise<boolean> {
    if (this.status === 'simulated') {
      this.stopSimulation();
    }

    if (!('serial' in navigator)) {
      alert('Web Serial API is not supported in this browser. Please use Google Chrome, Microsoft Edge, or the native Tauri desktop app.');
      return false;
    }

    try {
      this.setStatus('connecting');
      this.port = await (navigator as any).serial.requestPort();
      await this.port.open({ baudRate: 115200 });

      this.setStatus('connected');
      this.startReading();
      return true;
    } catch (err: any) {
      console.error('Serial connection error:', err);
      this.setStatus('disconnected');
      return false;
    }
  }

  public async disconnect(): Promise<void> {
    if (this.simInterval) {
      this.stopSimulation();
    }

    if (this.reader) {
      try {
        await this.reader.cancel();
      } catch (e) {}
      this.reader = null;
    }

    if (this.port) {
      try {
        await this.port.close();
      } catch (e) {}
      this.port = null;
    }

    this.setStatus('disconnected');
  }

  public async sendCommand(cmd: string): Promise<void> {
    const cleanCmd = cmd.trim();
    if (!cleanCmd) return;

    // Echo to listeners
    this.emitLine(`> ${cleanCmd}`);

    if (this.status === 'simulated') {
      this.handleSimulatedCommand(cleanCmd);
      return;
    }

    if (!this.port || !this.port.writable) {
      console.warn('Cannot send command: port not open');
      return;
    }

    try {
      const encoder = new TextEncoder();
      const writer = this.port.writable.getWriter();
      await writer.write(encoder.encode(cleanCmd + '\r\n'));
      writer.releaseLock();
    } catch (err) {
      console.error('Failed to write command to serial port:', err);
    }
  }

  private async startReading() {
    while (this.port && this.port.readable && this.status === 'connected') {
      try {
        const textDecoder = new (window as any).TextDecoderStream();
        const readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
        const reader = textDecoder.readable.getReader();
        this.reader = reader;

        let buffer = '';
        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) {
            buffer += value;
            const lines = buffer.split(/\r?\n/);
            buffer = lines.pop() || '';
            for (const line of lines) {
              const trimmed = line.trim();
              if (trimmed) {
                this.emitLine(trimmed);
                this.parseTelemetryLine(trimmed);
              }
            }
          }
        }
      } catch (err) {
        console.warn('Serial read loop error or closed:', err);
        break;
      }
    }
    this.setStatus('disconnected');
  }

  private emitLine(line: string) {
    this.lineListeners.forEach((cb) => cb(line));
  }

  private parseTelemetryLine(line: string) {
    // Example:
    // [DEAD-RECKONING] Home:BOOT | Dist: 2D= 0.00m, 3D= 0.10m | Pos: dN= +0.00m, dE= +0.00m, dAlt= +0.10m( 10cm) | Vel: Vn= +0.00m/s, Ve= +0.00m/s, Vz= +0.00m/s (Spd:0.00) | Acc: GPS[FLOW(Q:185,Spd:0.02)] Raw:(+0.00,+0.00)m Alt:LID[LIDAR:10.2]cm | DISARM [MDBCI|CAS]
    if (!line.includes('[DEAD-RECKONING]')) {
      // Check for mounting rotation or param responses
      const rotMatch = line.match(/Optical Flow Mounting Rotation set to: ([\d.-]+)/);
      if (rotMatch) {
        this.latestTelemetry.flowRotDeg = parseFloat(rotMatch[1]);
        this.emitTelemetry();
      }
      return;
    }

    try {
      const homeMatch = line.match(/Home:([A-Z]+)/);
      if (homeMatch) this.latestTelemetry.homeOrigin = homeMatch[1];

      const distMatch = line.match(/Dist:\s*2D=\s*([\d.-]+)m,\s*3D=\s*([\d.-]+)m/);
      if (distMatch) {
        this.latestTelemetry.dist2d = parseFloat(distMatch[1]);
        this.latestTelemetry.dist3d = parseFloat(distMatch[2]);
      }

      const posMatch = line.match(/Pos:\s*dN=\s*([+\-\d.]+)m,\s*dE=\s*([+\-\d.]+)m,\s*dAlt=\s*([+\-\d.]+)m\(\s*([\d.]+)cm\)/);
      if (posMatch) {
        this.latestTelemetry.posNorth = parseFloat(posMatch[1]);
        this.latestTelemetry.posEast = parseFloat(posMatch[2]);
        this.latestTelemetry.altDiff = parseFloat(posMatch[3]);
        this.latestTelemetry.heightCm = parseFloat(posMatch[4]);
      }

      const velMatch = line.match(/Vel:\s*Vn=\s*([+\-\d.]+)m\/s,\s*Ve=\s*([+\-\d.]+)m\/s,\s*Vz=\s*([+\-\d.]+)m\/s\s*\(Spd:([\d.]+)\)/);
      if (velMatch) {
        this.latestTelemetry.velNorth = parseFloat(velMatch[1]);
        this.latestTelemetry.velEast = parseFloat(velMatch[2]);
        this.latestTelemetry.velDown = parseFloat(velMatch[3]);
        this.latestTelemetry.speed3d = parseFloat(velMatch[4]);
      }

      const flowMatch = line.match(/FLOW\(Q:(\d+),Spd:([\d.]+)\)/);
      if (flowMatch) {
        this.latestTelemetry.flowQuality = parseInt(flowMatch[1], 10);
        this.latestTelemetry.flowSpeed = parseFloat(flowMatch[2]);
        this.latestTelemetry.gpsStatus = `FLOW(Q:${flowMatch[1]},Spd:${flowMatch[2]})`;
      }

      const lidarMatch = line.match(/Alt:([A-Z]+)\[LIDAR:([\d.]+)\]cm/);
      if (lidarMatch) {
        this.latestTelemetry.altSource = lidarMatch[1];
        this.latestTelemetry.lidarCm = parseFloat(lidarMatch[2]);
      }

      const endMatch = line.match(/\|\s*([A-Za-z0-9:]+)\s*\[([A-Za-z0-9]+)\|([A-Za-z0-9]+)\]$/);
      if (endMatch) {
        this.latestTelemetry.stateTag = endMatch[1];
        this.latestTelemetry.controller = endMatch[2];
        this.latestTelemetry.mode = endMatch[3];
        this.latestTelemetry.isArmed = endMatch[1].startsWith('ARM');
        this.latestTelemetry.isAirborne = endMatch[1].includes(':CLB') || endMatch[1].includes(':HOV') || endMatch[1].includes(':LND');
      }

      this.latestTelemetry.timestamp = Date.now();
      this.emitTelemetry();
    } catch (e) {
      console.warn('Error parsing telemetry line:', e);
    }
  }

  private emitTelemetry() {
    const data = { ...this.latestTelemetry };
    this.telemetryListeners.forEach((cb) => cb(data));
  }

  // --- DEMO / SIMULATOR MODE ---
  public startSimulation() {
    this.disconnect();
    this.setStatus('simulated');
    this.emitLine('>>> [DEMO MODE ACTIVE] Emulating flight controller telemetry stream @ 20 Hz');
    this.simTime = 0;

    this.simInterval = setInterval(() => {
      this.simTime += 0.05;
      const t = this.simTime;

      // Realistic drone attitude & altitude oscillation
      const roll = Math.sin(t * 1.5) * 4.2;
      const pitch = Math.cos(t * 1.2) * 3.5;
      const yaw = ((t * 8) % 360) - 180;

      const baseHeight = this.latestTelemetry.isArmed ? 45.0 + Math.sin(t * 2.0) * 2.5 : 9.5;
      const vz = this.latestTelemetry.isArmed ? Math.cos(t * 2.0) * 0.08 : 0.0;
      const vn = this.latestTelemetry.isArmed ? Math.sin(t * 0.8) * 0.25 : 0.0;
      const ve = this.latestTelemetry.isArmed ? Math.cos(t * 0.8) * 0.2 : 0.0;
      const quality = this.latestTelemetry.isArmed ? 190 + Math.floor(Math.sin(t) * 15) : 175;

      this.latestTelemetry = {
        ...this.latestTelemetry,
        rollDeg: roll,
        pitchDeg: pitch,
        yawDeg: yaw,
        heightCm: baseHeight,
        lidarCm: baseHeight,
        velNorth: vn,
        velEast: ve,
        velDown: vz,
        speed3d: Math.sqrt(vn * vn + ve * ve + vz * vz),
        posNorth: Math.sin(t * 0.3) * 0.4,
        posEast: Math.cos(t * 0.3) * 0.3,
        dist2d: 0.5,
        dist3d: 0.6,
        flowQuality: quality,
        flowSpeed: Math.sqrt(vn * vn + ve * ve),
        stateTag: this.latestTelemetry.isArmed ? 'ARM:HOV' : 'DISARM',
        isAirborne: this.latestTelemetry.isArmed,
        timestamp: Date.now(),
      };

      const line = `[DEAD-RECKONING] Home:BOOT | Dist: 2D=${this.latestTelemetry.dist2d.toFixed(2)}m, 3D=${this.latestTelemetry.dist3d.toFixed(2)}m | Pos: dN=${this.latestTelemetry.posNorth >= 0 ? '+' : ''}${this.latestTelemetry.posNorth.toFixed(2)}m, dE=${this.latestTelemetry.posEast >= 0 ? '+' : ''}${this.latestTelemetry.posEast.toFixed(2)}m, dAlt=+${(this.latestTelemetry.heightCm / 100).toFixed(2)}m(${Math.round(this.latestTelemetry.heightCm)}cm) | Vel: Vn=${vn >= 0 ? '+' : ''}${vn.toFixed(2)}m/s, Ve=${ve >= 0 ? '+' : ''}${ve.toFixed(2)}m/s, Vz=${vz >= 0 ? '+' : ''}${vz.toFixed(2)}m/s (Spd:${this.latestTelemetry.speed3d.toFixed(2)}) | Acc: GPS[FLOW(Q:${quality},Spd:${this.latestTelemetry.flowSpeed.toFixed(2)})] Raw:(+0.00,+0.00)m Alt:LID[LIDAR:${this.latestTelemetry.lidarCm.toFixed(1)}]cm | ${this.latestTelemetry.stateTag} [${this.latestTelemetry.controller}|CAS]`;

      this.emitLine(line);
      this.emitTelemetry();
    }, 50);
  }

  public stopSimulation() {
    if (this.simInterval) {
      clearInterval(this.simInterval);
      this.simInterval = null;
    }
    this.setStatus('disconnected');
    this.emitLine('>>> [DEMO MODE STOPPED]');
  }

  private handleSimulatedCommand(cmd: string) {
    if (cmd.startsWith('param set SENS_FLOW_ROT')) {
      const val = parseFloat(cmd.split(' ')[3]);
      this.latestTelemetry.flowRotDeg = val;
      this.emitLine(`--> Optical Flow Mounting Rotation set to: ${val.toFixed(1)} deg (Saved to Flash)`);
    } else if (cmd.startsWith('ctrl ') || cmd.startsWith('controller ')) {
      const c = cmd.split(' ')[1].toUpperCase();
      this.latestTelemetry.controller = c;
      this.emitLine(`--> Controller switched to: ${c}`);
    } else if (cmd === 'arm') {
      this.latestTelemetry.isArmed = true;
      this.emitLine('>>> ARMED! Motors spooled to 1050us. Liftoff sequence armed.');
    } else if (cmd === 'disarm') {
      this.latestTelemetry.isArmed = false;
      this.emitLine('!!! DISARMED: User command');
    } else if (cmd === 'status') {
      this.emitLine('=================== F330 FLIGHT CONTROLLER STATUS ===================');
      this.emitLine(`  • Controller   : ${this.latestTelemetry.controller}`);
      this.emitLine(`  • Flow Rot     : ${this.latestTelemetry.flowRotDeg} deg`);
      this.emitLine(`  • Flow Quality : ${this.latestTelemetry.flowQuality} / 255`);
      this.emitLine(`  • ToF LiDAR    : ${this.latestTelemetry.lidarCm.toFixed(1)} cm`);
      this.emitLine('=====================================================================');
    } else {
      this.emitLine(`OK: [Simulated Ack] ${cmd}`);
    }
  }
}

export const serialService = new SerialService();

