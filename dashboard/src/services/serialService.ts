import { DroneTelemetry, ConnectionState, FlowLivePoint } from '../types';

type LineCallback = (line: string) => void;
type TelemetryCallback = (data: DroneTelemetry) => void;
type StatusCallback = (status: ConnectionState) => void;
type FlowLiveCallback = (point: FlowLivePoint) => void;

class SerialService {
  private port: any = null;
  private reader: any = null;
  private writer: any = null;
  private status: ConnectionState = 'disconnected';
  private lineListeners: LineCallback[] = [];
  private telemetryListeners: TelemetryCallback[] = [];
  private statusListeners: StatusCallback[] = [];
  private flowLiveListeners: FlowLiveCallback[] = [];

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
    isSimMode: false,
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

  public onFlowLive(cb: FlowLiveCallback) {
    this.flowLiveListeners.push(cb);
    return () => {
      this.flowLiveListeners = this.flowLiveListeners.filter((l) => l !== cb);
    };
  }

  private setStatus(s: ConnectionState) {
    this.status = s;
    this.statusListeners.forEach((cb) => cb(s));
  }

  public async connect(): Promise<boolean> {
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

      // Query mounting rotation and flow parameters from Flash immediately on connection
      setTimeout(() => {
        if (this.status === 'connected') {
          this.sendCommand('param get SENS_FLOW_ROT');
        }
      }, 400);

      return true;
    } catch (err: any) {
      console.error('Serial connection error:', err);
      this.setStatus('disconnected');
      return false;
    }
  }

  public async disconnect(): Promise<void> {
    this.setStatus('disconnected');

    if (this.reader) {
      try {
        await this.reader.cancel();
      } catch (e) {
        console.warn('Reader cancel warning:', e);
      }
      try {
        this.reader.releaseLock();
      } catch (e) {}
      this.reader = null;
    }

    // Wait a brief tick for the reading loop to cleanly exit and release locks
    await new Promise((resolve) => setTimeout(resolve, 80));

    if (this.port) {
      try {
        await this.port.close();
      } catch (err) {
        console.warn('Port close warning:', err);
      }
      this.port = null;
    }

    this.setStatus('disconnected');
  }

  public async sendCommand(cmd: string): Promise<void> {
    const cleanCmd = cmd.trim();
    if (!cleanCmd) return;

    // Echo to listeners
    this.emitLine(`> ${cleanCmd}`);

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
    const textDecoder = new TextDecoder();
    while (this.port && this.port.readable && this.status === 'connected') {
      try {
        const reader = this.port.readable.getReader();
        this.reader = reader;

        let buffer = '';
        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) {
            buffer += textDecoder.decode(value, { stream: true });
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
      } catch (err: any) {
        if (this.status === 'connected') {
          console.warn('Serial read loop interrupted:', err);
        }
        break;
      } finally {
        if (this.reader) {
          try {
            this.reader.releaseLock();
          } catch (e) {}
          this.reader = null;
        }
      }
    }
    this.setStatus('disconnected');
  }

  private emitLine(line: string) {
    this.lineListeners.forEach((cb) => cb(line));
  }

  private parseTelemetryLine(line: string) {
    if (line.includes('--> SIM MODE ON')) {
      this.latestTelemetry.isSimMode = true;
      this.latestTelemetry.isArmed = true;
      this.latestTelemetry.stateTag = 'SIM';
      this.emitTelemetry();
      return;
    }
    if (line.includes('--> SIM MODE OFF')) {
      this.latestTelemetry.isSimMode = false;
      this.latestTelemetry.isArmed = false;
      this.latestTelemetry.stateTag = 'DISARM';
      this.emitTelemetry();
      return;
    }

    if (line.includes('[FLOW LIVE]')) {
      try {
        const qMatch = line.match(/Q:\s*(\d+)/);
        const rotLiveMatch = line.match(/Rot:\s*([\d.-]+)/i);
        const vfwdMatch = line.match(/V_fwd:\s*([+\-\d.]+)m\/s/);
        const vrightMatch = line.match(/V_right:\s*([+\-\d.]+)m\/s/);
        const ofMatch = line.match(/OF_x:\s*([+\-\d.]+)dps\s+OF_y:\s*([+\-\d.]+)dps/);
        const gyroMatch = line.match(/Gyro:\(\s*([+\-\d.]+),\s*([+\-\d.]+),\s*([+\-\d.]+)\)dps/);
        const hMatch = line.match(/H:\s*([\d.]+)m/);
        const dxyMatch = line.match(/dX:\s*([+\-\d]+)\s+dY:\s*([+\-\d]+)/);

        const quality = qMatch ? parseInt(qMatch[1], 10) : 0;
        if (rotLiveMatch) {
          this.latestTelemetry.flowRotDeg = parseFloat(rotLiveMatch[1]);
        }
        const vFwd = vfwdMatch ? parseFloat(vfwdMatch[1]) : 0;
        const vRight = vrightMatch ? parseFloat(vrightMatch[1]) : 0;
        const gyroX = gyroMatch ? parseFloat(gyroMatch[1]) : 0;
        const gyroY = gyroMatch ? parseFloat(gyroMatch[2]) : 0;
        const gyroZ = gyroMatch ? parseFloat(gyroMatch[3]) : 0;
        const heightM = hMatch ? parseFloat(hMatch[1]) : 0.8;
        const rawX = dxyMatch ? parseInt(dxyMatch[1], 10) : 0;
        const rawY = dxyMatch ? parseInt(dxyMatch[2], 10) : 0;

        let ofX = 0;
        let ofY = 0;
        if (ofMatch) {
          ofX = parseFloat(ofMatch[1]);
          ofY = parseFloat(ofMatch[2]);
        } else {
          // Fallback if older firmware string: estimate optical angular rates
          const safeH = Math.max(0.1, heightM);
          ofX = (vFwd / safeH) * (180.0 / Math.PI) + gyroY;
          ofY = (vRight / safeH) * (180.0 / Math.PI) + gyroX;
        }

        const point: FlowLivePoint = {
          time: Date.now(),
          quality,
          vFwd,
          vRight,
          ofX,
          ofY,
          gyroX,
          gyroY,
          gyroZ,
          heightM,
          rawX,
          rawY,
        };

        this.flowLiveListeners.forEach((cb) => cb(point));

        // Also keep telemetry fields up to date
        this.latestTelemetry.flowQuality = quality;
        this.latestTelemetry.flowSpeed = Math.sqrt(vFwd * vFwd + vRight * vRight);
        this.latestTelemetry.velNorth = vFwd;
        this.latestTelemetry.velEast = vRight;
        this.latestTelemetry.lidarCm = heightM * 100.0;
        this.emitTelemetry();
      } catch (err) {
        console.warn('Error parsing [FLOW LIVE] line:', err);
      }
      return;
    }

    // Example:
    // [DEAD-RECKONING] Home:BOOT | Dist: 2D= 0.00m, 3D= 0.10m | Pos: dN= +0.00m, dE= +0.00m, dAlt= +0.10m( 10cm) | Vel: Vn= +0.00m/s, Ve= +0.00m/s, Vz= +0.00m/s (Spd:0.00) | Acc: GPS[FLOW(Q:185,Spd:0.02)] Raw:(+0.00,+0.00)m Alt:LID[LIDAR:10.2]cm | DISARM [MDBCI|CAS]
    if (!line.includes('[DEAD-RECKONING]')) {
      // Check for mounting rotation responses from param get, param show, flow report, or flow rot
      const rotMatch = line.match(/(?:SENS_FLOW_ROT\s*=\s*|Mounting Rot\s*:\s*|Rot:\s*|Optical Flow Mounting Rotation set to:\s*)([\d.-]+)/i);
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

      const posMatch = line.match(/Pos:\s*(?:dN|X)=\s*([+\-\d.]+)m,\s*(?:dE|Y)=\s*([+\-\d.]+)m,\s*(?:dAlt|Z)=\s*([+\-\d.]+)m\(\s*([\d.]+)cm\)/);
      if (posMatch) {
        this.latestTelemetry.posNorth = parseFloat(posMatch[1]);
        this.latestTelemetry.posEast = parseFloat(posMatch[2]);
        this.latestTelemetry.altDiff = parseFloat(posMatch[3]);
        this.latestTelemetry.heightCm = parseFloat(posMatch[4]);
      }

      const velMatch = line.match(/Vel:\s*(?:Vn|Vx)=\s*([+\-\d.]+)m\/s,\s*(?:Ve|Vy)=\s*([+\-\d.]+)m\/s,\s*Vz=\s*([+\-\d.]+)m\/s\s*\(Spd:([\d.]+)\)/);
      if (velMatch) {
        this.latestTelemetry.velNorth = parseFloat(velMatch[1]);
        this.latestTelemetry.velEast = parseFloat(velMatch[2]);
        this.latestTelemetry.velDown = parseFloat(velMatch[3]);
        this.latestTelemetry.speed3d = parseFloat(velMatch[4]);
      }

      const attMatch = line.match(/(?:Att|Rot):\s*R=\s*([+\-\d.]+)[,\s]+P=\s*([+\-\d.]+)[,\s]+Y=\s*([+\-\d.]+)/i);
      if (attMatch) {
        this.latestTelemetry.rollDeg = parseFloat(attMatch[1]);
        this.latestTelemetry.pitchDeg = parseFloat(attMatch[2]);
        this.latestTelemetry.yawDeg = parseFloat(attMatch[3]);
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
        const tag = endMatch[1];
        this.latestTelemetry.stateTag = tag;
        this.latestTelemetry.controller = endMatch[2];
        this.latestTelemetry.mode = endMatch[3];
        this.latestTelemetry.isSimMode = tag === 'SIM';
        this.latestTelemetry.isArmed = tag.startsWith('ARM') || tag === 'SIM';
        this.latestTelemetry.isAirborne = tag.includes(':CLB') || tag.includes(':HOV') || tag.includes(':LND');
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
}

export const serialService = new SerialService();

