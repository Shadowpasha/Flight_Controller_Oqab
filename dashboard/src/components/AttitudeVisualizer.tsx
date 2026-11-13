import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';

interface AttitudeProps {
  roll: number;
  pitch: number;
  yaw: number;
  outRoll?: number;
  outPitch?: number;
  outYaw?: number;
  isSimMode?: boolean;
}

export const AttitudeVisualizer: React.FC<AttitudeProps> = ({
  roll,
  pitch,
  yaw,
  outRoll = 0,
  outPitch = 0,
  outYaw = 0,
  isSimMode = false,
}) => {
  const mountRef = useRef<HTMLDivElement>(null);
  const droneGroupRef = useRef<THREE.Group | null>(null);
  const arrowsGroupRef = useRef<THREE.Group | null>(null);
  const rollArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const pitchArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const motorArrowsRef = useRef<THREE.ArrowHelper[]>([]);

  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    const width = container.clientWidth;
    const height = container.clientHeight;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x050505);

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.position.set(0, 3.2, 5.0);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.4);
    dirLight.position.set(5, 10, 5);
    dirLight.castShadow = true;
    scene.add(dirLight);

    const grid = new THREE.GridHelper(10, 20, 0xdc2626, 0x1f1f1f);
    grid.position.y = -1.2;
    scene.add(grid);

    const droneGroup = new THREE.Group();
    droneGroup.rotation.order = 'YXZ';
    scene.add(droneGroup);
    droneGroupRef.current = droneGroup;

    // Center body
    const bodyGeo = new THREE.BoxGeometry(0.7, 0.12, 0.7);
    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x141414,
      roughness: 0.4,
      metalness: 0.8,
    });
    const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
    droneGroup.add(bodyMesh);

    // Flight controller board
    const fcGeo = new THREE.BoxGeometry(0.35, 0.05, 0.35);
    const fcMat = new THREE.MeshStandardMaterial({ color: 0x1c1917, roughness: 0.5 });
    const fcMesh = new THREE.Mesh(fcGeo, fcMat);
    fcMesh.position.y = 0.08;
    droneGroup.add(fcMesh);

    // Red status dot on FC
    const ledGeo = new THREE.SphereGeometry(0.025, 8, 8);
    const ledMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    const ledMesh = new THREE.Mesh(ledGeo, ledMat);
    ledMesh.position.set(0, 0.11, 0);
    droneGroup.add(ledMesh);

    // Forward Arrow in iconic red
    const arrowGeo = new THREE.ConeGeometry(0.12, 0.3, 8);
    const arrowMat = new THREE.MeshStandardMaterial({ color: 0xdc2626 });
    const arrowMesh = new THREE.Mesh(arrowGeo, arrowMat);
    arrowMesh.rotation.x = -Math.PI / 2;
    arrowMesh.position.set(0, 0.12, -0.45);
    droneGroup.add(arrowMesh);

    // Quad-X Arms
    const armGeo = new THREE.CylinderGeometry(0.04, 0.04, 1.3, 12);
    const armMat = new THREE.MeshStandardMaterial({ color: 0x262626, metalness: 0.8 });

    const arm1 = new THREE.Mesh(armGeo, armMat);
    arm1.rotation.z = Math.PI / 2;
    arm1.rotation.y = Math.PI / 4;
    droneGroup.add(arm1);

    const arm2 = new THREE.Mesh(armGeo, armMat);
    arm2.rotation.z = Math.PI / 2;
    arm2.rotation.y = -Math.PI / 4;
    droneGroup.add(arm2);

    // Motors & Propellers (Front: Red, Rear: Dark Graphite)
    const motorGeo = new THREE.CylinderGeometry(0.09, 0.09, 0.1, 16);
    const motorMat = new THREE.MeshStandardMaterial({ color: 0x171717, metalness: 0.9 });

    const propGeo = new THREE.BoxGeometry(0.65, 0.015, 0.08);
    const frontPropMat = new THREE.MeshStandardMaterial({
      color: 0xdc2626,
      roughness: 0.3,
    });
    const rearPropMat = new THREE.MeshStandardMaterial({
      color: 0x525252,
      roughness: 0.3,
    });

    const motorPositions = [
      { id: 1, x:  0.46, z: -0.46, isFront: true  }, // M1: Front-Right
      { id: 2, x:  0.46, z:  0.46, isFront: false }, // M2: Rear-Right
      { id: 3, x: -0.46, z:  0.46, isFront: false }, // M3: Rear-Left
      { id: 4, x: -0.46, z: -0.46, isFront: true  }, // M4: Front-Left
    ];

    const props: THREE.Mesh[] = [];

    motorPositions.forEach((pos) => {
      const motor = new THREE.Mesh(motorGeo, motorMat);
      motor.position.set(pos.x, 0.08, pos.z);
      droneGroup.add(motor);

      const prop = new THREE.Mesh(propGeo, pos.isFront ? frontPropMat : rearPropMat);
      prop.position.set(pos.x, 0.14, pos.z);
      droneGroup.add(prop);
      props.push(prop);
    });

    // --- Dynamic PID Correction Torque Arrows Group ---
    const arrowsGroup = new THREE.Group();
    droneGroup.add(arrowsGroup);
    arrowsGroupRef.current = arrowsGroup;

    // Pitch Correction Arrow (along body Z-axis: forward/backward torque)
    const pitchArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, -1),
      new THREE.Vector3(0, 0.25, 0),
      0.001,
      0x38bdf8, // Sky Blue for pitch
      0.15,
      0.08
    );
    arrowsGroup.add(pitchArrow);
    pitchArrowRef.current = pitchArrow;

    // Roll Correction Arrow (along body X-axis: right/left torque)
    const rollArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0.25, 0),
      0.001,
      0xf59e0b, // Amber for roll
      0.15,
      0.08
    );
    arrowsGroup.add(rollArrow);
    rollArrowRef.current = rollArrow;

    // Per-motor vertical thrust delta arrows (M1: FR, M2: RR, M3: RL, M4: FL)
    const motorArrows: THREE.ArrowHelper[] = [];
    motorPositions.forEach((pos) => {
      const mArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(pos.x, 0.20, pos.z),
        0.001,
        0x22c55e,
        0.12,
        0.06
      );
      arrowsGroup.add(mArrow);
      motorArrows.push(mArrow);
    });
    motorArrowsRef.current = motorArrows;

    let animationId: number;
    const animate = () => {
      animationId = requestAnimationFrame(animate);

      props.forEach((p, idx) => {
        p.rotation.y += (idx % 2 === 0 ? 0.25 : -0.25);
      });

      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      if (!container) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };

    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
      cancelAnimationFrame(animationId);
      renderer.dispose();
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement);
      }
    };
  }, []);

  useEffect(() => {
    if (!droneGroupRef.current) return;
    const deg2rad = Math.PI / 180;
    droneGroupRef.current.rotation.set(-pitch * deg2rad, yaw * deg2rad, -roll * deg2rad, 'YXZ');
  }, [roll, pitch, yaw]);

  // Update dynamic PID correction arrows
  useEffect(() => {
    const maxPid = 250.0;
    const maxArrowLen = 1.3;

    // Pitch torque arrow: positive pitch output commands nose down (forward acceleration / thrust from rear)
    // Three.js body frame: Forward is -Z, Right is +X, Up is +Y
    if (pitchArrowRef.current) {
      const magP = Math.min(Math.abs(outPitch) / maxPid, 1.0) * maxArrowLen;
      if (magP > 0.05) {
        pitchArrowRef.current.visible = true;
        // outPitch > 0 -> pitch nose down (forward push, vector along -Z); outPitch < 0 -> pitch nose up (vector along +Z)
        const dirZ = outPitch > 0 ? -1 : 1;
        pitchArrowRef.current.setDirection(new THREE.Vector3(0, 0, dirZ));
        pitchArrowRef.current.setLength(magP, Math.min(0.18, magP * 0.4), Math.min(0.09, magP * 0.25));
      } else {
        pitchArrowRef.current.visible = false;
      }
    }

    // Roll torque arrow: outRoll > 0 -> roll right down (push rightwards, vector along +X)
    if (rollArrowRef.current) {
      const magR = Math.min(Math.abs(outRoll) / maxPid, 1.0) * maxArrowLen;
      if (magR > 0.05) {
        rollArrowRef.current.visible = true;
        const dirX = outRoll > 0 ? 1 : -1;
        rollArrowRef.current.setDirection(new THREE.Vector3(dirX, 0, 0));
        rollArrowRef.current.setLength(magR, Math.min(0.18, magR * 0.4), Math.min(0.09, magR * 0.25));
      } else {
        rollArrowRef.current.visible = false;
      }
    }

    // Per-motor thrust delta arrows (Quad-X mixer):
    // M1 Front-Right (x: +0.46, z: -0.46): -out_roll - out_pitch
    // M2 Rear-Right  (x: +0.46, z: +0.46): -out_roll + out_pitch
    // M3 Rear-Left   (x: -0.46, z: +0.46): +out_roll + out_pitch
    // M4 Front-Left  (x: -0.46, z: -0.46): +out_roll - out_pitch
    const motorDeltas = [
      -outRoll - outPitch, // M1 FR
      -outRoll + outPitch, // M2 RR
      +outRoll + outPitch, // M3 RL
      +outRoll - outPitch, // M4 FL
    ];

    motorArrowsRef.current.forEach((mArrow, idx) => {
      const delta = motorDeltas[idx] ?? 0;
      const mag = Math.min(Math.abs(delta) / 200.0, 1.0) * 0.9;
      if (mag > 0.04) {
        mArrow.visible = true;
        // delta > 0: motor speeding up (thrust pushing UP); delta < 0: motor slowing down
        const dirY = delta >= 0 ? 1 : -1;
        mArrow.setDirection(new THREE.Vector3(0, dirY, 0));
        mArrow.setLength(mag, Math.min(0.12, mag * 0.35), Math.min(0.06, mag * 0.2));
        mArrow.setColor(new THREE.Color(delta >= 0 ? 0x22c55e : 0xef4444));
      } else {
        mArrow.visible = false;
      }
    });
  }, [outRoll, outPitch, outYaw]);

  return (
    <div className="relative w-full h-full min-h-[340px] rounded-lg overflow-hidden bg-[#0a0a0a] border border-neutral-800 flex flex-col">
      <div className="absolute top-3 left-4 z-10 flex items-center gap-2">
        <div className="w-2 h-2 rounded-full bg-red-600" />
        <span className="text-[11px] font-mono uppercase tracking-widest text-neutral-400">
          Attitude Horizon
        </span>
      </div>

      <div className="absolute top-3 right-4 z-10 flex gap-3 text-xs font-mono bg-black/90 px-3 py-1.5 rounded border border-neutral-800">
        <div>
          <span className="text-neutral-500">R: </span>
          <span className="font-semibold text-neutral-200">
            {roll >= 0 ? '+' : ''}{roll.toFixed(1)}°
          </span>
        </div>
        <div>
          <span className="text-neutral-500">P: </span>
          <span className="font-semibold text-neutral-200">
            {pitch >= 0 ? '+' : ''}{pitch.toFixed(1)}°
          </span>
        </div>
        <div>
          <span className="text-neutral-500">Y: </span>
          <span className="font-semibold text-neutral-200">
            {yaw.toFixed(0)}°
          </span>
        </div>
      </div>

      <div className="absolute bottom-3 left-4 z-10 flex items-center gap-3 text-[11px] text-neutral-400 font-mono">
        <div className="flex items-center gap-1.5">
          <div className="w-2.5 h-2.5 rounded bg-red-600" />
          <span>Front (Forward)</span>
        </div>
        <div className="flex items-center gap-1.5">
          <div className="w-2.5 h-2.5 rounded bg-neutral-600" />
          <span>Rear</span>
        </div>
        {isSimMode && (
          <div className="hidden sm:flex items-center gap-2.5 pl-2 border-l border-neutral-800 text-[10px]">
            <span className="flex items-center gap-1">
              <span className="w-2 h-0.5 bg-sky-400 inline-block" /> Pitch PID
            </span>
            <span className="flex items-center gap-1">
              <span className="w-2 h-0.5 bg-amber-400 inline-block" /> Roll PID
            </span>
            <span className="flex items-center gap-1">
              <span className="w-2 h-2 rounded-full bg-emerald-500 inline-block" /> Motor ΔPWM
            </span>
          </div>
        )}
      </div>

      <div ref={mountRef} className="w-full flex-1" />
    </div>
  );
};
