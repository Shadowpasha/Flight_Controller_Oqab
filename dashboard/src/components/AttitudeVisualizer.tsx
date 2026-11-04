import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';

interface AttitudeProps {
  roll: number;
  pitch: number;
  yaw: number;
}

export const AttitudeVisualizer: React.FC<AttitudeProps> = ({ roll, pitch, yaw }) => {
  const mountRef = useRef<HTMLDivElement>(null);
  const droneGroupRef = useRef<THREE.Group | null>(null);

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
      { x: 0.46, z: -0.46, isFront: true },  // Front-Right
      { x: -0.46, z: -0.46, isFront: true }, // Front-Left
      { x: 0.46, z: 0.46, isFront: false },  // Rear-Right
      { x: -0.46, z: 0.46, isFront: false }, // Rear-Left
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
    droneGroupRef.current.rotation.x = pitch * deg2rad;
    droneGroupRef.current.rotation.z = -roll * deg2rad;
    droneGroupRef.current.rotation.y = -yaw * deg2rad;
  }, [roll, pitch, yaw]);

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
      </div>

      <div ref={mountRef} className="w-full flex-1" />
    </div>
  );
};
