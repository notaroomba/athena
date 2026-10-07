import { useMemo, useRef, Component, type ReactNode } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import type { NoseAxis, Quat } from "@/lib/types";

interface RocketProps {
  q: Quat | null; // body -> NED
  nose: NoseAxis;
}

// The rocket is built with the nose along +Y, then rotated to the chosen body axis.
const NOSE_ROT: Record<NoseAxis, [number, number, number]> = {
  "+X": [0, 0, -Math.PI / 2],
  "-X": [0, 0, Math.PI / 2],
  "+Y": [0, 0, 0],
  "-Y": [Math.PI, 0, 0],
  "+Z": [Math.PI / 2, 0, 0],
  "-Z": [-Math.PI / 2, 0, 0],
};

class CanvasErrorBoundary extends Component<
  { children: ReactNode; fallback: ReactNode },
  { hasError: boolean }
> {
  constructor(props: { children: ReactNode; fallback: ReactNode }) {
    super(props);
    this.state = { hasError: false };
  }
  static getDerivedStateFromError() {
    return { hasError: true };
  }
  render() {
    if (this.state.hasError) return this.props.fallback;
    return this.props.children;
  }
}

/**
 * q rotates body -> NED. Three.js is Y-up right-handed: x = East, y = -Down, z = -North.
 * Writes the body axes (as columns) into `m`.
 */
function setMatrixFromQuat(m: THREE.Matrix4, q: Quat) {
  const [w, x, y, z] = q;
  const R = [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
  const toThree = (v: number[]) => [v[1], -v[2], -v[0]]; // NED (N,E,D) -> three (E,-D,-N)
  const bx = toThree([R[0][0], R[1][0], R[2][0]]);
  const by = toThree([R[0][1], R[1][1], R[2][1]]);
  const bz = toThree([R[0][2], R[1][2], R[2][2]]);
  m.set(bx[0], by[0], bz[0], 0, bx[1], by[1], bz[1], 0, bx[2], by[2], bz[2], 0, 0, 0, 0, 1);
}

function Rocket({ q, nose }: RocketProps) {
  const ref = useRef<THREE.Group>(null);
  const axes = useMemo(() => {
    const a = new THREE.AxesHelper(0.9); // body X, Y, Z in the series colours
    a.position.y = -0.8;
    a.setColors(new THREE.Color("#ea5a2c"), new THREE.Color("#1f9aa8"), new THREE.Color("#3b5fd0"));
    return a;
  }, []);

  useFrame(() => {
    if (!ref.current) return;
    if (q) setMatrixFromQuat(ref.current.matrix, q);
    else ref.current.matrix.identity();
    ref.current.matrixWorldNeedsUpdate = true;
  });

  const mat = { metalness: 0.15, roughness: 0.6 };
  return (
    <group ref={ref} matrixAutoUpdate={false}>
      <primitive object={axes} />
      <group rotation={NOSE_ROT[nose]}>
        <mesh>
          <cylinderGeometry args={[0.16, 0.16, 1.6, 32]} />
          <meshStandardMaterial color="#f3dfb0" {...mat} />
        </mesh>
        <mesh position={[0, 1.05, 0]}>
          <coneGeometry args={[0.16, 0.5, 32]} />
          <meshStandardMaterial color="#f2552a" {...mat} />
        </mesh>
        {[0, 1, 2, 3].map((i) => (
          <group key={i} rotation={[0, (i * Math.PI) / 2, 0]}>
            <mesh position={[0, -0.6, 0.3]}>
              <boxGeometry args={[0.02, 0.4, 0.3]} />
              <meshStandardMaterial color="#1f8a97" {...mat} />
            </mesh>
          </group>
        ))}
      </group>
    </group>
  );
}

export default function BoardVisualizer(props: RocketProps) {
  return (
    <CanvasErrorBoundary
      fallback={
        <div className="flex h-full items-center justify-center">
          <span className="text-sm tracking-wider text-ink-3">3D UNAVAILABLE</span>
        </div>
      }
    >
      <Canvas
        camera={{ position: [3.4, 2.0, 4.4], fov: 32 }}
        style={{ background: "transparent", touchAction: "pan-y" }}
        gl={{ antialias: true, alpha: true, powerPreference: "default" }}
        onCreated={({ gl }) => {
          gl.domElement.addEventListener("webglcontextlost", (e) => e.preventDefault());
        }}
      >
        <ambientLight intensity={0.55} />
        <directionalLight position={[3, 5, 4]} intensity={0.9} />
        <Rocket {...props} />
        <OrbitControls enableDamping dampingFactor={0.1} minDistance={2} maxDistance={12} target={[0, -0.1, 0]} />
        <gridHelper args={[4, 16, "#2a2a30", "#1b1b1f"]} position={[0, -1.5, 0]} />
      </Canvas>
    </CanvasErrorBoundary>
  );
}
