import { encodeFrame, G0, PKT } from "./protocol";
import type { NoseAxis, Quat } from "./types";

// Scripted flight pushed through the real frame encoder, so the dashboard's
// decoder/parsers/charts see exactly what the firmware would send.

function qmul(a: Quat, b: Quat): Quat {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

// base orientation mapping the chosen body nose axis to NED up (-D)
const BASE: Record<NoseAxis, Quat> = {
  "+X": [Math.SQRT1_2, 0, -Math.SQRT1_2, 0],
  "-X": [Math.SQRT1_2, 0, Math.SQRT1_2, 0],
  "+Y": [Math.SQRT1_2, Math.SQRT1_2, 0, 0],
  "-Y": [Math.SQRT1_2, -Math.SQRT1_2, 0, 0],
  "+Z": [0, 1, 0, 0],
  "-Z": [1, 0, 0, 0],
};

const text = (s: string) => encodeFrame(PKT.TEXT, new TextEncoder().encode(s));

/** Starts the demo; returns a stop function. `emit` receives raw link bytes. */
export function startDemo(nose: NoseAxis, emit: (bytes: Uint8Array) => void): () => void {
  let t = 0,
    alt = 0,
    vel = 0,
    phaseName = "",
    apogee = 0;
  const dt = 0.05,
    lat0 = 40.5,
    lon0 = -74.0;
  const ax = { X: 0, Y: 1, Z: 2 }[nose[1] as "X" | "Y" | "Z"];
  const sign = nose[0] === "-" ? -1 : 1;
  const rnd = (s: number) => (Math.random() - 0.5) * 2 * s;

  const timer = setInterval(() => {
    t += dt;
    let thrust = 0,
      phase = "pad";
    if (t > 5 && t < 8) {
      thrust = 5 * G0;
      phase = "boost";
    } else if (t >= 8 && vel > 0) phase = "coast";
    else if (t >= 8 && alt > 0) phase = "descent";
    let a = thrust - G0;
    if (phase === "descent") a = (-8 - vel) * 2; // chute: settle to -8 m/s
    if (phase === "pad") {
      a = 0;
      vel = 0;
      alt = 0;
    }
    vel += a * dt;
    alt += vel * dt;
    if (alt < 0) {
      alt = 0;
      vel = 0;
      if (t > 10) phase = "landed";
    }
    apogee = Math.max(apogee, alt);
    if (phase !== phaseName) {
      emit(text(`[demo] ${phase}` + (phase === "descent" ? ` (apogee ${apogee.toFixed(0)} m)` : "")));
      phaseName = phase;
    }
    const spin = phase === "pad" ? 0 : 0.8;
    const tilt = phase === "pad" ? 0.01 : 0.06 * Math.sin(t * 1.3);
    // attitude: nose up with a little sway and a slow roll about the nose axis
    const qz: Quat = [Math.cos(tilt / 2), Math.sin(tilt / 2), 0, 0];
    const roll = spin * t;
    const qr: Quat = [Math.cos(roll / 2), 0, 0, 0];
    qr[1 + ax] = Math.sin(roll / 2) * sign;
    const q = qmul(qmul(qz, BASE[nose]), qr);
    const spec = sign * (a + G0); // specific force along the nose axis (body)
    const acc = [rnd(0.4), rnd(0.4), rnd(0.4)];
    acc[ax] += spec;
    const gyro = [rnd(0.01), rnd(0.01), rnd(0.01)];
    gyro[ax] += spin * sign;
    const flags = (phase !== "pad" && phase !== "landed" ? 1 : 0) | 2 | 4 | 8 | 16;

    const b = new ArrayBuffer(96),
      v = new DataView(b);
    const F = (o: number, x: number) => v.setFloat32(o, x, true);
    v.setUint32(0, Math.round(t * 1e6), true);
    q.forEach((x, i) => F(4 + 4 * i, x));
    [rnd(0.5), rnd(0.5), -alt].forEach((x, i) => F(20 + 4 * i, x));
    [0, 0, -vel].forEach((x, i) => F(32 + 4 * i, x));
    acc.forEach((x, i) => F(44 + 4 * i, x));
    gyro.forEach((x, i) => F(56 + 4 * i, x));
    [0.22, 0.05, -0.41].forEach((x, i) => F(68 + 4 * i, x));
    F(80, alt + rnd(0.8));
    v.setInt32(84, Math.round(lat0 * 1e7), true);
    v.setInt32(88, Math.round(lon0 * 1e7), true);
    v.setUint8(92, 7);
    v.setUint8(93, flags);
    v.setUint16(94, 400, true);
    emit(encodeFrame(PKT.STATE, new Uint8Array(b)));

    if (Math.round(t / dt) % 4 === 0) {
      // 5 Hz GPS
      const gb = new ArrayBuffer(44),
        gv = new DataView(gb);
      gv.setUint32(0, Math.round(t * 1000), true);
      gv.setUint8(4, 3);
      gv.setUint8(5, 12);
      gv.setUint8(6, 1);
      gv.setInt32(8, Math.round((lat0 + rnd(2e-6)) * 1e7), true);
      gv.setInt32(12, Math.round((lon0 + rnd(2e-6)) * 1e7), true);
      gv.setInt32(16, Math.round((120 + alt) * 1000), true);
      gv.setInt32(20, Math.round(rnd(300)), true);
      gv.setInt32(24, Math.round(rnd(300)), true);
      gv.setInt32(28, Math.round(-vel * 1000), true);
      gv.setUint32(32, 1800, true);
      gv.setUint32(36, 2500, true);
      gv.setUint32(40, 300, true);
      emit(encodeFrame(PKT.GPS, new Uint8Array(gb)));
    }
    if (Math.round(t / dt) % 40 === 0) {
      // plain console text between frames, as the firmware's printf output arrives
      emit(new TextEncoder().encode(`alt ${alt.toFixed(1)} m  vD ${(-vel).toFixed(1)} m/s  | ${phase}\n`));
    }
    if (phase === "landed" && t > 200) clearInterval(timer);
  }, dt * 1000);

  return () => clearInterval(timer);
}
