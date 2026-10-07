import { encodeFrame, G0, PKT, nedToLatLon } from "./protocol";
import type { NoseAxis, Quat } from "./types";

// Scripted flight pushed through the real frame encoder, so the dashboard's decoder/parsers/charts/map
// see exactly what the firmware would send: STATE at 20 Hz, GPS at 5 Hz (with a dropout to show dead
// reckoning), SPU status at 2 Hz (phases, arming, pyro events, battery) and console text.

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
const PHASE: Record<string, number> = { pad: 0, boost: 1, coast: 2, apogee: 3, descent: 4, landed: 5 };

/** Starts the demo; returns a stop function. `emit` receives raw link bytes. */
export function startDemo(nose: NoseAxis, emit: (bytes: Uint8Array) => void): () => void {
  let t = 0,
    alt = 0,
    vel = 0,
    n = 0,
    e = 0,
    vn = 0,
    ve = 0,
    phaseName = "",
    apogee = 0,
    vmax = 0,
    fired = 0;
  const dt = 0.05,
    lat0 = 40.868, // Black Rock Desert playa, so the demo map shows land instead of New York bay
    lon0 = -119.075;
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
    if (phase === "descent") a = (-(fired & 2 ? 5 : 12) - vel) * 2; // drogue -12 m/s, main -5 m/s
    if (phase === "pad") {
      a = 0;
      vel = 0;
      alt = 0;
    }
    vel += a * dt;
    alt += vel * dt;
    // horizontal: a 3 deg tilt during boost drifts north-east, wind pushes east on the way down
    const an = phase === "boost" ? 0.4 * G0 * 0.05 : 0,
      ae = phase === "boost" ? 0.3 * G0 * 0.05 : 0;
    const windE = alt > 0 && phase !== "boost" ? 4 : 0;
    vn += an * dt - (phase === "descent" ? vn * 0.5 * dt : 0);
    ve += ae * dt + (phase === "descent" ? (windE - ve) * 0.5 * dt : 0);
    if (alt <= 0 && phase !== "pad") {
      vn = ve = 0;
    }
    n += vn * dt;
    e += ve * dt;
    if (alt < 0) {
      alt = 0;
      vel = 0;
      if (t > 10) phase = "landed";
    }
    apogee = Math.max(apogee, alt);
    vmax = Math.max(vmax, Math.abs(vel));
    if (phase === "descent" && !(fired & 1)) {
      fired |= 1;
      emit(text(`[demo] SPU: drogue fired at apogee ${apogee.toFixed(0)} m`));
    }
    if (phase === "descent" && alt < 150 && !(fired & 2)) {
      fired |= 2;
      emit(text(`[demo] SPU: main fired at ${alt.toFixed(0)} m`));
    }
    if (phase !== phaseName) {
      emit(text(`[demo] ${phase}`));
      phaseName = phase;
    }
    const gpsLost = t > 11 && t < 19; // show dead reckoning: no fixes, filter keeps going on IMU + baro
    const spin = phase === "pad" ? 0 : 0.8;
    const tilt = phase === "pad" ? 0.01 : 0.06 * Math.sin(t * 1.3);
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
    const flags = (phase !== "pad" && phase !== "landed" ? 1 : 0) | (gpsLost ? 0 : 2) | 4 | 8 | 16;

    const b = new ArrayBuffer(96),
      v = new DataView(b);
    const F = (o: number, x: number) => v.setFloat32(o, x, true);
    v.setUint32(0, Math.round(t * 1e6), true);
    q.forEach((x, i) => F(4 + 4 * i, x));
    [n + rnd(0.3), e + rnd(0.3), -alt].forEach((x, i) => F(20 + 4 * i, x));
    [vn, ve, -vel].forEach((x, i) => F(32 + 4 * i, x));
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

    if (Math.round(t / dt) % 4 === 0 && !gpsLost) {
      // 5 Hz GPS
      const [lat, lon] = nedToLatLon(lat0, lon0, n + rnd(1.5), e + rnd(1.5));
      const gb = new ArrayBuffer(44),
        gv = new DataView(gb);
      gv.setUint32(0, Math.round(t * 1000), true);
      gv.setUint8(4, 3);
      gv.setUint8(5, 12);
      gv.setUint8(6, 1);
      gv.setInt32(8, Math.round(lat * 1e7), true);
      gv.setInt32(12, Math.round(lon * 1e7), true);
      gv.setInt32(16, Math.round((120 + alt) * 1000), true);
      gv.setInt32(20, Math.round((vn + rnd(0.3)) * 1000), true);
      gv.setInt32(24, Math.round((ve + rnd(0.3)) * 1000), true);
      gv.setInt32(28, Math.round(-vel * 1000), true);
      gv.setUint32(32, 1800, true);
      gv.setUint32(36, 2500, true);
      gv.setUint32(40, 300, true);
      emit(encodeFrame(PKT.GPS, new Uint8Array(gb)));
    }
    if (Math.round(t / dt) % 10 === 0) {
      // 2 Hz SPU status: armed, phase, pyro events, a 2S pack discharging, USB-PD controller patched
      const sb = new ArrayBuffer(44),
        sv = new DataView(sb);
      sv.setUint32(0, Math.round(t * 1000), true);
      sv.setUint16(4, 8120 + Math.round(rnd(10)), true);
      sv.setUint16(6, 7900, true);
      sv.setUint16(8, 0, true);
      sv.setInt16(10, -420 + Math.round(rnd(30)), true);
      sv.setUint16(12, 0, true);
      sv.setUint16(14, 0, true);
      sv.setUint16(16, 150, true);
      sv.setUint8(18, PHASE[phase] ?? 0);
      sv.setUint8(19, (phase === "landed" && t > 200 ? 0 : 1) | 2 | 32 | 64);
      sv.setUint8(20, fired);
      sv.setUint8(21, 0);
      sv.setUint8(22, 2);
      sv.setUint8(23, 0);
      [1500, 1500, 0, 0, 0, 0].forEach((us, i) => sv.setUint16(24 + 2 * i, us, true));
      sv.setFloat32(36, apogee, true);
      sv.setFloat32(40, vmax, true);
      emit(encodeFrame(PKT.SPU, new Uint8Array(sb)));
    }
    if (Math.round(t / dt) % 40 === 0) {
      // plain console text between frames, as the firmware's printf output arrives
      emit(new TextEncoder().encode(`alt ${alt.toFixed(1)} m  vD ${(-vel).toFixed(1)} m/s  | ${phase}${gpsLost ? " | gps DR" : ""}\n`));
    }
    if (phase === "landed" && t > 200) clearInterval(timer);
  }, dt * 1000);

  return () => clearInterval(timer);
}
