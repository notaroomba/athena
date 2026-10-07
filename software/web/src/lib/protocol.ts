import type { AthenaState, GpsFix, Quat, SpuStatus, Telemetry } from "./types";

// Athena link frame (mirror of firmware/Athena/athena_link.h):
//   [0xA5][type u8][len u8][payload len bytes][crc16 lo][crc16 hi]
//   crc = CRC-16/CCITT-FALSE over type, len, payload

export const SOF = 0xa5;
export const MAX_PAYLOAD = 200;
export const PKT = { GPS: 0x01, STATE: 0x02, TELEM: 0x03, SPU: 0x04, CMD: 0x05, TEXT: 0x7f } as const;

export const STATE_FLAG = {
  IN_FLIGHT: 1 << 0,
  GPS_FRESH: 1 << 1,
  BARO_OK: 1 << 2,
  MAG_OK: 1 << 3,
  ORIGIN_OK: 1 << 4,
} as const;

export const SPU_PHASES = ["pad", "boost", "coast", "apogee", "descent", "landed"] as const;
export const SPU_FLAG = {
  ARMED: 1 << 0,
  MPU_LINK: 1 << 1,
  CHRG_OK: 1 << 2,
  PROCHOT: 1 << 3,
  CMPOUT: 1 << 4,
  PD_APP: 1 << 5,
  BQ_OK: 1 << 6,
} as const;
export const PD_MODES = ["none", "PTCH", "APP", "BOOT"] as const;
export const CMD = { PING: 1, ARM: 2, DISARM: 3, FIRE: 4, SERVO: 5, RESET_MPU: 6, SET_MAIN_ALT: 7 } as const;
/** "ARM!" - ARM and FIRE are refused by the SPU without it. */
export const CMD_KEY = 0x41524d21;

export function crc16(bytes: Uint8Array, off = 0, len = bytes.length - off): number {
  let crc = 0xffff;
  for (let i = 0; i < len; i++) {
    crc ^= bytes[off + i] << 8;
    for (let b = 0; b < 8; b++)
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
  }
  return crc;
}

export function encodeFrame(type: number, payload: Uint8Array): Uint8Array {
  const out = new Uint8Array(payload.length + 5);
  out[0] = SOF;
  out[1] = type;
  out[2] = payload.length;
  out.set(payload, 3);
  const c = crc16(out, 1, payload.length + 2);
  out[3 + payload.length] = c & 0xff;
  out[4 + payload.length] = c >> 8;
  return out;
}

/**
 * Stateful frame decoder. Bytes outside a frame are treated as console text
 * (ASCII never contains 0xA5), so a mixed debug/binary stream decodes cleanly.
 * Any length or CRC error drops back to hunting for the next SOF (resync).
 */
export class Decoder {
  ok = 0;
  bad = 0;
  private st = 0;
  private type = 0;
  private len = 0;
  private idx = 0;
  private crc = 0;
  private buf = new Uint8Array(MAX_PAYLOAD);
  private text = "";

  constructor(
    private onFrame: (type: number, payload: Uint8Array) => void,
    private onText: (line: string) => void,
  ) {}

  feed(bytes: Uint8Array) {
    for (let i = 0; i < bytes.length; i++) this.feedByte(bytes[i]);
  }

  feedByte(b: number) {
    switch (this.st) {
      case 0:
        if (b === SOF) this.st = 1;
        else if (b === 10) {
          this.onText(this.text);
          this.text = "";
        } else if (b >= 32 && b < 127) {
          this.text += String.fromCharCode(b);
          if (this.text.length > 300) {
            this.onText(this.text);
            this.text = "";
          }
        }
        break;
      case 1:
        this.type = b;
        this.st = 2;
        break;
      case 2:
        if (b > MAX_PAYLOAD) {
          this.st = 0;
          this.bad++;
          break;
        }
        this.len = b;
        this.idx = 0;
        this.st = b ? 3 : 4;
        break;
      case 3:
        this.buf[this.idx++] = b;
        if (this.idx >= this.len) this.st = 4;
        break;
      case 4:
        this.crc = b;
        this.st = 5;
        break;
      case 5: {
        this.crc |= b << 8;
        const tmp = new Uint8Array(this.len + 2);
        tmp[0] = this.type;
        tmp[1] = this.len;
        tmp.set(this.buf.subarray(0, this.len), 2);
        if (crc16(tmp) === this.crc) {
          this.ok++;
          this.onFrame(this.type, this.buf.slice(0, this.len));
        } else this.bad++;
        this.st = 0;
        break;
      }
    }
  }
}

const view = (p: Uint8Array) => new DataView(p.buffer, p.byteOffset, p.byteLength);

/** Athena_State, 96 bytes. */
export function parseState(p: Uint8Array): AthenaState | null {
  if (p.length < 96) return null;
  const v = view(p);
  const f = (o: number) => v.getFloat32(o, true);
  return {
    t_us: v.getUint32(0, true),
    q: [f(4), f(8), f(12), f(16)],
    pos: [f(20), f(24), f(28)],
    vel: [f(32), f(36), f(40)],
    acc: [f(44), f(48), f(52)],
    gyro: [f(56), f(60), f(64)],
    mag: [f(68), f(72), f(76)],
    baro_alt: f(80),
    origin_lat: v.getInt32(84, true) * 1e-7,
    origin_lon: v.getInt32(88, true) * 1e-7,
    imu_mask: p[92],
    flags: p[93],
    loop_hz: v.getUint16(94, true),
  };
}

/** Athena_GpsFix, 44 bytes. */
export function parseGps(p: Uint8Array): GpsFix | null {
  if (p.length < 44) return null;
  const v = view(p);
  const i = (o: number) => v.getInt32(o, true);
  const u = (o: number) => v.getUint32(o, true);
  return {
    itow: u(0),
    fix: p[4],
    sv: p[5],
    ok: p[6] & 1,
    lat: i(8) * 1e-7,
    lon: i(12) * 1e-7,
    hmsl: i(16) * 1e-3,
    vel: [i(20) * 1e-3, i(24) * 1e-3, i(28) * 1e-3],
    hacc: u(32) * 1e-3,
    vacc: u(36) * 1e-3,
    sacc: u(40) * 1e-3,
  };
}

/** Athena_Telemetry, 38 bytes. */
export function parseTelem(p: Uint8Array): Telemetry | null {
  if (p.length < 38) return null;
  const v = view(p);
  const s = (o: number) => v.getInt16(o, true);
  return {
    t_ms: v.getUint32(0, true),
    lat: v.getInt32(4, true) * 1e-7,
    lon: v.getInt32(8, true) * 1e-7,
    alt: v.getFloat32(12, true),
    baro_alt: v.getFloat32(16, true),
    vel: [s(20) / 10, s(22) / 10, s(24) / 10],
    q: [s(26) / 32767, s(28) / 32767, s(30) / 32767, s(32) / 32767],
    fix: p[34],
    sv: p[35],
    flags: p[36],
    imu_mask: p[37],
  };
}

export const G0 = 9.80665;
export const R2D = 57.29578;
export const FIX_NAMES = ["none", "DR", "2D", "3D", "3D+DR", "time"];

/** Quaternion (w,x,y,z) -> [roll, pitch, yaw] in degrees. */
export function quatToEuler(q: Quat): [number, number, number] {
  const [w, x, y, z] = q;
  const s = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  return [
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * R2D,
    Math.asin(s) * R2D,
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * R2D,
  ];
}

/** Athena_SpuStatus, 44 bytes. */
export function parseSpu(p: Uint8Array): SpuStatus | null {
  if (p.length < 44) return null;
  const v = view(p);
  const u16 = (o: number) => v.getUint16(o, true);
  return {
    t_ms: v.getUint32(0, true),
    vbat_mv: u16(4),
    vsys_mv: u16(6),
    vbus_mv: u16(8),
    ibat_ma: v.getInt16(10, true),
    iin_ma: u16(12),
    chg_status: u16(14),
    main_alt_m: u16(16),
    phase: p[18],
    flags: p[19],
    pyro_fired: p[20],
    pyro_on: p[21],
    pd_mode: p[22],
    pd_status: p[23],
    servo_us: [u16(24), u16(26), u16(28), u16(30), u16(32), u16(34)],
    apogee_m: v.getFloat32(36, true),
    vmax_ms: v.getFloat32(40, true),
  };
}

/** Athena_Cmd (8 bytes) wrapped in a CMD frame, ready for the serial/Bluetooth link. */
export function encodeCmd(cmd: number, arg = 0, value = 0, key = 0): Uint8Array {
  const b = new Uint8Array(8);
  const v = view(b);
  b[0] = cmd;
  b[1] = arg;
  v.setUint16(2, value, true);
  v.setUint32(4, key >>> 0, true);
  return encodeFrame(PKT.CMD, b);
}

const EARTH_R = 6371000;

/** Local NED offset (m) from an origin to lat/lon, flat-earth (fine for a few km). */
export function nedToLatLon(lat0: number, lon0: number, north: number, east: number): [number, number] {
  const lat = lat0 + (north / EARTH_R) * R2D;
  const lon = lon0 + (east / (EARTH_R * Math.cos(lat0 / R2D))) * R2D;
  return [lat, lon];
}

/** Great-circle distance (m) and initial bearing (deg) from a to b. */
export function distanceBearing(lat1: number, lon1: number, lat2: number, lon2: number): [number, number] {
  const p1 = lat1 / R2D,
    p2 = lat2 / R2D,
    dl = (lon2 - lon1) / R2D;
  const a = Math.sin((p2 - p1) / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  const d = 2 * EARTH_R * Math.asin(Math.sqrt(a));
  const y = Math.sin(dl) * Math.cos(p2);
  const x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
  return [d, (Math.atan2(y, x) * R2D + 360) % 360];
}
