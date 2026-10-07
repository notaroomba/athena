import type { AthenaState, GpsFix, Quat, Telemetry } from "./types";

// Athena link frame (mirror of firmware/Athena/athena_link.h):
//   [0xA5][type u8][len u8][payload len bytes][crc16 lo][crc16 hi]
//   crc = CRC-16/CCITT-FALSE over type, len, payload

export const SOF = 0xa5;
export const MAX_PAYLOAD = 200;
export const PKT = { GPS: 0x01, STATE: 0x02, TELEM: 0x03, TEXT: 0x7f } as const;

export const STATE_FLAG = {
  IN_FLIGHT: 1 << 0,
  GPS_FRESH: 1 << 1,
  BARO_OK: 1 << 2,
  MAG_OK: 1 << 3,
  ORIGIN_OK: 1 << 4,
} as const;

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
