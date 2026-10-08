// Replays a flight log (ATHnnnnn.BIN from the SD card, or a `athlog.py dump` of the flash ring) through
// the same decoder the live links use, paced by the timestamps inside the frames.
import { MAX_PAYLOAD, PKT, SOF, crc16 } from "./protocol";

export interface ReplayHandle {
  stop: () => void;
  setSpeed: (x: number) => void;
}

/** Timestamp (seconds) carried by a frame, or null for frames without one. */
function frameTime(type: number, p: Uint8Array): number | null {
  const v = new DataView(p.buffer, p.byteOffset, p.byteLength);
  if (type === PKT.STATE && p.length >= 4) return v.getUint32(0, true) / 1e6;
  if ((type === PKT.TELEM || type === PKT.SPU) && p.length >= 4) return v.getUint32(0, true) / 1e3;
  return null;
}

export function startReplay(
  data: Uint8Array,
  feed: (bytes: Uint8Array) => void,
  onProgress: (fraction: number, done: boolean) => void,
  speed = 1,
): ReplayHandle {
  let stopped = false;
  let rate = speed;
  const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

  (async () => {
    let i = 0;
    let logT0: number | null = null; // first timestamp seen in the log
    let wallT0 = performance.now();
    let lastClock: number | null = null;
    let paceType: number | null = null; // MPU, TPU and SPU clocks are unrelated: pace on the first frame type seen only
    let lastBucket = -1;
    while (i < data.length && !stopped) {
      // frame at i?  [A5][type][len][payload][crc lo][crc hi]
      let end = i + 1;
      if (data[i] === SOF && i + 2 < data.length) {
        const type = data[i + 1],
          len = data[i + 2];
        const fe = i + 3 + len + 2;
        if (len <= MAX_PAYLOAD && fe <= data.length) {
          const crc = data[fe - 2] | (data[fe - 1] << 8);
          if (crc16(data, i + 1, len + 2) === crc) {
            end = fe;
            const t = frameTime(type, data.subarray(i + 3, i + 3 + len));
            if (t !== null && (paceType ??= type) === type) {
              if (logT0 === null || lastClock === null || t < lastClock - 1 || t - lastClock > 3600) {
                logT0 = t; // start, or the MCU rebooted inside the log: rebase
                wallT0 = performance.now();
              }
              lastClock = t;
              const due = wallT0 + ((t - logT0) * 1000) / rate;
              const ahead = due - performance.now();
              if (ahead > 5) await sleep(Math.min(ahead, 500));
            }
          }
        }
      }
      feed(data.subarray(i, end));
      i = end;
      const bucket = i >> 14;
      if (bucket !== lastBucket) {
        lastBucket = bucket;
        onProgress(i / data.length, false);
      }
    }
    if (!stopped) onProgress(1, true);
  })();

  return {
    stop: () => {
      stopped = true;
    },
    setSpeed: (x) => {
      rate = x;
    },
  };
}
