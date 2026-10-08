import { useState, useEffect, useRef, useCallback, lazy, Suspense } from "react";
import AdminPanel from "./AdminPanel";
import DataCharts, { WINDOW_S } from "./DataCharts";
import RecoveryPanel from "./RecoveryPanel";
import ChecklistPanel, { type AutoCheck } from "./ChecklistPanel";
import EventsPanel, { type FlightSummary } from "./EventsPanel";
import { AxisValue, Flag, Metric, fixed } from "./SensorCard";
import type { MapStat } from "./MapPanel";
import {
  NOSE_AXES,
  type AthenaState,
  type FlightEvent,
  type GpsFix,
  type LinkStats,
  type NoseAxis,
  type Quat,
  type Sample,
  type SpuStatus,
  type Telemetry,
  type TrackPoint,
  type WSMessage,
  type WSStationMessage,
} from "@/lib/types";
import { Decoder, FIX_NAMES, G0, PKT, R2D, SPU_FLAG, SPU_PHASES, STATE_FLAG, distanceBearing, encodeCmd, nedToLatLon, parseGps, parseSpu, parseState, parseTelem, quatToEuler } from "@/lib/protocol";
import { connectSerial, disconnectSerial, isSerialSupported, writeSerial } from "@/lib/serial";
import { connectBluetooth, disconnectBluetooth, isBluetoothSupported, writeBluetooth } from "@/lib/bluetooth";
import { startReplay, type ReplayHandle } from "@/lib/replay";
import { startDemo } from "@/lib/demo";

const BoardVisualizer = lazy(() => import("./BoardVisualizer"));
const MapPanel = lazy(() => import("./MapPanel"));

const WS_URL: string =
  (typeof window !== "undefined" && new URLSearchParams(window.location.search).get("ws")) || // local ground station: ?ws=ws://localhost:3001/ws
  import.meta.env.VITE_WS_URL ||
  "wss://api.athena.notaroomba.dev/ws";
const MAX_LINES = 200;
const MAX_TRACK = 6000;
const MAX_EVENTS = 200;
const EVENT_COLOR = { phase: "#f3dfb0", pyro: "#f2552a", arm: "#f29a5a", launch: "#1f9aa8" } as const;
const X = "#ea5a2c",
  Y = "#1f9aa8",
  Z = "#3b5fd0";

function Panel({ children, className = "" }: { children: React.ReactNode; className?: string }) {
  return <div className={`panel ${className}`}>{children}</div>;
}

function Title({ children, sub }: { children: React.ReactNode; sub?: string }) {
  return (
    <h2 className="panel-title mb-3">
      {children}
      {sub && <small>{sub}</small>}
    </h2>
  );
}

function KV({ k, v }: { k: string; v: string }) {
  return (
    <>
      <dt className="text-ink-2">{k}</dt>
      <dd className="text-right font-mono tabular-nums">{v}</dd>
    </>
  );
}

function loadNose(): NoseAxis {
  try {
    const n = localStorage.getItem("athena.nose");
    if (n && (NOSE_AXES as string[]).includes(n)) return n as NoseAxis;
  } catch {
    /* ignore */
  }
  return "+Z";
}

export default function Dashboard() {
  const [isAdmin, setIsAdmin] = useState(false);
  const [serialConnected, setSerialConnected] = useState(false);
  const [portLabel, setPortLabel] = useState("");
  const [bleConnected, setBleConnected] = useState(false);
  const [bleLabel, setBleLabel] = useState("");
  const [wsConnected, setWsConnected] = useState(false);
  const [viewers, setViewers] = useState(0);
  const [adminOnline, setAdminOnline] = useState(false);
  const [showAdmin, setShowAdmin] = useState(false);
  const [showChecklist, setShowChecklist] = useState(false);
  const [station, setStation] = useState<WSStationMessage | null>(null);
  const [stationAt, setStationAt] = useState(0);
  const [demoMode, setDemoMode] = useState(false);
  const [replay, setReplay] = useState<{ name: string; progress: number; done: boolean } | null>(null);
  const [nose, setNose] = useState<NoseAxis>(loadNose);

  const [state, setState] = useState<AthenaState | null>(null);
  const [gps, setGps] = useState<GpsFix | null>(null);
  const [telem, setTelem] = useState<Telemetry | null>(null);
  const [spu, setSpu] = useState<SpuStatus | null>(null);
  const [spuAt, setSpuAt] = useState(0);
  const [history, setHistory] = useState<Sample[]>([]);
  const [fusedTrack, setFusedTrack] = useState<TrackPoint[]>([]);
  const [gpsTrack, setGpsTrack] = useState<TrackPoint[]>([]);
  const [apogee, setApogee] = useState(0);
  const [vmax, setVmax] = useState(0);
  const [gmax, setGmax] = useState(0);
  const [events, setEvents] = useState<FlightEvent[]>([]);
  const [rates, setRates] = useState({ state: 0, gps: 0, telem: 0, spu: 0, bytes: 0 });
  const [recording, setRecording] = useState(false);
  const [recordedBytes, setRecordedBytes] = useState(0);
  const [replaySpeed, setReplaySpeed] = useState(1);
  const [replayPaused, setReplayPaused] = useState(false);
  const [sound, setSound] = useState(() => {
    try {
      return localStorage.getItem("athena.sound") === "1";
    } catch {
      return false;
    }
  });
  const [lastFrameAt, setLastFrameAt] = useState(0);
  const [tMinus, setTMinus] = useState<number | null>(null); // launch countdown, seconds; null = off
  const [units, setUnits] = useState<"m" | "ft">(() => {
    try {
      return localStorage.getItem("athena.units") === "ft" ? "ft" : "m";
    } catch {
      return "m";
    }
  });
  useEffect(() => {
    try {
      localStorage.setItem("athena.units", units);
    } catch {
      /* ignore */
    }
  }, [units]);
  // display conversion only: the link, the SPU's main altitude and the logs stay in metres
  const ft = units === "ft";
  const L = (m: number) => (ft ? m * 3.28084 : m); // length
  const UL = ft ? "ft" : "m",
    US = ft ? "ft/s" : "m/s";
  const soundRef = useRef(false);
  useEffect(() => {
    soundRef.current = sound;
    try {
      localStorage.setItem("athena.sound", sound ? "1" : "0");
    } catch {
      /* ignore */
    }
  }, [sound]);
  const [lines, setLines] = useState<string[]>([]);
  const [link, setLink] = useState<LinkStats>({ ok: 0, bad: 0 });

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectRef = useRef<ReturnType<typeof setTimeout>>(undefined);
  const savedPasswordRef = useRef<string | null>(null);
  const isAdminRef = useRef(false);
  const localLinkRef = useRef(false); // a serial/Bluetooth board is open here: relayed bytes would duplicate it
  const decoderRef = useRef<Decoder | null>(null);
  const stateRef = useRef<AthenaState | null>(null); // mirror of `state` for the frame handler (no setState-updater side effects)
  const consoleRef = useRef<HTMLDivElement>(null);
  const replayRef = useRef<ReplayHandle | null>(null);
  const fileRef = useRef<HTMLInputElement>(null);
  const lastFusedRef = useRef<{ t: number; n: number; e: number } | null>(null);
  const lastGpsRef = useRef(0);
  const prevSpuRef = useRef<SpuStatus | null>(null);
  const prevFlightRef = useRef(false);
  const launchWallRef = useRef(0);
  const landedWallRef = useRef(0);
  const firstFrameWallRef = useRef(0);
  const countsRef = useRef({ state: 0, gps: 0, telem: 0, spu: 0, bytes: 0 });
  const lastCountsRef = useRef({ state: 0, gps: 0, telem: 0, spu: 0, bytes: 0 });
  const recordRef = useRef<Uint8Array[] | null>(null);
  const recordLenRef = useRef(0);

  useEffect(() => {
    isAdminRef.current = isAdmin;
  }, [isAdmin]);
  useEffect(() => {
    localLinkRef.current = serialConnected || bleConnected;
  }, [serialConnected, bleConnected]);

  // 1 s tick: freshness checks (SPU live/stale) re-evaluate without new data, and per-type frame rates
  const [, setTick] = useState(0);
  useEffect(() => {
    const id = setInterval(() => {
      setTick((n) => n + 1);
      const c = countsRef.current,
        l = lastCountsRef.current;
      setRates({ state: c.state - l.state, gps: c.gps - l.gps, telem: c.telem - l.telem, spu: c.spu - l.spu, bytes: c.bytes - l.bytes });
      lastCountsRef.current = { ...c };
    }, 1000);
    return () => clearInterval(id);
  }, []);

  const beep = useCallback((freq: number, ms: number, times = 1) => {
    if (!soundRef.current) return;
    try {
      const Ctx = window.AudioContext || (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
      const ctx = new Ctx();
      for (let i = 0; i < times; i++) {
        const o = ctx.createOscillator();
        const g = ctx.createGain();
        o.frequency.value = freq;
        o.type = "square";
        g.gain.value = 0.08;
        o.connect(g).connect(ctx.destination);
        const t0 = ctx.currentTime + i * (ms / 1000 + 0.08);
        o.start(t0);
        o.stop(t0 + ms / 1000);
      }
      setTimeout(() => void ctx.close(), times * (ms + 120) + 200);
    } catch {
      /* no audio */
    }
  }, []);

  useEffect(() => {
    if (tMinus === null) return;
    if (tMinus <= 0) {
      beep(1760, 600);
      const t = setTimeout(() => setTMinus(null), 3000);
      return () => clearTimeout(t);
    }
    if (tMinus <= 10) beep(tMinus <= 3 ? 1320 : 880, 90);
    const t = setTimeout(() => setTMinus((x) => (x === null ? null : x - 1)), 1000);
    return () => clearTimeout(t);
  }, [tMinus, beep]);

  const pushEvent = useCallback((label: string, detail: string, color: string) => {
    const now = Date.now();
    if (label === "LAUNCH") beep(880, 120, 2);
    else if (label === "APOGEE") beep(1320, 180);
    else if (label.startsWith("PYRO")) beep(660, 250, 3);
    else if (label === "LANDED") beep(440, 400);
    else if (label === "ARMED") beep(1760, 80, 2);
    if (!firstFrameWallRef.current) firstFrameWallRef.current = now;
    const ev: FlightEvent = { when: new Date(now).toTimeString().slice(0, 8), t: (now - firstFrameWallRef.current) / 1000, label, detail, color };
    setEvents((prev) => {
      const next = [...prev, ev];
      return next.length > MAX_EVENTS ? next.slice(-MAX_EVENTS) : next;
    });
  }, []);

  useEffect(() => {
    try {
      localStorage.setItem("athena.nose", nose);
    } catch {
      /* ignore */
    }
  }, [nose]);

  const logLine = useCallback((s: string) => {
    if (!s) return;
    const line = `${new Date().toTimeString().slice(0, 8)}  ${s}`;
    setLines((prev) => {
      const next = [...prev, line];
      return next.length > MAX_LINES ? next.slice(-MAX_LINES) : next;
    });
  }, []);

  const pushSample = useCallback((s: Sample) => {
    setHistory((prev) => {
      if (prev.length && s.t < prev[prev.length - 1].t) return [s]; // MCU clock restarted: start the window over
      const next = [...prev, s];
      let i = 0;
      while (i < next.length && s.t - next[i].t > WINDOW_S) i++;
      return i ? next.slice(i) : next;
    });
  }, []);

  const pushTrack = useCallback((setter: typeof setFusedTrack, p: TrackPoint) => {
    setter((prev) => {
      const next = [...prev, p];
      return next.length > MAX_TRACK ? next.slice(-MAX_TRACK) : next;
    });
  }, [beep]);

  const onFrame = useCallback(
    (type: number, p: Uint8Array) => {
      if (!firstFrameWallRef.current) firstFrameWallRef.current = Date.now();
      setLastFrameAt(Date.now());
      if (type === PKT.STATE) {
        const s = parseState(p);
        if (!s) return;
        countsRef.current.state++;
        stateRef.current = s;
        setState(s);
        const alt = -s.pos[2];
        setApogee((a) => (alt > a ? alt : a));
        setVmax((v) => (Math.abs(s.vel[2]) > v ? Math.abs(s.vel[2]) : v));
        const g = Math.hypot(s.acc[0], s.acc[1], s.acc[2]) / G0;
        setGmax((m) => (g > m ? g : m));
        const inFlight = !!(s.flags & STATE_FLAG.IN_FLIGHT);
        if (inFlight && !prevFlightRef.current) {
          launchWallRef.current = Date.now();
          landedWallRef.current = 0;
          setTMinus(null);
          pushEvent("LAUNCH", "MPU launch detector", EVENT_COLOR.launch);
        }
        prevFlightRef.current = inFlight;
        pushSample({
          t: s.t_us / 1e6,
          acc: [s.acc[0] / G0, s.acc[1] / G0, s.acc[2] / G0],
          gyro: [s.gyro[0] * R2D, s.gyro[1] * R2D, s.gyro[2] * R2D],
          alt,
          baro: s.baro_alt,
        });
        // ground track from the filter: NED offset from the pad mapped back to lat/lon, thinned to ~4 Hz or 1 m
        if (s.flags & STATE_FLAG.ORIGIN_OK) {
          const t = s.t_us / 1e6;
          const last = lastFusedRef.current;
          if (!last || t < last.t || t - last.t > 0.25 || Math.hypot(s.pos[0] - last.n, s.pos[1] - last.e) > 1) {
            lastFusedRef.current = { t, n: s.pos[0], e: s.pos[1] };
            const [lat, lon] = nedToLatLon(s.origin_lat, s.origin_lon, s.pos[0], s.pos[1]);
            pushTrack(setFusedTrack, { lat, lon, alt, t, dr: !(s.flags & STATE_FLAG.GPS_FRESH) });
          }
        }
      } else if (type === PKT.GPS) {
        const g = parseGps(p);
        if (!g) return;
        countsRef.current.gps++;
        setGps(g);
        if (g.fix >= 2 && g.ok && g.itow !== lastGpsRef.current) {
          lastGpsRef.current = g.itow;
          pushTrack(setGpsTrack, { lat: g.lat, lon: g.lon, alt: g.hmsl, t: g.itow / 1e3, dr: false });
        }
      } else if (type === PKT.TELEM) {
        const t = parseTelem(p);
        if (!t) return;
        countsRef.current.telem++;
        setTelem(t);
        // TPU port or radio only: the compact frame carries the fused position, build what we can from it
        if (!stateRef.current) {
          pushSample({ t: t.t_ms / 1e3, alt: t.alt, baro: t.baro_alt });
          setApogee((a) => (t.alt > a ? t.alt : a));
          if (t.lat || t.lon) pushTrack(setFusedTrack, { lat: t.lat, lon: t.lon, alt: t.alt, t: t.t_ms / 1e3, dr: !(t.flags & STATE_FLAG.GPS_FRESH) });
        }
      } else if (type === PKT.SPU) {
        const s = parseSpu(p);
        if (!s) return;
        countsRef.current.spu++;
        setSpu(s);
        setSpuAt(Date.now());
        // events: phase changes, pyro firings, arming
        const prev = prevSpuRef.current;
        prevSpuRef.current = s;
        if (prev) {
          if (s.phase !== prev.phase) {
            // the SPU's APOGEE phase lasts one 50 ms frame, so a 2 Hz status frame usually jumps COAST -> DESCENT
            if (prev.phase === 2 && s.phase === 4) pushEvent("APOGEE", `${s.apogee_m.toFixed(0)} m`, EVENT_COLOR.phase);
            pushEvent(SPU_PHASES[s.phase]?.toUpperCase() ?? `PHASE ${s.phase}`, s.phase >= 2 ? `apogee so far ${s.apogee_m.toFixed(0)} m, max ${s.vmax_ms.toFixed(0)} m/s` : "", EVENT_COLOR.phase);
            if (s.phase === 5) landedWallRef.current = Date.now();
          }
          const newlyFired = s.pyro_fired & ~prev.pyro_fired;
          for (let ch = 0; ch < 6; ch++)
            if (newlyFired & (1 << ch)) pushEvent(`PYRO ${ch + 1} FIRED`, ch === 0 ? "drogue" : ch === 1 ? "main" : "manual", EVENT_COLOR.pyro);
          if ((s.flags & SPU_FLAG.ARMED) !== (prev.flags & SPU_FLAG.ARMED)) pushEvent(s.flags & SPU_FLAG.ARMED ? "ARMED" : "DISARMED", "", EVENT_COLOR.arm);
        }
      } else if (type === PKT.TEXT) {
        logLine(new TextDecoder().decode(p));
      }
    },
    [pushSample, pushTrack, logLine, pushEvent],
  );

  const resetData = useCallback(() => {
    stateRef.current = null;
    setState(null);
    setLastFrameAt(0);
    setGps(null);
    setTelem(null);
    setSpu(null);
    setHistory([]);
    setFusedTrack([]);
    setGpsTrack([]);
    setApogee(0);
    setVmax(0);
    setGmax(0);
    setEvents([]);
    lastFusedRef.current = null;
    lastGpsRef.current = 0;
    prevSpuRef.current = null;
    prevFlightRef.current = false;
    launchWallRef.current = landedWallRef.current = firstFrameWallRef.current = 0;
    setLink({ ok: 0, bad: 0 });
    decoderRef.current = new Decoder(onFrame, logLine);
  }, [onFrame, logLine]);

  useEffect(() => {
    if (!decoderRef.current) decoderRef.current = new Decoder(onFrame, logLine);
  }, [onFrame, logLine]);

  /** Raw link bytes from any source (serial, Bluetooth, replay, demo, or relayed by the server). */
  const feed = useCallback((bytes: Uint8Array) => {
    const d = decoderRef.current;
    if (!d) return;
    countsRef.current.bytes += bytes.length;
    if (recordRef.current) {
      recordRef.current.push(bytes.slice());
      recordLenRef.current += bytes.length;
      setRecordedBytes(recordLenRef.current);
    }
    d.feed(bytes);
    setLink({ ok: d.ok, bad: d.bad });
  }, []);

  // ---- recording: everything fed to the decoder, in the same raw format as the SD/flash logs (replayable)
  const toggleRecording = useCallback(() => {
    if (recordRef.current) {
      const parts = recordRef.current;
      recordRef.current = null;
      setRecording(false);
      const blob = new Blob(parts as BlobPart[], { type: "application/octet-stream" });
      const a = document.createElement("a");
      const stamp = new Date().toISOString().replace(/[-:]/g, "").slice(0, 15);
      a.href = URL.createObjectURL(blob);
      a.download = `athena-${stamp}.bin`;
      a.click();
      setTimeout(() => URL.revokeObjectURL(a.href), 10000);
      logLine(`[dashboard] saved ${a.download} (${(recordLenRef.current / 1024).toFixed(0)} KB)`);
    } else {
      recordRef.current = [];
      recordLenRef.current = 0;
      setRecordedBytes(0);
      setRecording(true);
      logLine("[dashboard] recording raw link bytes");
    }
  }, [logLine]);

  useEffect(() => {
    if (consoleRef.current) consoleRef.current.scrollTop = consoleRef.current.scrollHeight;
  }, [lines]);

  // ---- flight report: everything the dashboard derived, as one JSON file (events, summary, tracks, last frames)
  const saveReport = useCallback(() => {
    const stamp = new Date().toISOString().replace(/[-:]/g, "").slice(0, 15);
    const report = {
      generated: new Date().toISOString(),
      summary: { apogee_m: apogee, vmax_ms: vmax, gmax_g: gmax, flight_time_s: launchWallRef.current ? ((landedWallRef.current || Date.now()) - launchWallRef.current) / 1000 : 0 },
      pad: state && state.flags & STATE_FLAG.ORIGIN_OK ? { lat: state.origin_lat, lon: state.origin_lon } : fusedTrack[0] ? { lat: fusedTrack[0].lat, lon: fusedTrack[0].lon } : null,
      last_position: fusedTrack.length ? fusedTrack[fusedTrack.length - 1] : null,
      events,
      spu,
      gps,
      state,
      telem,
      fused_track: fusedTrack,
      gps_track: gpsTrack,
      link,
      console: lines,
    };
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([JSON.stringify(report, null, 1)], { type: "application/json" }));
    a.download = `athena-report-${stamp}.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 10000);
    logLine(`[dashboard] saved ${a.download}`);
  }, [apogee, vmax, gmax, state, fusedTrack, gpsTrack, events, spu, gps, telem, link, lines, logLine]);

  // ---- demo
  useEffect(() => {
    if (!demoMode) return;
    resetData();
    logLine("[dashboard] demo: simulated flight through the real frame encoder/decoder (GPS drops out 11-19 s)");
    const stop = startDemo(nose, feed);
    return stop;
    // nose is read once at demo start on purpose; changing it mid-demo only re-renders the model
  }, [demoMode, feed, resetData, logLine]); // eslint-disable-line react-hooks/exhaustive-deps

  // ---- replay of a log file
  const stopReplay = useCallback(() => {
    replayRef.current?.stop();
    replayRef.current = null;
    setReplay(null);
    setReplayPaused(false);
  }, []);

  const handleReplayFile = useCallback(
    async (file: File) => {
      stopReplay();
      setDemoMode(false);
      const data = new Uint8Array(await file.arrayBuffer());
      resetData();
      logLine(`[dashboard] replay ${file.name} (${(data.length / 1024).toFixed(0)} KB) at real-time pace`);
      setReplay({ name: file.name, progress: 0, done: false });
      replayRef.current = startReplay(data, feed, (progress, done) => setReplay((r) => (r ? { ...r, progress, done } : r)), replaySpeed);
    },
    [stopReplay, resetData, logLine, feed, replaySpeed],
  );

  const changeReplaySpeed = useCallback((x: number) => {
    setReplaySpeed(x);
    setReplayPaused(false);
    replayRef.current?.setSpeed(x);
  }, []);

  const toggleReplayPause = useCallback(() => {
    setReplayPaused((p) => {
      replayRef.current?.setSpeed(p ? replaySpeed : 0);
      return !p;
    });
  }, [replaySpeed]);

  // ---- websocket
  const connectWebSocket = useCallback(() => {
    const ws = new WebSocket(WS_URL);
    ws.binaryType = "arraybuffer";
    wsRef.current = ws;

    ws.onopen = () => {
      setWsConnected(true);
      if (savedPasswordRef.current) {
        ws.send(JSON.stringify({ type: "auth", password: savedPasswordRef.current }));
      }
    };
    ws.onclose = () => {
      setWsConnected(false);
      reconnectRef.current = setTimeout(connectWebSocket, 2000);
    };
    ws.onerror = () => ws.close();

    ws.onmessage = (event) => {
      if (event.data instanceof ArrayBuffer) {
        // relayed raw link bytes from a ground station or another admin's board; the decoder resyncs on any chunk boundary
        if (!localLinkRef.current) feed(new Uint8Array(event.data));
        return;
      }
      try {
        const msg: WSMessage = JSON.parse(event.data);
        if (msg.type === "status") {
          setViewers(msg.viewers);
          setAdminOnline(msg.adminConnected);
        } else if (msg.type === "auth_result") {
          setIsAdmin(msg.success);
          if (!msg.success) savedPasswordRef.current = null;
        } else if (msg.type === "admin_disconnected") {
          setAdminOnline(false);
        } else if (msg.type === "station") {
          setStation(msg);
          setStationAt(Date.now());
        } else if (msg.type === "cmd_result") {
          logLine(msg.delivered ? `[relay] command handed to ${msg.delivered} ground station${msg.delivered > 1 ? "s" : ""}` : "[relay] NO ground station connected: command dropped");
        } else if (msg.type === "cmd") {
          // another admin asked a ground station to send a command; nothing to do in a browser
        }
      } catch {
        /* ignore */
      }
    };
  }, [feed, logLine]);

  useEffect(() => {
    connectWebSocket();
    return () => {
      clearTimeout(reconnectRef.current);
      wsRef.current?.close();
    };
  }, [connectWebSocket]);

  const authenticate = useCallback((password: string) => {
    savedPasswordRef.current = password;
    wsRef.current?.send(JSON.stringify({ type: "auth", password }));
  }, []);

  /** Bytes from a real board: decode here and, when admin, relay to every viewer. */
  const onBoardChunk = useCallback(
    (bytes: Uint8Array) => {
      feed(bytes);
      if (isAdminRef.current && wsRef.current?.readyState === WebSocket.OPEN) {
        wsRef.current.send(bytes);
      }
    },
    [feed],
  );

  // ---- serial
  const handleConnect = useCallback(async () => {
    setDemoMode(false);
    stopReplay();
    const label = await connectSerial({
      onChunk: onBoardChunk,
      onDisconnect: () => {
        setSerialConnected(false);
        setPortLabel("");
        logLine("[dashboard] port closed");
      },
    });
    resetData();
    setSerialConnected(true);
    setPortLabel(label);
    logLine(`[dashboard] port opened (${label})`);
  }, [onBoardChunk, resetData, logLine, stopReplay]);

  const handleDisconnect = useCallback(() => {
    void disconnectSerial();
  }, []);

  // ---- bluetooth (DA14531 on the TPU)
  const handleBluetooth = useCallback(async () => {
    if (bleConnected) {
      disconnectBluetooth();
      return;
    }
    setDemoMode(false);
    stopReplay();
    const label = await connectBluetooth({
      onChunk: onBoardChunk,
      onInfo: logLine,
      onDisconnect: () => {
        setBleConnected(false);
        setBleLabel("");
        logLine("[dashboard] bluetooth disconnected");
      },
    });
    resetData();
    setBleConnected(true);
    setBleLabel(label);
  }, [bleConnected, onBoardChunk, resetData, logLine, stopReplay]);

  // ---- commands to the SPU. Paths, in order: a local serial/Bluetooth link, the desktop ground station's
  // uplink (window.pywebview bridge), or the relay (logged in: the server hands the frame to a ground station).
  // The SPU enforces arming and the key whatever the path.
  const sendCommand = useCallback(
    async (cmd: number, arg = 0, value = 0, key = 0) => {
      const frame = encodeCmd(cmd, arg, value, key);
      const hex = Array.from(frame, (b) => b.toString(16).padStart(2, "0")).join("");
      let via = "";
      const bridge = (window as unknown as { pywebview?: { api?: { send_command?: (h: string) => Promise<boolean> } } }).pywebview?.api?.send_command;
      if (serialConnected && (await writeSerial(frame))) via = "serial";
      else if (bleConnected && (await writeBluetooth(frame))) via = "bluetooth";
      else if (bridge && (await bridge(hex))) via = "ground station";
      else if (isAdminRef.current && wsRef.current?.readyState === WebSocket.OPEN) {
        wsRef.current.send(JSON.stringify({ type: "cmd", frame: hex }));
        via = "relay";
      }
      logLine(`[dashboard] command ${cmd} ch=${arg} val=${value} ${via ? "sent via " + via : "NOT sent (no link: connect serial/Bluetooth or log in)"}`);
    },
    [serialConnected, bleConnected, logLine],
  );
  const hasBridge = typeof window !== "undefined" && !!(window as unknown as { pywebview?: unknown }).pywebview;

  // ---- keyboard shortcuts
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement | null)?.tagName;
      if (e.metaKey || e.ctrlKey || e.altKey || tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
      if (e.key === "d") setDemoMode((v) => !v);
      else if (e.key === "s") setSound((v) => !v);
      else if (e.key === "c") setShowChecklist((v) => !v);
      else if (e.key === "l") setShowAdmin((v) => !v);
      else if (e.key === "t") setTMinus((v) => (v === null ? 60 : null));
      else if (e.key === "r") toggleRecording();
      else if (e.key === " " && replayRef.current) toggleReplayPause();
      else return;
      e.preventDefault();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [toggleRecording, toggleReplayPause]);

  // ---- derived values
  const s = state;
  const acc = s ? s.acc.map((a) => a / G0) : [0, 0, 0];
  const gyro = s ? s.gyro.map((g) => g * R2D) : [0, 0, 0];
  const alt = s ? -s.pos[2] : telem ? telem.alt : 0;
  const vz = s ? -s.vel[2] : telem ? -telem.vel[2] : 0;
  const baro = s ? s.baro_alt : telem ? telem.baro_alt : 0;
  const q: Quat | null = s ? s.q : telem ? telem.q : null;
  const flags = s ? s.flags : telem ? telem.flags : 0;
  const imuMask = s ? s.imu_mask : telem ? telem.imu_mask : 0;
  const rpy = q ? quatToEuler(q).map((x) => x.toFixed(1)).join(" / ") + " °" : "-";
  const fix = gps
    ? `${FIX_NAMES[gps.fix] ?? gps.fix} · ${gps.sv} sv · ±${gps.hacc.toFixed(1)} m`
    : telem
      ? `${FIX_NAMES[telem.fix] ?? telem.fix} · ${telem.sv} sv`
      : "none";
  const pos = gps ?? telem;
  const isLive = adminOnline || serialConnected || bleConnected || demoMode || !!replay;
  const serialSupported = isSerialSupported();
  const bleSupported = isBluetoothSupported();
  const spuFresh = !!spu && Date.now() - spuAt < 3000;

  // map: current fused position, pad, landing extrapolation, flight stats
  const cur = fusedTrack.length ? fusedTrack[fusedTrack.length - 1] : null;
  const pad: [number, number] | null = s && s.flags & STATE_FLAG.ORIGIN_OK ? [s.origin_lat, s.origin_lon] : fusedTrack.length ? [fusedTrack[0].lat, fusedTrack[0].lon] : null;
  const velNE: [number, number] = s ? [s.vel[0], s.vel[1]] : telem ? [telem.vel[0], telem.vel[1]] : gps ? [gps.vel[0], gps.vel[1]] : [0, 0];
  let landing: [number, number] | null = null;
  let eta = 0;
  if (cur && alt > 5 && vz < -0.5) {
    eta = alt / -vz;
    landing = nedToLatLon(cur.lat, cur.lon, velNE[0] * eta, velNE[1] * eta);
  }
  const predApogee = vz > 0.5 ? alt + (vz * vz) / (2 * G0) : 0;
  const [dist, brg] = cur && pad ? distanceBearing(pad[0], pad[1], cur.lat, cur.lon) : [0, 0];
  const flightTime = launchWallRef.current ? ((landedWallRef.current || Date.now()) - launchWallRef.current) / 1000 : 0;
  const linkAge = lastFrameAt ? (Date.now() - lastFrameAt) / 1000 : 0;
  const linkLost = isLive && !demoMode && !replay && lastFrameAt > 0 && linkAge > 5;
  const stationFresh = !!station && Date.now() - stationAt < 5000;
  const fixNum = gps ? gps.fix : telem ? telem.fix : 0;
  // pre-flight checks read from the live data (the hand-ticked half lives in ChecklistPanel)
  const checks: AutoCheck[] = [
    { label: "flight computer data", ok: !!(state || telem) && linkAge < 5, detail: state ? `${rates.state}/s` : telem ? `${rates.telem}/s` : "none" },
    { label: "3 IMUs", ok: imuMask === 7, detail: `${[0, 1, 2].filter((i) => imuMask & (1 << i)).length}/3` },
    { label: "barometer", ok: !!(flags & STATE_FLAG.BARO_OK) },
    { label: "magnetometer", ok: !!(flags & STATE_FLAG.MAG_OK) },
    { label: "GPS 3D fix", ok: fixNum >= 3 && (!gps || gps.hacc < 10), detail: gps ? `${gps.sv} sv ±${gps.hacc.toFixed(0)} m` : telem ? `${telem.sv} sv` : undefined },
    { label: "pad origin set", ok: !!(flags & STATE_FLAG.ORIGIN_OK) },
    { label: "SPU status live", ok: spuFresh },
    { label: "SPU sees the MPU", ok: !!(spu && spu.flags & SPU_FLAG.MPU_LINK) },
    { label: "on the pad", ok: !!spu && spu.phase === 0, detail: spu ? SPU_PHASES[spu.phase] : undefined },
    { label: "main altitude set", ok: !!spu && spu.main_alt_m > 0, detail: spu ? `${spu.main_alt_m} m` : undefined },
    { label: "radio link", ok: rates.telem > 0 || stationFresh, detail: stationFresh ? `${station!.level_db.toFixed(0)} dB` : rates.telem ? `${rates.telem}/s` : "no TPU frames" },
    { label: "armed", ok: !!(spu && spu.flags & SPU_FLAG.ARMED) },
  ];
  useEffect(() => {
    const phaseName = spu ? SPU_PHASES[spu.phase] : flags & STATE_FLAG.IN_FLIGHT ? "flight" : "";
    document.title = isLive && (state || telem) ? `${fixed(L(alt), 0)} ${UL} ${phaseName ? "· " + phaseName + " " : ""}· Athena` : "Athena Telemetry";
  }, [alt, spu, flags, isLive, state, telem, ft]); // eslint-disable-line react-hooks/exhaustive-deps
  const summary: FlightSummary | null =
    apogee > 0 || events.length
      ? { apogee: L(apogee), vmax: L(vmax), gmax, flightTime, landingDist: spu?.phase === 5 && cur && pad ? L(dist) : 0, lengthUnit: UL, speedUnit: US, phase: spu ? (SPU_PHASES[spu.phase] ?? "?") : flags & STATE_FLAG.IN_FLIGHT ? "flight" : "pad" }
      : null;
  const stats: MapStat[] = [
    { label: "from pad", value: cur && pad ? `${L(dist).toFixed(0)} ${UL} @ ${brg.toFixed(0)}°` : "-" },
    { label: "apogee", value: apogee > 0 ? `${L(apogee).toFixed(0)} ${UL}` : "-" },
    { label: "max speed", value: vmax > 0 ? `${L(vmax).toFixed(0)} ${US}` : "-" },
    { label: predApogee ? "pred. apogee" : "landing in", value: predApogee ? `${L(predApogee).toFixed(0)} ${UL}` : eta ? `${eta.toFixed(0)} s` : "-" },
    { label: "ground speed", value: cur ? `${L(Math.hypot(velNE[0], velNE[1])).toFixed(1)} ${US}` : "-" },
    { label: "position", value: cur ? (cur.dr ? "DEAD RECKONING" : "GPS aided") : "-" },
  ];

  return (
    <div className="flex min-h-screen w-full flex-col gap-2 overflow-y-auto p-2 xl:h-screen xl:overflow-hidden xl:gap-3 xl:p-3">
      {/* 4x3 grid on wide screens (16:9 viewport), 3 columns on tablets, single column on phones */}
      <div className="grid grid-cols-1 gap-2 md:grid-cols-3 xl:min-h-0 xl:flex-1 xl:grid-cols-4 xl:grid-rows-3 xl:gap-3">
        {/* ═══ HERO — center (top on mobile) ═══ */}
        <Panel className="scroll order-0 flex flex-col items-center justify-center-safe p-4 xl:col-start-2 xl:row-start-2 xl:min-h-0">
          <img src="/logo.png" alt="Athena logo" className="h-16 w-16" />
          <div className="wordmark mt-1">ATHENA</div>
          <div className="stripebar mt-2 w-40" />

          <div className="mt-3 flex flex-col items-center gap-1">
            <div
              className={`text-sm font-bold tracking-[.22em] ${
                isLive ? (demoMode ? "text-orange-2" : replay ? "text-cream" : "text-teal") : "text-ink-3"
              }`}
            >
              {isLive ? (demoMode ? "DEMO" : replay ? `REPLAY ${Math.round(replay.progress * 100)}%` : "CONNECTED") : "NO DATA"}
            </div>
            <div className="flex items-center gap-1.5">
              <div className={`h-1.5 w-1.5 rounded-full ${wsConnected ? "pulse-dot bg-teal" : "bg-orange"}`} />
              <span className="text-[10px] text-ink-3">
                {viewers} viewer{viewers !== 1 ? "s" : ""}
                {serialConnected ? ` · ${portLabel}` : bleConnected ? ` · ${bleLabel}` : stationFresh ? " · via ground station" : adminOnline && !demoMode && !replay ? " · via relay" : ""}
              </span>
            </div>
          </div>

          {linkLost && (
            <div className="mt-2 rounded border border-orange bg-[#2a120c] px-3 py-1 text-xs font-semibold tracking-wider text-orange">
              NO DATA FOR {linkAge.toFixed(0)} s
            </div>
          )}
          {tMinus !== null && (
            <div className={`mt-2 font-mono text-3xl font-bold tabular-nums ${tMinus <= 0 ? "text-orange" : tMinus <= 10 ? "text-cream" : "text-teal"}`} title="launch countdown (t to cancel)">
              {tMinus <= 0 ? "LIFTOFF" : `T-${String(Math.floor(tMinus / 60)).padStart(2, "0")}:${String(tMinus % 60).padStart(2, "0")}`}
            </div>
          )}
          <div className="mt-3 flex flex-wrap justify-center gap-2">
            <button onClick={() => setDemoMode(!demoMode)} className={`btn ${demoMode ? "active" : ""}`} title="simulated flight (d)">
              DEMO
            </button>
            <button onClick={() => setTMinus(tMinus === null ? 60 : null)} className={`btn ${tMinus !== null ? "active" : ""}`} title="60 s launch countdown with beeps in the last 10 s; stops itself at launch (t)">
              {tMinus === null ? "T-60" : "ABORT"}
            </button>
            <button
              onClick={() => (replay ? stopReplay() : fileRef.current?.click())}
              className={`btn ${replay ? "active" : ""}`}
              title="replay an ATHnnnnn.BIN from the SD card, a flash dump or a dashboard recording"
            >
              {replay ? "STOP" : "REPLAY"}
            </button>
            {replay && !replay.done && (
              <button onClick={toggleReplayPause} className={`btn ${replayPaused ? "active" : ""}`} title="pause / resume the replay (space)">
                {replayPaused ? "RESUME" : "PAUSE"}
              </button>
            )}
            {replay && (
              <select value={replaySpeed} onChange={(e) => changeReplaySpeed(Number(e.target.value))} className="text-[10px]" title="replay speed">
                {[1, 2, 5, 10, 50].map((x) => (
                  <option key={x} value={x}>
                    {x}×
                  </option>
                ))}
              </select>
            )}
            <input
              ref={fileRef}
              type="file"
              accept=".bin,.BIN,application/octet-stream"
              className="hidden"
              onChange={(e) => {
                const f = e.target.files?.[0];
                if (f) void handleReplayFile(f).catch((err) => logLine(`[dashboard] ${err.message}`));
                e.target.value = "";
              }}
            />
            <button
              onClick={() => void handleBluetooth().catch((e) => logLine(`[dashboard] ${e.message}`))}
              disabled={!bleSupported}
              title={bleSupported ? "connect to the DA14531 on the TPU" : "Web Bluetooth needs Chrome/Edge over https"}
              className={`btn ${bleConnected ? "active" : ""}`}
            >
              {bleConnected ? "DISCONNECT" : "BLUETOOTH"}
            </button>
            <button
              onClick={() => (serialConnected ? handleDisconnect() : void handleConnect().catch((e) => logLine(`[dashboard] ${e.message}`)))}
              disabled={!serialSupported}
              title={serialSupported ? "open the MPU, TPU or SPU USB port" : "WebSerial needs Chrome/Edge over https or localhost"}
              className={`btn ${serialConnected ? "active" : ""}`}
            >
              {serialConnected ? "DISCONNECT" : "SERIAL"}
            </button>
            <button onClick={() => setShowAdmin(!showAdmin)} className={`btn ${showAdmin ? "active" : ""}`}>
              {isAdmin ? "ADMIN" : "LOGIN"}
            </button>
            <button onClick={() => setShowChecklist(!showChecklist)} className={`btn ${showChecklist ? "active" : ""}`} title="pre-flight GO/NO-GO from live data plus a hand-ticked list (c)">
              {showChecklist ? "CHECKLIST" : checks.every((c) => c.ok) ? "GO" : "CHECKLIST"}
            </button>
            <button
              onClick={() => {
                setSound(!sound);
                if (!sound) {
                  soundRef.current = true;
                  beep(880, 80);
                }
              }}
              className={`btn ${sound ? "active" : ""}`}
              title="beep on launch, apogee, pyro firings, landing, countdown (s)"
            >
              {sound ? "SOUND ON" : "SOUND"}
            </button>
          </div>

          {replay && (
            <input
              type="range"
              min={0}
              max={1000}
              value={Math.round(replay.progress * 1000)}
              onChange={(e) => {
                resetData(); // a rewind replays events and tracks from that point; no duplicates from before it
                replayRef.current?.seek(Number(e.target.value) / 1000);
              }}
              className="mt-2 w-full"
              title={`${replay.name} · drag to seek`}
            />
          )}
          {showChecklist && (
            <div className="mt-3 w-full border-t border-line pt-3">
              <ChecklistPanel auto={checks} />
            </div>
          )}
          {showAdmin && (
            <div className="mt-3 w-full border-t border-line pt-3">
              <AdminPanel
                isAdmin={isAdmin}
                serialConnected={serialConnected}
                portLabel={portLabel}
                link={link}
                loopHz={s?.loop_hz ?? 0}
                imuMask={imuMask}
                onAuth={authenticate}
                onConnect={handleConnect}
                onDisconnect={handleDisconnect}
              />
            </div>
          )}
        </Panel>

        {/* (1,1) Acceleration */}
        <Panel className="order-1 flex flex-col justify-center p-4 xl:col-start-1 xl:row-start-1">
          <Title sub="body frame">Acceleration</Title>
          <div className="flex flex-col gap-2">
            <AxisValue axis="X" value={acc[0]} color={X} unit="g" />
            <AxisValue axis="Y" value={acc[1]} color={Y} unit="g" />
            <AxisValue axis="Z" value={acc[2]} color={Z} unit="g" />
          </div>
        </Panel>

        {/* (2,1) Accel chart */}
        <Panel className="order-2 flex min-h-48 flex-col xl:col-start-1 xl:row-start-2 xl:min-h-0">
          <div className="px-4 pt-3">
            <Title sub="last 20 s, g">Accel</Title>
          </div>
          <div className="flex-1" style={{ minHeight: 0 }}>
            <DataCharts history={history} type="accel" />
          </div>
        </Panel>

        {/* (1,3) Gyroscope */}
        <Panel className="order-3 flex flex-col justify-center p-4 xl:col-start-3 xl:row-start-1">
          <Title sub="bias removed">Gyroscope</Title>
          <div className="flex flex-col gap-2">
            <AxisValue axis="X" value={gyro[0]} color={X} unit="°/s" decimals={1} />
            <AxisValue axis="Y" value={gyro[1]} color={Y} unit="°/s" decimals={1} />
            <AxisValue axis="Z" value={gyro[2]} color={Z} unit="°/s" decimals={1} />
          </div>
        </Panel>

        {/* (2,3) Gyro chart */}
        <Panel className="order-4 flex min-h-48 flex-col xl:col-start-3 xl:row-start-2 xl:min-h-0">
          <div className="px-4 pt-3">
            <Title sub="last 20 s, °/s">Gyro</Title>
          </div>
          <div className="flex-1" style={{ minHeight: 0 }}>
            <DataCharts history={history} type="gyro" />
          </div>
        </Panel>

        {/* (1,2) 3D attitude */}
        <Panel className="order-5 relative min-h-72 xl:col-start-2 xl:row-start-1 xl:min-h-0">
          <Suspense fallback={<div className="flex h-full items-center justify-center text-ink-3">Loading 3D...</div>}>
            <BoardVisualizer q={q} nose={nose} />
          </Suspense>
          <label className="absolute top-3 right-3 flex items-center gap-1 text-[10px] text-ink-2">
            nose
            <select
              value={nose}
              onChange={(e) => setNose(e.target.value as NoseAxis)}
              title="body axis that points along the rocket (reads +1 g on the pad)"
              className="text-[11px]"
            >
              {NOSE_AXES.map((n) => (
                <option key={n}>{n}</option>
              ))}
            </select>
          </label>
        </Panel>

        {/* (3,1) Flight */}
        <Panel className="order-6 flex flex-col justify-center p-4 xl:col-start-1 xl:row-start-3">
          <Title>Flight</Title>
          <div className="grid grid-cols-3 gap-3">
            <Metric label="Altitude" value={L(alt)} unit={`${UL} above pad`} />
            <Metric label="Vertical speed" value={L(vz)} unit={US} />
            <Metric label="Baro altitude" value={L(baro)} unit={UL} />
          </div>
          <div className="mt-3 flex flex-wrap gap-1.5">
            <Flag label="IN FLIGHT" on={!!(flags & STATE_FLAG.IN_FLIGHT)} />
            <Flag label="GPS FRESH" on={!!(flags & STATE_FLAG.GPS_FRESH)} />
            <Flag label="DEAD RECKONING" on={!!s && !(flags & STATE_FLAG.GPS_FRESH) && !!(flags & STATE_FLAG.IN_FLIGHT)} warn />
            <Flag label="BARO" on={!!(flags & STATE_FLAG.BARO_OK)} />
            <Flag label="MAG" on={!!(flags & STATE_FLAG.MAG_OK)} />
            <Flag label="ORIGIN" on={!!(flags & STATE_FLAG.ORIGIN_OK)} />
            {[1, 2, 3].map((i) => (
              <Flag key={i} label={`IMU${i}`} on={!!(imuMask & (1 << (i - 1)))} />
            ))}
          </div>
        </Panel>

        {/* (3,2) Altitude chart */}
        <Panel className="order-7 flex min-h-48 flex-col xl:col-start-2 xl:row-start-3 xl:min-h-0">
          <div className="px-4 pt-3">
            <Title sub="fused vs barometer, m">Altitude</Title>
          </div>
          <div className="flex-1" style={{ minHeight: 0 }}>
            <DataCharts history={history} type="alt" />
          </div>
        </Panel>

        {/* (3,3) Attitude & GPS */}
        <Panel className="order-8 flex flex-col justify-center p-4 xl:col-start-3 xl:row-start-3">
          <Title>Attitude &amp; GPS</Title>
          <dl className="grid grid-cols-[auto_1fr] gap-x-4 gap-y-1 text-xs">
            <KV k="roll / pitch / yaw" v={rpy} />
            <KV k="fix" v={fix} />
            <KV k="latitude" v={pos ? pos.lat.toFixed(6) + " °" : "-"} />
            <KV k="longitude" v={pos ? pos.lon.toFixed(6) + " °" : "-"} />
            <KV k="height MSL" v={gps ? L(gps.hmsl).toFixed(1) + " " + UL : "-"} />
            <KV k="ground speed" v={gps ? L(Math.hypot(gps.vel[0], gps.vel[1])).toFixed(1) + " " + US : "-"} />
            <KV k="magnetometer" v={s ? s.mag.map((x) => (x * 1000).toFixed(0)).join(" ") + " mG" : "-"} />
            <KV k="link" v={`${link.ok} frames, ${link.bad} bad` + (s ? ` · ${s.loop_hz} Hz` : "")} />
          </dl>
        </Panel>

        {/* (1-2,4) Map: ground track, dead reckoning, landing estimate */}
        <Panel className="order-9 flex min-h-96 flex-col xl:col-start-4 xl:row-span-2 xl:row-start-1 xl:min-h-0">
          <Suspense fallback={<div className="flex h-full items-center justify-center text-ink-3">Loading map...</div>}>
            <MapPanel fused={fusedTrack} gps={gpsTrack} pad={pad} cur={cur} hacc={gps?.hacc ?? 0} landing={landing} stats={stats} />
          </Suspense>
        </Panel>

        {/* (3,4) Recovery & power (SPU) */}
        <Panel className="order-10 flex flex-col xl:col-start-4 xl:row-start-3">
          <RecoveryPanel spu={spu} fresh={spuFresh} canCommand={serialConnected || bleConnected || isAdmin || hasBridge} onCommand={sendCommand} />
        </Panel>
      </div>

      {/* Bottom strip: console text from the link (left) and the flight event timeline (right) */}
      <div className="grid h-28 shrink-0 grid-cols-1 gap-2 md:h-32 md:grid-cols-[3fr_2fr]">
        <div ref={consoleRef} className="console h-full min-h-0">
          {lines.length ? (
            lines.map((l, i) => (
              <div key={i}>
                <span className="t">{l.slice(0, 8)}</span>
                {l.slice(8)}
              </div>
            ))
          ) : (
            <span className="t">console — text from the link appears here</span>
          )}
        </div>
        <div className="panel h-full min-h-0">
          <EventsPanel events={events} summary={summary} />
        </div>
      </div>
      <div className="flex items-center justify-between text-[10px] tracking-wider text-ink-3">
        <span className="font-mono tabular-nums">
          state {rates.state}/s · gps {rates.gps}/s · telem {rates.telem}/s · spu {rates.spu}/s · {(rates.bytes / 1024).toFixed(1)} kB/s · {link.bad} bad
          {stationFresh && (
            <span className="text-teal" title="RTL-SDR ground station: signal over noise, carrier offset, good packets / decoded, command uplink">
              {" "}
              · RF {station!.level_db.toFixed(0)} dB · cfo {station!.cfo_khz >= 0 ? "+" : ""}
              {station!.cfo_khz.toFixed(1)} kHz · {station!.ok}/{station!.packets} pkts
              {station!.last_rx ? ` · rx ${Math.max(0, Date.now() / 1000 - station!.last_rx).toFixed(0)} s ago` : ""} · uplink {station!.uplink ? "yes" : "no"}
            </span>
          )}
        </span>
        <span className="flex items-center gap-2">
          <button onClick={toggleRecording} className={`btn ${recording ? "danger" : ""}`} style={{ padding: "2px 8px" }} title="save the raw link stream as a replayable .bin (r)">
            {recording ? `STOP · ${(recordedBytes / 1024).toFixed(0)} KB` : "REC"}
          </button>
          <button onClick={saveReport} className="btn" style={{ padding: "2px 8px" }} title="download a JSON flight report: summary, events, tracks, last frames, console">
            REPORT
          </button>
          <button onClick={() => setUnits(ft ? "m" : "ft")} className="btn" style={{ padding: "2px 8px" }} title="display units for altitude, distance and speed (the link and the SPU stay metric)">
            {ft ? "FT" : "M"}
          </button>
          <a href="https://github.com/NotARoomba/Athena" className="text-ink-2">
            Athena
          </a>
        </span>
      </div>
    </div>
  );
}
