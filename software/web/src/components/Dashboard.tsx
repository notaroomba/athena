import { useState, useEffect, useRef, useCallback, lazy, Suspense } from "react";
import AdminPanel from "./AdminPanel";
import DataCharts, { WINDOW_S } from "./DataCharts";
import { AxisValue, Flag, Metric } from "./SensorCard";
import { NOSE_AXES, type AthenaState, type GpsFix, type LinkStats, type NoseAxis, type Quat, type Sample, type Telemetry, type WSMessage } from "@/lib/types";
import { Decoder, FIX_NAMES, G0, PKT, R2D, STATE_FLAG, parseGps, parseState, parseTelem, quatToEuler } from "@/lib/protocol";
import { connectSerial, disconnectSerial, isSerialSupported } from "@/lib/serial";
import { startDemo } from "@/lib/demo";

const BoardVisualizer = lazy(() => import("./BoardVisualizer"));

const WS_URL: string = import.meta.env.VITE_WS_URL ?? "wss://api.athena.notaroomba.dev/ws";
const MAX_LINES = 200;
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
  const [wsConnected, setWsConnected] = useState(false);
  const [viewers, setViewers] = useState(0);
  const [adminOnline, setAdminOnline] = useState(false);
  const [showAdmin, setShowAdmin] = useState(false);
  const [demoMode, setDemoMode] = useState(false);
  const [nose, setNose] = useState<NoseAxis>(loadNose);

  const [state, setState] = useState<AthenaState | null>(null);
  const [gps, setGps] = useState<GpsFix | null>(null);
  const [telem, setTelem] = useState<Telemetry | null>(null);
  const [history, setHistory] = useState<Sample[]>([]);
  const [lines, setLines] = useState<string[]>([]);
  const [link, setLink] = useState<LinkStats>({ ok: 0, bad: 0 });

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectRef = useRef<ReturnType<typeof setTimeout>>(undefined);
  const savedPasswordRef = useRef<string | null>(null);
  const isAdminRef = useRef(false);
  const decoderRef = useRef<Decoder | null>(null);
  const consoleRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    isAdminRef.current = isAdmin;
  }, [isAdmin]);

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
      const next = [...prev, s];
      let i = 0;
      while (i < next.length && s.t - next[i].t > WINDOW_S) i++;
      return i ? next.slice(i) : next;
    });
  }, []);

  const onFrame = useCallback(
    (type: number, p: Uint8Array) => {
      if (type === PKT.STATE) {
        const s = parseState(p);
        if (!s) return;
        setState(s);
        pushSample({
          t: s.t_us / 1e6,
          acc: [s.acc[0] / G0, s.acc[1] / G0, s.acc[2] / G0],
          gyro: [s.gyro[0] * R2D, s.gyro[1] * R2D, s.gyro[2] * R2D],
          alt: -s.pos[2],
          baro: s.baro_alt,
        });
      } else if (type === PKT.GPS) {
        const g = parseGps(p);
        if (g) setGps(g);
      } else if (type === PKT.TELEM) {
        const t = parseTelem(p);
        if (!t) return;
        setTelem(t);
        // TPU port only: build what we can from the compact frame
        setState((cur) => {
          if (!cur) pushSample({ t: t.t_ms / 1e3, alt: t.alt, baro: t.baro_alt });
          return cur;
        });
      } else if (type === PKT.TEXT) {
        logLine(new TextDecoder().decode(p));
      }
    },
    [pushSample, logLine],
  );

  const resetData = useCallback(() => {
    setState(null);
    setGps(null);
    setTelem(null);
    setHistory([]);
    setLink({ ok: 0, bad: 0 });
    decoderRef.current = new Decoder(onFrame, logLine);
  }, [onFrame, logLine]);

  useEffect(() => {
    if (!decoderRef.current) decoderRef.current = new Decoder(onFrame, logLine);
  }, [onFrame, logLine]);

  /** Raw link bytes from any source (serial, demo, or relayed by the server). */
  const feed = useCallback((bytes: Uint8Array) => {
    const d = decoderRef.current;
    if (!d) return;
    d.feed(bytes);
    setLink({ ok: d.ok, bad: d.bad });
  }, []);

  useEffect(() => {
    if (consoleRef.current) consoleRef.current.scrollTop = consoleRef.current.scrollHeight;
  }, [lines]);

  // ---- demo
  useEffect(() => {
    if (!demoMode) return;
    resetData();
    logLine("[dashboard] demo: simulated flight through the real frame encoder/decoder");
    const stop = startDemo(nose, feed);
    return stop;
    // nose is read once at demo start on purpose; changing it mid-demo only re-renders the model
  }, [demoMode, feed, resetData, logLine]); // eslint-disable-line react-hooks/exhaustive-deps

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
        // relayed raw link bytes from the admin's serial port; the decoder resyncs on any chunk boundary
        if (!isAdminRef.current) feed(new Uint8Array(event.data));
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
        }
      } catch {
        /* ignore */
      }
    };
  }, [feed]);

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

  // ---- serial
  const handleConnect = useCallback(async () => {
    setDemoMode(false);
    const label = await connectSerial({
      onChunk: (bytes) => {
        feed(bytes);
        if (isAdminRef.current && wsRef.current?.readyState === WebSocket.OPEN) {
          wsRef.current.send(bytes);
        }
      },
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
  }, [feed, resetData, logLine]);

  const handleDisconnect = useCallback(() => {
    void disconnectSerial();
  }, []);

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
  const isLive = adminOnline || serialConnected || demoMode;
  const serialSupported = isSerialSupported();

  return (
    <div className="flex min-h-screen w-full flex-col gap-2 overflow-y-auto p-2 md:h-screen md:overflow-hidden xl:gap-3 xl:p-3">
      {/* 3x3 grid — fits a 16:9 viewport, single column on phones */}
      <div className="grid grid-cols-1 gap-2 md:min-h-0 md:flex-1 md:grid-cols-3 md:grid-rows-3 xl:gap-3">
        {/* ═══ HERO — center (top on mobile) ═══ */}
        <Panel className="order-0 flex flex-col items-center justify-center p-4 md:col-start-2 md:row-start-2">
          <img src="/logo.png" alt="Athena logo" className="h-16 w-16" />
          <div className="wordmark mt-1">ATHENA</div>
          <div className="stripebar mt-2 w-40" />

          <div className="mt-3 flex flex-col items-center gap-1">
            <div
              className={`text-sm font-bold tracking-[.22em] ${
                isLive ? (demoMode ? "text-orange-2" : "text-teal") : "text-ink-3"
              }`}
            >
              {isLive ? (demoMode ? "DEMO" : "CONNECTED") : "NO DATA"}
            </div>
            <div className="flex items-center gap-1.5">
              <div className={`h-1.5 w-1.5 rounded-full ${wsConnected ? "pulse-dot bg-teal" : "bg-orange"}`} />
              <span className="text-[10px] text-ink-3">
                {viewers} viewer{viewers !== 1 ? "s" : ""}
              </span>
            </div>
          </div>

          <div className="mt-3 flex flex-wrap justify-center gap-2">
            <button onClick={() => setDemoMode(!demoMode)} className={`btn ${demoMode ? "active" : ""}`}>
              DEMO
            </button>
            <button
              onClick={() => (serialConnected ? handleDisconnect() : void handleConnect().catch((e) => logLine(`[dashboard] ${e.message}`)))}
              disabled={!serialSupported}
              title={serialSupported ? "open the MPU or TPU USB port" : "WebSerial needs Chrome/Edge over https or localhost"}
              className={`btn ${serialConnected ? "active" : ""}`}
            >
              {serialConnected ? "DISCONNECT" : "CONNECT"}
            </button>
            <button onClick={() => setShowAdmin(!showAdmin)} className={`btn ${showAdmin ? "active" : ""}`}>
              {isAdmin ? "ADMIN" : "LOGIN"}
            </button>
          </div>

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
        <Panel className="order-1 flex flex-col justify-center p-4 md:col-start-1 md:row-start-1">
          <Title sub="body frame">Acceleration</Title>
          <div className="flex flex-col gap-2">
            <AxisValue axis="X" value={acc[0]} color={X} unit="g" />
            <AxisValue axis="Y" value={acc[1]} color={Y} unit="g" />
            <AxisValue axis="Z" value={acc[2]} color={Z} unit="g" />
          </div>
        </Panel>

        {/* (2,1) Accel chart */}
        <Panel className="order-2 flex min-h-48 flex-col md:col-start-1 md:row-start-2 md:min-h-0">
          <div className="px-4 pt-3">
            <Title sub="last 20 s, g">Accel</Title>
          </div>
          <div className="flex-1" style={{ minHeight: 0 }}>
            <DataCharts history={history} type="accel" />
          </div>
        </Panel>

        {/* (1,3) Gyroscope */}
        <Panel className="order-3 flex flex-col justify-center p-4 md:col-start-3 md:row-start-1">
          <Title sub="bias removed">Gyroscope</Title>
          <div className="flex flex-col gap-2">
            <AxisValue axis="X" value={gyro[0]} color={X} unit="°/s" decimals={1} />
            <AxisValue axis="Y" value={gyro[1]} color={Y} unit="°/s" decimals={1} />
            <AxisValue axis="Z" value={gyro[2]} color={Z} unit="°/s" decimals={1} />
          </div>
        </Panel>

        {/* (2,3) Gyro chart */}
        <Panel className="order-4 flex min-h-48 flex-col md:col-start-3 md:row-start-2 md:min-h-0">
          <div className="px-4 pt-3">
            <Title sub="last 20 s, °/s">Gyro</Title>
          </div>
          <div className="flex-1" style={{ minHeight: 0 }}>
            <DataCharts history={history} type="gyro" />
          </div>
        </Panel>

        {/* (1,2) 3D attitude */}
        <Panel className="order-5 relative min-h-72 md:col-start-2 md:row-start-1 md:min-h-0">
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
        <Panel className="order-6 flex flex-col justify-center p-4 md:col-start-1 md:row-start-3">
          <Title>Flight</Title>
          <div className="grid grid-cols-3 gap-3">
            <Metric label="Altitude" value={alt} unit="m above pad" />
            <Metric label="Vertical speed" value={vz} unit="m/s" />
            <Metric label="Baro altitude" value={baro} unit="m" />
          </div>
          <div className="mt-3 flex flex-wrap gap-1.5">
            <Flag label="IN FLIGHT" on={!!(flags & STATE_FLAG.IN_FLIGHT)} />
            <Flag label="GPS FRESH" on={!!(flags & STATE_FLAG.GPS_FRESH)} />
            <Flag label="BARO" on={!!(flags & STATE_FLAG.BARO_OK)} />
            <Flag label="MAG" on={!!(flags & STATE_FLAG.MAG_OK)} />
            <Flag label="ORIGIN" on={!!(flags & STATE_FLAG.ORIGIN_OK)} />
            {[1, 2, 3].map((i) => (
              <Flag key={i} label={`IMU${i}`} on={!!(imuMask & (1 << (i - 1)))} />
            ))}
          </div>
        </Panel>

        {/* (3,2) Altitude chart */}
        <Panel className="order-7 flex min-h-48 flex-col md:col-start-2 md:row-start-3 md:min-h-0">
          <div className="px-4 pt-3">
            <Title sub="fused vs barometer, m">Altitude</Title>
          </div>
          <div className="flex-1" style={{ minHeight: 0 }}>
            <DataCharts history={history} type="alt" />
          </div>
        </Panel>

        {/* (3,3) Attitude & GPS */}
        <Panel className="order-8 flex flex-col justify-center p-4 md:col-start-3 md:row-start-3">
          <Title>Attitude &amp; GPS</Title>
          <dl className="grid grid-cols-[auto_1fr] gap-x-4 gap-y-1 text-xs">
            <KV k="roll / pitch / yaw" v={rpy} />
            <KV k="fix" v={fix} />
            <KV k="latitude" v={pos ? pos.lat.toFixed(6) + " °" : "-"} />
            <KV k="longitude" v={pos ? pos.lon.toFixed(6) + " °" : "-"} />
            <KV k="height MSL" v={gps ? gps.hmsl.toFixed(1) + " m" : "-"} />
            <KV k="ground speed" v={gps ? Math.hypot(gps.vel[0], gps.vel[1]).toFixed(1) + " m/s" : "-"} />
            <KV k="magnetometer" v={s ? s.mag.map((x) => (x * 1000).toFixed(0)).join(" ") + " mG" : "-"} />
            <KV k="link" v={`${link.ok} frames, ${link.bad} bad` + (s ? ` · ${s.loop_hz} Hz` : "")} />
          </dl>
        </Panel>
      </div>

      {/* Console: text lines from the link */}
      <div ref={consoleRef} className="console h-24 shrink-0 md:h-28">
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
      <div className="flex justify-between text-[10px] tracking-wider text-ink-3">
        <span>MPU emits STATE frames at 20 Hz, TPU emits GPS + TELEM frames · framing from firmware/Athena/athena_link.h</span>
        <a href="https://github.com/NotARoomba/Athena" className="text-ink-2">
          Athena
        </a>
      </div>
    </div>
  );
}
