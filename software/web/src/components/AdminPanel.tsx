import { useState } from "react";
import { isSerialSupported } from "@/lib/serial";
import type { LinkStats } from "@/lib/types";

interface AdminPanelProps {
  isAdmin: boolean;
  serialConnected: boolean;
  portLabel: string;
  link: LinkStats;
  loopHz: number;
  imuMask: number;
  onAuth: (password: string) => void;
  onConnect: () => Promise<void>;
  onDisconnect: () => void;
}

export default function AdminPanel({
  isAdmin,
  serialConnected,
  portLabel,
  link,
  loopHz,
  imuMask,
  onAuth,
  onConnect,
  onDisconnect,
}: AdminPanelProps) {
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const serialSupported = isSerialSupported();

  const handleAuth = () => {
    if (!password.trim()) return;
    setError("");
    onAuth(password);
  };

  const handleConnect = async () => {
    setError("");
    try {
      await onConnect();
    } catch (e) {
      setError(e instanceof Error ? e.message : "Serial connection failed");
    }
  };

  const mask = [1, 2, 3].map((i) => (imuMask & (1 << (i - 1)) ? `${i}` : "-")).join("");

  return (
    <div className="flex flex-col items-center gap-2">
      {!isAdmin ? (
        <>
          <div className="flex items-center gap-1">
            <input
              type="password"
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              onKeyDown={(e) => e.key === "Enter" && handleAuth()}
              className="w-32 text-xs"
            />
            <button onClick={handleAuth} className="btn active">
              AUTH
            </button>
          </div>
          {error && <span className="text-[10px] text-orange">{error}</span>}
        </>
      ) : (
        <>
          {!serialSupported ? (
            <span className="text-[10px] text-orange">No WebSerial. Use Chrome/Edge over https or localhost.</span>
          ) : serialConnected ? (
            <div className="flex items-center gap-2">
              <div className="flex items-center gap-1.5">
                <div className="pulse-dot h-1.5 w-1.5 rounded-full bg-teal" />
                <span className="font-mono text-xs text-teal">{portLabel || "serial"}</span>
              </div>
              <button onClick={onDisconnect} className="btn danger">
                DISCONNECT
              </button>
            </div>
          ) : (
            <button onClick={handleConnect} className="btn active">
              CONNECT SERIAL
            </button>
          )}
          <dl className="grid grid-cols-[auto_1fr] gap-x-4 gap-y-1 font-mono text-[11px]">
            <dt className="text-ink-2">frames</dt>
            <dd className="text-right tabular-nums">
              {link.ok} ok / <span className={link.bad ? "text-orange-2" : ""}>{link.bad} bad</span>
            </dd>
            <dt className="text-ink-2">loop</dt>
            <dd className="text-right tabular-nums">{loopHz} Hz</dd>
            <dt className="text-ink-2">imu mask</dt>
            <dd className="text-right tabular-nums">{mask}</dd>
          </dl>
          {error && <span className="text-[10px] text-orange">{error}</span>}
        </>
      )}
    </div>
  );
}
