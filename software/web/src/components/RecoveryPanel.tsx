import { useEffect, useState } from "react";
import { Flag } from "./SensorCard";
import { CMD, CMD_KEY, PD_MODES, SPU_FLAG, SPU_PHASES } from "@/lib/protocol";
import type { SpuStatus } from "@/lib/types";

interface RecoveryPanelProps {
  spu: SpuStatus | null;
  fresh: boolean; // a status frame arrived in the last 3 s
  canCommand: boolean; // a writable link (serial or Bluetooth) is open
  onCommand: (cmd: number, arg?: number, value?: number, key?: number) => void;
}

/** BQ25713 ChargerStatus bits (0x21 in the high byte). */
function chargerText(st: number): string {
  if (!st) return "-";
  const parts: string[] = [];
  if (st & 0x8000) parts.push("input ok");
  if (st & 0x0400) parts.push("fast charge");
  else if (st & 0x0200) parts.push("precharge");
  if (st & 0x0100) parts.push("OTG");
  if (st & 0x00ff) parts.push(`fault 0x${(st & 0xff).toString(16)}`);
  return parts.length ? parts.join(", ") : "idle";
}

export default function RecoveryPanel({ spu, fresh, canCommand, onCommand }: RecoveryPanelProps) {
  const [pending, setPending] = useState<string | null>(null); // two-step confirmation for arm/fire
  const [mainAlt, setMainAlt] = useState("150");
  const [servo, setServo] = useState(1);
  const [servoUs, setServoUs] = useState(1500);

  useEffect(() => {
    if (!pending) return;
    const t = setTimeout(() => setPending(null), 4000);
    return () => clearTimeout(t);
  }, [pending]);

  useEffect(() => {
    if (spu) setMainAlt(String(spu.main_alt_m));
  }, [spu?.main_alt_m]); // eslint-disable-line react-hooks/exhaustive-deps

  const armed = !!(spu && spu.flags & SPU_FLAG.ARMED);
  const confirm = (key: string, run: () => void) => {
    if (pending === key) {
      setPending(null);
      run();
    } else setPending(key);
  };
  const label = (key: string, text: string) => (pending === key ? "CONFIRM" : text);

  return (
    <div className="flex h-full flex-col gap-2 p-4">
      <h2 className="panel-title">
        Recovery &amp; power
        <small>{spu ? (fresh ? "SPU live" : "SPU stale") : "no SPU data"}</small>
      </h2>

      {/* flight phase as the SPU sees it */}
      <div className="flex flex-wrap gap-1">
        {SPU_PHASES.map((p, i) => (
          <span key={p} className={`flag ${spu && spu.phase === i ? "on" : ""}`}>
            {p.toUpperCase()}
          </span>
        ))}
        <Flag label="ARMED" on={armed} warn />
        <Flag label="MPU LINK" on={!!(spu && spu.flags & SPU_FLAG.MPU_LINK)} />
      </div>

      {/* pyro channels */}
      <div className="grid grid-cols-6 gap-1">
        {[1, 2, 3, 4, 5, 6].map((ch) => {
          const bit = 1 << (ch - 1);
          const on = !!(spu && spu.pyro_on & bit),
            fired = !!(spu && spu.pyro_fired & bit);
          return (
            <div
              key={ch}
              className={`flex flex-col items-center rounded border px-1 py-1 text-[10px] ${on ? "pulse-dot border-orange text-orange" : fired ? "border-orange-2 text-orange-2" : "border-line text-ink-3"}`}
              title={ch === 1 ? "drogue at apogee" : ch === 2 ? "main below the set altitude" : "manual"}
            >
              <span className="font-semibold tracking-wider">P{ch}</span>
              <span>{on ? "FIRING" : fired ? "fired" : ch === 1 ? "drogue" : ch === 2 ? "main" : "-"}</span>
            </div>
          );
        })}
      </div>

      {/* battery / USB-PD */}
      <dl className="grid grid-cols-[auto_1fr_auto_1fr] gap-x-3 gap-y-0.5 text-[11px]">
        <dt className="text-ink-2">battery</dt>
        <dd className="font-mono tabular-nums">
          {spu && spu.flags & SPU_FLAG.BQ_OK && spu.vbat_mv ? `${(spu.vbat_mv / 1000).toFixed(2)} V  ${spu.ibat_ma > 0 ? "+" : ""}${(spu.ibat_ma / 1000).toFixed(2)} A` : spu ? "no charger bus" : "-"}
        </dd>
        <dt className="text-ink-2">USB-C</dt>
        <dd className="font-mono tabular-nums">
          {spu
            ? `${PD_MODES[spu.pd_mode] ?? spu.pd_mode}${spu.pd_status & 1 ? " · plug" : ""}${
                spu.vbus_mv ? (spu.flags & SPU_FLAG.BQ_OK ? ` · ${(spu.vbus_mv / 1000).toFixed(1)} V` : ` · PD ${(spu.vbus_mv / 1000).toFixed(0)} V/${(spu.iin_ma / 1000).toFixed(1)} A`) : ""
              }`
            : "-"}
        </dd>
        <dt className="text-ink-2">charger</dt>
        <dd className="font-mono tabular-nums">{spu ? chargerText(spu.chg_status) : "-"}</dd>
        <dt className="text-ink-2">system</dt>
        <dd className="font-mono tabular-nums">{spu && spu.flags & SPU_FLAG.BQ_OK && spu.vsys_mv ? `${(spu.vsys_mv / 1000).toFixed(2)} V · in ${(spu.iin_ma / 1000).toFixed(2)} A` : "-"}</dd>
        <dt className="text-ink-2">apogee</dt>
        <dd className="font-mono tabular-nums">{spu ? `${spu.apogee_m.toFixed(0)} m · ${spu.vmax_ms.toFixed(0)} m/s max` : "-"}</dd>
        <dt className="text-ink-2">pins</dt>
        <dd className="font-mono tabular-nums">
          {spu ? [spu.flags & SPU_FLAG.CHRG_OK ? "CHRG_OK" : "", spu.flags & SPU_FLAG.PROCHOT ? "PROCHOT" : "", spu.flags & SPU_FLAG.BQ_OK ? "BQ" : ""].filter(Boolean).join(" ") || "-" : "-"}
        </dd>
      </dl>

      {/* commands: only with an open link; destructive ones need a second click within 4 s */}
      {canCommand && (
        <div className="mt-auto flex flex-col gap-1.5 border-t border-line pt-2">
          <div className="flex flex-wrap items-center gap-1">
            <button className={`btn ${armed ? "danger" : ""}`} onClick={() => confirm("arm", () => onCommand(CMD.ARM, 0, 0, CMD_KEY))} disabled={armed}>
              {label("arm", "ARM")}
            </button>
            <button className="btn" onClick={() => onCommand(CMD.DISARM)} disabled={!armed}>
              DISARM
            </button>
            {[1, 2, 3, 4, 5, 6].map((ch) => (
              <button key={ch} className="btn danger" style={{ padding: "6px 8px" }} disabled={!armed} onClick={() => confirm(`fire${ch}`, () => onCommand(CMD.FIRE, ch, 0, CMD_KEY))}>
                {pending === `fire${ch}` ? "SURE?" : `FIRE ${ch}`}
              </button>
            ))}
            <button className="btn" onClick={() => confirm("rst", () => onCommand(CMD.RESET_MPU))}>
              {label("rst", "RESET MPU")}
            </button>
          </div>
          <div className="flex flex-wrap items-center gap-2 text-[10px] text-ink-2">
            <label className="flex items-center gap-1">
              main at
              <input type="text" value={mainAlt} onChange={(e) => setMainAlt(e.target.value)} className="w-14 text-xs" />
              m
            </label>
            <button className="btn" style={{ padding: "3px 8px" }} onClick={() => onCommand(CMD.SET_MAIN_ALT, 0, Math.round(Number(mainAlt) || 0))}>
              SET
            </button>
            <label className="flex items-center gap-1">
              servo
              <select value={servo} onChange={(e) => setServo(Number(e.target.value))} className="text-[11px]">
                {[1, 2, 3, 4, 5, 6].map((n) => (
                  <option key={n} value={n}>
                    {n}
                  </option>
                ))}
              </select>
            </label>
            <input
              type="range"
              min={1000}
              max={2000}
              step={10}
              value={servoUs}
              onChange={(e) => setServoUs(Number(e.target.value))}
              onMouseUp={() => onCommand(CMD.SERVO, servo, servoUs)}
              onTouchEnd={() => onCommand(CMD.SERVO, servo, servoUs)}
              className="w-24"
            />
            <span className="font-mono">{servoUs} µs</span>
            <button className="btn" style={{ padding: "3px 8px" }} onClick={() => onCommand(CMD.SERVO, servo, 0)}>
              RELEASE
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
