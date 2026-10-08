import { useEffect, useState } from "react";

export interface AutoCheck {
  label: string;
  ok: boolean;
  detail?: string;
}

const MANUAL = [
  "Igniters installed, continuity checked",
  "Recovery packed, shock cord and chutes attached",
  "SD card inserted, antennas on (LoRa, GPS)",
  "ARM switch on (pyro bus powered)",
  "Range clear, RSO go",
] as const;

function loadManual(): boolean[] {
  try {
    const v = JSON.parse(localStorage.getItem("athena.checklist") || "[]");
    if (Array.isArray(v) && v.length === MANUAL.length) return v.map(Boolean);
  } catch {
    /* ignore */
  }
  return MANUAL.map(() => false);
}

/** Pre-flight GO/NO-GO: items read from live telemetry plus a hand-ticked list (kept in localStorage). */
export default function ChecklistPanel({ auto }: { auto: AutoCheck[] }) {
  const [manual, setManual] = useState<boolean[]>(loadManual);
  useEffect(() => {
    try {
      localStorage.setItem("athena.checklist", JSON.stringify(manual));
    } catch {
      /* ignore */
    }
  }, [manual]);

  const open = auto.filter((a) => !a.ok).length + manual.filter((m) => !m).length;
  return (
    <div className="flex flex-col gap-1 text-left text-[11px]">
      <div className="flex items-baseline justify-between">
        <span className="panel-title">Pre-flight</span>
        <span className={`font-mono text-xs font-bold tracking-[.2em] ${open ? "text-orange" : "text-teal"}`}>{open ? `NO-GO · ${open}` : "GO"}</span>
      </div>
      <ul className="grid grid-cols-1 gap-x-4 gap-y-0.5 sm:grid-cols-2">
        {auto.map((a) => (
          <li key={a.label} className={`flex items-baseline gap-1.5 ${a.ok ? "text-ink-2" : "text-orange"}`} title={a.detail}>
            <span className="font-mono">{a.ok ? "✓" : "✗"}</span>
            <span>{a.label}</span>
            {a.detail && <span className="ml-auto font-mono text-ink-3 tabular-nums">{a.detail}</span>}
          </li>
        ))}
      </ul>
      <ul className="mt-1 grid grid-cols-1 gap-x-4 gap-y-0.5 border-t border-line pt-1 sm:grid-cols-2">
        {MANUAL.map((m, i) => (
          <li key={m}>
            <label className={`flex cursor-pointer items-baseline gap-1.5 ${manual[i] ? "text-ink-2" : "text-cream"}`}>
              <input type="checkbox" checked={manual[i]} onChange={(e) => setManual((prev) => prev.map((v, j) => (j === i ? e.target.checked : v)))} />
              {m}
            </label>
          </li>
        ))}
      </ul>
      <button className="btn self-end" style={{ padding: "2px 8px" }} onClick={() => setManual(MANUAL.map(() => false))}>
        RESET
      </button>
    </div>
  );
}
