import { useEffect, useRef } from "react";
import type { FlightEvent } from "@/lib/types";

export interface FlightSummary {
  apogee: number; // m above pad
  vmax: number; // m/s
  gmax: number; // g
  flightTime: number; // s, launch -> landed (or so far)
  landingDist: number; // m from pad
  phase: string;
}

interface EventsPanelProps {
  events: FlightEvent[];
  summary: FlightSummary | null;
}

/** Flight events (phases, pyro firings, arming, launch) with a one-line summary; sits beside the console. */
export default function EventsPanel({ events, summary }: EventsPanelProps) {
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight;
  }, [events]);

  return (
    <div className="flex h-full min-h-0 flex-col">
      <div className="flex items-baseline justify-between px-3 pt-2">
        <span className="panel-title">Flight events</span>
        {summary && (
          <span className="font-mono text-[10px] text-ink-2 tabular-nums">
            {summary.phase} · apogee {summary.apogee.toFixed(0)} m · {summary.vmax.toFixed(0)} m/s · {summary.gmax.toFixed(1)} g · {summary.flightTime.toFixed(0)} s
            {summary.landingDist > 0 ? ` · ${summary.landingDist.toFixed(0)} m from pad` : ""}
          </span>
        )}
      </div>
      <div ref={ref} className="console mt-1 min-h-0 flex-1 border-0 bg-transparent px-3 py-1">
        {events.length ? (
          events.map((e, i) => (
            <div key={i}>
              <span className="t">{e.when}</span>
              {"  "}
              <span style={{ color: e.color }}>{e.label}</span>
              {e.detail ? `  ${e.detail}` : ""}
            </div>
          ))
        ) : (
          <span className="t">launch, burnout, apogee, pyro firings and landing appear here</span>
        )}
      </div>
    </div>
  );
}
