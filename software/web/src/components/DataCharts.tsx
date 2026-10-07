import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  ResponsiveContainer,
  Tooltip,
} from "recharts";
import type { Sample } from "@/lib/types";

interface DataChartsProps {
  history: Sample[]; // already trimmed to the 20 s window by Dashboard
  type: "accel" | "gyro" | "alt";
}

export const WINDOW_S = 20;

const X = "#ea5a2c",
  Y = "#1f9aa8",
  Z = "#3b5fd0";

export default function DataCharts({ history, type }: DataChartsProps) {
  if (history.length === 0) {
    return (
      <div className="flex h-full items-center justify-center text-ink-3">
        <span className="text-xs tracking-wider">WAITING...</span>
      </div>
    );
  }

  const step = Math.max(1, Math.floor(history.length / 100));
  const downsampled = history.filter((_, i) => i % step === 0);
  const t1 = history[history.length - 1].t;
  const rel = (s: Sample) => (s.t - t1).toFixed(1);
  const r = (v: number | undefined, d: number) => (v === undefined ? null : parseFloat(v.toFixed(d)));

  const data: Record<string, string | number | null>[] =
    type === "accel"
      ? downsampled.map((s) => ({ t: rel(s), X: r(s.acc?.[0], 3), Y: r(s.acc?.[1], 3), Z: r(s.acc?.[2], 3) }))
      : type === "gyro"
        ? downsampled.map((s) => ({ t: rel(s), X: r(s.gyro?.[0], 1), Y: r(s.gyro?.[1], 1), Z: r(s.gyro?.[2], 1) }))
        : downsampled.map((s) => ({ t: rel(s), fused: r(s.alt, 1), baro: r(s.baro, 1) }));

  const gridStroke = "#1b1b1f";
  const axisStyle = { fill: "#6b665e", fontSize: 10, fontFamily: "JetBrains Mono, monospace" };
  const tooltipStyle = {
    contentStyle: {
      backgroundColor: "#19191c",
      border: "1px solid #232326",
      borderRadius: 6,
      color: "#ece6da",
      fontSize: 11,
      fontFamily: "JetBrains Mono, monospace",
    },
  };
  const line = (key: string, stroke: string) => (
    <Line key={key} type="monotone" dataKey={key} stroke={stroke} dot={false} strokeWidth={1.5} isAnimationActive={false} />
  );

  return (
    <div className="h-full w-full p-2" style={{ minHeight: 0 }}>
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={data} margin={{ top: 4, right: 8, bottom: 4, left: -16 }}>
          <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
          <XAxis dataKey="t" tick={axisStyle} stroke={gridStroke} minTickGap={40} unit="s" />
          <YAxis tick={axisStyle} stroke={gridStroke} domain={["auto", "auto"]} />
          <Tooltip {...tooltipStyle} labelFormatter={(l) => `t ${l} s`} />
          {type === "alt" ? [line("fused", X), line("baro", Y)] : [line("X", X), line("Y", Y), line("Z", Z)]}
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
