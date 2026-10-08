// Value tiles and X/Y/Z readouts shared by the dashboard panels.

/** toFixed without the "-0.0" that a tiny negative value produces. */
export function fixed(v: number, decimals: number): string {
  const s = v.toFixed(decimals);
  return /^-0(\.0+)?$/.test(s) ? s.slice(1) : s;
}

export function Metric({
  label,
  value,
  unit,
  decimals = 1,
  color = "#f3dfb0",
  size = "md",
}: {
  label: string;
  value: number;
  unit: string;
  decimals?: number;
  color?: string;
  size?: "sm" | "md" | "lg";
}) {
  const textSize =
    size === "lg"
      ? "text-4xl xl:text-5xl"
      : size === "md"
        ? "text-3xl xl:text-4xl"
        : "text-xl xl:text-2xl";
  return (
    <div className="flex flex-col items-center justify-center gap-0.5 text-center">
      <span className="text-[10px] font-semibold tracking-[.14em] text-ink-2 uppercase">
        {label}
      </span>
      <span
        className={`font-mono font-bold tabular-nums leading-none ${textSize}`}
        style={{ color }}
      >
        {fixed(value, decimals)}
      </span>
      <span className="text-[11px] text-ink-3">{unit}</span>
    </div>
  );
}

export function AxisValue({
  axis,
  value,
  color,
  unit,
  decimals = 2,
}: {
  axis: string;
  value: number;
  color: string;
  unit: string;
  decimals?: number;
}) {
  return (
    <div className="grid grid-cols-[auto_1fr_auto] items-baseline gap-x-3">
      <span className="flex items-center gap-2 text-xs font-semibold text-ink-2">
        <i className="inline-block h-2.5 w-2.5 rounded-xs" style={{ backgroundColor: color }} />
        {axis}
      </span>
      <span className="text-right font-mono text-2xl font-bold tabular-nums xl:text-3xl" style={{ color }}>
        {fixed(value, decimals)}
      </span>
      <span className="text-xs text-ink-3">{unit}</span>
    </div>
  );
}

export function Flag({ label, on, warn = false }: { label: string; on: boolean; warn?: boolean }) {
  return <span className={`flag ${warn ? "warn" : ""} ${on ? "on" : ""}`}>{label}</span>;
}
