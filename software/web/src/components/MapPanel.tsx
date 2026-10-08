import { useEffect, useMemo, useRef, useState } from "react";
import { MapContainer, TileLayer, Polyline, CircleMarker, Circle, useMap } from "react-leaflet";
import type { LatLngBoundsExpression, LatLngExpression } from "leaflet";
import "leaflet/dist/leaflet.css";
import type { TrackPoint } from "@/lib/types";

export interface MapStat {
  label: string;
  value: string;
}

interface MapPanelProps {
  fused: TrackPoint[]; // filter estimate (dead-reckoned segments flagged)
  gps: TrackPoint[]; // raw receiver fixes
  pad: [number, number] | null;
  cur: { lat: number; lon: number; dr: boolean } | null;
  hacc: number; // m, 0 = unknown
  landing: [number, number] | null; // ballistic/drift extrapolation of the current descent
  stats: MapStat[];
}

const CREAM = "#f3dfb0",
  ORANGE = "#f2552a",
  TEAL = "#1f9aa8",
  AMBER = "#f29a5a";

/** First position: zoom in. Afterwards keep the current position centred while `follow` is on. */
function Follow({ target, follow, fitTo }: { target: LatLngExpression | null; follow: boolean; fitTo: LatLngBoundsExpression | null }) {
  const map = useMap();
  const zoomed = useRef(false);
  useEffect(() => {
    if (!target) return;
    if (!zoomed.current) {
      zoomed.current = true;
      map.setView(target, 16);
    } else if (follow) map.panTo(target, { animate: false });
  }, [map, target, follow]);
  useEffect(() => {
    if (fitTo) map.fitBounds(fitTo, { padding: [20, 20], animate: false });
  }, [map, fitTo]);
  return null;
}

/** Splits the fused track into runs with the same dead-reckoning flag so DR stretches can be dashed. */
function segments(track: TrackPoint[]): { pts: LatLngExpression[]; dr: boolean }[] {
  const out: { pts: LatLngExpression[]; dr: boolean }[] = [];
  for (const p of track) {
    const last = out[out.length - 1];
    if (!last || last.dr !== p.dr) {
      const start: LatLngExpression[] = last ? [last.pts[last.pts.length - 1]] : [];
      out.push({ pts: [...start, [p.lat, p.lon]], dr: p.dr });
    } else last.pts.push([p.lat, p.lon]);
  }
  return out;
}

function gpx(fused: TrackPoint[], gps: TrackPoint[], landing: [number, number] | null): string {
  const pt = (p: TrackPoint) => `<trkpt lat="${p.lat.toFixed(7)}" lon="${p.lon.toFixed(7)}"><ele>${p.alt.toFixed(1)}</ele></trkpt>`;
  const seg = (name: string, pts: TrackPoint[]) => (pts.length ? `<trk><name>${name}</name><trkseg>${pts.map(pt).join("")}</trkseg></trk>` : "");
  const last = fused[fused.length - 1];
  const wpt = (name: string, lat: number, lon: number) => `<wpt lat="${lat.toFixed(7)}" lon="${lon.toFixed(7)}"><name>${name}</name></wpt>`;
  return `<?xml version="1.0" encoding="UTF-8"?><gpx version="1.1" creator="Athena dashboard" xmlns="http://www.topografix.com/GPX/1/1">${
    fused.length ? wpt("pad", fused[0].lat, fused[0].lon) + wpt("last position", last.lat, last.lon) : ""
  }${landing ? wpt("landing estimate", landing[0], landing[1]) : ""}${seg("filter", fused)}${seg("gps", gps)}</gpx>`;
}

function download(name: string, text: string, type: string) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 10000);
}

export default function MapPanel({ fused, gps, pad, cur, hacc, landing, stats }: MapPanelProps) {
  const [follow, setFollow] = useState(true);
  const [fitKey, setFitKey] = useState(0);
  const [copied, setCopied] = useState(false);
  const here = cur ?? (landing ? { lat: landing[0], lon: landing[1], dr: true } : null);
  const coords = here ? `${here.lat.toFixed(6)}, ${here.lon.toFixed(6)}` : "";
  const mapsUrl = here ? `https://www.google.com/maps/search/?api=1&query=${here.lat.toFixed(6)},${here.lon.toFixed(6)}` : "";
  const copy = async () => {
    try {
      await navigator.clipboard.writeText(coords);
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    } catch {
      /* clipboard blocked */
    }
  };
  const segs = useMemo(() => segments(fused), [fused]);
  const gpsLine = useMemo<LatLngExpression[]>(() => gps.map((p) => [p.lat, p.lon]), [gps]);
  const target = useMemo<LatLngExpression | null>(() => (cur ? [cur.lat, cur.lon] : pad), [cur, pad]);
  const fitTo = useMemo<LatLngBoundsExpression | null>(() => {
    if (!fitKey) return null;
    const pts: [number, number][] = [...fused.map((p) => [p.lat, p.lon] as [number, number]), ...gps.map((p) => [p.lat, p.lon] as [number, number])];
    if (pad) pts.push(pad);
    if (landing) pts.push(landing);
    return pts.length >= 2 ? (pts as LatLngBoundsExpression) : null;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [fitKey]);

  return (
    <div className="flex h-full flex-col">
      <div className="flex items-center justify-between px-4 pt-3">
        <h2 className="panel-title">
          Ground track
          <small>{cur ? (cur.dr ? "dead reckoning" : "GPS + filter") : "waiting for a position"}</small>
        </h2>
        <div className="flex gap-1">
          <button className={`btn ${follow ? "active" : ""}`} style={{ padding: "3px 8px" }} onClick={() => setFollow(!follow)}>
            FOLLOW
          </button>
          <button className="btn" style={{ padding: "3px 8px" }} onClick={() => setFitKey((k) => k + 1)} disabled={fused.length + gps.length < 2}>
            FIT
          </button>
          <button className="btn" style={{ padding: "3px 8px" }} onClick={copy} disabled={!here} title={coords ? `copy ${coords}` : "no position yet"}>
            {copied ? "COPIED" : "COPY"}
          </button>
          <a className={`btn ${here ? "" : "pointer-events-none opacity-35"}`} style={{ padding: "3px 8px" }} href={mapsUrl || "#"} target="_blank" rel="noreferrer" title="open the last position in Google Maps (walk to the rocket)">
            MAPS
          </a>
          <button
            className="btn"
            style={{ padding: "3px 8px" }}
            onClick={() => download(`athena-track-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-")}.gpx`, gpx(fused, gps, landing), "application/gpx+xml")}
            disabled={!fused.length && !gps.length}
            title="download the tracks and landing estimate as GPX (Google Earth, phone map apps)"
          >
            GPX
          </button>
        </div>
      </div>
      <div className="relative mt-2 min-h-0 flex-1">
        <MapContainer center={[0, 0]} zoom={2} zoomControl={false} attributionControl={true} className="h-full w-full" style={{ background: "#0e0e10" }}>
          <TileLayer
            url="https://tile.openstreetmap.org/{z}/{x}/{y}.png"
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
            className="dark-tiles"
            maxZoom={19}
          />
          <Follow target={target} follow={follow} fitTo={fitTo} />
          {gpsLine.length > 1 && <Polyline positions={gpsLine} pathOptions={{ color: TEAL, weight: 2, dashArray: "2 6", opacity: 0.8 }} />}
          {segs.map((s, i) => (
            <Polyline key={i} positions={s.pts} pathOptions={{ color: s.dr ? AMBER : CREAM, weight: 2.5, dashArray: s.dr ? "6 6" : undefined }} />
          ))}
          {pad && <CircleMarker center={pad} radius={6} pathOptions={{ color: CREAM, fillColor: "#0c0c0d", fillOpacity: 1, weight: 2 }} />}
          {cur && hacc > 0 && <Circle center={[cur.lat, cur.lon]} radius={hacc} pathOptions={{ color: TEAL, weight: 1, fillOpacity: 0.08 }} />}
          {cur && landing && <Polyline positions={[[cur.lat, cur.lon], landing]} pathOptions={{ color: ORANGE, weight: 1.5, dashArray: "2 4" }} />}
          {landing && <CircleMarker center={landing} radius={5} pathOptions={{ color: ORANGE, fillColor: ORANGE, fillOpacity: 0.6, weight: 1.5 }} />}
          {cur && (
            <CircleMarker center={[cur.lat, cur.lon]} radius={7} pathOptions={{ color: cur.dr ? AMBER : ORANGE, fillColor: cur.dr ? AMBER : ORANGE, fillOpacity: 0.9, weight: 2 }} />
          )}
        </MapContainer>
        {!cur && !pad && (
          <div className="pointer-events-none absolute inset-0 flex items-center justify-center">
            <span className="rounded bg-[#0c0c0dcc] px-3 py-1 text-xs tracking-wider text-ink-3">NO POSITION YET</span>
          </div>
        )}
      </div>
      <div className="grid grid-cols-3 gap-x-3 gap-y-1 px-4 py-2 text-[10px] md:grid-cols-6 xl:grid-cols-3">
        {stats.map((s) => (
          <div key={s.label} className="flex flex-col">
            <span className="tracking-[.12em] text-ink-3 uppercase">{s.label}</span>
            <span className="font-mono text-xs text-ink tabular-nums">{s.value}</span>
          </div>
        ))}
      </div>
      <div className="flex gap-3 px-4 pb-2 text-[10px] text-ink-3">
        <span>
          <i className="inline-block h-0.5 w-4 align-middle" style={{ background: CREAM }} /> filter
        </span>
        <span>
          <i className="inline-block h-0.5 w-4 border-t border-dashed align-middle" style={{ borderColor: AMBER }} /> dead reckoning
        </span>
        <span>
          <i className="inline-block h-0.5 w-4 border-t border-dotted align-middle" style={{ borderColor: TEAL }} /> raw GPS
        </span>
        <span>
          <i className="inline-block h-2 w-2 rounded-full align-middle" style={{ background: ORANGE }} /> landing estimate
        </span>
      </div>
    </div>
  );
}
