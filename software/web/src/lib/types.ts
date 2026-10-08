export type Vec3 = [number, number, number];
/** Quaternion as (w, x, y, z), body -> NED. */
export type Quat = [number, number, number, number];
export type NoseAxis = "+X" | "-X" | "+Y" | "-Y" | "+Z" | "-Z";
export const NOSE_AXES: NoseAxis[] = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];

/** Athena_State, 96 bytes (MPU, 20 Hz). */
export interface AthenaState {
  t_us: number;
  q: Quat;
  pos: Vec3; // NED, m from pad
  vel: Vec3; // NED, m/s
  acc: Vec3; // body specific force, m/s^2
  gyro: Vec3; // body rate, rad/s
  mag: Vec3; // gauss
  baro_alt: number;
  origin_lat: number;
  origin_lon: number;
  imu_mask: number;
  flags: number;
  loop_hz: number;
}

/** Athena_GpsFix, 44 bytes (TPU). */
export interface GpsFix {
  itow: number;
  fix: number;
  sv: number;
  ok: number;
  lat: number;
  lon: number;
  hmsl: number;
  vel: Vec3; // NED, m/s
  hacc: number;
  vacc: number;
  sacc: number;
}

/** Athena_Telemetry, 38 bytes (TPU -> ground). */
export interface Telemetry {
  t_ms: number;
  lat: number;
  lon: number;
  alt: number;
  baro_alt: number;
  vel: Vec3;
  q: Quat;
  fix: number;
  sv: number;
  flags: number;
  imu_mask: number;
}

/** Athena_SpuStatus, 44 bytes (SPU -> MPU -> TPU, 2 Hz). */
export interface SpuStatus {
  t_ms: number;
  vbat_mv: number;
  vsys_mv: number;
  vbus_mv: number;
  ibat_ma: number; // >0 charging
  iin_ma: number;
  chg_status: number; // BQ25713 ChargerStatus, 0x21 high byte
  main_alt_m: number;
  phase: number; // SPU_PHASE index
  flags: number; // SPU_FLAG bits
  pyro_fired: number;
  pyro_on: number;
  pd_mode: number; // 0 none, 1 PTCH, 2 APP, 3 BOOT
  pd_status: number;
  servo_us: number[];
  apogee_m: number;
  vmax_ms: number;
}

/** A point on the ground track. `dr` = dead-reckoned (GPS was not fresh when the filter produced it). */
export interface TrackPoint {
  lat: number;
  lon: number;
  alt: number;
  t: number;
  dr: boolean;
}

/** A flight event for the timeline (derived from SPU phases, pyro bits and the MPU launch flag). */
export interface FlightEvent {
  when: string; // wall clock HH:MM:SS
  t: number; // seconds since the dashboard saw the first frame
  label: string;
  detail: string;
  color: string;
}

/** One chart sample: acc in g, gyro in deg/s, altitudes in m. */
export interface Sample {
  t: number;
  acc?: Vec3;
  gyro?: Vec3;
  alt: number;
  baro: number;
}

export interface LinkStats {
  ok: number;
  bad: number;
}

export interface WSStatusMessage {
  type: "status";
  viewers: number;
  adminConnected: boolean;
}

export interface WSAuthResult {
  type: "auth_result";
  success: boolean;
}

export interface WSAdminDisconnected {
  type: "admin_disconnected";
}

export type WSMessage = WSStatusMessage | WSAuthResult | WSAdminDisconnected;
