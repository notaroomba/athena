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
