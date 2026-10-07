#include "fusion.h"
#include <math.h>
#include <string.h>

#define G0        9.80665f
#define DEG2RAD   0.017453292519943295f
#define R_EARTH   6371000.0

/* ------------------------------------------------------------------ math */

static float vnorm3(const float v[3]) { return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }

static void vcross(const float a[3], const float b[3], float o[3])
{
    o[0] = a[1]*b[2] - a[2]*b[1];
    o[1] = a[2]*b[0] - a[0]*b[2];
    o[2] = a[0]*b[1] - a[1]*b[0];
}

static int vunit(float v[3])
{
    float n = vnorm3(v);
    if (n < 1e-6f) return -1;
    v[0] /= n; v[1] /= n; v[2] /= n;
    return 0;
}

static void qnorm(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    for (int i = 0; i < 4; i++) q[i] /= n;
}

/* rotate body vector into NED with q (body->NED) */
static void qrot(const float q[4], const float v[3], float o[3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    o[0] = (1 - 2*(y*y + z*z))*v[0] + 2*(x*y - w*z)*v[1] + 2*(x*z + w*y)*v[2];
    o[1] = 2*(x*y + w*z)*v[0] + (1 - 2*(x*x + z*z))*v[1] + 2*(y*z - w*x)*v[2];
    o[2] = 2*(x*z - w*y)*v[0] + 2*(y*z + w*x)*v[1] + (1 - 2*(x*x + y*y))*v[2];
}

/* rotate NED vector into body (inverse rotation) */
static void qrot_inv(const float q[4], const float v[3], float o[3])
{
    float qc[4] = { q[0], -q[1], -q[2], -q[3] };
    qrot(qc, v, o);
}

/* quaternion from rotation matrix rows (R maps body->NED), Shepperd's method */
static void quat_from_rows(const float n[3], const float e[3], const float d[3], float q[4])
{
    float m[3][3] = { { n[0], n[1], n[2] }, { e[0], e[1], e[2] }, { d[0], d[1], d[2] } };
    float tr = m[0][0] + m[1][1] + m[2][2];
    if (tr > 0) {
        float s = sqrtf(tr + 1.f) * 2.f;
        q[0] = 0.25f * s;
        q[1] = (m[2][1] - m[1][2]) / s;
        q[2] = (m[0][2] - m[2][0]) / s;
        q[3] = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        float s = sqrtf(1.f + m[0][0] - m[1][1] - m[2][2]) * 2.f;
        q[0] = (m[2][1] - m[1][2]) / s; q[1] = 0.25f * s;
        q[2] = (m[0][1] + m[1][0]) / s; q[3] = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        float s = sqrtf(1.f + m[1][1] - m[0][0] - m[2][2]) * 2.f;
        q[0] = (m[0][2] - m[2][0]) / s; q[1] = (m[0][1] + m[1][0]) / s;
        q[2] = 0.25f * s;               q[3] = (m[1][2] + m[2][1]) / s;
    } else {
        float s = sqrtf(1.f + m[2][2] - m[0][0] - m[1][1]) * 2.f;
        q[0] = (m[1][0] - m[0][1]) / s; q[1] = (m[0][2] + m[2][0]) / s;
        q[2] = (m[1][2] + m[2][1]) / s; q[3] = 0.25f * s;
    }
    qnorm(q);
}

void Fusion_QuatToEuler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    *roll  = atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y));
    float s = 2*(w*y - z*x);
    if (s > 1) s = 1;
    if (s < -1) s = -1;
    *pitch = asinf(s);
    *yaw   = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

float Fusion_PressureToAlt(float pa)
{
    /* ISA troposphere, 101325 Pa reference. Only differences matter (pad-relative). */
    return 44330.77f * (1.f - powf(pa / 101325.f, 0.190263f));
}

float Fusion_Median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;
}

/* --------------------------------------------------------- per-axis KF */

static void kf_init_axis(Fusion *f, int i)
{
    memset(f->x[i], 0, sizeof(f->x[i]));
    memset(f->P[i], 0, sizeof(f->P[i]));
    f->P[i][0][0] = 1.f;      /* 1 m */
    f->P[i][1][1] = 1.f;      /* 1 m/s */
    f->P[i][2][2] = 0.25f;    /* 0.5 m/s^2 bias */
}

static void kf_predict(Fusion *f, int i, float u, float dt)
{
    float *x = f->x[i];
    float (*P)[3] = f->P[i];
    float a = u - x[2];
    /* x = F x + B u */
    x[0] += x[1]*dt + 0.5f*a*dt*dt;
    x[1] += a*dt;
    /* F = [1 dt -dt^2/2; 0 1 -dt; 0 0 1] ;  P = F P F' + Q */
    float F[3][3] = { {1, dt, -0.5f*dt*dt}, {0, 1, -dt}, {0, 0, 1} };
    float FP[3][3], N[3][3];
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) {
        FP[r][c] = F[r][0]*P[0][c] + F[r][1]*P[1][c] + F[r][2]*P[2][c];
    }
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) {
        N[r][c] = FP[r][0]*F[c][0] + FP[r][1]*F[c][1] + FP[r][2]*F[c][2];
    }
    float sa2 = f->p.acc_noise * f->p.acc_noise;
    float sb2 = f->p.acc_bias_walk * f->p.acc_bias_walk;
    N[0][0] += sa2 * 0.25f*dt*dt*dt*dt;
    N[0][1] += sa2 * 0.5f*dt*dt*dt;  N[1][0] = N[0][1];
    N[1][1] += sa2 * dt*dt;
    N[2][2] += sb2 * dt;
    memcpy(P, N, sizeof(N));
}

/* scalar measurement of state element h (0 pos, 1 vel) with variance r */
static void kf_update(Fusion *f, int i, int h, float z, float r)
{
    float *x = f->x[i];
    float (*P)[3] = f->P[i];
    float S = P[h][h] + r;
    if (S <= 0) return;
    float K[3] = { P[0][h]/S, P[1][h]/S, P[2][h]/S };
    float y = z - x[h];
    for (int k = 0; k < 3; k++) x[k] += K[k]*y;
    float Ph[3] = { P[h][0], P[h][1], P[h][2] };
    for (int rr = 0; rr < 3; rr++) for (int c = 0; c < 3; c++) P[rr][c] -= K[rr]*Ph[c];
}

/* ------------------------------------------------------------- public */

void Fusion_DefaultParams(Fusion_Params *p)
{
    memset(p, 0, sizeof(*p));
    p->acc_noise        = 0.5f;    /* vibration-heavy airframe: raise to 1-2 */
    p->acc_bias_walk    = 0.02f;
    p->baro_noise       = 1.0f;
    p->gps_pos_min      = 2.0f;
    p->gps_vel_min      = 0.3f;
    p->zupt_noise       = 0.05f;
    p->mahony_kp_pad    = 2.0f;
    p->mahony_ki_pad    = 0.3f;    /* bias settles in ~5 s of sitting still */
    p->mahony_kp_flight = 0.05f;
    p->mahony_kp_mag    = 0.5f;
    p->launch_acc       = 1.5f * G0;  /* kinematic (gravity removed): fires for thrust/weight > 2.5 */
    p->launch_alt       = 10.0f;      /* ...or when the barometer says we are clearly off the pad */
    p->still_acc        = 1.0f;       /* m/s^2 kinematic accel below which the pad ZUPT may run */
    p->still_gyro       = 0.05f;      /* rad/s */
    p->gps_latency_s    = 0.15f;      /* NAV-PVT age at the filter: nav epoch + UART relay */
    p->mag_decl_rad     = 0.f;     /* ponytail: set per launch site, or yaw is magnetic not true */
    p->gps_timeout_s    = 1.5f;
}

void Fusion_Init(Fusion *f, const Fusion_Params *p)
{
    memset(f, 0, sizeof(*f));
    if (p) f->p = *p; else Fusion_DefaultParams(&f->p);
    f->q[0] = 1.f;
    for (int i = 0; i < 3; i++) kf_init_axis(f, i);
}

/* Build the initial attitude from "where is down" (accel at rest) and
 * "where is north" (mag, if any). Without mag, yaw is arbitrary (body x). */
static void att_init(Fusion *f, const float acc[3])
{
    float d[3] = { -acc[0], -acc[1], -acc[2] };        /* gravity direction in body */
    if (vunit(d)) return;
    float n[3], e[3];
    if (f->mag_ok) {
        float m[3] = { f->mag_body[0], f->mag_body[1], f->mag_body[2] };
        vcross(d, m, e);                                /* east = down x mag */
        if (vunit(e)) goto nomag;
        vcross(e, d, n);                                /* north = east x down */
    } else {
    nomag:
        { float bx[3] = {1, 0, 0}; vcross(d, bx, e); if (vunit(e)) { float by[3] = {0, 1, 0}; vcross(d, by, e); vunit(e); } vcross(e, d, n); }
    }
    quat_from_rows(n, e, d, f->q);
    f->have_att = 1;
}

void Fusion_Imu(Fusion *f, const float acc[3], const float gyro[3], float dt, uint32_t t_us)
{
    if (dt <= 0.f || dt > 0.1f) dt = 0.0025f;          /* glitch guard */
    f->t_us = t_us;
    memcpy(f->acc_body, acc, sizeof(f->acc_body));

    if (!f->have_att) {
        if (!f->mag_ok && f->init_wait < 200) { f->init_wait++; return; }   /* give the mag ~0.5 s to show up */
        att_init(f, acc);
        if (!f->have_att) return;
    }

    /* ---- Mahony attitude ---- */
    float w[3] = { gyro[0] + f->ierr[0], gyro[1] + f->ierr[1], gyro[2] + f->ierr[2] };
    float e[3] = { 0, 0, 0 };
    float anorm = vnorm3(acc);
    int acc_ok = anorm > 0.8f*G0 && anorm < 1.2f*G0;     /* only trust accel when it is ~gravity */
    float kp = f->in_flight ? f->p.mahony_kp_flight : f->p.mahony_kp_pad;
    if (acc_ok) {
        float a[3] = { acc[0]/anorm, acc[1]/anorm, acc[2]/anorm };
        float up_ned[3] = { 0, 0, -1 }, up_b[3];
        qrot_inv(f->q, up_ned, up_b);                   /* where the filter thinks "up" is */
        float ea[3]; vcross(a, up_b, ea);
        for (int i = 0; i < 3; i++) e[i] += kp * ea[i];
        if (!f->in_flight)                              /* learn gyro bias only when still */
            for (int i = 0; i < 3; i++) f->ierr[i] += f->p.mahony_ki_pad * ea[i] * dt;
    }
    if (f->mag_ok && (t_us - f->last_mag_us) < 500000u) {
        /* Yaw-only correction: compare the HORIZONTAL part of the measured field
         * (rotated into NED) with magnetic north, and apply the resulting error as a
         * rotation about the NED vertical only. Using the full 3-D cross product here
         * would also torque roll/pitch wherever the field has a vertical component,
         * which is everywhere except the equator. */
        float mn[3]; qrot(f->q, f->mag_body, mn);
        float hh = sqrtf(mn[0]*mn[0] + mn[1]*mn[1]);
        if (hh > 0.02f) {
            float cz = (mn[0]*sinf(f->p.mag_decl_rad) - mn[1]*cosf(f->p.mag_decl_rad)) / hh; /* (m_h x ref_h).z */
            float e_ned[3] = { 0, 0, cz }, em[3];
            qrot_inv(f->q, e_ned, em);
            for (int i = 0; i < 3; i++) e[i] += f->p.mahony_kp_mag * em[i];
            if (!f->in_flight)                          /* yaw-axis gyro bias is only observable from mag */
                for (int i = 0; i < 3; i++) f->ierr[i] += f->p.mahony_ki_pad * em[i] * dt;
        }
    }
    for (int i = 0; i < 3; i++) w[i] += e[i];
    memcpy(f->gyro_body, w, sizeof(f->gyro_body));
    {   /* q += 0.5 * q (x) (0,w) * dt */
        float q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];
        f->q[0] += 0.5f*dt*(-q1*w[0] - q2*w[1] - q3*w[2]);
        f->q[1] += 0.5f*dt*( q0*w[0] + q2*w[2] - q3*w[1]);
        f->q[2] += 0.5f*dt*( q0*w[1] - q1*w[2] + q3*w[0]);
        f->q[3] += 0.5f*dt*( q0*w[2] + q1*w[1] - q2*w[0]);
        qnorm(f->q);
    }

    /* ---- kinematic acceleration in NED, launch detect ---- */
    qrot(f->q, acc, f->acc_ned);
    f->acc_ned[2] += G0;                                /* remove gravity (NED: g points +D) */
    float a_kin = vnorm3(f->acc_ned);
    if (!f->in_flight && (a_kin > f->p.launch_acc || f->baro_alt > f->p.launch_alt)) f->in_flight = 1;

    /* ---- position KF predict (dead reckoning step) ---- */
    for (int i = 0; i < 3; i++) kf_predict(f, i, f->acc_ned[i], dt);

    /* On the pad we know pos = 0 and vel = 0: these pseudo-measurements are what
     * calibrates the accel bias states before launch. They are gated on measured
     * stillness, not just on !in_flight, so the thrust ramp before the launch
     * threshold trips is not absorbed into the bias states. */
    int still = a_kin < f->p.still_acc && vnorm3(gyro) < f->p.still_gyro;
    if (!f->in_flight && still) {
        float rv = f->p.zupt_noise * f->p.zupt_noise;
        for (int i = 0; i < 3; i++) { kf_update(f, i, 1, 0.f, rv); kf_update(f, i, 0, 0.f, 0.25f); }
    }
}

void Fusion_Mag(Fusion *f, const float mag[3], uint32_t t_us)
{
    for (int i = 0; i < 3; i++) f->mag_body[i] = mag[i] - f->p.mag_offset[i];
    f->mag_ok = vnorm3(f->mag_body) > 0.05f;            /* sane Earth field: 0.25-0.65 G */
    f->last_mag_us = t_us;
}

void Fusion_Baro(Fusion *f, float pressure_pa, uint32_t t_us)
{
    if (pressure_pa < 1000.f || pressure_pa > 120000.f) return;
    float alt = Fusion_PressureToAlt(pressure_pa);
    if (!f->have_baro0) { f->baro_alt0 = alt; f->have_baro0 = 1; }
    else if (!f->in_flight) f->baro_alt0 += 0.02f * (alt - f->baro_alt0);  /* track pad pressure drift */
    f->baro_alt = alt - f->baro_alt0;
    f->baro_ok = 1;
    f->last_baro_us = t_us;
    if (f->in_flight)
        kf_update(f, 2, 0, -f->baro_alt, f->p.baro_noise * f->p.baro_noise);
}

static float a_norm(const Fusion *f) { return vnorm3(f->acc_ned); }

void Fusion_Gps(Fusion *f, const Athena_GpsFix *g, uint32_t t_us)
{
    if (g->fix_type < 3 || !(g->flags & 0x01)) return;  /* need a 3D fix flagged OK */
    double lat = g->lat_1e7 * 1e-7, lon = g->lon_1e7 * 1e-7;
    float  h   = g->h_msl_mm * 1e-3f;
    f->last_gps_us = t_us;

    if (!f->have_origin) { f->lat0 = lat; f->lon0 = lon; f->h0 = h; f->have_origin = 1; }
    if (!f->in_flight) {                                /* average the pad position while waiting */
        f->lat0 += 0.05 * (lat - f->lat0);
        f->lon0 += 0.05 * (lon - f->lon0);
        f->h0   += 0.05f * (h - f->h0);
        return;                                         /* ZUPT already pins pos/vel to 0 */
    }
    float n = (float)((lat - f->lat0) * DEG2RAD * R_EARTH);
    float e = (float)((lon - f->lon0) * DEG2RAD * R_EARTH * cos(f->lat0 * DEG2RAD));
    float d = -(h - f->h0);
    float hs = g->h_acc_mm * 1e-3f, vs = g->v_acc_mm * 1e-3f, ss = g->s_acc_mms * 1e-3f;
    if (hs < f->p.gps_pos_min) hs = f->p.gps_pos_min;
    if (vs < f->p.gps_pos_min) vs = f->p.gps_pos_min;
    if (ss < f->p.gps_vel_min) ss = f->p.gps_vel_min;
    /* The fix describes where the rocket was ~gps_latency_s ago. Instead of back-dating
     * the state, widen each measurement by how far the vehicle moved in that time
     * (speed x latency for position, acceleration x latency for velocity): on the pad
     * this adds nothing, in boost the GPS is trusted much less. */
    float tau = f->p.gps_latency_s;
    float pos_lag[3], vel_lag = a_norm(f) * tau;
    for (int i = 0; i < 3; i++) pos_lag[i] = f->x[i][1] * tau;
    kf_update(f, 0, 0, n, hs*hs + pos_lag[0]*pos_lag[0]);
    kf_update(f, 1, 0, e, hs*hs + pos_lag[1]*pos_lag[1]);
    kf_update(f, 2, 0, d, vs*vs + pos_lag[2]*pos_lag[2]);
    for (int i = 0; i < 3; i++) kf_update(f, i, 1, g->vel_ned_mms[i] * 1e-3f, ss*ss + vel_lag*vel_lag);
}

void Fusion_GetState(const Fusion *f, Athena_State *s, uint8_t imu_mask, uint16_t loop_hz)
{
    memset(s, 0, sizeof(*s));
    s->t_us = f->t_us;
    memcpy(s->q, f->q, sizeof(s->q));
    for (int i = 0; i < 3; i++) { s->pos_ned[i] = f->x[i][0]; s->vel_ned[i] = f->x[i][1]; }
    memcpy(s->acc_body,  f->acc_body,  sizeof(s->acc_body));
    memcpy(s->gyro_body, f->gyro_body, sizeof(s->gyro_body));
    memcpy(s->mag_body,  f->mag_body,  sizeof(s->mag_body));
    s->baro_alt = f->baro_alt;
    s->origin_lat_1e7 = (int32_t)(f->lat0 * 1e7);
    s->origin_lon_1e7 = (int32_t)(f->lon0 * 1e7);
    s->imu_mask = imu_mask;
    s->loop_hz  = loop_hz;
    /* signed: a fix stamped after the last IMU sample (normal) must not read as "stale" */
    int32_t age = (int32_t)(f->t_us - f->last_gps_us);
    s->flags = (f->in_flight ? STATE_FLAG_IN_FLIGHT : 0)
             | ((f->last_gps_us && age < (int32_t)(f->p.gps_timeout_s * 1e6f)) ? STATE_FLAG_GPS_FRESH : 0)
             | (f->baro_ok ? STATE_FLAG_BARO_OK : 0)
             | (f->mag_ok ? STATE_FLAG_MAG_OK : 0)
             | (f->have_origin ? STATE_FLAG_ORIGIN_OK : 0);
}
