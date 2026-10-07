/*
 * fusion.h - Athena navigation filter (runs on the MPU, pure C, host-testable)
 *
 *  Attitude : Mahony complementary filter on the gyro, corrected by the
 *             accelerometer (only when it is measuring ~1 g, i.e. on the pad
 *             or hanging under a chute) and by the magnetometer (yaw).
 *  Position : three independent linear Kalman filters, one per NED axis,
 *             state [pos, vel, accel_bias]. The IMU drives the prediction
 *             (dead reckoning); barometer corrects Down, GPS corrects all
 *             three axes + velocity when a fix is fresh. With GPS lost the
 *             filter keeps propagating on IMU + baro.
 *
 *  Frame: NED, origin at the launch pad, z positive DOWN (altitude = -pos[2]).
 *  Units: SI (m, m/s, m/s^2, rad/s, Pa, gauss).
 */
#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>
#include "athena_link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float acc_noise;        /* m/s^2        accelerometer noise + vibration driving the KF */
    float acc_bias_walk;    /* m/s^2/sqrt(s) how fast the accel bias state may wander */
    float baro_noise;       /* m            barometer altitude std dev */
    float gps_pos_min;      /* m            floor applied to GPS hAcc/vAcc */
    float gps_vel_min;      /* m/s          floor applied to GPS sAcc */
    float zupt_noise;       /* m/s          zero-velocity "measurement" std dev while on the pad */
    float mahony_kp_pad;    /* accel correction gain on the pad */
    float mahony_ki_pad;    /* gyro-bias integrator gain on the pad */
    float mahony_kp_flight; /* accel correction gain in flight (small: accel != gravity) */
    float mahony_kp_mag;    /* magnetometer yaw correction gain */
    float launch_acc;       /* m/s^2        kinematic acceleration that declares launch */
    float launch_alt;       /* m            baro altitude above pad that also declares launch */
    float still_acc;        /* m/s^2        kinematic accel below which the pad ZUPT runs */
    float still_gyro;       /* rad/s        gyro rate below which the pad ZUPT runs */
    float gps_latency_s;    /* s            NAV-PVT age when it reaches the filter */
    float mag_decl_rad;     /* rad          magnetic declination, east positive */
    float mag_offset[3];    /* gauss        hard-iron offsets, subtracted from raw mag */
    float gps_timeout_s;    /* s            after this a fix is "stale" -> dead reckoning only */
} Fusion_Params;

typedef struct {
    Fusion_Params p;
    /* attitude */
    float q[4];             /* body -> NED, (w,x,y,z) */
    float ierr[3];          /* Mahony integral term == -gyro bias */
    int   have_att, init_wait;
    /* position KF, per axis N,E,D */
    float x[3][3];          /* [pos, vel, abias] */
    float P[3][3][3];
    /* references */
    int    have_origin, have_baro0;
    double lat0, lon0;      /* deg */
    float  h0;              /* m MSL */
    float  baro_alt0;       /* m, ISA altitude of the pad */
    float  baro_alt;        /* m above pad, last baro sample */
    /* bookkeeping */
    float    acc_body[3], gyro_body[3], mag_body[3], acc_ned[3];
    int      in_flight, mag_ok, baro_ok;
    uint32_t t_us, last_gps_us, last_baro_us, last_mag_us;
} Fusion;

void  Fusion_DefaultParams(Fusion_Params *p);
void  Fusion_Init(Fusion *f, const Fusion_Params *p);   /* p may be NULL for defaults */

/* Call at the IMU rate. acc = specific force (m/s^2), gyro = rad/s, dt = seconds since last call. */
void  Fusion_Imu (Fusion *f, const float acc[3], const float gyro[3], float dt, uint32_t t_us);
void  Fusion_Mag (Fusion *f, const float mag_gauss[3], uint32_t t_us);
void  Fusion_Baro(Fusion *f, float pressure_pa, uint32_t t_us);
void  Fusion_Gps (Fusion *f, const Athena_GpsFix *g, uint32_t t_us);

void  Fusion_GetState(const Fusion *f, Athena_State *s, uint8_t imu_mask, uint16_t loop_hz);

/* helpers (exposed for tests and for the IMU voter) */
float Fusion_PressureToAlt(float pa);
float Fusion_Median3(float a, float b, float c);
void  Fusion_QuatToEuler(const float q[4], float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif
#endif
