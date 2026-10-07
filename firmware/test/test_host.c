/*
 * Host self-check for the pure-C Athena modules (no HAL).
 *   cc -std=c99 -Wall -I Athena test/test_host.c Athena/athena_link.c Athena/ubx.c Athena/fusion.c -lm
 * Fails loudly (assert) if the link framing, UBX parsing or the filter break.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "athena_link.h"
#include "ubx.h"
#include "fusion.h"

static int got_type = -1; static uint8_t got_len; static uint8_t got_buf[LINK_MAX_PAYLOAD];
static void on_pkt(uint8_t type, const uint8_t *p, uint8_t len, void *u) { (void)u; got_type = type; got_len = len; memcpy(got_buf, p, len); }

static void test_link(void)
{
    Link l; Link_Init(&l, on_pkt, NULL);
    Athena_GpsFix g = { .itow_ms = 123456, .fix_type = 3, .num_sv = 9, .flags = 1, .lat_1e7 = 405000000, .lon_1e7 = -740000000, .h_msl_mm = 12345 };
    uint8_t frame[LINK_MAX_PAYLOAD + LINK_OVERHEAD];
    size_t n = Link_Encode(frame, LINK_PKT_GPS, &g, sizeof g);
    assert(n == sizeof g + LINK_OVERHEAD);
    /* garbage before the frame must be ignored, frame must decode */
    uint8_t junk[] = { 0x00, 0xA5, 0x02, 0xFF, 0x11 };
    for (size_t i = 0; i < sizeof junk; i++) Link_RxPush(&l, junk[i]);
    for (size_t i = 0; i < n; i++) Link_RxPush(&l, frame[i]);
    Link_Process(&l);
    assert(got_type == LINK_PKT_GPS && got_len == sizeof g && memcmp(got_buf, &g, sizeof g) == 0);
    /* a flipped bit must be rejected */
    got_type = -1; frame[10] ^= 0x01;
    for (size_t i = 0; i < n; i++) Link_FeedByte(&l, frame[i]);
    assert(got_type == -1 && l.rx_bad == 2 && l.rx_ok == 1);   /* 1 bad from junk prefix, 1 from flipped bit */
    assert(sizeof(Athena_State) == 96 && sizeof(Athena_Telemetry) == 38 && sizeof(Athena_GpsFix) == 44);
    printf("link      ok\n");
}

static void test_ubx(void)
{
    uint8_t pvt[92] = { 0 };
    pvt[20] = 3; pvt[21] = 0x01; pvt[23] = 11;
    int32_t lon = -740000000, lat = 405000000, hmsl = 50000, veld = -123000; uint32_t hacc = 2500, sacc = 400;
    memcpy(pvt + 24, &lon, 4); memcpy(pvt + 28, &lat, 4); memcpy(pvt + 36, &hmsl, 4);
    memcpy(pvt + 40, &hacc, 4); memcpy(pvt + 56, &veld, 4); memcpy(pvt + 68, &sacc, 4);
    uint8_t frame[128]; size_t n = Ubx_Frame(frame, UBX_CLASS_NAV, UBX_NAV_PVT, pvt, 92);
    Ubx u; Ubx_Init(&u); int done = 0;
    Ubx_Feed(&u, 0xFF); Ubx_Feed(&u, 0xB5); Ubx_Feed(&u, 0xFF);     /* SPI idle bytes + false sync */
    for (size_t i = 0; i < n; i++) done |= Ubx_Feed(&u, frame[i]);
    assert(done && u.cls == UBX_CLASS_NAV && u.id == UBX_NAV_PVT && u.len == 92);
    Athena_GpsFix g; assert(Ubx_ParseNavPvt(u.payload, u.len, &g) == 0);
    assert(g.lat_1e7 == lat && g.lon_1e7 == lon && g.h_msl_mm == hmsl && g.vel_ned_mms[2] == veld && g.h_acc_mm == hacc && g.s_acc_mms == sacc && g.fix_type == 3 && g.num_sv == 11);
    printf("ubx       ok\n");
}

static float frand(float s) { return s * ((float)rand() / (float)RAND_MAX - 0.5f) * 2.f; }

/* Vertical flight: 3 s boost at 50 m/s^2, coast, apogee, fall. Noisy IMU, baro, 10 Hz GPS.
 * GPS is dropped from t=8 s onward to exercise dead reckoning. */
static void test_fusion(void)
{
    Fusion f; Fusion_Init(&f, NULL);
    const float dt = 0.0025f; const float g = 9.80665f;
    float alt = 0, vel = 0, acc_bias = 0.15f;
    uint32_t t_us = 0; float t = 0;
    /* body frame: rocket nose = body -z ... keep it simple: nose along -x_body pointing UP,
       i.e. body x points down. accel at rest then reads +g on x? no: at rest accelerometer
       measures reaction = -gravity = "up"; up is body -x => acc_x = -g. */
    float max_err_alt = 0, max_err_vel = 0, alt_apogee_true = 0, alt_apogee_est = 0;
    float yaw0 = 0;
    int landed = 0;
    for (int k = 0; k < (int)(80.f / dt) && !landed; k++, t += dt, t_us += 2500) {
        float thrust = (t > 30.f && t < 33.f) ? 50.f : 0.f;   /* 30 s on the pad, then 3 s burn */
        float a_up = thrust - g;
        if (alt <= 0 && a_up < 0) { a_up = 0; vel = 0; alt = 0; }      /* sitting on the pad */
        vel += a_up * dt; alt += vel * dt;
        if (alt < 0) { alt = 0; vel = 0; }
        if (alt_apogee_true > 100.f && alt <= 0.f) landed = 1;   /* stop judging at touchdown */
        if (alt > alt_apogee_true) alt_apogee_true = alt;
        /* specific force in body: up = -x_body, kinematic a_up along up, plus gravity reaction g up */
        float acc[3] = { -(a_up + g) + acc_bias + frand(0.3f), frand(0.3f), frand(0.3f) };
        float gyr[3] = { 0.01f + frand(0.002f), 0.015f + frand(0.002f), -0.02f + frand(0.002f) };   /* biased gyros, x is the vertical axis */
        Fusion_Imu(&f, acc, gyr, dt, t_us);
        if (k % 4 == 0) { float m[3] = { -0.4f, 0.2f, 0.0f }; Fusion_Mag(&f, m, t_us); }
        if (k % 16 == 0) { float pa = 101325.f * powf(1.f - alt / 44330.77f, 5.2559f) + frand(8.f); Fusion_Baro(&f, pa, t_us); }
        if (k % 40 == 0 && t < 36.f) {                              /* GPS lost 3 s after burnout */
            Athena_GpsFix gf = { .fix_type = 3, .flags = 1, .num_sv = 10, .lat_1e7 = 405000000 + (int32_t)frand(10), .lon_1e7 = -740000000 + (int32_t)frand(10),
                                 .h_msl_mm = (int32_t)((100.f + alt + frand(2.f)) * 1000), .h_acc_mm = 2000, .v_acc_mm = 3000, .s_acc_mms = 300 };
            gf.vel_ned_mms[2] = (int32_t)((-vel + frand(0.3f)) * 1000);
            Fusion_Gps(&f, &gf, t_us);
        }
        if (t == 0) { float r, p; Fusion_QuatToEuler(f.q, &r, &p, &yaw0); }
        if (t > 34.f && !landed) {   /* judge once in flight, not the touchdown step */
            float ea = fabsf(-f.x[2][0] - alt), ev = fabsf(-f.x[2][1] - vel);
            if (ea > max_err_alt) max_err_alt = ea;
            if (ev > max_err_vel) max_err_vel = ev;
        }
        if (-f.x[2][0] > alt_apogee_est) alt_apogee_est = -f.x[2][0];
    }
    Athena_State s; Fusion_GetState(&f, &s, 0x7, 400);
    float roll, pitch, yaw; Fusion_QuatToEuler(f.q, &roll, &pitch, &yaw);
    printf("fusion    apogee true %.1f est %.1f | max alt err %.2f m, max vel err %.2f m/s | horiz DR err %.1f,%.1f m | in_flight=%d gps_fresh=%d pitch=%.1fdeg bias=%.4f,%.4f,%.4f\n",
           alt_apogee_true, alt_apogee_est, max_err_alt, max_err_vel, f.x[0][0], f.x[1][0], !!(s.flags & STATE_FLAG_IN_FLIGHT), !!(s.flags & STATE_FLAG_GPS_FRESH), pitch * 57.3f, -f.ierr[0], -f.ierr[1], -f.ierr[2]);
    assert(s.flags & STATE_FLAG_IN_FLIGHT);
    assert(!(s.flags & STATE_FLAG_GPS_FRESH));           /* GPS was cut: dead reckoning */
    assert(fabsf(alt_apogee_est - alt_apogee_true) < 5.f);
    assert(max_err_alt < 6.f && max_err_vel < 4.f);
    assert(fabsf(-f.ierr[2] + 0.02f) < 0.004f);          /* horizontal-axis gyro bias learned from accel on the pad */
    assert(fabsf(-f.ierr[1] - 0.015f) < 0.004f);
    assert(fabsf(-f.ierr[0] - 0.01f) < 0.006f);          /* vertical-axis bias learned from the mag */
    assert(fabsf(pitch * 57.3f - 90.f) < 3.f || fabsf(pitch * 57.3f + 90.f) < 3.f);   /* nose vertical */
    assert(fabsf(f.x[0][0]) < 25.f && fabsf(f.x[1][0]) < 25.f);                        /* horizontal DR without GPS stayed sane */
    /* telemetry round trip of the estimate back to lat/lon */
    Athena_Telemetry tm; Link_MakeTelemetry(&tm, &s, NULL, 1234);
    assert(abs(tm.lat_1e7 - 405000000) < 2000 && abs(tm.lon_1e7 + 740000000) < 2000);
    assert(Fusion_Median3(3, 1, 2) == 2 && Fusion_Median3(1, 2, 3) == 2 && Fusion_Median3(2, 3, 1) == 2);
    printf("fusion    ok\n");
}

int main(void)
{
    test_link();
    test_ubx();
    test_fusion();
    printf("ALL OK\n");
    return 0;
}
