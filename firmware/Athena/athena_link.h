/*
 * athena_link.h - framed packet protocol shared by all three MCUs
 *
 * One frame on the wire (UART between MCUs, and also over LoRa):
 *
 *   [0xA5][type][len][payload ... len bytes][crc lo][crc hi]
 *
 *   crc = CRC-16/CCITT-FALSE over type, len and payload.
 *
 * Payload structs below are packed little-endian so the same header can be
 * compiled into a ground-station tool unchanged.
 */
#ifndef ATHENA_LINK_H
#define ATHENA_LINK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINK_SOF          0xA5
#define LINK_MAX_PAYLOAD  200
#define LINK_OVERHEAD     5          /* sof + type + len + crc16 */
#define LINK_RX_RING      512

enum {
    LINK_PKT_GPS   = 0x01,  /* TPU -> MPU   : Athena_GpsFix       */
    LINK_PKT_STATE = 0x02,  /* MPU -> TPU   : Athena_State        */
    LINK_PKT_TELEM = 0x03,  /* TPU -> ground: Athena_Telemetry    */
    LINK_PKT_TEXT  = 0x7F,  /* free-form debug text               */
};

/* Parsed UBX-NAV-PVT, only the fields the filter and telemetry need.
 * packed = exact wire layout; aligned(4) = every float/int32 member stays word aligned
 * so float pointers into the struct are safe on Cortex-M7 (sizes are multiples of 4). */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t itow_ms;
    uint8_t  fix_type;      /* 0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time only */
    uint8_t  num_sv;
    uint8_t  flags;         /* bit0 gnssFixOK (from NAV-PVT flags) */
    uint8_t  _pad;
    int32_t  lat_1e7;       /* deg * 1e7 */
    int32_t  lon_1e7;
    int32_t  h_msl_mm;      /* height above mean sea level */
    int32_t  vel_ned_mms[3];/* north, east, down velocity, mm/s */
    uint32_t h_acc_mm;      /* horizontal accuracy estimate */
    uint32_t v_acc_mm;      /* vertical accuracy estimate */
    uint32_t s_acc_mms;     /* speed accuracy estimate */
} Athena_GpsFix;            /* 44 bytes */

/* Athena_State.flags bits */
#define STATE_FLAG_IN_FLIGHT  (1u << 0)
#define STATE_FLAG_GPS_FRESH  (1u << 1)
#define STATE_FLAG_BARO_OK    (1u << 2)
#define STATE_FLAG_MAG_OK     (1u << 3)
#define STATE_FLAG_ORIGIN_OK  (1u << 4)

/* Navigation solution produced by the MPU. NED frame, origin = launch pad. */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t t_us;
    float    q[4];            /* body -> NED quaternion (w, x, y, z) */
    float    pos_ned[3];      /* m from pad */
    float    vel_ned[3];      /* m/s */
    float    acc_body[3];     /* fused IMU specific force, m/s^2 */
    float    gyro_body[3];    /* fused IMU rate, rad/s (bias removed) */
    float    mag_body[3];     /* gauss */
    float    baro_alt;        /* m above pad, from barometer alone */
    int32_t  origin_lat_1e7;  /* pad position so pos_ned can be mapped back */
    int32_t  origin_lon_1e7;
    uint8_t  imu_mask;        /* bit n set = IMU n healthy */
    uint8_t  flags;           /* STATE_FLAG_* */
    uint16_t loop_hz;         /* measured fusion rate */
} Athena_State;               /* 96 bytes */

/* Compact radio frame, ~40 bytes so it fits a 2 Hz LoRa budget at SF7. */
typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int32_t  lat_1e7;         /* fused estimate (dead-reckoned when GPS is lost) */
    int32_t  lon_1e7;
    float    alt_agl;         /* m above pad, fused */
    float    baro_alt;        /* m above pad, barometer only */
    int16_t  vel_ned_dms[3];  /* dm/s  (+-3276 m/s range) */
    int16_t  q_q15[4];        /* quaternion * 32767 */
    uint8_t  fix_type;
    uint8_t  num_sv;
    uint8_t  flags;           /* STATE_FLAG_* */
    uint8_t  imu_mask;
} Athena_Telemetry;           /* 38 bytes */

typedef void (*Link_Handler)(uint8_t type, const uint8_t *payload, uint8_t len, void *user);

typedef struct {
    /* ISR -> main loop ring buffer */
    uint8_t  ring[LINK_RX_RING];
    volatile uint16_t head, tail;
    /* decoder */
    uint8_t  st, type, len, idx;
    uint8_t  buf[LINK_MAX_PAYLOAD];
    uint16_t crc;
    Link_Handler on_packet;
    void    *user;
    uint32_t rx_ok, rx_bad;
} Link;

uint16_t Link_Crc16(const uint8_t *d, size_t n);
/* Writes a full frame to out (needs len + LINK_OVERHEAD bytes). Returns frame size, 0 on error. */
size_t   Link_Encode(uint8_t *out, uint8_t type, const void *payload, uint8_t len);
void     Link_Init(Link *l, Link_Handler h, void *user);
void     Link_RxPush(Link *l, uint8_t b);      /* call from the UART RX interrupt */
void     Link_Process(Link *l);                /* call from the main loop */
void     Link_FeedByte(Link *l, uint8_t b);    /* decoder; Process() calls this */

void     Link_MakeTelemetry(Athena_Telemetry *t, const Athena_State *s,
                            const Athena_GpsFix *g, uint32_t t_ms);

#ifdef __cplusplus
}
#endif
#endif
