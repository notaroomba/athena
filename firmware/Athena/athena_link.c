#include "athena_link.h"
#include <string.h>
#include <math.h>

uint16_t Link_Crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;                     /* CRC-16/CCITT-FALSE */
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t Link_Encode(uint8_t *out, uint8_t type, const void *payload, uint8_t len)
{
    if (len > LINK_MAX_PAYLOAD) return 0;
    out[0] = LINK_SOF;
    out[1] = type;
    out[2] = len;
    if (len) memcpy(&out[3], payload, len);
    uint16_t crc = Link_Crc16(&out[1], (size_t)len + 2);
    out[3 + len] = (uint8_t)(crc & 0xFF);
    out[4 + len] = (uint8_t)(crc >> 8);
    return (size_t)len + LINK_OVERHEAD;
}

void Link_Init(Link *l, Link_Handler h, void *user)
{
    memset(l, 0, sizeof(*l));
    l->on_packet = h;
    l->user = user;
}

void Link_RxPush(Link *l, uint8_t b)
{
    uint16_t next = (uint16_t)((l->head + 1) % LINK_RX_RING);
    if (next == l->tail) return;               /* full: drop, decoder resyncs on next SOF */
    l->ring[l->head] = b;
    l->head = next;
}

enum { ST_SOF, ST_TYPE, ST_LEN, ST_PAYLOAD, ST_CRC_LO, ST_CRC_HI };

void Link_FeedByte(Link *l, uint8_t b)
{
    switch (l->st) {
    case ST_SOF:
        if (b == LINK_SOF) l->st = ST_TYPE;
        break;
    case ST_TYPE:
        l->type = b; l->st = ST_LEN;
        break;
    case ST_LEN:
        if (b > LINK_MAX_PAYLOAD) { l->st = ST_SOF; l->rx_bad++; break; }
        l->len = b; l->idx = 0;
        l->st = b ? ST_PAYLOAD : ST_CRC_LO;
        break;
    case ST_PAYLOAD:
        l->buf[l->idx++] = b;
        if (l->idx >= l->len) l->st = ST_CRC_LO;
        break;
    case ST_CRC_LO:
        l->crc = b; l->st = ST_CRC_HI;
        break;
    case ST_CRC_HI: {
        l->crc |= (uint16_t)b << 8;
        uint8_t hdr[2] = { l->type, l->len };
        uint16_t c = 0xFFFF;
        /* crc over type,len,payload: compute incrementally to avoid a copy */
        {
            uint8_t tmp[LINK_MAX_PAYLOAD + 2];
            tmp[0] = hdr[0]; tmp[1] = hdr[1];
            memcpy(&tmp[2], l->buf, l->len);
            c = Link_Crc16(tmp, (size_t)l->len + 2);
        }
        if (c == l->crc) {
            l->rx_ok++;
            if (l->on_packet) l->on_packet(l->type, l->buf, l->len, l->user);
        } else {
            l->rx_bad++;
        }
        l->st = ST_SOF;
        break;
    }
    default:
        l->st = ST_SOF;
    }
}

void Link_Process(Link *l)
{
    while (l->tail != l->head) {
        uint8_t b = l->ring[l->tail];
        l->tail = (uint16_t)((l->tail + 1) % LINK_RX_RING);
        Link_FeedByte(l, b);
    }
}

static int16_t clamp16(float v)
{
    if (v > 32767.f) return 32767;
    if (v < -32768.f) return -32768;
    return (int16_t)v;
}

void Link_MakeTelemetry(Athena_Telemetry *t, const Athena_State *s,
                        const Athena_GpsFix *g, uint32_t t_ms)
{
    memset(t, 0, sizeof(*t));
    t->t_ms = t_ms;
    if (s->flags & STATE_FLAG_ORIGIN_OK) {
        /* map fused NED offset back onto the pad's lat/lon: this is what keeps
         * the ground station getting a position while GPS is lost */
        const double re = 6371000.0, d2r = 3.14159265358979 / 180.0;
        double lat0 = s->origin_lat_1e7 * 1e-7, lon0 = s->origin_lon_1e7 * 1e-7;
        double lat = lat0 + (s->pos_ned[0] / re) / d2r;
        double lon = lon0 + (s->pos_ned[1] / (re * cos(lat0 * d2r))) / d2r;
        t->lat_1e7 = (int32_t)(lat * 1e7);
        t->lon_1e7 = (int32_t)(lon * 1e7);
    } else if (g) {
        t->lat_1e7 = g->lat_1e7;
        t->lon_1e7 = g->lon_1e7;
    }
    t->alt_agl  = -s->pos_ned[2];
    t->baro_alt = s->baro_alt;
    for (int i = 0; i < 3; i++) t->vel_ned_dms[i] = clamp16(s->vel_ned[i] * 10.f);
    for (int i = 0; i < 4; i++) t->q_q15[i] = clamp16(s->q[i] * 32767.f);
    if (g) { t->fix_type = g->fix_type; t->num_sv = g->num_sv; }
    t->flags = s->flags;
    t->imu_mask = s->imu_mask;
}
