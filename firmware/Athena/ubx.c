#include "ubx.h"
#include <string.h>

enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB };

void Ubx_Init(Ubx *u) { memset(u, 0, sizeof(*u)); }

static void cksum(uint8_t *a, uint8_t *b, uint8_t x) { *a = (uint8_t)(*a + x); *b = (uint8_t)(*b + *a); }

int Ubx_Feed(Ubx *u, uint8_t b)
{
    switch (u->st) {
    case S_SYNC1: if (b == 0xB5) u->st = S_SYNC2; break;
    case S_SYNC2: u->st = (b == 0x62) ? S_CLASS : S_SYNC1; break;
    case S_CLASS: u->cls = b; u->ck_a = u->ck_b = 0; cksum(&u->ck_a, &u->ck_b, b); u->st = S_ID; break;
    case S_ID:    u->id = b;  cksum(&u->ck_a, &u->ck_b, b); u->st = S_LEN1; break;
    case S_LEN1:  u->len = b; cksum(&u->ck_a, &u->ck_b, b); u->st = S_LEN2; break;
    case S_LEN2:
        u->len |= (uint16_t)b << 8; cksum(&u->ck_a, &u->ck_b, b);
        if (u->len > UBX_MAX_PAYLOAD) { u->st = S_SYNC1; break; }
        u->idx = 0;
        u->st = u->len ? S_PAYLOAD : S_CKA;
        break;
    case S_PAYLOAD:
        u->payload[u->idx++] = b; cksum(&u->ck_a, &u->ck_b, b);
        if (u->idx >= u->len) u->st = S_CKA;
        break;
    case S_CKA: u->st = (b == u->ck_a) ? S_CKB : S_SYNC1; break;
    case S_CKB: u->st = S_SYNC1; return b == u->ck_b;
    default: u->st = S_SYNC1;
    }
    return 0;
}

size_t Ubx_Frame(uint8_t *out, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    out[0] = 0xB5; out[1] = 0x62; out[2] = cls; out[3] = id;
    out[4] = (uint8_t)(len & 0xFF); out[5] = (uint8_t)(len >> 8);
    if (len) memcpy(&out[6], payload, len);
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < (size_t)len + 6; i++) cksum(&a, &b, out[i]);
    out[6 + len] = a; out[7 + len] = b;
    return (size_t)len + 8;
}

static int32_t  rd_i4(const uint8_t *p) { return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24); }
static uint32_t rd_u4(const uint8_t *p) { return (uint32_t)rd_i4(p); }

int Ubx_ParseNavPvt(const uint8_t *p, uint16_t len, Athena_GpsFix *f)
{
    if (len < 92) return -1;
    f->itow_ms   = rd_u4(p + 0);
    f->fix_type  = p[20];
    f->flags     = p[21] & 0x01;          /* gnssFixOK */
    f->num_sv    = p[23];
    f->_pad      = 0;
    f->lon_1e7   = rd_i4(p + 24);
    f->lat_1e7   = rd_i4(p + 28);
    f->h_msl_mm  = rd_i4(p + 36);
    f->h_acc_mm  = rd_u4(p + 40);
    f->v_acc_mm  = rd_u4(p + 44);
    f->vel_ned_mms[0] = rd_i4(p + 48);
    f->vel_ned_mms[1] = rd_i4(p + 52);
    f->vel_ned_mms[2] = rd_i4(p + 56);
    f->s_acc_mms = rd_u4(p + 68);
    return 0;
}
