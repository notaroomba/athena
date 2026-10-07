/*
 * ubx.h - minimal u-blox UBX protocol: frame builder, byte-fed parser,
 *         NAV-PVT decoder. No hardware dependencies.
 */
#ifndef UBX_H
#define UBX_H

#include <stdint.h>
#include <stddef.h>
#include "athena_link.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_MAX_PAYLOAD 256

#define UBX_CLASS_NAV  0x01
#define UBX_CLASS_ACK  0x05
#define UBX_CLASS_CFG  0x06
#define UBX_NAV_PVT    0x07
#define UBX_ACK_NAK    0x00
#define UBX_ACK_ACK    0x01
#define UBX_CFG_PRT    0x00
#define UBX_CFG_MSG    0x01
#define UBX_CFG_RATE   0x08
#define UBX_CFG_NAV5   0x24
#define UBX_CFG_NAVX5  0x23
#define UBX_CLASS_MON  0x0A
#define UBX_MON_HW     0x09

typedef struct {
    uint8_t  st;
    uint8_t  cls, id;
    uint16_t len, idx;
    uint8_t  ck_a, ck_b;
    uint8_t  payload[UBX_MAX_PAYLOAD];
} Ubx;

void   Ubx_Init(Ubx *u);
/* Returns 1 when a complete, checksum-valid frame is available in u->cls/id/len/payload. */
int    Ubx_Feed(Ubx *u, uint8_t b);
/* Builds a frame into out (needs len + 8 bytes). Returns frame length. */
size_t Ubx_Frame(uint8_t *out, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
/* Decodes a NAV-PVT payload. Returns 0 on success. */
int    Ubx_ParseNavPvt(const uint8_t *p, uint16_t len, Athena_GpsFix *fix);

#ifdef __cplusplus
}
#endif
#endif
