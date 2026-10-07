/*
 * w25q.h - Winbond W25Q256JV (32 MB) on QUADSPI (TPU), single-line 1-1-1 commands with
 * 4-byte addresses (0x13 read, 0x12 page program, 0x21 sector erase), so no address-mode
 * switching is needed. W25Q256JV datasheet rev. K, section 8.
 */
#ifndef W25Q_H
#define W25Q_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define W25Q_SIZE        (32u * 1024u * 1024u)
#define W25Q_SECTOR      4096u
#define W25Q_PAGE        256u
#define W25Q_JEDEC_ID    0xEF4019u

int  W25Q_Init(uint32_t *jedec_id);                                     /* 0 ok, -1 bus error, -2 wrong ID */
int  W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len);
int  W25Q_Program(uint32_t addr, const uint8_t *buf, uint32_t len);     /* within one 256 B page */
int  W25Q_EraseSector(uint32_t addr);                                   /* 4 KB, ~50 ms typical, blocking */
int  W25Q_EraseSectorStart(uint32_t addr);                              /* same, returns immediately */
int  W25Q_Busy(void);                                                   /* 1 while a program/erase runs */
int  W25Q_IsErased(uint32_t addr, uint32_t len);                        /* 1 if all 0xFF */

#ifdef __cplusplus
}
#endif
#endif
