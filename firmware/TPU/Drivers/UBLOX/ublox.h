/*
 * ublox.h - u-blox NEO-M8U over SPI3 (TPU)
 *
 *   D_SEL (PD3) low selects SPI. SAFEBOOT_N (PD4) and RESET_N (PB6) must idle HIGH.
 *   LNA_EN (PD5) is an OUTPUT of the module: the MCU pin must be an input.
 *   CS = PD7 active low, SCK PB3, MISO PB4, MOSI PD6. SPI mode 0, 781 kHz (module limit is 125 kB/s).
 *   The module clocks out 0xFF when it has nothing to say and treats 0xFF on MOSI as idle.
 */
#ifndef UBLOX_H
#define UBLOX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "athena_link.h"

/* Resets the module and pushes the config (UBX only on SPI, NAV-PVT at 5 Hz requested,
 * airborne-4g dynamics, built-in dead reckoning OFF). Returns number of config
 * messages that were NOT acknowledged (0 = all good). */
int Ublox_Init(void);

/* Call often. Clocks the module every 10 ms and parses what comes back.
 * Returns 1 and fills fix when a new NAV-PVT arrived. */
int Ublox_Poll(Athena_GpsFix *fix);

/* Antenna supervisor / RF front-end health from UBX-MON-HW, refreshed every 5 s. */
typedef struct {
    uint8_t  valid;
    uint8_t  ant_status;     /* 0 init, 1 don't know, 2 OK, 3 short, 4 open */
    uint8_t  ant_power;      /* 0 off, 1 on, 2 don't know */
    uint8_t  jam_ind;        /* 0 none .. 255 strong CW jammer */
    uint16_t noise_per_ms;
    uint16_t agc_cnt;        /* 0..8191, ~2000-4000 typical with a working antenna */
} Ublox_Hw;
const Ublox_Hw *Ublox_HwStatus(void);
const char     *Ublox_AntStatusStr(uint8_t st);

uint32_t Ublox_AckCount(void);
uint32_t Ublox_NakCount(void);

#ifdef __cplusplus
}
#endif
#endif
