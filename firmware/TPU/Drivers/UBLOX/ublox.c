#include "ublox.h"
#include "ubx.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;

#define POLL_BYTES 64
#define POLL_MS    10

static Ubx      ubx;
static uint32_t acks, naks, last_poll_ms, last_hw_poll_ms;
static uint8_t  last_ack_cls, last_ack_id;
static Ublox_Hw hw;

static void cs(int on) { HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

/* NEO-M8 SPI limits (NEO-M8 data sheet, SPI timing): <= 5.5 MHz clock, <= 125 kB/s byte rate
 * (so >= 8 us per byte: SPI3 runs at 781 kHz = 10.2 us/byte), >= 10 us from CS low to the
 * first clock, >= 1 ms between transactions (callers poll every 10 ms). */
static void short_delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000u) * us / 3u;   /* ~3 cycles per iteration */
    for (volatile uint32_t i = 0; i < cycles; i++) {}
}

static int xfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    cs(1);
    short_delay_us(15);                                          /* tINIT */
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)tx, rx, n, 100);
    cs(0);
    return st == HAL_OK ? 0 : -1;
}

/* Feed received bytes through the UBX parser; returns 1 if a NAV-PVT landed in fix. */
static int feed(const uint8_t *rx, uint16_t n, Athena_GpsFix *fix)
{
    int got = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (!Ubx_Feed(&ubx, rx[i])) continue;
        if (ubx.cls == UBX_CLASS_NAV && ubx.id == UBX_NAV_PVT) {
            if (fix && Ubx_ParseNavPvt(ubx.payload, ubx.len, fix) == 0) got = 1;
        } else if (ubx.cls == UBX_CLASS_MON && ubx.id == UBX_MON_HW && ubx.len >= 60) {
            /* UBX-MON-HW (protocol spec 32.15.5): noisePerMS u2 @16, agcCnt u2 @18, aStatus u1 @20,
             * aPower u1 @21, jamInd u1 @45 */
            hw.noise_per_ms = (uint16_t)(ubx.payload[16] | ubx.payload[17] << 8);
            hw.agc_cnt      = (uint16_t)(ubx.payload[18] | ubx.payload[19] << 8);
            hw.ant_status   = ubx.payload[20];
            hw.ant_power    = ubx.payload[21];
            hw.jam_ind      = ubx.payload[45];
            hw.valid        = 1;
        } else if (ubx.cls == UBX_CLASS_ACK && ubx.len >= 2) {
            last_ack_cls = ubx.payload[0]; last_ack_id = ubx.payload[1];
            if (ubx.id == UBX_ACK_ACK) acks++; else naks++;
        }
    }
    return got;
}

/* Send one CFG message and wait up to 300 ms for its ACK. */
static int send_cfg(uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t tx[128], rx[128];
    size_t n = Ubx_Frame(tx, UBX_CLASS_CFG, id, payload, len);
    if (xfer(tx, rx, (uint16_t)n)) return -1;
    feed(rx, (uint16_t)n, NULL);
    uint32_t a0 = acks, t0 = HAL_GetTick();
    memset(tx, 0xFF, sizeof(tx));
    HAL_Delay(2);                                                /* >= 1 ms between transactions */
    while (HAL_GetTick() - t0 < 1000) {                          /* spec: ACK within one second */
        if (xfer(tx, rx, 32)) return -1;
        feed(rx, 32, NULL);
        if (acks != a0 && last_ack_cls == UBX_CLASS_CFG && last_ack_id == id) return 0;
        HAL_Delay(2);
    }
    return -1;
}

int Ublox_Init(void)
{
    Ubx_Init(&ubx);
    cs(0);
    HAL_GPIO_WritePin(GPS_SEL_GPIO_Port,      GPS_SEL_Pin,      GPIO_PIN_RESET);  /* D_SEL = 0 -> SPI */
    HAL_GPIO_WritePin(GPS_SAFEBOOT_GPIO_Port, GPS_SAFEBOOT_Pin, GPIO_PIN_SET);    /* not safeboot */
    HAL_GPIO_WritePin(GPS_RESET_GPIO_Port,    GPS_RESET_Pin,    GPIO_PIN_RESET);  /* RESET_N pulse */
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPS_RESET_GPIO_Port,    GPS_RESET_Pin,    GPIO_PIN_SET);
    HAL_Delay(1200);                                                               /* boot */

    int failed = 0;

    /* CFG-PRT: SPI port (id 4), UBX in/out only, SPI mode 0 */
    uint8_t prt[20] = { 0 };
    prt[0] = 4; prt[12] = 0x01; prt[14] = 0x01;
    failed += send_cfg(UBX_CFG_PRT, prt, sizeof prt) != 0;

    /* CFG-MSG: NAV-PVT once per navigation epoch on the SPI port */
    uint8_t msg[8] = { UBX_CLASS_NAV, UBX_NAV_PVT, 0, 0, 0, 0, 1, 0 };
    failed += send_cfg(UBX_CFG_MSG, msg, sizeof msg) != 0;

    /* CFG-RATE: 200 ms measurement period -> 5 Hz requested. The NEO-M8U data sheet rates
     * the standard (non-UDR) PVT output at up to 2 Hz for some GNSS configurations; the
     * receiver keeps the fastest rate it supports, and the TPU status line prints the
     * fix rate actually achieved. The filter only needs ~1 Hz to bound dead reckoning. */
    uint8_t rate[6] = { 200, 0, 1, 0, 0, 0 };
    failed += send_cfg(UBX_CFG_RATE, rate, sizeof rate) != 0;

    /* CFG-NAV5: dynamic model 8 = airborne < 4 g (the highest the receiver offers) */
    uint8_t nav5[36] = { 0 };
    nav5[0] = 0x01; nav5[2] = 8;
    failed += send_cfg(UBX_CFG_NAV5, nav5, sizeof nav5) != 0;

    /* CFG-NAVX5 v2: useAdr = 0. The M8U's untethered dead reckoning assumes a car;
     * the MPU does the dead reckoning for the rocket with the real IMUs instead. */
    uint8_t navx5[40] = { 0 };
    navx5[0] = 0x02; navx5[4] = 0x40; navx5[39] = 0;
    failed += send_cfg(UBX_CFG_NAVX5, navx5, sizeof navx5) != 0;

    return failed;
}

int Ublox_Poll(Athena_GpsFix *fix)
{
    uint32_t now = HAL_GetTick();
    if (now - last_poll_ms < POLL_MS) return 0;
    last_poll_ms = now;
    uint8_t tx[POLL_BYTES], rx[POLL_BYTES];
    memset(tx, 0xFF, sizeof tx);
    if (now - last_hw_poll_ms >= 5000u) {                       /* ask for MON-HW (antenna supervisor) every 5 s */
        last_hw_poll_ms = now;
        Ubx_Frame(tx, UBX_CLASS_MON, UBX_MON_HW, NULL, 0);     /* 8-byte poll request, rest stays 0xFF */
    }
    if (xfer(tx, rx, POLL_BYTES)) return 0;
    return feed(rx, POLL_BYTES, fix);
}

const Ublox_Hw *Ublox_HwStatus(void) { return &hw; }

const char *Ublox_AntStatusStr(uint8_t st)
{
    static const char *n[] = { "init", "unknown", "ok", "SHORT", "OPEN" };
    return st < 5 ? n[st] : "?";
}

uint32_t Ublox_AckCount(void) { return acks; }
uint32_t Ublox_NakCount(void) { return naks; }
