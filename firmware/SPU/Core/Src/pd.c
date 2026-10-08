#include <string.h>
#include "pd.h"
#include "athena.h"

/* patch bundle ("low region binary") exported as a C array by TI's Application Customization Tool */
extern const char tps25750x_lowRegion_i2c_array[];
extern int gSizeLowRegionArray;

#define REG_MODE          0x03
#define REG_CMD1          0x08
#define REG_DATA1         0x09
#define REG_INT_EVENT1    0x14
#define REG_INT_CLEAR1    0x18
#define REG_STATUS        0x1A
#define REG_POWER_STATUS  0x3F
#define REG_ACTIVE_PDO    0x34   /* 6 bytes: USB PD sink contract PDO */
#define I2C_TMO           50

static const char *const mode_names[] = { "none", "PTCH", "APP", "BOOT" };
const char *PD_ModeName(uint8_t m) { return m < 4 ? mode_names[m] : "?"; }

/* Unique-address register protocol (TRM 1.3.1): write = [reg][count][data...]; read = [reg] then [count][data...] */
static int reg_read(PD *pd, uint8_t reg, uint8_t *out, uint8_t n)
{
    uint8_t buf[66];
    if (n > 64) n = 64;
    if (HAL_I2C_Mem_Read(pd->i2c, TPS25751_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)(n + 1), I2C_TMO) != HAL_OK) { pd->errors++; return -1; }
    uint8_t cnt = buf[0] < n ? buf[0] : n;
    memcpy(out, buf + 1, cnt);
    if (cnt < n) memset(out + cnt, 0, (size_t)(n - cnt));
    return cnt;
}

static int reg_write(PD *pd, uint8_t reg, const uint8_t *data, uint8_t n)
{
    uint8_t buf[66];
    if (n > 64) n = 64;
    buf[0] = reg; buf[1] = n; memcpy(buf + 2, data, n);
    if (HAL_I2C_Master_Transmit(pd->i2c, TPS25751_ADDR << 1, buf, (uint16_t)(n + 2), I2C_TMO) != HAL_OK) { pd->errors++; return -1; }
    return 0;
}

static uint8_t read_mode(PD *pd)
{
    uint8_t m[4];
    if (reg_read(pd, REG_MODE, m, 4) < 0) return PD_MODE_NONE;
    if (!memcmp(m, "APP ", 4)) return PD_MODE_APP;
    if (!memcmp(m, "PTCH", 4)) return PD_MODE_PTCH;
    if (!memcmp(m, "BOOT", 4)) return PD_MODE_BOOT;
    return PD_MODE_NONE;
}

/* 4CC task (TRM chapter 4): DATA1 <- in, CMD1 <- "XXXX", wait until CMD1 reads 0 (done) or "!CMD" (unknown),
 * then DATA1 -> [return code][out...]. Returns the task return code (0 = success) or <0 on bus/timeout. */
static int task(PD *pd, const char *cmd, const uint8_t *in, uint8_t in_n, uint8_t *out, uint8_t out_n, uint32_t timeout_ms)
{
    if (in_n && reg_write(pd, REG_DATA1, in, in_n) < 0) return -1;
    if (reg_write(pd, REG_CMD1, (const uint8_t *)cmd, 4) < 0) return -1;
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        uint8_t c[4];
        HAL_Delay(1);
        if (reg_read(pd, REG_CMD1, c, 4) < 0) return -1;
        if (!memcmp(c, "!CMD", 4)) return -2;
        if (!(c[0] | c[1] | c[2] | c[3])) break;
        if (HAL_GetTick() - t0 > timeout_ms) return -3;
    }
    uint8_t d[65];
    uint8_t n = (uint8_t)(out_n + 1 > 64 ? 64 : out_n + 1);
    if (reg_read(pd, REG_DATA1, d, n) < 0) return -1;
    if (out && out_n) memcpy(out, d + 1, out_n);
    return d[0];
}

/* Patch Burst Mode (TRM 5.2, app note SLVAFV8): PBMs(size, burst address, timeout) -> raw I2C writes of the
 * bundle to the burst address -> PBMc (CRC check, run) -> MODE must read 'APP '. */
static int load_patch(PD *pd)
{
    uint32_t size = (uint32_t)gSizeLowRegionArray;
    uint8_t ev[11];
    for (int i = 0; i < 40; i++) {                                   /* INT_EVENT1.ReadyForPatch (bit 81) */
        if (reg_read(pd, REG_INT_EVENT1, ev, 11) == 11 && (ev[10] & 0x02)) break;
        HAL_Delay(50);
    }
    uint8_t in[6] = { (uint8_t)size, (uint8_t)(size >> 8), (uint8_t)(size >> 16), (uint8_t)(size >> 24), TPS_BURST_ADDR, 0x32 };
    int ok = 0;                                                      /* SLVAFV8 step 5: in PTCH mode the DATA1 write may not take, verify it */
    for (int i = 0; i < 5 && !ok; i++) {
        uint8_t chk[6];
        if (reg_write(pd, REG_DATA1, in, 6) < 0) { HAL_Delay(1); continue; }
        HAL_Delay(1);
        ok = reg_read(pd, REG_DATA1, chk, 6) == 6 && !memcmp(chk, in, 6);
    }
    if (!ok) { print("pd: DATA1 would not take the PBMs parameters\r\n"); return -1; }
    int rc = task(pd, "PBMs", NULL, 0, NULL, 0, 1000);
    if (rc != 0) { print("pd: PBMs rejected (%d)\r\n", rc); return -1; }
    const uint8_t *p = (const uint8_t *)tps25750x_lowRegion_i2c_array;
    for (uint32_t off = 0; off < size; ) {
        uint16_t n = (uint16_t)((size - off) > 4095u ? 4095u : (size - off));
        if (HAL_I2C_Master_Transmit(pd->i2c, TPS_BURST_ADDR << 1, (uint8_t *)(p + off), n, 2000) != HAL_OK) {
            print("pd: burst write failed at %lu/%lu\r\n", (unsigned long)off, (unsigned long)size);
            task(pd, "PBMe", NULL, 0, NULL, 0, 500);
            return -1;
        }
        off += n;
        HAL_Delay(1);                                                /* >= 500 us between bursts */
    }
    uint8_t out[40] = { 0 };
    rc = task(pd, "PBMc", NULL, 0, out, 40, 3000);
    HAL_Delay(20);                                                   /* the controller applies the image */
    pd->mode = read_mode(pd);
    /* PBMc output: acState at byte 26, acFailCode at byte 27 (1-based, byte 1 = return code) */
    print("pd: patch %lu B -> PBMc rc=%d acState=%u acFail=%u, mode %s\r\n", (unsigned long)size, rc, out[24], out[25], PD_ModeName(pd->mode));
    return pd->mode == PD_MODE_APP ? 0 : -1;
}

/* charger register access through the TPS25751's I2Cc port */
#if PD_CHARGER_BRIDGE
static int bq_read(PD *pd, uint8_t reg, uint8_t *out, uint8_t n)
{
    uint8_t in[3] = { BQ25713_ADDR, reg, n };
    return task(pd, "I2Cr", in, 3, out, n, 200) == 0 ? 0 : -1;
}
static int bq_write(PD *pd, uint8_t reg, const uint8_t *data, uint8_t n)
{
    uint8_t in[14] = { BQ25713_ADDR, n, reg };
    if (n > 11) return -1;
    memcpy(in + 3, data, n);
    return task(pd, "I2Cw", in, (uint8_t)(3 + n), NULL, 0, 200) == 0 ? 0 : -1;
}
#endif

#if PD_CHARGER_BRIDGE
static int bq_start_adc(PD *pd)
{
    uint8_t id[2], opt0[2];
    if (bq_read(pd, 0x2E, id, 2) < 0) return -1;                    /* ManufacturerID (0x40), DeviceID */
    if (bq_read(pd, 0x00, opt0, 2) < 0) return -1;                  /* ChargeOption0 */
    opt0[1] &= (uint8_t)~0x80;                                       /* EN_LWPWR = 0: ADC keeps running on battery */
    bq_write(pd, 0x00, opt0, 2);
    uint8_t adc[2] = { 0xFF, 0xE0 };                                 /* 0x3A all channels; 0x3B ADC_CONV|ADC_START|3.06 V full scale */
    if (bq_write(pd, 0x3A, adc, 2) < 0) return -1;
    print("pd: BQ25713 id 0x%02X/0x%02X, ADC continuous\r\n", id[0], id[1]);
    return 0;
}

static void bq_poll(PD *pd)
{
    uint8_t v[2], i[2], in[2], vs[2], st[2];
    if (bq_read(pd, 0x26, v, 2) < 0 || bq_read(pd, 0x28, i, 2) < 0 || bq_read(pd, 0x2A, in, 2) < 0 ||
        bq_read(pd, 0x2C, vs, 2) < 0 || bq_read(pd, 0x20, st, 2) < 0) { pd->bq_ok = 0; return; }
    pd->bq_ok = 1;
    pd->vbus_mv = v[1] ? (uint16_t)(3200 + 64 * v[1]) : 0;                       /* 0x27 VBUS: 64 mV/LSB from 3.2 V */
    pd->ibat_ma = (int16_t)(64 * (i[1] & 0x7F)) - (int16_t)(256 * (i[0] & 0x7F));  /* 0x29 ICHG 64 mA/LSB, 0x28 IDCHG 256 mA/LSB */
    pd->iin_ma  = (uint16_t)(50 * in[1]);                                        /* 0x2B IIN 50 mA/LSB */
    pd->vbat_mv = vs[0] ? (uint16_t)(2880 + 64 * vs[0]) : 0;                     /* 0x2C VBAT 64 mV/LSB from 2.88 V */
    pd->vsys_mv = vs[1] ? (uint16_t)(2880 + 64 * vs[1]) : 0;                     /* 0x2D VSYS */
    pd->chg_status = (uint16_t)((st[1] << 8) | st[0]);
}
#endif /* PD_CHARGER_BRIDGE */

void PD_Init(PD *pd, I2C_HandleTypeDef *i2c)
{
    memset(pd, 0, sizeof *pd);
    pd->i2c = i2c;
    pd->present = HAL_I2C_IsDeviceReady(i2c, TPS25751_ADDR << 1, 3, 100) == HAL_OK;
    if (!pd->present) { print("pd: TPS25751 not answering at 0x%02X\r\n", TPS25751_ADDR); return; }
    pd->mode = read_mode(pd);
    print("pd: TPS25751 mode %s\r\n", PD_ModeName(pd->mode));
    if (pd->mode == PD_MODE_PTCH) load_patch(pd);
}

void PD_Task(PD *pd, uint32_t now, int allow_slow)
{
    if ((int32_t)(now - pd->next_ms) < 0) return;
    pd->next_ms = now + 1000;
    if (!pd->present) {
        if (now - pd->patch_retry_ms < 5000u) return;
        pd->patch_retry_ms = now;
        pd->present = HAL_I2C_IsDeviceReady(pd->i2c, TPS25751_ADDR << 1, 2, 20) == HAL_OK;
        if (!pd->present) return;
    }
    uint8_t prev = pd->mode;
    pd->mode = read_mode(pd);
    if (prev == PD_MODE_APP && pd->mode != PD_MODE_APP) { pd->resets++; print("pd: TPS25751 left APP mode (%s), reset #%lu\r\n", PD_ModeName(pd->mode), (unsigned long)pd->resets); }
    if (pd->mode == PD_MODE_PTCH) {                                  /* controller restarted: push the patch again */
        pd->adc_started = 0; pd->bq_ok = 0; pd->pdo_mv = pd->pdo_ma = 0;
        if (allow_slow && now - pd->patch_retry_ms >= 10000u) { pd->patch_retry_ms = now; load_patch(pd); }
        return;
    }
    if (pd->mode != PD_MODE_APP) { pd->bq_ok = 0; return; }
    uint8_t ps[2];
    reg_read(pd, REG_STATUS, pd->status, 5);
    if (reg_read(pd, REG_POWER_STATUS, ps, 2) == 2) pd->power_status = (uint16_t)((ps[1] << 8) | ps[0]);
    uint8_t pdo[6];
    if (reg_read(pd, REG_ACTIVE_PDO, pdo, 6) == 6) {                /* fixed-supply PDO: bits 19:10 voltage in 50 mV, 9:0 current in 10 mA */
        uint32_t w = (uint32_t)pdo[0] | (uint32_t)pdo[1] << 8 | (uint32_t)pdo[2] << 16 | (uint32_t)pdo[3] << 24;
        pd->pdo_mv = (uint16_t)(((w >> 10) & 0x3FF) * 50); pd->pdo_ma = (uint16_t)((w & 0x3FF) * 10);
    }
#if PD_CHARGER_BRIDGE
    if (!pd->adc_started) pd->adc_started = bq_start_adc(pd) == 0;
    if (pd->adc_started) bq_poll(pd);
#endif
}

void PD_Fill(const PD *pd, Athena_SpuStatus *st)
{
    st->pd_mode = pd->mode;
    st->pd_status = pd->status[0];
    st->vbat_mv = pd->bq_ok ? pd->vbat_mv : 0;
    st->vsys_mv = pd->bq_ok ? pd->vsys_mv : 0;
    st->vbus_mv = pd->bq_ok ? pd->vbus_mv : pd->pdo_mv;    /* no charger data: the negotiated USB-PD contract voltage */
    st->ibat_ma = pd->bq_ok ? pd->ibat_ma : 0;
    st->iin_ma  = pd->bq_ok ? pd->iin_ma : pd->pdo_ma;     /* ... and its current limit */
    st->chg_status = pd->bq_ok ? pd->chg_status : 0;
    if (pd->mode == PD_MODE_APP) st->flags |= SPU_FLAG_PD_APP;
    if (pd->bq_ok) st->flags |= SPU_FLAG_BQ_OK;
}
