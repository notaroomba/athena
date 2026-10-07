/*
 * pd.h - USB-PD controller (TI TPS25751, I2C1 @ 0x20) and the battery charger (TI BQ25713) behind it.
 *
 * Board wiring (schematic sheet 2): the SPU's I2C1 reaches only the TPS25751 host port (I2Ct, pins
 * 8/9). The BQ25713 sits on the TPS25751's controller port (I2Cc, pins 16/17), so every charger
 * access goes through the 'I2Cr'/'I2Cw' 4CC tasks of the TPS25751 host interface (TRM SLVUCR8).
 * There is no EEPROM on the board, and the ADCIN strapping is address index #1 / AlwaysEnableSink:
 * the controller boots into 'PTCH' mode and waits for the SPU to push the patch bundle generated
 * by TI's Application Customization Tool (Core/Src/athena_pd_lr.c). Until that happens it is a dumb
 * 5 V sink with USB PD disabled and the charger unreachable.
 */
#ifndef PD_H
#define PD_H
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "athena_link.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TPS25751_ADDR   0x20   /* 7-bit, ADCIN1=7 / ADCIN2=5 -> address index #1 (data sheet table 8-6) */
#define TPS_BURST_ADDR  0x30   /* second target address used while the patch streams in (TI SLVAFV8 example) */
#define BQ25713_ADDR    0x6B   /* 7-bit (D6h/D7h) */

enum { PD_MODE_NONE = 0, PD_MODE_PTCH, PD_MODE_APP, PD_MODE_BOOT };

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t  present;          /* answered on I2C1 */
    uint8_t  mode;             /* PD_MODE_* */
    uint8_t  bq_ok;            /* last charger read succeeded */
    uint8_t  adc_started;
    uint8_t  status[5];        /* TPS25751 STATUS (0x1A) */
    uint16_t power_status;     /* TPS25751 POWER_STATUS (0x3F) */
    uint16_t vbat_mv, vsys_mv, vbus_mv, iin_ma, chg_status;
    int16_t  ibat_ma;
    uint32_t errors, next_ms, patch_retry_ms;
} PD;

void        PD_Init(PD *pd, I2C_HandleTypeDef *i2c);   /* probe, load the patch if the controller waits for one */
void        PD_Task(PD *pd, uint32_t now_ms);          /* 1 Hz: mode, PD status, charger ADC */
void        PD_Fill(const PD *pd, Athena_SpuStatus *st);
const char *PD_ModeName(uint8_t mode);

#ifdef __cplusplus
}
#endif
#endif
