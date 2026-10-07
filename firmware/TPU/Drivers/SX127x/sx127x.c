/*
 * Minimal LoRa point-to-point driver for the SX1278 (Ai-Thinker RA-02).
 * Register map and bit definitions come from Semtech's official LoRaMac-node
 * header sx1276Regs-LoRa.h (github.com/Lora-net/LoRaMac-node, BSD-3-Clause);
 * the SX1276/77/78/79 share one register map. Sequences follow the SX1276/77/78/79
 * datasheet (Rev. 7) sections 4.1.6 (LoRa modem), 4.1.2.2 (FIFO) and 5.4 (SPI).
 */
#include "sx127x.h"
#include "sx1276Regs-LoRa.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define XTAL_HZ          32000000ULL
#define RSSI_OFFSET_LF   (-164)       /* datasheet 5.5.5, RegPktRssiValue, LF port */
#define LORA_LF          (RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_FREQMODE_ACCESS_LF)

static void cs(int on) { HAL_GPIO_WritePin(RF_CS_GPIO_Port, RF_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

/* SPI (datasheet 4.3): first byte = wnr bit7 (1 = write) + 7-bit address, then data; burst auto-increments. */
static void wr(uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), v };
    cs(1); HAL_SPI_Transmit(&hspi1, tx, 2, 100); cs(0);
}

uint8_t SX127x_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0 }, rx[2] = { 0, 0 };
    cs(1); HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100); cs(0);
    return rx[1];
}

static void wr_burst(uint8_t reg, const uint8_t *d, uint8_t n)
{
    uint8_t tx[256 + 1];
    tx[0] = (uint8_t)(reg | 0x80);
    memcpy(&tx[1], d, n);
    cs(1); HAL_SPI_Transmit(&hspi1, tx, (uint16_t)(n + 1), 100); cs(0);
}

static void rd_burst(uint8_t reg, uint8_t *d, uint8_t n)
{
    uint8_t tx[256 + 1] = { 0 }, rx[256 + 1];
    tx[0] = (uint8_t)(reg & 0x7F);
    cs(1); HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(n + 1), 100); cs(0);
    memcpy(d, &rx[1], n);
}

static void set_mode(uint8_t m) { wr(REG_LR_OPMODE, (uint8_t)(LORA_LF | m)); }

int SX127x_Init(uint32_t freq_hz, int8_t tx_power_dbm)
{
    cs(0);
    /* 7.2.2 manual reset: NRESET low >= 100 us, then wait >= 5 ms */
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    if (SX127x_ReadReg(REG_LR_VERSION) != 0x12) return -1;      /* silicon revision, 6.4 */

    set_mode(RFLR_OPMODE_SLEEP);                                 /* LongRangeMode only writable in sleep */
    HAL_Delay(2);
    set_mode(RFLR_OPMODE_STANDBY);

    /* 4.1.4: Frf = f_RF * 2^19 / F_XOSC */
    uint64_t frf = ((uint64_t)freq_hz << 19) / XTAL_HZ;
    uint8_t f[3] = { (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf };
    wr_burst(REG_LR_FRFMSB, f, 3);

    wr(REG_LR_FIFOTXBASEADDR, 0x00);
    wr(REG_LR_FIFORXBASEADDR, 0x00);
    wr(REG_LR_LNA, RFLR_LNA_GAIN_G1 | RFLR_LNA_BOOST_HF_ON);
    wr(REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_BW_125_KHZ | RFLR_MODEMCONFIG1_CODINGRATE_4_5 | RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF);
    wr(REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_SF_7 | RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON);
    wr(REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_AGCAUTO_ON);
    wr(REG_LR_PREAMBLEMSB, 0x00);
    wr(REG_LR_PREAMBLELSB, 0x08);
    wr(REG_LR_SYNCWORD, 0x12);                                   /* private network sync word */
    wr(REG_LR_DIOMAPPING1, 0x00);                                /* IRQs are polled, DIOs unused */

    /* RA-02 wires the PA_BOOST output: 2..17 dBm (5.4.3); OCP 100 mA (5.4.4) */
    if (tx_power_dbm > 17) tx_power_dbm = 17;
    if (tx_power_dbm < 2)  tx_power_dbm = 2;
    wr(REG_LR_PADAC, 0x80 | RFLR_PADAC_20DBM_OFF);
    wr(REG_LR_OCP, RFLR_OCP_ON | 0x0B);
    wr(REG_LR_PACONFIG, (uint8_t)(RFLR_PACONFIG_PASELECT_PABOOST | (tx_power_dbm - 2)));

    SX127x_StartRx();
    return 0;
}

void SX127x_StartRx(void)
{
    wr(REG_LR_IRQFLAGS, 0xFF);
    wr(REG_LR_FIFOADDRPTR, 0x00);
    set_mode(RFLR_OPMODE_RECEIVER);                              /* RXCONTINUOUS */
}

int SX127x_Send(const uint8_t *data, uint8_t len)
{
    set_mode(RFLR_OPMODE_STANDBY);
    wr(REG_LR_FIFOADDRPTR, 0x00);
    wr_burst(REG_LR_FIFO, data, len);
    wr(REG_LR_PAYLOADLENGTH, len);
    wr(REG_LR_IRQFLAGS, 0xFF);
    set_mode(RFLR_OPMODE_TRANSMITTER);
    uint32_t t0 = HAL_GetTick(); int rc = -1;
    uint32_t budget_ms = 150u + (uint32_t)len * 4u;             /* SF7/125k: ~90 ms for 45 B, ~420 ms for 255 B */
    while (HAL_GetTick() - t0 < budget_ms) {
        if (SX127x_ReadReg(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE) { rc = 0; break; }
    }
    SX127x_StartRx();
    return rc;
}

int SX127x_Receive(uint8_t *buf, uint8_t maxlen, int16_t *rssi_dbm, int8_t *snr_db)
{
    uint8_t irq = SX127x_ReadReg(REG_LR_IRQFLAGS);
    if (!(irq & RFLR_IRQFLAGS_RXDONE)) return 0;
    wr(REG_LR_IRQFLAGS, 0xFF);
    if (irq & RFLR_IRQFLAGS_PAYLOADCRCERROR) return 0;
    uint8_t n = SX127x_ReadReg(REG_LR_RXNBBYTES);
    if (n > maxlen) n = maxlen;
    wr(REG_LR_FIFOADDRPTR, SX127x_ReadReg(REG_LR_FIFORXCURRENTADDR));
    rd_burst(REG_LR_FIFO, buf, n);
    if (snr_db)   *snr_db   = (int8_t)((int8_t)SX127x_ReadReg(REG_LR_PKTSNRVALUE) / 4);
    if (rssi_dbm) *rssi_dbm = (int16_t)(RSSI_OFFSET_LF + SX127x_ReadReg(REG_LR_PKTRSSIVALUE));
    return n;
}
