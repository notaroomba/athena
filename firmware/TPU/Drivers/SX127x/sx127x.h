/*
 * sx127x.h - Ai-Thinker RA-02 (Semtech SX1278) LoRa radio on SPI1 (TPU)
 *
 *   NSS = PA4 (active low), NRESET = PA0 (active low), DIO0..5 = PB1,PB0,PC5,PC4,PA2,PA3 (inputs).
 *   SPI mode 0, <= 10 MHz. Register names come from Semtech's sx1276Regs-LoRa.h.
 *   Interrupt flags are polled from RegIrqFlags so the DIO
 *   lines are not needed; they must simply not be driven by the MCU.
 */
#ifndef SX127X_H
#define SX127X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Returns 0 when RegVersion reads 0x12. SF7 / BW125 kHz / CR4-5 / CRC on / explicit header. */
int  SX127x_Init(uint32_t freq_hz, int8_t tx_power_dbm);
/* Blocking transmit (<= 255 bytes), then re-arms continuous receive. Returns 0 on TxDone,
 * -1 if TxDone never came (radio fault: caller should re-init). */
int  SX127x_Send(const uint8_t *data, uint8_t len);
/* Non-blocking: returns payload length if a CRC-good packet is waiting, 0 otherwise. */
int  SX127x_Receive(uint8_t *buf, uint8_t maxlen, int16_t *rssi_dbm, int8_t *snr_db);
void SX127x_StartRx(void);
uint8_t SX127x_ReadReg(uint8_t reg);

#ifdef __cplusplus
}
#endif
#endif
