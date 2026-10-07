/*
 * logger.h - flight log on the TPU: every link frame (MPU state, GPS fix, telemetry, status
 * text) is appended to a file on the microSD card AND to a ring log in the W25Q256 flash.
 *
 *  - The SD card is treated as removable: card detect (PD1, low = inserted, pull-up on the
 *    board) is debounced, the card is initialised only when present, every write has a
 *    timeout, the file is f_sync'ed once a second so a yanked card loses at most ~1 s, and a
 *    re-inserted card is re-mounted and a new file started. Nothing blocks when it is absent.
 *  - The flash log survives without a card and without a filesystem. It is the raw frame
 *    stream (self-synchronising, CRC per frame): 'D' on the USB console dumps it, 'E' restarts
 *    it. tools/athlog.py does both and converts the stream to CSV.
 */
#ifndef LOGGER_H
#define LOGGER_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stddef.h>

void Logger_Init(void);
void Logger_Write(const uint8_t *data, uint32_t len);   /* append raw frame bytes (non-blocking) */
void Logger_Task(void);                                  /* call every main-loop pass */
void Logger_StatusLine(char *out, size_t n);
void Logger_UsbRx(const uint8_t *buf, uint32_t len);     /* console commands from USB CDC (ISR context) */

#ifdef __cplusplus
}
#endif
#endif
