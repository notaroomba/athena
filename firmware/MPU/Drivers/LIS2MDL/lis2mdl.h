/*
 * lis2mdl.h - LIS2MDL magnetometer on SPI3 (MPU), 4-wire SPI, CS = PB1.
 *
 * Thin platform glue around ST's official register driver (lis2mdl_reg.c/.h,
 * github.com/STMicroelectronics/lis2mdl-pid, BSD-3-Clause), same pattern as
 * the InvenSense ICM/ICP drivers already in this tree.
 *
 * Wiring (from the PCB): SCK PC10, MOSI PB2 (LIS2MDL pin 4 SDI), MISO PC11 (pin 7 SDO).
 * The part boots in 3-wire SPI; the first write switches it to 4-wire and disables I2C.
 * SPI mode 3 (clock idle high, sample on the second edge), <= 10 MHz.
 */
#ifndef LIS2MDL_H
#define LIS2MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int LIS2MDL_Init(void);              /* 0 ok, -1 WHO_AM_I mismatch, -2 bus error */
int LIS2MDL_Read(float gauss[3]);    /* 0 new sample, 1 nothing new yet, -1 bus error */
int LIS2MDL_ReadTemp(float *deg_c);  /* 0 ok */

#ifdef __cplusplus
}
#endif
#endif
