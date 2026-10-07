/*
 * imu_interface.h - the three ICM-45686 IMUs on SPI1 (MPU)
 *
 *   IMU1 CS = PE11, IMU2 CS = PE13, IMU3 CS = PE15, shared SCK/MISO/MOSI, FSYNC on PE9.
 *   SPI mode 0, 10 MHz. Chip select is ACTIVE LOW (pulled low only for a transfer).
 */
#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "athena.h"

#define IMU_COUNT          3
#define IMU_ACCEL_FSR_G    32.0f    /* rockets pull >4 g: use the full +-32 g range */
#define IMU_GYRO_FSR_DPS   2000.0f
#define IMU_ODR_HZ         400

/* Initialises all three IMUs. Returns a bitmask: bit n set = IMU n+1 answered WHO_AM_I and configured. */
uint8_t IMU_Init(void);

/* Reads IMU idx (0..2) into d: accel_g, gyro_dps, temperature_c, timestamp. Returns 0 on success. */
int IMU_Read(int idx, IMU_Data *d);

/* Reads every IMU in mask and votes: median of 3, mean of 2, or the single survivor.
 * Returns the mask of IMUs whose reads succeeded (0 = no data). */
uint8_t IMU_ReadFused(uint8_t mask, IMU_Data *out);

#ifdef __cplusplus
}
#endif
#endif /* IMU_INTERFACE_H */
