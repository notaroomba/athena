/**
  ******************************************************************************
  * @file    pd.h
  * @brief   TPS25751 USB-PD Controller Configuration Header
  ******************************************************************************
  */

#ifndef __PD_H
#define __PD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Exported defines ----------------------------------------------------------*/
// TPS25751 I2C addresses (7-bit format for use with HAL functions)
// HAL functions automatically shift these left by 1 to create 8-bit format
#define TPS25751_I2C_ADDR        0x20  // Address Index #1: 7-bit addr (becomes 0x40 in 8-bit)

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
// External firmware arrays from athena_pd_lr.c and athena_pd_ff.c
extern const char tps25750x_lowRegion_i2c_array[];
extern int gSizeLowRegionArray;
extern const char tps25750x_fullFlash_i2c_array[];
extern int gSizeFullFlashArray;

/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef TPS25751_LoadFirmware(I2C_HandleTypeDef *hi2c, 
                                        const char *firmware_data, 
                                        int firmware_size, 
                                        uint8_t device_addr);

HAL_StatusTypeDef TPS25751_WriteEEPROM(I2C_HandleTypeDef *hi2c, 
                                       const char *data, 
                                       int size);

#ifdef __cplusplus
}
#endif

#endif /* __PD_H */
