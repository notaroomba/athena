/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/** @defgroup DriverIcp201xx Icp201xx driver
 *  @brief    Low-level driver for Icp201xx devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICP201XX_DEFS_H_
#define _INV_ICP201XX_DEFS_H_

#include "InvError.h"
#include "Icp201xxSerif.h"

#include <stdint.h>
#include <assert.h>		
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXPECTED_DEVICE_ID 0x63

/* ICP 201XX Operation Mode */
typedef enum icp201xx_op_mode{  
	ICP201XX_OP_MODE0 = 0 ,  /* Mode 0: Bw:6.25 Hz ODR: 25Hz */
	ICP201XX_OP_MODE1     ,  /* Mode 1: Bw:30 Hz ODR: 120Hz */
	ICP201XX_OP_MODE2     ,  /* Mode 2: Bw:10 Hz ODR: 40Hz */
	ICP201XX_OP_MODE3     ,  /* Mode 3: Bw:0.5 Hz ODR: 2Hz */ 
	ICP201XX_OP_MODE4     ,  /* Mode 4: User configurable Mode */ 
	ICP201XX_OP_MODE_MAX
}icp201xx_op_mode_t;

typedef enum icp201xx_forced_meas_trigger {
	ICP201XX_FORCE_MEAS_STANDBY = 0,			/* Stay in Stand by */
	ICP201XX_FORCE_MEAS_TRIGGER_FORCE_MEAS = 1	/* Trigger for forced measurements */
}icp201xx_forced_meas_trigger_t;

typedef enum icp201xx_meas_mode
{
	ICP201XX_MEAS_MODE_FORCED_TRIGGER = 0, /* Force trigger mode based on icp201xx_forced_meas_trigger_t **/
	ICP201XX_MEAS_MODE_CONTINUOUS = 1   /* Continuous measurements based on selected mode ODR settings*/
}icp201xx_meas_mode_t;

typedef enum icp201xx_power_mode
{
	ICP201XX_POWER_MODE_NORMAL = 0,  /* Normal Mode: Device is in standby and goes to active mode during the execution of a measurement */
	ICP201XX_POWER_MODE_ACTIVE = 1   /* Active Mode: Power on DVDD and enable the high frequency clock */
}icp201xx_power_mode_t;


typedef enum icp201xx_FIFO_readout_mode
{
	ICP201XX_FIFO_READOUT_MODE_PRES_TEMP = 0,   /* Pressure and temperature as pair and address wraps to the start address of the Pressure value ( pressure first ) */
	ICP201XX_FIFO_READOUT_MODE_TEMP_ONLY = 1,   /* Temperature only reporting */
	ICP201XX_FIFO_READOUT_MODE_TEMP_PRES = 2,   /* Pressure and temperature as pair and address wraps to the start address of the Temperature value ( Temperature first ) */
	ICP201XX_FIFO_READOUT_MODE_PRES_ONLY = 3    /* Pressure only reporting */
}icp201xx_FIFO_readout_mode_t;


#define ICP201XX_INT_MASK_PRESS_DELTA      (0X01 << 6)
#define ICP201XX_INT_MASK_PRESS_ABS        (0X01 << 5)
#define ICP201XX_INT_MASK_FIFO_WMK_LOW     (0X01 << 3)
#define ICP201XX_INT_MASK_FIFO_WMK_HIGH    (0X01 << 2)
#define ICP201XX_INT_MASK_FIFO_UNDER_FLOW  (0X01 << 1)
#define ICP201XX_INT_MASK_FIFO_OVER_FLOW   (0X01 << 0)	



#define ICP201XX_INT_STATUS_PRESS_DELTA      (0X01 << 6)
#define ICP201XX_INT_STATUS_PRESS_ABS        (0X01 << 5)
#define ICP201XX_INT_STATUS_FIFO_WMK_LOW     (0X01 << 3)
#define ICP201XX_INT_STATUS_FIFO_WMK_HIGH    (0X01 << 2)
#define ICP201XX_INT_STATUS_FIFO_UNDER_FLOW  (0X01 << 1)
#define ICP201XX_INT_STATUS_FIFO_OVER_FLOW   (0X01 << 0)


#ifdef __cplusplus
}
#endif

#endif /* _INV_ICP201XX_DEFS_H_ */
