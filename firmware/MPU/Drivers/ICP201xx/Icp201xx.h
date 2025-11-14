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
 *  @brief    High-level driver for Icp201xx devices
 *  @ingroup  DriverIcp201xx
 *  @{
 */

#ifndef _INV_ICP201XX_H_
#define _INV_ICP201XX_H_


#include "Icp201xxSerif.h"
#include "Icp201xxDefs.h"

#include <stdint.h>
#include <assert.h>		
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct inv_icp201xx {
	inv_icp201xx_serif_t			serif;				/*!< serial interface structure*/
	icp201xx_FIFO_readout_mode_t	fifo_readout_mode;  /* ICP 201xx FIFO read out mode  */ 
}inv_icp201xx_t;

/** @brief Initialize ICP201xx .
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] serif Pointer to serial interface struct updated with read,write function pointers and interface mode. 
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_init(inv_icp201xx_t * s,const inv_icp201xx_serif_t * serif);


/** @brief Configure ICP201xx to Start/Stop Pressure and/or Temperature measurements.( All other required fields are set to default values )
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] op_mode  Operation mode to configure BW and ODR 
 * @param[in] fifo_read_mode FIFO read out mode configuration.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_config(inv_icp201xx_t * s,icp201xx_op_mode_t op_mode,icp201xx_FIFO_readout_mode_t fifo_read_mode);
									
/** @brief Configure ICP201xx to Start/Stop Pressure and/or Temperature measurements.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] op_mode  Operation mode to configure BW and ODR .
 * @param[in] meas_mode  Measurements mode .
 * @param[in] forced_meas_trigger  Meas trigger mode .
 * @param[in] pow_mode  Power mode .
 * @param[in] fifo_readout_mode  FIFO Read out mode  .
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_adv_config(inv_icp201xx_t * s,
									   icp201xx_op_mode_t op_mode,
									   icp201xx_meas_mode_t meas_mode,
									   icp201xx_forced_meas_trigger_t forced_meas_trigger,
									   icp201xx_power_mode_t  pow_mode,
									   icp201xx_FIFO_readout_mode_t fifo_readout_mode);


/** @brief  Stop ICP201xx from doing measurements and set ICP201xx in standby mode
 * other configuration ( FIFO, FIFO read out mode etc  ) are not changed.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_set_standby(inv_icp201xx_t * s);


/** @brief  Start ICP201xx in continuous meas mode with existing mode select.
 *          Other configuration ( FIFO, FIFO read out mode, mode select, int_mask etc  ) are not changed. 
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_set_continuous_meas_mode(inv_icp201xx_t * s);

/** @brief Trigger ICP201xx to perform one measurement and update FIFO
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_trigger_meas(inv_icp201xx_t * s);

/** @brief Config Pressure sensor notifications.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] press_int_mask interrupt mask for Pressure sensor( need to be set with ICP201XX_INT_MASK_PRESS_XXXXX)
 * @param[in] press_abs  Pressure overrun/underrun value. Valid only if ICP201XX_INT_MASK_PRESS_ABS is set in press_int_mask interrupt.
 * @param[in] press_delta  Delta pressure overrun/underrun value. Valid only if ICP201XX_INT_MASK_PRESS_DELTA is set in press_int_mask interrupt.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_set_press_notification_config(inv_icp201xx_t * s,uint8_t press_int_mask,int16_t press_abs , int16_t press_delta);

/** @brief Read Pressure sensor notifications from ICP201xx.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[out] press_int_mask interrupt mask for Pressure sensor( need to be set with ICP201XX_INT_MASK_PRESS_XXXXX)
 * @param[out] press_abs  Pressure overrun/underrun value. Valid only if ICP201XX_INT_MASK_PRESS_ABS is set in press_int_mask interrupt.
 * @param[out] press_delta  Delta pressure overrun/underrun value. Valid only if ICP201XX_INT_MASK_PRESS_DELTA is set in press_int_mask interrupt.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_get_press_notification_config(inv_icp201xx_t * s,uint8_t *press_int_mask,int16_t *press_abs , int16_t *press_delta);

/** @brief Config FIFO notifications.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] fifo_int_mask interrupt mask ( need to be set with ICP201XX_INT_MASK_FIFO_XXXXX)
 * @param[in] fifo_wmk_high  FIFO high watermark value. Valid only if ICP201XX_INT_MASK_FIFO_WMK_HIGH is set in fifo_int_mask. 
 *                             if set to 0 high watermark check is disabled.
 * @param[in] fifo_wmk_low  FIFO Low watermark value. Valid only if ICP201XX_INT_MASK_FIFO_WMK_LOW is set in fifo_int_mask.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_set_fifo_notification_config(inv_icp201xx_t * s,uint8_t fifo_int_mask, uint8_t fifo_wmk_high,uint8_t fifo_wmk_low);


/** @brief Read FIFO notifications from ICP201xx.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[out] fifo_int_mask interrupt mask ( need to be set with ICP201XX_INT_MASK_FIFO_XXXXX)
 * @param[out] fifo_wmk_high  FIFO high watermark value. Valid only if ICP201XX_INT_MASK_FIFO_WMK_HIGH is set in fifo_int_mask. 
 *                             if set to 0 high watermark check is disabled.
 * @param[out] fifo_wmk_low  FIFO Low watermark value. Valid only if ICP201XX_INT_MASK_FIFO_WMK_LOW is set in fifo_int_mask.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_get_fifo_notification_config(inv_icp201xx_t * s,uint8_t *fifo_int_mask, uint8_t *fifo_wmk_high,uint8_t *fifo_wmk_low);

/** @brief Flush FIFO.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_flush_fifo(inv_icp201xx_t * s);

/** @brief Reads interrupt status.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[out] int_status current interrupt status as read from mems ( ICP201XX_INT_STATUS_XXXXX ).
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_get_int_status(inv_icp201xx_t * s,uint8_t *int_status);

int inv_icp201xx_clear_int_status(inv_icp201xx_t * s,uint8_t int_status);

/** @brief Reads device status.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[out] dev_status current device status as read from mems 
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_get_device_status(inv_icp201xx_t * s,uint8_t *dev_status);

/** @brief Reads number of packets in FIFO.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[out] fifo_cnt current number of packets in FIFO.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_get_fifo_count(inv_icp201xx_t * s,uint8_t *fifo_cnt);

/** @brief Reads up to requested number of packets from FIFO. 
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] req_packet_cnt Number of packets to be read from FIFO.
 * @param[out] data raw data read from FIFO.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_get_fifo_data(inv_icp201xx_t * s,uint8_t req_packet_cnt, uint8_t *data);

/** @brief Process raw data based on FIFO read out mode.
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] packet_cnt Number of packets .
 * @param[in] data raw data.
 * @param[out] pressure raw pressure data .
 * @param[out] temperature raw temperature data.
 * @return    0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_process_raw_data(inv_icp201xx_t * s,uint8_t packet_cnt, uint8_t *data,int32_t * pressure, int32_t * temperature);

int inv_icp201xx_soft_reset(inv_icp201xx_t * s);


int inv_icp201xx_OTP_bootup_cfg(inv_icp201xx_t * s);

/** @brief Get Device ID and version from ICP201xx 
 * @param[in] s Pointer to the driver context structure inv_icp201xx_t
 * @param[in] pointer to device id.
 * @param[in] pointer to version.
 * @return    0 in case of success( device is validated), negative value on error. See enum inv_status
 */
int inv_icp201xx_get_devid_version(inv_icp201xx_t * s,uint8_t *device_id,uint8_t *ver);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICP201XX_H_ */
