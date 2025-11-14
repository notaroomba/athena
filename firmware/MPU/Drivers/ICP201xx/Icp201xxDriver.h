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
 *  @ingroup  DriverIcp201xx
 *  @{
 */
#ifndef _INV_ICP201XX_DRIVER_H_
#define _INV_ICP201XX_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Generic read write api's if upper layer know reg addr and vlue to be writen/read */
int inv_icp201xx_rd_reg(inv_icp201xx_serif_t  * s, uint8_t reg, uint8_t *value);
int inv_icp201xx_wr_reg(inv_icp201xx_serif_t  * s, uint8_t reg, uint8_t new_value);


/* Main/ OTP Registers */
#define MPUREG_TRIM1_MSB						0X05
#define MPUREG_TRIM2_LSB						0X06
#define MPUREG_TRIM2_MSB						0X07
#define MPUREG_DEVICE_ID						0X0C
#define MPUREG_OTP_MTP_OTP_CFG1					0XAC
#define MPUREG_OTP_MTP_MR_LSB					0XAD
#define MPUREG_OTP_MTP_MR_MSB					0XAE
#define MPUREG_OTP_MTP_MRA_LSB					0XAF
#define MPUREG_OTP_MTP_MRA_MSB					0XB0
#define MPUREG_OTP_MTP_MRB_LSB					0XB1
#define MPUREG_OTP_MTP_MRB_MSB					0XB2
#define MPUREG_OTP_MTP_OTP_ADDR					0XB5
#define MPUREG_OTP_MTP_OTP_CMD					0XB6
#define MPUREG_OTP_MTP_RD_DATA					0XB8
#define MPUREG_OTP_MTP_OTP_STATUS				0xB9
#define MPUREG_OTP_DEBUG2						0XBC
#define MPUREG_MASTER_LOCK						0XBE
#define MPUREG_OTP_MTP_OTP_STATUS2				0XBF


#define MPUREG_MODE_SELECT			0xC0
#define MPUREG_INTERRUPT_STATUS     0xC1
#define MPUREG_INTERRUPT_MASK		0xC2
#define MPUREG_FIFO_CONFIG			0xC3
#define MPUREG_FIFO_FILL			0xC4
#define MPUREG_SPI_MODE				0xC5
#define MPUREG_PRESS_ABS_LSB		0xC7
#define MPUREG_PRESS_ABS_MSB		0xC8
#define MPUREG_PRESS_DELTA_LSB      0xC9
#define MPUREG_PRESS_DELTA_MSB      0xCA
#define MPUREG_DEVICE_STATUS		0xCD
#define MPUREG_I3C_INFO				0xCE
#define MPUREG_VERSION   			0XD3
#define MPUREG_FIFO_BASE            0XFA

/********************************************
Register Name:
Bank         : Main registers
Register Type: READ/WRITE
Register Address: 5(Decimal); 5 (Hex)
********************************************/
#define BIT_PEFE_OFFSET_TRIM_MASK			0x3F

int inv_icp201xx_wr_pefe_offset_trim(inv_icp201xx_serif_t  * s,uint8_t new_value);


/********************************************
Register Name:
Bank         : Main registers
Register Type: READ/WRITE
Register Address: 6(Decimal); 6 (Hex)
********************************************/
#define BIT_HFOSC_OFFSET_TRIM_MASK			0x7F

int inv_icp201xx_wr_hfosc_trim(inv_icp201xx_serif_t  * s,uint8_t new_value);

/********************************************
Register Name:
Bank         : Main registers
Register Type: READ/WRITE
Register Address: 7(Decimal); 7 (Hex)
********************************************/
#define BIT_PEFE_GAIN_TRIM_MASK			0x70

#define BIT_PEFE_GAIN_TRIM_POS			4

int inv_icp201xx_wr_pefe_gain_trim(inv_icp201xx_serif_t  * s, uint8_t new_value);

/********************************************
Register Name:
Bank         : Main registers
Register Type: READ/WRITE
Register Address: 12(Decimal); 0X0C (Hex)
********************************************/
int inv_icp201xx_rd_device_id(inv_icp201xx_serif_t  * s, uint8_t *value);


/********************************************
Register Name: OTP Config 1
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 172 (Decimal); AC (Hex)
********************************************/
#define BIT_OTP_CONFIG1_WRITE_SWITCH_MASK			0x02
#define BIT_OTP_CONFIG1_OTP_ENABLE_MASK				0x01
#define BIT_OTP_CONFIG1_WRITE_SWITCH_POS			1
int inv_icp201xx_wr_otp_write_switch(inv_icp201xx_serif_t  * s,uint8_t new_value);
int inv_icp201xx_wr_otp_enable(inv_icp201xx_serif_t  * s,uint8_t new_value);


/********************************************
Register Name: OTP MR LSB
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 173 (Decimal); AD (Hex)
********************************************/
int inv_icp201xx_wr_otp_mr_lsb(inv_icp201xx_serif_t  * s,uint8_t new_value);

/********************************************
Register Name: OTP MR MSB
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 174 (Decimal); AE (Hex)
********************************************/
int inv_icp201xx_wr_otp_mr_msb(inv_icp201xx_serif_t  * s,uint8_t new_value);
/********************************************
Register Name: OTP MRA LSB
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 175 (Decimal); AF (Hex)
********************************************/
int inv_icp201xx_wr_otp_mra_lsb(inv_icp201xx_serif_t  * s,uint8_t new_value);
int inv_icp201xx_rd_otp_mra_lsb(inv_icp201xx_serif_t  * s, uint8_t *value);

/********************************************
Register Name: OTP MRA MSB
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 176 (Decimal); B0 (Hex)
********************************************/
int inv_icp201xx_wr_otp_mra_msb(inv_icp201xx_serif_t  * s,uint8_t new_value);
int inv_icp201xx_rd_otp_mra_msb(inv_icp201xx_serif_t  * s, uint8_t *value);

/********************************************
Register Name: OTP MRB LSB
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 177 (Decimal); B1 (Hex)
********************************************/
int inv_icp201xx_wr_otp_mrb_lsb(inv_icp201xx_serif_t  * s,uint8_t new_value);

/********************************************
Register Name: OTP MRB MSB
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 178 (Decimal); B2 (Hex)
********************************************/
int inv_icp201xx_wr_otp_mrb_msb(inv_icp201xx_serif_t  * s,uint8_t new_value);

/********************************************
Register Name: OTP ADDR
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 181 (Decimal); B5 (Hex)
********************************************/
int inv_icp201xx_wr_otp_addr(inv_icp201xx_serif_t  * s,uint8_t new_value);
int inv_icp201xx_rd_otp_addr(inv_icp201xx_serif_t  * s, uint8_t *value);

/********************************************
Register Name: OTP CMD
Bank         : otp register
Register Type: READ/WRITE
Register Address: 182 (Decimal); B6 (Hex)
********************************************/
int inv_icp201xx_wr_otp_cmd(inv_icp201xx_serif_t  * s,uint8_t new_value);
int inv_icp201xx_rd_otp_cmd(inv_icp201xx_serif_t  * s, uint8_t *value);


/********************************************
Register Name: OTP Read Reg
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 184 (Decimal); B8 (Hex)
********************************************/
int inv_icp201xx_rd_otp_reg_data(inv_icp201xx_serif_t  * s, uint8_t *value);


/********************************************
Register Name: OTP status
Bank         : otp registers
Register Type: READ/WRITE
Register Address: 185 (Decimal); B9 (Hex)
********************************************/
int inv_icp201xx_rd_otp_status(inv_icp201xx_serif_t  * s, uint8_t *value);

/********************************************
Register Name: OTP Debug2
Bank         : OTP registers
Register Type: READ/WRITE
Register Address:  180(Decimal); BC (Hex)
********************************************/
#define BIT_OTP_DBG2_RESET_MASK   0x80
#define BIT_OTP_DBG2_RESET_POS	  7

int inv_icp201xx_wr_otp_dbg2_reset(inv_icp201xx_serif_t  * s,uint8_t new_value);


/********************************************
Register Name: Master lock
Bank         : OTP registers
Register Type: READ/WRITE
Register Address:  190(Decimal); BE (Hex)
********************************************/
int inv_icp201xx_wr_master_lock(inv_icp201xx_serif_t  * s,uint8_t new_value);

/********************************************
Register Name: OTP Status2
Bank         : OTP registers
Register Type: READ/WRITE
Register Address:  191(Decimal); BF (Hex)
********************************************/
#define BIT_OTP_STATUS2_BOOTUP_STATUS_MASK   0x01

int inv_icp201xx_rd_boot_up_status(inv_icp201xx_serif_t  * s, uint8_t *value);
int inv_icp201xx_wr_boot_up_status (inv_icp201xx_serif_t  * s,uint8_t new_value);



/********************************************
Register Name: MODE_SELECT
Register Type: READ/WRITE
Register Address: 192 (Decimal); C0 (Hex)
********************************************/
#define BIT_MEAS_CONFIG_MASK          0xE0
#define BIT_FORCED_MEAS_TRIGGER_MASK  0x10
#define BIT_MEAS_MODE_MASK            0x08
#define BIT_POWER_MODE_MASK           0x04
#define BIT_FIFO_READOUT_MODE_MASK    0x03

#define BIT_MEAS_CONFIG_POS				5
#define BIT_FORCED_MEAS_TRIGGER_POS	    4
#define BIT_FORCED_MEAS_MODE_POS	    3
#define BIT_FORCED_POW_MODE_POS			2	

int inv_icp201xx_wr_mode_select(inv_icp201xx_serif_t  * s, uint8_t new_value);
int inv_icp201xx_rd_mode_select(inv_icp201xx_serif_t * s, uint8_t *value);

int inv_icp201xx_wr_meas_config(inv_icp201xx_serif_t  * s, icp201xx_op_mode_t new_value);
int inv_icp201xx_rd_meas_config(inv_icp201xx_serif_t * s, icp201xx_op_mode_t *value);

int inv_icp201xx_wr_forced_meas_trigger(inv_icp201xx_serif_t  * s, icp201xx_forced_meas_trigger_t new_value);
int inv_icp201xx_rd_forced_meas_trigger(inv_icp201xx_serif_t * s, icp201xx_forced_meas_trigger_t *value);

int inv_icp201xx_wr_meas_mode(inv_icp201xx_serif_t  * s, icp201xx_meas_mode_t new_value);
int inv_icp201xx_rd_meas_mode(inv_icp201xx_serif_t * s, icp201xx_meas_mode_t *value);

int inv_icp201xx_wr_pow_mode(inv_icp201xx_serif_t  * s, icp201xx_power_mode_t new_value);
int inv_icp201xx_rd_pow_mode(inv_icp201xx_serif_t * s, icp201xx_power_mode_t *value);

int inv_icp201xx_wr_fifo_readout_mode(inv_icp201xx_serif_t  * s, icp201xx_FIFO_readout_mode_t new_value);
int inv_icp201xx_rd_fifo_readout_mode(inv_icp201xx_serif_t * s, icp201xx_FIFO_readout_mode_t *value);


/********************************************
Register Name: INTERRUPT_STATUS
Register Type: READ/WRITE
Register Address: 193 (Decimal); C1(Hex)
********************************************/
int inv_icp201xx_wr_int_status(inv_icp201xx_serif_t * s, uint8_t new_value);

int inv_icp201xx_rd_int_status(inv_icp201xx_serif_t * s, uint8_t *value);

/********************************************
Register Name: INTERRUPT_MASK
Register Type: READ/WRITE
Register Address: 194 (Decimal); C2(Hex)
********************************************/
int inv_icp201xx_wr_int_mask(inv_icp201xx_serif_t * s, uint8_t new_value);

int inv_icp201xx_rd_int_mask(inv_icp201xx_serif_t * s, uint8_t *value);

/********************************************
Register Name: FIFO_CONFIG
Register Type: READ/WRITE
Register Address: 195 (Decimal); C3(Hex)
********************************************/
#define BIT_FIFO_WM_HIGH_MASK    0xF0
#define BIT_FIFO_WM_LOW_MASK     0x0F

#define BIT_FIFO_WM_HIGH_POS     3

int inv_icp201xx_wr_fifo_config(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_fifo_config(inv_icp201xx_serif_t * s, uint8_t *value);

int inv_icp201xx_wr_fifo_wm_high(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_fifo_wm_high(inv_icp201xx_serif_t * s, uint8_t *value);

int inv_icp201xx_wr_fifo_wm_low(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_fifo_wm_low(inv_icp201xx_serif_t * s, uint8_t *value);


/********************************************
Register Name: FIFO_FILL
Register Type: READ/WRITE
Register Address: 196 (Decimal); C4 (Hex)
********************************************/
#define BIT_FIFO_FLUSH_MASK				0x80
#define BIT_FIFO_EMPTY_STATUS_MASK		0x40
#define BIT_FIFO_FULL_STATUS_MASK		0x20
#define BIT_FIFO_LEVEL_MASK				0x1F

#define BIT_FIFO_EMPTY_POS     6
#define BIT_FIFO_FULL_POS      5

int inv_icp201xx_wr_fifo_fill(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_fifo_fill(inv_icp201xx_serif_t * s, uint8_t *value);


int inv_icp201xx_wr_flush_fifo(inv_icp201xx_serif_t * s);
int inv_icp201xx_rd_fifo_empty_status(inv_icp201xx_serif_t * s, uint8_t *value);
int inv_icp201xx_rd_fifo_full_status(inv_icp201xx_serif_t * s, uint8_t *value);
int inv_icp201xx_rd_fifo_level(inv_icp201xx_serif_t * s, uint8_t *value);

/********************************************
Register Name: SPI_MODE
Register Type: READ/WRITE
Register Address: 197 (Decimal); C5 (Hex)
********************************************/
typedef enum icp201xx_spi_mode {
	ICP201XX_SPI_MODE_4_WIRE = 0,
	ICP201XX_SPI_MODE_3_WIRE,
}icp201xx_spi_mode_t;

#define BIT_FIFO_SPI_MODE_MASK    0x01  
                                 // 0: SPI 4-WIRE
								 // 1: SPI 3-WIRE
int inv_icp201xx_wr_spi_mode(inv_icp201xx_serif_t * s, icp201xx_spi_mode_t new_value);
int inv_icp201xx_rd_spi_mode(inv_icp201xx_serif_t * s, icp201xx_spi_mode_t *value);

/********************************************
Register Name: PRESS_ABS_LSB
Register Type: READ/WRITE
Register Address: 199 (Decimal); C7 (Hex)
********************************************/
int inv_icp201xx_wr_press_abs_lsb(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_press_abs_lsb(inv_icp201xx_serif_t * s, uint8_t *value);
/********************************************
Register Name: PRESS_ABS_MSB
Register Type: READ/WRITE
Register Address: 200 (Decimal); C8 (Hex)
********************************************/
int inv_icp201xx_wr_press_abs_msb(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_press_abs_msb(inv_icp201xx_serif_t * s, uint8_t *value);
/********************************************
Register Name: PRESS_DELTA_LSB
Register Type: READ/WRITE
Register Address: 201 (Decimal); C9 (Hex)
********************************************/
int inv_icp201xx_wr_press_delta_lsb(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_press_delta_lsb(inv_icp201xx_serif_t * s, uint8_t *value);
/********************************************
Register Name: PRESS_DELTA_MSB
Register Type: READ/WRITE
Register Address: 202 (Decimal); CA (Hex)
********************************************/
int inv_icp201xx_wr_press_delta_msb(inv_icp201xx_serif_t * s, uint8_t new_value);
int inv_icp201xx_rd_press_delta_msb(inv_icp201xx_serif_t * s, uint8_t *value);
/********************************************
Register Name: DEVICE_STATUS
Register Type: READ
Register Address: 205 (Decimal); CD (Hex)
********************************************/
#define BIT_DEVICE_STATUS_LP_SEQ_STATE_MASK			0X06
#define BIT_DEVICE_STATUS_MODE_SYNC_STATUS_MASK     0x01
                                      // 0 : Mode sync is going on, MODE_SELECT Reg is NOT accessible.
									  // 1 : MODE_SELECT Reg is accessible.
									  
#define BIT_DEVICE_STATUS_LP_SEQ_STATE_POS			1
int inv_icp201xx_rd_device_status(inv_icp201xx_serif_t * s, uint8_t *value);
int inv_icp201xx_rd_mode_sync_status(inv_icp201xx_serif_t * s, uint8_t *value);
int inv_icp201xx_rd_lp_seq_sync_status(inv_icp201xx_serif_t * s, uint8_t *value);

/********************************************
Register Name: SPI_MODE
Register Type: READ/WRITE
Register Address: 206 (Decimal); CE (Hex)
********************************************/

int inv_icp201xx_rd_fifo(inv_icp201xx_serif_t * s, uint8_t len, uint8_t* value, uint8_t fifo_read_offset);

/********************************************
Register Name:
Bank         : Main registers
Register Type: READ/WRITE
Register Address: 211(Decimal); 0XD3 (Hex)
********************************************/

int inv_icp201xx_rd_version(inv_icp201xx_serif_t  * s, uint8_t *value);
#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _INV_ICP201XX_DRIVER_H_ */
