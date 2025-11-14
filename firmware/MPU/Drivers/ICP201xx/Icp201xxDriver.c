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


#include "Icp201xxDefs.h"
#include "Icp201xxSerif.h"
#include "Icp201xxDriver.h"
#include "Icp201xxExtFunc.h"

#include "InvError.h"

/*********************** Generic reg read/write ***********************************/
int inv_icp201xx_rd_reg(inv_icp201xx_serif_t  * s, uint8_t reg, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, reg, 1, value);
}
int inv_icp201xx_wr_reg(inv_icp201xx_serif_t  * s, uint8_t reg, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, reg, 1, &new_value);
}


int inv_icp201xx_wr_pefe_offset_trim(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	int status;
	uint8_t reg_value;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_TRIM1_MSB, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_PEFE_OFFSET_TRIM_MASK) ) | ( new_value  ) ;

	status = inv_icp201xx_serif_write_reg(s, MPUREG_TRIM1_MSB, 1, &reg_value);

	return status;
}

int inv_icp201xx_wr_pefe_gain_trim(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	int status;
	uint8_t reg_value;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_TRIM2_MSB, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_PEFE_GAIN_TRIM_MASK) ) | ( new_value <<BIT_PEFE_GAIN_TRIM_POS ) ;

	status = inv_icp201xx_serif_write_reg(s, MPUREG_TRIM2_MSB, 1, &reg_value);

	return status;
}

int inv_icp201xx_wr_hfosc_trim(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	int status;
	uint8_t reg_value;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_TRIM2_LSB, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_HFOSC_OFFSET_TRIM_MASK) ) | ( new_value  ) ;

	status = inv_icp201xx_serif_write_reg(s, MPUREG_TRIM2_LSB, 1, &reg_value);

	return status;
}
int inv_icp201xx_rd_device_id(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_DEVICE_ID, 1, value);
}

/********************************************
Register Name: OTP Config 1
Register Type: READ/WRITE
Register Address: 172 (Decimal); AC (Hex)
********************************************/
int inv_icp201xx_wr_otp_write_switch(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_OTP_CFG1, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_OTP_CONFIG1_WRITE_SWITCH_MASK) ) | ( new_value << BIT_OTP_CONFIG1_WRITE_SWITCH_POS ) ;

	status = inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_OTP_CFG1, 1, &reg_value);

	return status;

}

int inv_icp201xx_wr_otp_enable(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_OTP_CFG1, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_OTP_CONFIG1_OTP_ENABLE_MASK) ) | ( new_value  ) ;

	status = inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_OTP_CFG1, 1, &reg_value);

	return status;

}


/********************************************
Register Name: OTP MR LSB
Register Type: READ/WRITE
Register Address: 173 (Decimal); AD (Hex)
********************************************/
int inv_icp201xx_wr_otp_mr_lsb(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_MR_LSB, 1, &new_value);
	
}

/********************************************
Register Name: OTP MR MSB
Register Type: READ/WRITE
Register Address: 174 (Decimal); AE (Hex)
********************************************/
int inv_icp201xx_wr_otp_mr_msb(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_MR_MSB, 1, &new_value);
	
}

/********************************************
Register Name: OTP MRA LSB
Register Type: READ/WRITE
Register Address: 175 (Decimal); AF (Hex)
********************************************/
int inv_icp201xx_wr_otp_mra_lsb(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_MRA_LSB, 1, &new_value);
	
}
int inv_icp201xx_rd_otp_mra_lsb(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_MRA_LSB, 1, value);
}
/********************************************
Register Name: OTP MRA MSB
Register Type: READ/WRITE
Register Address: 176 (Decimal); B0 (Hex)
********************************************/
int inv_icp201xx_wr_otp_mra_msb(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_MRA_MSB, 1, &new_value);
}

int inv_icp201xx_rd_otp_mra_msb(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_MRA_MSB, 1, value);
}


/********************************************
Register Name: OTP MRB LSB
Register Type: READ/WRITE
Register Address: 177 (Decimal); B1 (Hex)
********************************************/
int inv_icp201xx_wr_otp_mrb_lsb(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_MRB_LSB, 1, &new_value);
}
/********************************************
Register Name: OTP MRB MSB
Register Type: READ/WRITE
Register Address: 178 (Decimal); B2 (Hex)
********************************************/
int inv_icp201xx_wr_otp_mrb_msb(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_MRB_MSB, 1, &new_value);
}

/********************************************
Register Name: OTP ADDR
Register Type: READ/WRITE
Register Address: 181 (Decimal); B5 (Hex)
********************************************/
int inv_icp201xx_wr_otp_addr(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_OTP_ADDR, 1, &new_value);
}

int inv_icp201xx_rd_otp_addr(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_OTP_ADDR, 1, value);
}

/********************************************
Register Name: OTP CMD
Register Type: READ/WRITE
Register Address: 182 (Decimal); B6 (Hex)
********************************************/
int inv_icp201xx_wr_otp_cmd(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_OTP_CMD, 1, &new_value);
}

int inv_icp201xx_rd_otp_cmd(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_OTP_CMD, 1, value);
}

/********************************************
Register Name: OTP Read Reg
Register Type: READ
Register Address: 184 (Decimal); B8 (Hex)
********************************************/

int inv_icp201xx_rd_otp_reg_data(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_RD_DATA, 1, value);
}

/********************************************
Register Name: OTP status
Register Type: READ
Register Address: 185 (Decimal); B9 (Hex)
********************************************/
int inv_icp201xx_rd_otp_status(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_OTP_STATUS, 1, value);
}

/********************************************
Register Name: OTP Debug2
Register Type: READ/WRITE
Register Address:  180(Decimal); BC (Hex)
********************************************/
int inv_icp201xx_wr_otp_dbg2_reset(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_OTP_DEBUG2, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_OTP_DBG2_RESET_MASK) ) | ( new_value << BIT_OTP_DBG2_RESET_POS  ) ;

	status = inv_icp201xx_serif_write_reg(s, MPUREG_OTP_DEBUG2, 1, &reg_value);

	return status;

}

/********************************************
Register Name: Master lock
Register Type: WRITE
Register Address:  190(Decimal); BE (Hex)
********************************************/
int inv_icp201xx_wr_master_lock(inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_MASTER_LOCK, 1, &new_value);
}

/********************************************
Register Name: OTP STATUS2
Register Type: WRITE
Register Address:  191(Decimal); BF (Hex)
********************************************/
int inv_icp201xx_rd_boot_up_status (inv_icp201xx_serif_t  * s, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	status =  inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_OTP_STATUS2, 1, &reg_value);
	if(status)
		return status;
		
	*value = reg_value & BIT_OTP_STATUS2_BOOTUP_STATUS_MASK ;
	
	return status;
}

int inv_icp201xx_wr_boot_up_status (inv_icp201xx_serif_t  * s,uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	status =  inv_icp201xx_serif_read_reg(s, MPUREG_OTP_MTP_OTP_STATUS2, 1, &reg_value);
	if(status)
		return status;
	
	reg_value = reg_value & (~BIT_OTP_STATUS2_BOOTUP_STATUS_MASK) | new_value  ;
	
	return inv_icp201xx_serif_write_reg(s, MPUREG_OTP_MTP_OTP_STATUS2, 1, &reg_value);;
}

/* ########################################################################################################## */


int inv_icp201xx_wr_mode_select(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	int status;
	uint8_t read_value=0;
	
	do {
		inv_icp201xx_rd_mode_sync_status(s,&read_value);
		if ( read_value  )
			break;
		inv_icp201xx_sleep_us(500);
	}while(1);

	status = inv_icp201xx_serif_write_reg(s, MPUREG_MODE_SELECT, 1, &new_value);

	return status;
}


int inv_icp201xx_rd_mode_select(inv_icp201xx_serif_t * s, uint8_t *value)
{
	
	int status;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, value);

	return status;
}
/***********************************************************************************************/
int inv_icp201xx_wr_meas_config(inv_icp201xx_serif_t  * s, icp201xx_op_mode_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	uint8_t read_value=0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_MEAS_CONFIG_MASK) ) | ( new_value << BIT_MEAS_CONFIG_POS) ;
	
	do {
		inv_icp201xx_rd_mode_sync_status(s,&read_value);
		if ( read_value  )
			break;

		inv_icp201xx_sleep_us(500);
	}while(1);
	
	status = inv_icp201xx_serif_write_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_meas_config(inv_icp201xx_serif_t * s, icp201xx_op_mode_t *value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
		return status;

	*value = (icp201xx_op_mode_t)(( reg_value & BIT_MEAS_CONFIG_MASK ) >> BIT_MEAS_CONFIG_POS) ;
	
	return status;
}

int inv_icp201xx_wr_forced_meas_trigger(inv_icp201xx_serif_t  * s, icp201xx_forced_meas_trigger_t new_value)
{
	int status;
	uint8_t reg_value = 0, read_value =0 ;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_FORCED_MEAS_TRIGGER_MASK) ) | ( new_value << BIT_FORCED_MEAS_TRIGGER_POS) ;
	do {
		inv_icp201xx_rd_mode_sync_status(s,&read_value);
		if ( read_value  )
			break;
	
		inv_icp201xx_sleep_us(500);
	}while(1);
	
	status = inv_icp201xx_serif_write_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_forced_meas_trigger(inv_icp201xx_serif_t * s, icp201xx_forced_meas_trigger_t *value)
{
	int status;
	uint8_t reg_value;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
		return status;
	
	*value = (icp201xx_forced_meas_trigger_t)(( reg_value & BIT_FORCED_MEAS_TRIGGER_MASK ) >> BIT_FORCED_MEAS_TRIGGER_POS) ; 
	
	return status;
}

int inv_icp201xx_wr_meas_mode(inv_icp201xx_serif_t  * s, icp201xx_meas_mode_t new_value)
{
	int status;
	uint8_t reg_value = 0, read_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_MEAS_MODE_MASK) ) | ( new_value << BIT_FORCED_MEAS_MODE_POS) ;
	do {
		inv_icp201xx_rd_mode_sync_status(s,&read_value);
		if ( read_value  )
			break;
		inv_icp201xx_sleep_us(500);
	}while(1);
	
	status = inv_icp201xx_serif_write_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_meas_mode(inv_icp201xx_serif_t * s, icp201xx_meas_mode_t *value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
		return status;
	
	*value = (icp201xx_meas_mode_t)(( reg_value & BIT_MEAS_MODE_MASK ) >> BIT_FORCED_MEAS_MODE_POS) ; 
	
	return status;
}


int inv_icp201xx_wr_pow_mode(inv_icp201xx_serif_t  * s, icp201xx_power_mode_t new_value)
{
	int status;
	uint8_t reg_value = 0, read_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_POWER_MODE_MASK) ) | ( new_value << BIT_FORCED_POW_MODE_POS) ;
	
	do {
		inv_icp201xx_rd_mode_sync_status(s,&read_value);
		if ( read_value  )
			break;
		inv_icp201xx_sleep_us(500);
	}while(1);
		
	status = inv_icp201xx_serif_write_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	return status;
}

int inv_icp201xx_rd_pow_mode(inv_icp201xx_serif_t * s, icp201xx_power_mode_t *value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
		return status;
	
	*value = (icp201xx_power_mode_t)(( reg_value & BIT_POWER_MODE_MASK ) >> BIT_FORCED_POW_MODE_POS) ; 
	
	return status;
}


int inv_icp201xx_wr_fifo_readout_mode(inv_icp201xx_serif_t  * s, icp201xx_FIFO_readout_mode_t new_value)
{
	int status;
	uint8_t reg_value = 0, read_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_FIFO_READOUT_MODE_MASK) ) | ( new_value ) ;
	
	do {
		inv_icp201xx_rd_mode_sync_status(s,&read_value);
		if ( read_value  )
			break;
		//return 0;
		inv_icp201xx_sleep_us(500);
	}while(1);
	
	status = inv_icp201xx_serif_write_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);
	
	return status;
}


int inv_icp201xx_rd_fifo_readout_mode(inv_icp201xx_serif_t * s, icp201xx_FIFO_readout_mode_t *value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_MODE_SELECT, 1, &reg_value);

	if(status)
		return status;
	
	*value = (icp201xx_FIFO_readout_mode_t)(( reg_value & BIT_FIFO_READOUT_MODE_MASK )) ; 
	
	return status;
}

/**************************************    End of Mode Select ********************************************************/

int inv_icp201xx_wr_int_status(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_INTERRUPT_STATUS, 1, &new_value);
}

int inv_icp201xx_rd_int_status(inv_icp201xx_serif_t * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_INTERRUPT_STATUS, 1, value);
}


int inv_icp201xx_wr_int_mask(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_INTERRUPT_MASK, 1, &new_value);
}

int inv_icp201xx_rd_int_mask(inv_icp201xx_serif_t * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_INTERRUPT_MASK, 1, value);
}


int inv_icp201xx_rd_fifo_config(inv_icp201xx_serif_t * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_CONFIG, 1, value);
}

int inv_icp201xx_wr_fifo_wm_high(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_CONFIG, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_FIFO_WM_HIGH_MASK) ) | ( new_value << BIT_FIFO_WM_HIGH_POS) ;
	
	return inv_icp201xx_serif_write_reg(s, MPUREG_FIFO_CONFIG, 1, &reg_value);

}
int inv_icp201xx_rd_fifo_wm_high(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_CONFIG, 1, &reg_value);

	if(status)
		return status;
	
	*value = ( reg_value & BIT_FIFO_WM_HIGH_MASK ) >> BIT_FIFO_WM_HIGH_POS; 
	
	return status;
	
}
int inv_icp201xx_wr_fifo_wm_low(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_CONFIG, 1, &reg_value);

	if(status)
	return status;

	reg_value = ( reg_value & (~BIT_FIFO_WM_LOW_MASK) ) | ( new_value ) ;
	
	status = inv_icp201xx_serif_write_reg(s, MPUREG_FIFO_CONFIG, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_fifo_wm_low(inv_icp201xx_serif_t * s, uint8_t  *value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_CONFIG, 1, &reg_value);

	if(status)
		return status;
	
	*value = ( reg_value & BIT_FIFO_WM_LOW_MASK ); 
	
	return status;
}


int inv_icp201xx_wr_fifo_config(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_FIFO_CONFIG, 1, &new_value);
}


int inv_icp201xx_wr_spi_mode(inv_icp201xx_serif_t * s, icp201xx_spi_mode_t new_value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_SPI_MODE, 1, &reg_value);

	if(status)
		return status;

	reg_value = ( reg_value & (~BIT_FIFO_SPI_MODE_MASK) ) | ( new_value ) ;
	
	status = inv_icp201xx_serif_write_reg(s, MPUREG_SPI_MODE, 1, &reg_value);
	
	return status;
}

int inv_icp201xx_rd_spi_mode(inv_icp201xx_serif_t * s, icp201xx_spi_mode_t *value)
{
	int status;
	uint8_t reg_value = 0;
	status = inv_icp201xx_serif_read_reg(s, MPUREG_SPI_MODE, 1, &reg_value);

	if(status)
		return status;
	
	*value = (icp201xx_spi_mode_t)( reg_value & BIT_FIFO_SPI_MODE_MASK ); 
	
	return status;
	
}




/********************************************************** FIFO FILL ********************************************/

int inv_icp201xx_rd_fifo_fill(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_FILL, 1, value);

	return status;
}

int inv_icp201xx_wr_flush_fifo(inv_icp201xx_serif_t * s)
{
	int status;
	uint8_t read_val = 0;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_FILL, 1, &read_val);
	if ( status )
		return status;
	read_val |= 0x80;

    status = inv_icp201xx_serif_write_reg(s, MPUREG_FIFO_FILL, 1, &read_val);

	return status;
}

int inv_icp201xx_rd_fifo_empty_status(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	uint8_t read_val = 0;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_FILL, 1, &read_val);
	if ( status )
		return status;
	*value = ( read_val & BIT_FIFO_EMPTY_STATUS_MASK ) >>  BIT_FIFO_EMPTY_POS;
	
	return status;
}

int inv_icp201xx_rd_fifo_full_status(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	uint8_t read_val = 0;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_FILL, 1, &read_val);
	if ( status )
		return status;
	*value = ( read_val & BIT_FIFO_FULL_STATUS_MASK ) >>  BIT_FIFO_FULL_POS;
	
	return status;
}

int inv_icp201xx_rd_fifo_level(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	uint8_t read_val = 0;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_FIFO_FILL, 1, &read_val);
	if ( status )
		return status;
	*value = (uint8_t)( read_val & BIT_FIFO_LEVEL_MASK ) ;
	/* Max value for fifo level is 0x10 for any values higher than 0x10 function should return error */
	if (( *value & 0x10 )  && ( *value & 0x0F) )
		status = INV_ERROR_TRANSPORT;
	return status;
}

/************************************** End of FIFO FILL *************************************************/

int inv_icp201xx_rd_fifo(inv_icp201xx_serif_t * s, uint8_t len, uint8_t* value,uint8_t fifo_read_offset)
{
	return (inv_icp201xx_serif_read_reg(s, (MPUREG_FIFO_BASE+fifo_read_offset), len, value));
	
}


int inv_icp201xx_wr_press_abs_lsb(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_PRESS_ABS_LSB, 1, &new_value);
}
int inv_icp201xx_rd_press_abs_lsb(inv_icp201xx_serif_t * s, uint8_t *value)
{
	return (inv_icp201xx_serif_read_reg(s, MPUREG_PRESS_ABS_LSB, 1, value));
}

int inv_icp201xx_wr_press_abs_msb(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_PRESS_ABS_MSB, 1, &new_value);
}
int inv_icp201xx_rd_press_abs_msb(inv_icp201xx_serif_t * s, uint8_t *value)
{
	return (inv_icp201xx_serif_read_reg(s, MPUREG_PRESS_ABS_MSB, 1, value));
}

int inv_icp201xx_wr_press_delta_lsb(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_PRESS_DELTA_LSB, 1, &new_value);
}
int inv_icp201xx_rd_press_delta_lsb(inv_icp201xx_serif_t * s, uint8_t *value)
{
	return (inv_icp201xx_serif_read_reg(s, MPUREG_PRESS_DELTA_LSB, 1, value));
}

int inv_icp201xx_wr_press_delta_msb(inv_icp201xx_serif_t * s, uint8_t new_value)
{
	return inv_icp201xx_serif_write_reg(s, MPUREG_PRESS_DELTA_MSB, 1, &new_value);
}
int inv_icp201xx_rd_press_delta_msb(inv_icp201xx_serif_t * s, uint8_t *value)
{
	return (inv_icp201xx_serif_read_reg(s, MPUREG_PRESS_DELTA_MSB, 1, value));
}


int inv_icp201xx_rd_device_status(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	uint8_t read_val;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_DEVICE_STATUS, 1, &read_val);
	
	return status;
}


int inv_icp201xx_rd_mode_sync_status(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	uint8_t read_val;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_DEVICE_STATUS, 1, &read_val);
	
	if ( status )
		return status;
	
	*value = read_val & BIT_DEVICE_STATUS_MODE_SYNC_STATUS_MASK ;
	
	return status;
}

int inv_icp201xx_rd_lp_seq_sync_status(inv_icp201xx_serif_t * s, uint8_t *value)
{
	int status;
	uint8_t read_val;
	
	status = inv_icp201xx_serif_read_reg(s, MPUREG_DEVICE_STATUS, 1, &read_val);
	
	if ( status )
		return status;
	
	*value = (read_val & BIT_DEVICE_STATUS_LP_SEQ_STATE_MASK) >> BIT_DEVICE_STATUS_LP_SEQ_STATE_POS ;
	
	return status;
}

int inv_icp201xx_rd_version(inv_icp201xx_serif_t  * s, uint8_t *value)
{
	return inv_icp201xx_serif_read_reg(s, MPUREG_VERSION, 1, value);
}

/** @} */

