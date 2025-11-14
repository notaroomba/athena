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


#include "Icp201xx.h"
#include "Icp201xxDriver.h"
#include "Icp201xxExtFunc.h"

int inv_icp201xx_soft_reset(inv_icp201xx_t * s)
{
	int status = INV_ERROR_SUCCESS;
	uint8_t int_status;
	status |= inv_icp201xx_wr_mode_select(&(s->serif),0); 
	inv_icp201xx_sleep_us(2000);
	
	status |= inv_icp201xx_flush_fifo(s);
	status |= inv_icp201xx_set_fifo_notification_config(s,0,0,0);
		
	status |= inv_icp201xx_wr_int_mask(&(s->serif),0xFF);
	status |= inv_icp201xx_rd_int_status(&(s->serif),&int_status);
	if ( int_status )
		status |= inv_icp201xx_clear_int_status(s,int_status);
	
	return status;
}

int inv_icp201xx_init(inv_icp201xx_t * s,const inv_icp201xx_serif_t * serif)
{
	int status = INV_ERROR_SUCCESS;
	s->serif = *serif;
	if ( s->serif.if_mode == ICP201XX_IF_3_WIRE_SPI )
	{
		status |= inv_icp201xx_wr_spi_mode(&(s->serif),ICP201XX_SPI_MODE_3_WIRE);
	} else if ( s->serif.if_mode == ICP201XX_IF_4_WIRE_SPI )
	{
		status |= inv_icp201xx_wr_spi_mode(&(s->serif),ICP201XX_SPI_MODE_4_WIRE);
	} else if ( s->serif.if_mode == ICP201XX_IF_I2C || s->serif.if_mode == ICP201XX_IF_I3C)
	{
		/** dummy write transaction to address 0xEE to make sure we give enough time for ICP201xx to init **/
		do {
			status = inv_icp201xx_wr_reg(&(s->serif),0xEE,0xf0);
			if ( !status )
				break;
			inv_icp201xx_sleep_us(5);
		}while(1);
	}

	return status;
}

int  inv_icp201xx_config(inv_icp201xx_t * s,icp201xx_op_mode_t op_mode,icp201xx_FIFO_readout_mode_t fifo_read_mode)
{
	uint8_t reg_value = 0;
	int status = INV_ERROR_SUCCESS;
	if ( op_mode >= ICP201XX_OP_MODE_MAX )
		return INV_ERROR ;
	
	status |= inv_icp201xx_wr_mode_select(&(s->serif),reg_value); /* Set Sensor in Standby mode */

	status |= inv_icp201xx_wr_flush_fifo(&(s->serif));
	
	status |= inv_icp201xx_rd_int_status(&(s->serif),&reg_value);
	if ( reg_value )
		status |= inv_icp201xx_clear_int_status(s,reg_value);
	
	
	status |= inv_icp201xx_wr_forced_meas_trigger(&(s->serif),ICP201XX_FORCE_MEAS_STANDBY);
	
	status |= inv_icp201xx_wr_pow_mode(&(s->serif),ICP201XX_POWER_MODE_NORMAL);
	
	status |= inv_icp201xx_wr_fifo_readout_mode(&(s->serif),fifo_read_mode);
	s->fifo_readout_mode = fifo_read_mode;
	
	status |= inv_icp201xx_wr_meas_config(&(s->serif),op_mode);
	
	status |= inv_icp201xx_wr_meas_mode(&(s->serif),ICP201XX_MEAS_MODE_CONTINUOUS);
	
	return status;
}

int  inv_icp201xx_adv_config(inv_icp201xx_t * s,
							icp201xx_op_mode_t op_mode,
							icp201xx_meas_mode_t meas_mode,
							icp201xx_forced_meas_trigger_t forced_meas_trigger,
							icp201xx_power_mode_t  pow_mode,
							icp201xx_FIFO_readout_mode_t fifo_readout_mode)
{
	uint8_t reg_value = 0;
	int status = INV_ERROR_SUCCESS;
	if ( op_mode >= ICP201XX_OP_MODE_MAX )
		return INV_ERROR ;
	
	status |= inv_icp201xx_wr_mode_select(&(s->serif),reg_value); /* Set Sensor in Standby mode */
	
	status |= inv_icp201xx_wr_flush_fifo(&(s->serif));
	
	status |= inv_icp201xx_wr_meas_config(&(s->serif),op_mode);
	
	status |= inv_icp201xx_wr_forced_meas_trigger(&(s->serif),forced_meas_trigger);
	
	status |= inv_icp201xx_wr_pow_mode(&(s->serif),pow_mode);
	
	status |= inv_icp201xx_wr_fifo_readout_mode(&(s->serif),fifo_readout_mode);
	s->fifo_readout_mode = fifo_readout_mode;
	
	status |= inv_icp201xx_wr_meas_mode(&(s->serif),meas_mode);

	return status;
}

int inv_icp201xx_trigger_meas(inv_icp201xx_t * s)
{
	return inv_icp201xx_wr_forced_meas_trigger(&(s->serif),ICP201XX_FORCE_MEAS_TRIGGER_FORCE_MEAS);;
}


int inv_icp201xx_set_standby(inv_icp201xx_t * s)
{
	int status = INV_ERROR_SUCCESS;
	status |= inv_icp201xx_wr_forced_meas_trigger(&(s->serif),ICP201XX_FORCE_MEAS_STANDBY);
	
	status |= inv_icp201xx_wr_pow_mode(&(s->serif),ICP201XX_POWER_MODE_NORMAL);
	
	status |= inv_icp201xx_wr_meas_mode(&(s->serif),ICP201XX_MEAS_MODE_FORCED_TRIGGER);
	
	status |= inv_icp201xx_wr_flush_fifo(&(s->serif));
	
	return status;
}

int inv_icp201xx_set_continuous_meas_mode(inv_icp201xx_t * s)
{
	int status = INV_ERROR_SUCCESS;

	status |= inv_icp201xx_wr_meas_mode(&(s->serif),ICP201XX_MEAS_MODE_CONTINUOUS);
	
	return status;
}

int inv_icp201xx_set_press_notification_config(inv_icp201xx_t * s,uint8_t press_int_mask,int16_t press_abs , int16_t press_delta)
{
	uint8_t reg_value = 0;
	int status = INV_ERROR_SUCCESS;
	
	status |= inv_icp201xx_rd_int_mask( &(s->serif),&reg_value);
	
	reg_value =  (reg_value |  (ICP201XX_INT_MASK_PRESS_ABS | ICP201XX_INT_MASK_PRESS_DELTA)) & ~press_int_mask;
	
	status |= inv_icp201xx_wr_int_mask(&(s->serif),reg_value);
	
	if ( press_int_mask & ICP201XX_INT_MASK_PRESS_ABS  )
	{
		status |= inv_icp201xx_wr_press_abs_lsb(&(s->serif),(uint8_t) (press_abs & 0xff));
		status |= inv_icp201xx_wr_press_abs_msb(&(s->serif),(uint8_t) ((press_abs >> 8) & 0xff));
	}
	else
	{
		status |= inv_icp201xx_wr_press_abs_lsb(&(s->serif),0);
		status |= inv_icp201xx_wr_press_abs_msb(&(s->serif),0);
	}
	
	
	if ( press_int_mask & ICP201XX_INT_MASK_PRESS_DELTA  )
	{
		status |= inv_icp201xx_wr_press_delta_lsb(&(s->serif),(uint8_t) (press_delta & 0xff));
		status |= inv_icp201xx_wr_press_delta_msb(&(s->serif),(uint8_t) ((press_delta >> 8) & 0xff));
	}
	else
	{
		status |= inv_icp201xx_wr_press_delta_lsb(&(s->serif),0);
		status |= inv_icp201xx_wr_press_delta_msb(&(s->serif),0);
	}
	return status;
}


int inv_icp201xx_get_press_notification_config(inv_icp201xx_t * s,uint8_t *press_int_mask,int16_t *press_abs , int16_t *press_delta)
{
	uint8_t reg_value = 0,lsb=0,msb=0;
	int status = INV_ERROR_SUCCESS;
	
	status |= inv_icp201xx_rd_int_mask( &(s->serif),&reg_value);
	
	*press_int_mask = ( reg_value & (ICP201XX_INT_MASK_PRESS_ABS|ICP201XX_INT_MASK_PRESS_DELTA ) );
	
	status |= inv_icp201xx_rd_press_abs_lsb(&(s->serif),&lsb);
	status |= inv_icp201xx_rd_press_abs_msb(&(s->serif),&msb);
	
	*press_abs = (msb << 8) | lsb ;
	
	status |= inv_icp201xx_rd_press_delta_lsb(&(s->serif),&lsb);
	status |= inv_icp201xx_rd_press_delta_msb(&(s->serif),&msb);
	
	*press_delta = (msb << 8) | lsb ;
	
	return status;
}

int inv_icp201xx_set_fifo_notification_config(inv_icp201xx_t * s,uint8_t fifo_int_mask, uint8_t fifo_wmk_high,uint8_t fifo_wmk_low)
{
	uint8_t reg_value = 0;
	int status = INV_ERROR_SUCCESS;
	
	if ( fifo_wmk_high > 0xf || fifo_wmk_low > 0xf )
		return 0;
	/** FIFO config **/
	reg_value = (fifo_wmk_high << 4) | fifo_wmk_low;
		
	status |= inv_icp201xx_wr_fifo_config(&(s->serif),reg_value);
	
	status |=inv_icp201xx_rd_int_mask( &(s->serif),&reg_value);
	reg_value =  (reg_value |  (ICP201XX_INT_MASK_FIFO_WMK_HIGH | ICP201XX_INT_MASK_FIFO_OVER_FLOW |ICP201XX_INT_MASK_FIFO_WMK_LOW |ICP201XX_INT_MASK_FIFO_UNDER_FLOW)) & ~fifo_int_mask;
	
	status |= inv_icp201xx_wr_int_mask(&(s->serif),reg_value);
	return status;
}

int inv_icp201xx_get_fifo_notification_config(inv_icp201xx_t * s,uint8_t *fifo_int_mask, uint8_t *fifo_wmk_high,uint8_t *fifo_wmk_low)
{
	uint8_t reg_value = 0;
	int status = INV_ERROR_SUCCESS;
	
	status |= inv_icp201xx_rd_fifo_config(&(s->serif),&reg_value);
	
	*fifo_wmk_low = (reg_value ) & 0x0f ;
	*fifo_wmk_high = (reg_value >> 4) & 0x0f ;
	
	status |= inv_icp201xx_rd_int_mask( &(s->serif) ,&reg_value);
	
	*fifo_int_mask = reg_value  & (ICP201XX_INT_MASK_FIFO_WMK_HIGH | ICP201XX_INT_MASK_FIFO_OVER_FLOW |ICP201XX_INT_MASK_FIFO_WMK_LOW |ICP201XX_INT_MASK_FIFO_UNDER_FLOW ) ;
	
	return status;
}

int inv_icp201xx_flush_fifo(inv_icp201xx_t * s)
{
	return (inv_icp201xx_wr_flush_fifo(&(s->serif)));
}

int inv_icp201xx_get_int_status(inv_icp201xx_t * s,uint8_t *int_status)
{
	return (inv_icp201xx_rd_int_status(&(s->serif),int_status));
}


int inv_icp201xx_clear_int_status(inv_icp201xx_t * s,uint8_t int_status)
{
	return ( inv_icp201xx_wr_int_status(&(s->serif),int_status));
}

int inv_icp201xx_get_device_status(inv_icp201xx_t * s,uint8_t *dev_status)
{
	return (inv_icp201xx_rd_device_status(&(s->serif),dev_status));
}


int inv_icp201xx_get_fifo_count(inv_icp201xx_t * s,uint8_t *fifo_cnt)
{
	return inv_icp201xx_rd_fifo_level(&(s->serif),fifo_cnt);
}

int inv_icp201xx_get_fifo_data(inv_icp201xx_t * s,uint8_t req_packet_cnt, uint8_t *data)
{
	uint8_t fifo_read_offset = (( s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_PRES_ONLY )|| (s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_TEMP_ONLY)) ? 3 : 0;
	uint8_t packet_cnt = (( s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_PRES_ONLY )|| (s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_TEMP_ONLY)) ? (req_packet_cnt * 1 * 3 ) : (req_packet_cnt * 2 * 3) ;
	return (inv_icp201xx_rd_fifo(&(s->serif),packet_cnt,data,fifo_read_offset));
}


int inv_icp201xx_process_raw_data(inv_icp201xx_t * s,uint8_t packet_cnt, uint8_t *data,int32_t * pressure, int32_t * temperature)
{
	uint8_t i,offset=0;

	for ( i = 0 ; i < packet_cnt ; i++ )
	{
		if ( s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_PRES_TEMP)
		{
			pressure[i] = (int32_t)((( data[offset+2] & 0x0f) << 16) | (data[offset+1] << 8) | data[offset]) ;
			offset += 3;
			temperature[i] = (int32_t)(((data[offset+2] & 0x0f) << 16) | (data[offset+1] << 8) | data[offset]) ;
			offset += 3;
		} else if ( s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_TEMP_ONLY) {
			temperature[i] = (int32_t)(((data[offset+2] & 0x0f) << 16) | (data[offset+1] << 8) | data[offset]) ;
			offset += 3;
		} else if( s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_TEMP_PRES) {
			temperature[i] = (int32_t)(((data[offset+2] & 0x0f) << 16) | (data[offset+1] << 8) | data[offset]) ;
			offset += 3;
			pressure[i] = (int32_t)((( data[offset+2] & 0x0f) << 16) | (data[offset+1] << 8) | data[offset]) ;
			offset += 3;
		} else if( s->fifo_readout_mode == ICP201XX_FIFO_READOUT_MODE_PRES_ONLY) {
			pressure[i] = (int32_t)((( data[offset+2] & 0x0f) << 16) | (data[offset+1] << 8) | data[offset]) ;
			offset += 3;
		}
	}
	return 0;
}


static void inv_icp201xx_enable_write_switch_OTP_read(inv_icp201xx_t * s)
{
	volatile uint8_t reg_value = 0;
	
	/*5) set to power mode. Bring the ASIC in power mode to activate the OTP power domain and get access to the main registers
	�	mode_select.power_mode = 1 */
	reg_value = BIT_POWER_MODE_MASK;
	inv_icp201xx_wr_mode_select(&(s->serif),reg_value);
	
	inv_icp201xx_sleep_us(4000);
	
	/* 
	6)	Unlock the main registers
	�	master_lock.lock 	= 0x1f
	*/
	inv_icp201xx_wr_master_lock(&(s->serif),0x1f);

	/*
	7)	Enable the OTP and the write switch
	�	otp_config1.otp_enable = 1;
	�	otp_config1.otp_write_switch = 1;
	�	wait 10us;
	*/
	inv_icp201xx_wr_otp_enable(&(s->serif),0x01);
	inv_icp201xx_wr_otp_write_switch(&(s->serif),0x01);
	
	inv_icp201xx_sleep_us(10);
	
	/*
	
	8)	Toggle the OTP reset pin
	�	otp_dbg2.reset = 1
	�	wait 10us
	�	otp_dbg2.reset = 0
	�	wait 10us
	
	*/
	
	inv_icp201xx_wr_otp_dbg2_reset(&(s->serif),1);
	
	inv_icp201xx_sleep_us(10);
	
	inv_icp201xx_wr_otp_dbg2_reset(&(s->serif),0);
	
	inv_icp201xx_sleep_us(10);
	
	/*
	9)	Program redundant read
	�	otp_mra_lsb		= 0x04
	�	otp_mra_msb		= 0x04
	�	otp_mrb_lsb		= 0x21
	�	otp_mrb_msb		= 0x20
	�	otp_mr_lsb		= 0x10
	�	otp_mr_msb		= 0x80
	*/
	inv_icp201xx_wr_otp_mra_lsb(&(s->serif),0x04);
	inv_icp201xx_wr_otp_mra_msb(&(s->serif),0x04);
	
	inv_icp201xx_wr_otp_mrb_lsb(&(s->serif),0x21);
	inv_icp201xx_wr_otp_mrb_msb(&(s->serif),0x20);
	
	inv_icp201xx_wr_otp_mr_lsb(&(s->serif),0x10);
	inv_icp201xx_wr_otp_mr_msb(&(s->serif),0x80);
	
}


int inv_icp201xx_OTP_bootup_cfg(inv_icp201xx_t * s)
{
	int status = 0;
	uint8_t otp_status;
	uint8_t offset = 0, gain = 0,Hfosc = 0;
	uint8_t version= 0;
	volatile uint8_t bootup_status = 0;
	/* 	1)	Power-on the ASIC ( Asic is already powered on )
	    2)  Do init ( already initialized ) 
		*/
	/*3) read version register */
	status = inv_icp201xx_rd_version(&(s->serif),&version);
	if ( status )
		return status;
	if ( version == 0xB2 )
	{
		/* B2 version Asic is detected. Boot up sequence is not required for B2 Asic, so returning */
		return INV_ERROR_SUCCESS;
	}
	
	/* 4) Read boot up status and avoid re running boot up sequence if it is already done */
	status = inv_icp201xx_rd_boot_up_status(&(s->serif),&bootup_status);
	if ( status )
		return status;
	if ( bootup_status )
	{
		/* Boot up sequence is already done, not required to repeat boot up sequence */
		return INV_ERROR_SUCCESS;
	}

	/* Continue with boot up sequence for A1 */
	inv_icp201xx_enable_write_switch_OTP_read(s);
	
	/****************************************************************************/
	/*
	10)	Write the address content and read command
	�	otp_address_reg.address		= 8�hF8 	// for offset
	�	otp_command_reg.address		= 4�h0
	�	otp_command_reg.command	    = 1    // read action
	*/
	inv_icp201xx_wr_otp_addr(&(s->serif),0xf8);   // for offset
	inv_icp201xx_wr_otp_cmd(&(s->serif),0x10);
	
	/*
	11)	Wait for the OTP read to finish
	�	Monitor otp_status.busy to be 0
	*/
	do 	{
		inv_icp201xx_rd_otp_status(&(s->serif),&otp_status);
		if ( otp_status == 0 ) {
			break;
		}
		inv_icp201xx_sleep_us(1);
	}while(1);
	/*
	12)	Read the data from register otp_rdata_reg.value
	*/
	
	status |=inv_icp201xx_rd_otp_reg_data(&(s->serif),&offset);
		
		
	/****************************************************************************/
	/* 3 bit gain (OTP 249 [2:0] to MAIN, TRIM2_MSB [6:4]
	
	13)	Write the address content and read command
	�	otp_address_reg.address		= 8�hF9	// for gain
	�	otp_command_reg.address		= 4�h0
	�	otp_command_reg.command	    = 1    // read action
	*/
	inv_icp201xx_wr_otp_addr(&(s->serif),0xf9); // for gain
	inv_icp201xx_wr_otp_cmd(&(s->serif),0x10);
	
	/*
	14)	Wait for the OTP read to finish
	�	Monitor otp_status.busy to be 0
	*/
	do
	{
		inv_icp201xx_rd_otp_status(&(s->serif),&otp_status);
		if ( otp_status == 0 )
		{
			break;
		}
		inv_icp201xx_sleep_us(1);
	}while(1);
	/*
	15)	Read the data from register otp_rdata_reg.value
	*/
	status |= inv_icp201xx_rd_otp_reg_data(&(s->serif),&gain);
	
	/****************************************************************************/
	/* HFOSC */
	/* 16)
		Write the address content and read command
	�	otp_address_reg.address		= 8�hFA	// for HFosc
	�	otp_command_reg.address		= 4�h0
	�	otp_command_reg.command	    = 1    // read action
	*/
	inv_icp201xx_wr_otp_addr(&(s->serif),0xFA); // for HFosc
	inv_icp201xx_wr_otp_cmd(&(s->serif),0x10);
	
	/*
	17)	Wait for the OTP read to finish
	�	Monitor otp_status.busy to be 0
	*/
	do
	{
		inv_icp201xx_rd_otp_status(&(s->serif),&otp_status);
		if ( otp_status == 0 )
		{
			break;
		}
		inv_icp201xx_sleep_us(1);
	}while(1);
	/*
	18)	Read the data from register otp_rdata_reg.value
	*/
	status |= inv_icp201xx_rd_otp_reg_data(&(s->serif),&Hfosc);
	
	/* 
	 19) Disable OTP and write switch
	 */
	inv_icp201xx_wr_otp_enable(&(s->serif),0x0);
	inv_icp201xx_wr_otp_write_switch(&(s->serif),0x00);
	inv_icp201xx_sleep_us(10);

	/* 20,21,22 Write OTP values to main registers */
	/** Updating main reg */
	status |= inv_icp201xx_wr_pefe_offset_trim(&(s->serif),(offset & 0x3f));
	status |= inv_icp201xx_wr_pefe_gain_trim(&(s->serif),(gain & 0x07));
	status |= inv_icp201xx_wr_hfosc_trim(&(s->serif),(Hfosc & 0x7f));
		
	/* 23) Lock the main register */
	inv_icp201xx_wr_master_lock(&(s->serif),0x00);
	
	/*24) Move to stand by */
	inv_icp201xx_wr_mode_select(&(s->serif),0x00);
	
	/* 25) Update boot up status to 1 */
	status |= inv_icp201xx_wr_boot_up_status(&(s->serif),1);
	
	return status; 
		
}


int inv_icp201xx_get_devid_version(inv_icp201xx_t * s,uint8_t *device_id,uint8_t *ver)
{
	int status ; 
	
	status = inv_icp201xx_rd_device_id(&(s->serif),device_id);
	if ( status )
		return status;
	
	if ( *device_id != EXPECTED_DEVICE_ID )
		return INV_ERROR;
	
	
	
	status = inv_icp201xx_rd_version(&(s->serif),ver);
	if ( status )
		return status;
		
	return INV_ERROR_SUCCESS;  	
}

/** @} */