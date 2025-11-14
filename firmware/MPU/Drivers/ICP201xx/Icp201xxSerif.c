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


/** @defgroup DriverIcp201xxSerif Icp201xx driver serif
 *  @brief Interface for low-level serial (I2C/I3C/SPI) access
 *  @ingroup  DriverIcp201xx
 *  @{
 */


#include "InvError.h"
#include "Icp201xxSerif.h"



#define ICP201XX_SERIF_SPI_REG_WRITE_CMD   0X33
#define ICP201XX_SERIF_SPI_REG_READ_CMD    0X3C


/** @brief Reads data from a register on mems.
 * @param[in]  s Pointer to the driver serial interface context structure inv_icp201xx_serif_t
 * @param[in]  reg    register address
 * @param[in]  len length of data
 * @param[out] buf   output data from the register
 * @return     0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_serif_read_reg(inv_icp201xx_serif_t * s,uint8_t reg, uint32_t len, uint8_t * buf )
{
	if ( 0 == s)
		return INV_ERROR_BAD_ARG;

	if(len > s->max_read)
		return INV_ERROR_SIZE;
	
	if ( s->if_mode == ICP201XX_IF_3_WIRE_SPI || s->if_mode == ICP201XX_IF_4_WIRE_SPI )
	{
		uint8_t cmd_buf[2] = {0};
		cmd_buf[0] = ICP201XX_SERIF_SPI_REG_READ_CMD;
		cmd_buf[1] = reg;

		if(s->write_reg(s->context, 0, cmd_buf, 2) != 0)
			return INV_ERROR_TRANSPORT;
	}
	
	if(s->read_reg(s->context, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;


	return INV_ERROR_SUCCESS;
}


/** @brief Writes data to a register on mems.
 * @param[in]  s Pointer to the driver serial interface context structure inv_icp201xx_serif_t
 * @param[in]  reg    register address
 * @param[in]  buf   data to be written to the register
 * @param[in]  len number of bytes to be written
 * @return     0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_serif_write_reg(inv_icp201xx_serif_t * s,uint8_t reg, uint32_t len, const uint8_t * buf)
{
	if ( 0 == s)
		return INV_ERROR_BAD_ARG;

	if(len > s->max_write)
		return INV_ERROR_SIZE;

	if ( s->if_mode == ICP201XX_IF_3_WIRE_SPI || s->if_mode == ICP201XX_IF_4_WIRE_SPI )
	{
		uint8_t cmd_buf[2] = {0};
		cmd_buf[0] = ICP201XX_SERIF_SPI_REG_WRITE_CMD;
		cmd_buf[1] = reg;

		if(s->write_reg(s->context, 0, cmd_buf, 2) != 0)
			return INV_ERROR_TRANSPORT;
	}

	if(s->write_reg(s->context, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;

	return INV_ERROR_SUCCESS;
}


/** @} */