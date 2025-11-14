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

#ifndef _INV_ICP201XX_SERIF_H_
#define _INV_ICP201XX_SERIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "InvError.h"
#include <stdint.h>

#define ICP201XX_AD0_LOW_I2C_ADDR			(0x63)
#define ICP201XX_AD0_HIGH_I2C_ADDR			(0x64)


typedef enum icp201xx_if {
	ICP201XX_IF_I2C = 0,
	ICP201XX_IF_I3C,
	ICP201XX_IF_3_WIRE_SPI,
	ICP201XX_IF_4_WIRE_SPI
}icp201xx_if_t;

/** @brief Invpres serial interface
 */
typedef struct inv_icp201xx_serif {
	void *     context;   /*!< reserved */
	int      (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len);        /*!< function pointer to the low-level serial interface read function implemented by application layer */
	int      (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len); /*!< function pointer to the low-level serial interface write function implemented by application laye*/
	uint32_t   max_read;  /*!< maximum number of bytes allowed per serial read */
	uint32_t   max_write; /*!< maximum number of bytes allowed per serial write */
	icp201xx_if_t if_mode; /*!< Interface mode */
}inv_icp201xx_serif_t;



/** @brief Reads data from a register on mems.
 * @param[in]  s Pointer to the driver serial interface context structure inv_icp201xx_serif_t
 * @param[in]  reg    register address
 * @param[in]  len length of data
 * @param[out] buf   output data from the register
 * @return     0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_serif_read_reg(inv_icp201xx_serif_t * s,uint8_t reg, uint32_t len, uint8_t * buf );

/** @brief Writes data to a register on mems.
 * @param[in]  s Pointer to the driver serial interface context structure inv_icp201xx_serif_t
 * @param[in]  reg    register address
 * @param[in]  buf   data to be written to the register
 * @param[in]  len number of bytes to be written
 * @return     0 in case of success, negative value on error. See enum inv_status
 */
int inv_icp201xx_serif_write_reg(inv_icp201xx_serif_t * s,uint8_t reg, uint32_t len, const uint8_t * buf);


#ifdef __cplusplus
}
#endif

#endif /* _INV_ICP201XX_SERIF_H_ */

/** @} */
